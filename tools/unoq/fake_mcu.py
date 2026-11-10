#!/usr/bin/env python3
# Author: Roberto Navoni, member of the ArduPilot Dev Team
# Contact: r.navoni74@gmail.com
# Developed by Roberto Navoni — DelphyAI LAB
# For information: r.navoni74@gmail.com
"""Stand-in for the Uno Q microcontroller and the relay in front of it.

Listens on UDP where ArduPilot's joint-bus port points (ardurover --serial2 udp:127.0.0.1:9003)
and behaves like sketch.ino as seen from the host: a state packet every 10 ms, the command
acceptance rules (source age, sequence order, ARM only with a fresh IMU), torque state, the
calibration and servo-setup sessions with their 250 ms host watchdog, voltage and raw-encoder
reports. Servos are first-order lags towards the target; the IMU reports a fixed attitude that a
test can change. Nothing here models the robot's dynamics: it checks the link and the autopilot's
behaviour on it, not the walk.

    tools/unoq/fake_mcu.py                 # run, print one status line per second
    tools/unoq/fake_mcu.py --tilt 80       # lying on its face

Tests use the FakeMcu class directly (tools/unoq/bench_test.py).
"""
from __future__ import annotations

import argparse
import math
import select
import socket
import struct
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'unoq' / 'app' / 'python'))
import frames  # noqa: E402

STATE = struct.Struct('<4sBBH10I6h45f')
VOLTAGE = struct.Struct('<4sIIH15B')
COMMAND = struct.Struct('<4sIIIBBH15f')
CAL = struct.Struct('<4sIBB')
CAL_REPLY = struct.Struct('<4sIBB')
SERVO = struct.Struct('<4sBBBBBBH')
RAW = struct.Struct('<4sHH15H')
SERVO_REPLY = struct.Struct('<4sBBBBHHBB')

DISARM, ARM, POLICY, HOLD = range(4)
HOST_NORMAL, HOST_CAL, HOST_SERVO = range(3)
MOTOR_IDS = [10, 11, 12, 13, 14, 20, 21, 22, 23, 24, 30, 31, 32, 33, 34]
TICK = 0.01
MAX_SOURCE_AGE_US = 150000
HOST_HEARTBEAT_S = 0.25
SERVO_TAU = 0.03            # s, position lag of the simulated servos


def u32(v: float) -> int:
    return int(v) & 0xFFFFFFFF


class FakeMcu:
    def __init__(self, host='127.0.0.1', port=9003, rest_deg=None, tilt_deg=0.0):
        self.addr = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(self.addr)
        self.peer = None
        self.deframer = frames.Deframer(accept=(frames.COMMAND, frames.CAL, frames.SERVO))
        self.lock = threading.Lock()
        self.t0 = time.perf_counter()
        # servos
        self.ids = list(MOTOR_IDS)
        self.present = [True] * 15
        self.angle = list(rest_deg) if rest_deg is not None else [0.0] * 15
        self.velocity = [0.0] * 15
        self.target = list(self.angle)
        self.kp = [5] * 15
        self.kd = [20] * 15
        self.protection = 0x0D
        self.protection_current = 154
        # imu: gravity direction in the robot frame, pitch about y
        self.tilt_deg = tilt_deg
        self.imu_ok = True
        # control
        self.enabled = False
        self.timed_out = False
        self.host_mode = HOST_NORMAL
        self.have_command = False
        self.accepted_seq = 0
        self.invalid = 0
        self.last_host_rx = 0.0
        self.pending_command = None
        self.pending_ops = []
        self.cal_held = 0
        self.cal_job = None             # (index, seq, done_at)
        self.mid_job = None             # (id, done_at)
        self.seq = 0
        # observation for tests
        self.modes = [0, 0, 0, 0]
        self.last_mode = None
        self.last_source_age_us = None
        self.first_policy_step = None
        self.max_target_step = 0.0     # largest change of a joint target between two commands
        self.rx_frames = 0
        self.last_policy_rx = None
        self.policy_gaps = []           # s between consecutive POLICY commands
        self.step_log = []              # (time, deg) of every target change above 1 deg
        self.long_gaps = []             # (time, gap ms, sequence numbers skipped, tick lateness ms)
        self.last_policy_seq = 0
        self.last_tick = time.perf_counter()
        self.events = []
        self._stop = threading.Event()
        self._thread = None

    # ------------------------------------------------------------------ helpers
    def now_us(self) -> int:
        return u32((time.perf_counter() - self.t0) * 1e6)

    def event(self, text):
        self.events.append((time.perf_counter() - self.t0, text))

    def send(self, packet: bytes):
        if self.peer is not None:
            try:
                self.sock.sendto(frames.frame(packet), self.peer)
            except OSError:
                pass

    def set_tilt(self, deg: float):
        with self.lock:
            self.tilt_deg = float(deg)

    def reset_step_watch(self):
        with self.lock:
            self.max_target_step = 0.0

    def command_timing(self, reset=True) -> dict:
        """Rate and spacing of the POLICY commands since the last call."""
        with self.lock:
            gaps = sorted(self.policy_gaps)
            if reset:
                self.policy_gaps = []
        if not gaps:
            return dict(n=0, hz=0.0, p50_ms=0.0, p99_ms=0.0, max_ms=0.0)
        return dict(n=len(gaps), hz=len(gaps) / sum(gaps), p50_ms=1e3 * gaps[len(gaps) // 2],
                    p99_ms=1e3 * gaps[min(len(gaps) - 1, int(len(gaps) * 0.99))], max_ms=1e3 * gaps[-1])

    def set_imu_ok(self, ok: bool):
        with self.lock:
            self.imu_ok = bool(ok)

    def snapshot(self) -> dict:
        with self.lock:
            return dict(enabled=self.enabled, host_mode=self.host_mode, angle=list(self.angle),
                        target=list(self.target), modes=list(self.modes), last_mode=self.last_mode,
                        invalid=self.invalid, rx_frames=self.rx_frames, timed_out=self.timed_out,
                        last_source_age_us=self.last_source_age_us, first_policy_step=self.first_policy_step, max_target_step=self.max_target_step, crc_errors=self.deframer.crc_errors,
                        peer=self.peer, cal_held=self.cal_held, ids=list(self.ids))

    # ------------------------------------------------------------------ receive (sketch receive*())
    def receive(self, packet: bytes):
        magic = packet[:4]
        now = time.perf_counter()
        self.rx_frames += 1
        if magic == frames.COMMAND:
            x = COMMAND.unpack(packet)
            valid = x[4] <= HOLD and not x[5] and not x[6] and all(math.isfinite(v) for v in x[7:])
            if valid:
                self.pending_command = x
                self.last_host_rx = now
            else:
                self.invalid += 1
        elif magic == frames.CAL:
            _, seq, op, index = CAL.unpack(packet)
            if op <= 2 and (op != 2 or index < 15):
                self.pending_ops.append(('cal', seq, op, index))
                self.last_host_rx = now
            else:
                self.invalid += 1
        elif magic == frames.SERVO:
            x = SERVO.unpack(packet)
            self.pending_ops.append(('servo',) + x[1:])
            self.last_host_rx = now

    # ------------------------------------------------------------------ setup sessions
    def servo_reply(self, op, target_id, ok, raw_pos=0, kp=0, kd=0, flags=0):
        self.send(SERVO_REPLY.pack(b'DQS3', op, target_id, 1 if ok else 0, flags, raw_pos, 2047, kp, kd))

    def raw_pos(self, i) -> int:
        if self.cal_held & (1 << i):
            return 2048
        return max(0, min(4095, int(round(2047 + self.angle[i] / 360.0 * 4096))))

    def apply_op(self, op, now):
        if op[0] == 'cal':
            _, seq, code, index = op
            if code == 0:
                self.host_mode, self.cal_held, self.cal_job = HOST_CAL, 0, None
                self.enabled = False
                self.event('cal enter')
            elif code == 1:
                if self.host_mode == HOST_CAL:
                    self.host_mode, self.cal_held, self.cal_job = HOST_NORMAL, 0, None
                    self.enabled = False
                    self.event('cal exit')
            else:
                if (self.host_mode != HOST_CAL or self.cal_job is not None
                        or self.cal_held & (1 << index) or not self.present[index]):
                    self.send(CAL_REPLY.pack(b'DQA1', seq, index, 0))
                else:
                    self.cal_job = (index, seq, now + 0.3)
            return
        _, code, target_id, new_id, kp, kd, flags, current = op
        if code == 0:
            self.host_mode, self.enabled = HOST_SERVO, False
            self.event('servo enter')
            self.servo_reply(code, target_id, True)
            return
        if code == 1:
            if self.host_mode == HOST_SERVO:
                self.host_mode = HOST_NORMAL
                self.event('servo exit')
            self.servo_reply(code, target_id, True)
            return
        if self.host_mode != HOST_SERVO or self.mid_job is not None:
            self.servo_reply(code, target_id, False)
            return
        if code in (9, 10):
            if code == 9:
                self.protection = flags
                self.servo_reply(code, 0xFE, True, flags=flags)
            else:
                self.protection_current = current
                self.servo_reply(code, 0xFE, True, raw_pos=current)
            return
        index = self.ids.index(target_id) if target_id in self.ids else None
        present = index is not None and self.present[index]
        if not present:
            self.servo_reply(code, target_id, False)
        elif code == 3:
            self.ids[index] = new_id
            self.event(f'id {target_id} -> {new_id}')
            self.servo_reply(code, target_id, True)
        elif code == 5:
            self.kp[index], self.kd[index] = kp, kd
            self.servo_reply(code, target_id, True, self.raw_pos(index), kp, kd)
        elif code == 8:
            self.mid_job = (target_id, now + 0.03)
        else:   # unlock, read, ping
            self.servo_reply(code, target_id, True, self.raw_pos(index), self.kp[index], self.kd[index])

    # ------------------------------------------------------------------ one control tick (sketch loop)
    def tick(self):
        now = time.perf_counter()
        prev_tick, self.last_tick = self.last_tick, now
        if now - prev_tick > 0.025:
            self.event(f'emulator tick late: {1e3 * (now - prev_tick):.0f} ms')
        start_us = self.now_us()
        ops, self.pending_ops = self.pending_ops, []
        for op in ops:
            self.apply_op(op, now)
        if self.cal_job and now >= self.cal_job[2]:
            index, seq, _ = self.cal_job
            self.cal_held |= 1 << index
            self.cal_job = None
            self.send(CAL_REPLY.pack(b'DQA1', seq, index, 1))
        if self.mid_job and now >= self.mid_job[1]:
            target_id = self.mid_job[0]
            self.mid_job = None
            if target_id in self.ids:
                self.angle[self.ids.index(target_id)] = 0.0
            self.servo_reply(8, target_id, True)

        if self.host_mode != HOST_NORMAL and now - self.last_host_rx > HOST_HEARTBEAT_S:
            self.event('setup session lost its host')
            self.host_mode, self.cal_held, self.cal_job, self.mid_job = HOST_NORMAL, 0, None, None
            self.enabled, self.timed_out = False, True
        taken = self.host_mode != HOST_NORMAL
        cmd, self.pending_command = self.pending_command, None
        if cmd is not None and not taken:
            _, seq, source_us, _ttl, mode, _r, _r2, *targets = cmd
            age = u32(start_us - source_us)
            fresh = mode == DISARM or age <= MAX_SOURCE_AGE_US
            ordered = (not self.have_command or mode == DISARM
                       or 0 < ((seq - self.accepted_seq) & 0xFFFFFFFF) < 0x80000000)
            if fresh and ordered:
                self.accepted_seq, self.have_command = seq, True
                self.modes[mode] += 1
                if mode != self.last_mode:
                    self.event(f'command mode {("DISARM", "ARM", "POLICY", "HOLD")[mode]}')
                    if mode == POLICY:
                        # first target after enabling: how far it is from where the joints are
                        self.first_policy_step = max(abs(t - a) for t, a in zip(targets[:14], self.angle[:14]))
                self.last_mode = mode
                self.last_source_age_us = age if mode != DISARM else None
                if mode == DISARM:
                    self.enabled, self.timed_out = False, False
                if mode == ARM and not self.enabled and self.imu_ok:
                    self.enabled, self.timed_out = True, False
                    self.event('torque on')
                if mode == POLICY:
                    if self.last_policy_rx is not None:
                        self.policy_gaps.append(now - self.last_policy_rx)
                        if now - self.last_policy_rx > 0.035:
                            self.long_gaps.append((round(now - self.t0, 3), round(1e3 * (now - self.last_policy_rx), 1),
                                                   seq - self.last_policy_seq, round(1e3 * (now - prev_tick), 1)))
                    self.last_policy_seq = seq
                    self.last_policy_rx = now
                else:
                    self.last_policy_rx = None
                if mode == POLICY and self.enabled:
                    step = max(abs(t - o) for t, o in zip(targets[:14], self.target[:14]))
                    self.max_target_step = max(self.max_target_step, step)
                    if step > 1.0:
                        self.step_log.append((round(now - self.t0, 3), round(step, 2)))
                self.target = list(targets)
            else:
                self.invalid += 1
        if taken:
            self.enabled, self.timed_out = False, False
        elif self.enabled and not self.imu_ok:
            self.enabled, self.timed_out = False, True
            self.event('torque off: IMU lost')

        # servos: powered ones follow the target, released ones stay where they are
        k = 1.0 - math.exp(-TICK / SERVO_TAU)
        for i in range(15):
            old = self.angle[i]
            if self.enabled:
                self.angle[i] += (self.target[i] - self.angle[i]) * k
            self.velocity[i] = (self.angle[i] - old) / TICK

        mask = sum(1 << i for i in range(15) if self.present[i])
        tilt = math.radians(self.tilt_deg)
        acc = [int(round(16384 * math.sin(tilt))), 0, int(round(-16384 * math.cos(tilt)))]
        self.seq += 1
        flags = ((1 if self.imu_ok else 0) | (2 if self.enabled else 0) | (4 if self.timed_out else 0)
                 | (8 if self.host_mode == HOST_CAL else 0) | (16 if self.host_mode == HOST_SERVO else 0))
        sample_us = self.now_us()
        self.send(STATE.pack(b'DQS1', 1, flags, mask, self.seq, sample_us, sample_us, self.accepted_seq,
                             0, int(TICK * 1e6), 500, 0, 0, self.invalid,
                             *acc, 0, 0, 0, *self.angle, *self.velocity, *self.target))
        if taken:
            self.send(RAW.pack(b'DQR2', mask, self.cal_held, *[self.raw_pos(i) for i in range(15)]))
        if self.seq % 100 == 0:
            self.send(VOLTAGE.pack(b'DQV1', self.seq, sample_us, mask, *([74] * 15)))

    # ------------------------------------------------------------------ thread
    def run(self):
        deadline = time.perf_counter()
        while not self._stop.is_set():
            deadline += TICK
            while True:
                wait = deadline - time.perf_counter()
                if wait <= 0:
                    break
                ready, _, _ = select.select([self.sock], [], [], wait)
                if not ready:
                    break
                try:
                    data, peer = self.sock.recvfrom(4096)
                except OSError:
                    continue
                with self.lock:
                    self.peer = peer
                    for packet in self.deframer.feed(data):
                        self.receive(packet)
            if time.perf_counter() - deadline > 0.5:
                deadline = time.perf_counter()      # the process was stopped for a while
            with self.lock:
                self.tick()

    def start(self):
        self._thread = threading.Thread(target=self.run, name='fake_mcu', daemon=True)
        self._thread.start()
        return self

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        self.sock.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=9003)
    ap.add_argument('--tilt', type=float, default=0.0, help='pitch of the reported gravity, degrees')
    ap.add_argument('--rest', type=float, default=0.0, help='angle of every servo at power-up, degrees')
    args = ap.parse_args()
    mcu = FakeMcu(args.host, args.port, rest_deg=[args.rest] * 15, tilt_deg=args.tilt).start()
    print(f'fake MCU on udp:{args.host}:{args.port}  (ardurover --serial2 udp:{args.host}:{args.port})')
    seen = 0
    try:
        while True:
            time.sleep(1.0)
            s = mcu.snapshot()
            for t, text in mcu.events[seen:]:
                print(f'  {t:8.2f}s  {text}')
            seen = len(mcu.events)
            names = ('DISARM', 'ARM', 'POLICY', 'HOLD')
            print(f"peer={s['peer']} torque={'on' if s['enabled'] else 'off'} "
                  f"last={names[s['last_mode']] if s['last_mode'] is not None else '-'} "
                  f"commands={s['modes']} refused={s['invalid']} "
                  f"leg={' '.join(f'{a:6.1f}' for a in s['angle'][:5])}")
    except KeyboardInterrupt:
        pass
    finally:
        mcu.stop()


if __name__ == '__main__':
    main()
