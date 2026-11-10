#!/usr/bin/env python3
# Author: Roberto Navoni, member of the ArduPilot Dev Team
# Contact: r.navoni74@gmail.com
# Developed by Roberto Navoni — DelphyAI LAB
# For information: r.navoni74@gmail.com
"""Bench test of ArduRover + AP_NNMixer + AP_JointBus against the emulated microcontroller.

Starts fake_mcu.FakeMcu, starts ardurover pointed at it, then drives the autopilot over MAVLink the
way the web gateway does and checks what reaches the "servos". No robot dynamics: a pass means the
link, the state machines and the MAVLink interface behave, not that the duck walks.

    # the Uno Q binary, inside the build container (unoq/build_ardurover.sh test)
    tools/unoq/bench_test.py --binary /build/unoq/bin/ardurover
    # the SITL binary on the host (IMU from SITL: the fall test is skipped)
    tools/unoq/bench_test.py --sitl --binary ardupilot/build/sitl/bin/ardurover
"""
from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(Path(__file__).resolve().parent))
from fake_mcu import FakeMcu, POLICY  # noqa: E402

os.environ.setdefault('MAVLINK20', '1')
from pymavlink import mavutil  # noqa: E402

JOINT_SLOT = [0, 1, 2, 3, 4, 10, 11, 12, 13, 5, 6, 7, 8, 9]
ST_LINK, ST_IMU, ST_TORQUE, ST_TIMED_OUT, ST_CAL, ST_SERVO, ST_RELEASED, ST_RAW = (1 << i for i in range(8))
REST_DEG = 12.0


class Bench:
    def __init__(self, mcu: FakeMcu, port: int):
        self.mcu = mcu
        self.m = mavutil.mavlink_connection(f'udpin:127.0.0.1:{port}', source_system=255, source_component=190)
        self.named = {}
        self.arrays = {}
        self.replies = []
        self.acks = []
        self.params = {}
        self.texts = []
        self.armed = False
        self.heartbeats = 0
        self.history = []           # (t, policy, skill, getup) whenever one of them changes
        self.timing = {}            # task timing reports of the autopilot
        self.rc = None
        self._hb = 0.0
        self._rc = 0.0
        self.seq = 100

    # -------------------------------------------------------------- transport
    def pump(self, duration=0.0):
        end = time.monotonic() + duration
        while True:
            now = time.monotonic()
            if self.heartbeats and now - self._hb >= 1.0:
                self._hb = now
                self.m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            if self.rc is not None and self.heartbeats and now - self._rc >= 0.05:
                self._rc = now
                self.m.mav.rc_channels_override_send(self.m.target_system, self.m.target_component, *self.rc)
            msg = self.m.recv_match(blocking=True, timeout=0.02)
            if msg is not None:
                self.handle(msg)
            if time.monotonic() >= end:
                return

    def handle(self, msg):
        kind = msg.get_type()
        if kind == 'HEARTBEAT':
            if msg.get_srcComponent() == 1:
                self.heartbeats += 1
                self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        elif kind == 'NAMED_VALUE_FLOAT':
            self.named[msg.name] = msg.value
            if msg.name in ('PPO_HZ', 'PPO_GAP', 'PPO_LHZ', 'PPO_LGAP'):
                self.timing.setdefault(msg.name, []).append(msg.value)
        elif kind == 'DEBUG_FLOAT_ARRAY':
            self.arrays[msg.name] = list(msg.data)
            if msg.name == 'NNM_SRVR':
                self.replies.append(list(msg.data))
            elif msg.name == 'NNM_JNT':
                state = tuple(int(v) for v in msg.data[52:55])
                if not self.history or self.history[-1][1:] != state:
                    self.history.append((time.monotonic(),) + state)
        elif kind == 'COMMAND_ACK':
            self.acks.append((msg.command, msg.result))
        elif kind == 'PARAM_VALUE':
            self.params[msg.param_id] = msg.param_value
        elif kind == 'STATUSTEXT':
            self.texts.append(msg.text)

    def wait(self, cond, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.pump(0.02)
            if cond():
                return True
        return False

    # -------------------------------------------------------------- requests
    def array(self, name, data):
        self.m.mav.debug_float_array_send(int(time.time() * 1e6), name.encode(), 0, list(data) + [0.0] * (58 - len(data)))

    def service(self, op, ident=0, a0=0.0, a1=0.0, timeout=2.0):
        """NNM_SRV request; the NNM_SRVR reply as a dict, or None."""
        self.seq += 1
        seq = self.seq
        self.array('NNM_SRV', [seq, op, ident, a0, a1])
        found = []

        def got():
            found[:] = [r for r in self.replies if int(r[0]) == seq and int(r[1]) == op]
            return bool(found)
        if not self.wait(got, timeout):
            return None
        r = found[0]
        return dict(ok=int(r[2]), id=int(r[3]), raw_pos=int(r[4]), center=int(r[5]), kp=int(r[6]), kd=int(r[7]),
                    protection=int(r[8]), current=int(r[9]), source=int(r[10]))

    def command(self, cmd, *p, timeout=3.0):
        p = list(p) + [0.0] * (7 - len(p))
        self.acks.clear()
        self.m.mav.command_long_send(self.m.target_system, self.m.target_component, cmd, 0, *p)
        if not self.wait(lambda: any(c == cmd for c, _ in self.acks), timeout):
            return None
        return next(r for c, r in self.acks if c == cmd)

    def param_get(self, name, timeout=3.0):
        self.params.pop(name, None)
        self.m.mav.param_request_read_send(self.m.target_system, self.m.target_component, name.encode(), -1)
        return self.params[name] if self.wait(lambda: name in self.params, timeout) else None

    def param_set(self, name, value, timeout=3.0):
        self.params.pop(name, None)
        self.m.mav.param_set_send(self.m.target_system, self.m.target_component, name.encode(), float(value),
                                  mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        return self.params[name] if self.wait(lambda: name in self.params, timeout) else None

    def arm(self, on):
        r = self.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1.0 if on else 0.0, 21196.0)
        self.wait(lambda: self.armed == on, 3.0)
        return r

    # -------------------------------------------------------------- views
    def jnt(self):
        return self.arrays.get('NNM_JNT')

    def status(self):
        j = self.jnt()
        return int(j[0]) if j else 0

    def policy(self):
        j = self.jnt()
        return int(j[52]) if j else -1

    def skill(self):
        j = self.jnt()
        return int(j[53]) if j else -1

    def getup(self):
        j = self.jnt()
        return int(j[54]) if j else -1


def run(bench: Bench, mcu: FakeMcu, home_slot_deg, sitl: bool, report: list):
    def check(name, ok, detail=''):
        report.append(dict(name=name, ok=bool(ok), detail=str(detail)))
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f'  — {detail}' if detail else ''), flush=True)
        return ok

    def skip(name, why):
        report.append(dict(name=name, ok=None, detail=why))
        print(f'  [SKIP] {name}  — {why}', flush=True)

    def at_home(tol=1.0):
        s = mcu.snapshot()
        return s['enabled'] and max(abs(a - h) for a, h in zip(s['angle'][:14], home_slot_deg[:14])) < tol

    # ---- boot
    print('boot', flush=True)
    if not check('heartbeat from the autopilot', bench.wait(lambda: bench.heartbeats > 0, 40.0)):
        return
    bench.pump(1.5)     # our heartbeat makes the channel active
    if not check('joint telemetry (NNM_JNT) arrives', bench.wait(lambda: bench.jnt() is not None, 15.0)):
        return
    check('link and IMU flags set', bench.wait(lambda: bench.status() & (ST_LINK | ST_IMU) == (ST_LINK | ST_IMU), 10.0),
          f'status={bench.status()}')
    j = bench.jnt() or [0.0] * 58
    check('all 15 servos reported', int(j[1]) == 0x7FFF, f'mask=0x{int(j[1]):04x}')
    wanted = dict(NNM_ENABLE=1, NNM_ROBOT=21, NNM_OUT=1, NNM_DIS_POSE=1, NNM_DEPLOY=1, SERIAL2_PROTOCOL=51,
                  SCHED_LOOP_RATE=50, NNM_GETUP_IDX=1)
    got = {k: bench.param_get(k) for k in wanted}
    check('board defaults applied', all(got[k] is not None and abs(got[k] - v) < 1e-6 for k, v in wanted.items()), got)

    # ---- disarmed: balance pose, reached with a ramp
    print('disarmed pose', flush=True)
    check('torque enabled while disarmed', bench.wait(lambda: mcu.snapshot()['enabled'], 10.0))
    check('joints reach the balance pose', bench.wait(at_home, 6.0),
          'angle=' + ' '.join(f'{a:.1f}' for a in mcu.snapshot()['angle'][:14]))
    s = mcu.snapshot()
    check('first target after enabling is where the joints are', s['first_policy_step'] is not None
          and s['first_policy_step'] < 1.5, f"{s['first_policy_step']} deg")
    check('pose reached without steps', 0.0 < s['max_target_step'] < 3.0, f"largest step {s['max_target_step']:.2f} deg "
          f'from {REST_DEG:.0f} deg on every joint')
    check('no command refused by the MCU', s['invalid'] == 0 and bench.jnt()[50] == 0, f"mcu={s['invalid']}")
    check('no CRC error in either direction', s['crc_errors'] == 0 and bench.jnt()[51] == 0,
          f"mcu={s['crc_errors']} autopilot={bench.jnt()[51]:.0f}")
    check('command stamped with a fresh MCU time', s['last_source_age_us'] is not None and s['last_source_age_us'] < 60000,
          f"{s['last_source_age_us']} us")
    check('policy idle while disarmed (PPO_FAIL 1)', bench.named.get('PPO_FAIL') == 1.0, f"PPO_FAIL={bench.named.get('PPO_FAIL')}")

    # ---- torque off / on
    print('torque', flush=True)
    r = bench.service(1)
    check('torque-off request accepted', r is not None and r['ok'] == 1, r)
    check('servos released', bench.wait(lambda: not mcu.snapshot()['enabled'], 2.0))
    check('released state reported', bench.wait(lambda: bench.status() & ST_RELEASED and not bench.status() & ST_TORQUE, 2.0),
          f'status={bench.status()}')
    with mcu.lock:      # somebody moves the legs by hand
        for i in range(14):
            mcu.angle[i] = -REST_DEG
        mcu.first_policy_step = None
        mcu.max_target_step = 0.0
        mcu.last_mode = None
    bench.pump(0.5)
    r = bench.service(2)
    check('default-pose request accepted', r is not None and r['ok'] == 1, r)
    check('balance pose again', bench.wait(at_home, 6.0))
    s = mcu.snapshot()
    check('ramp starts from the moved joints', s['first_policy_step'] is not None and s['first_policy_step'] < 1.5
          and s['max_target_step'] < 3.0, f"first {s['first_policy_step']} deg, largest step {s['max_target_step']:.2f} deg")

    # ---- joint centering session
    print('calibration session', flush=True)
    r = bench.service(10)
    check('calibration session opens', r is not None and r['ok'] == 1, r)
    check('MCU in calibration, torque off', bench.wait(lambda: mcu.snapshot()['host_mode'] == 1 and not mcu.snapshot()['enabled'], 2.0))
    check('raw encoders reported', bench.wait(lambda: bench.status() & ST_CAL and bench.status() & ST_RAW, 2.0), f'status={bench.status()}')
    j = bench.jnt()
    expect_raw = round(2047 + mcu.snapshot()['angle'][2] / 360.0 * 4096)
    check('raw encoder value of slot 2', abs(j[35] - expect_raw) <= 2, f'{j[35]:.0f} (servo at {expect_raw})')
    r = bench.service(12, 3, timeout=3.0)
    check('centre slot 3', r is not None and r['ok'] == 1 and r['id'] == 3 and r['source'] == 0, r)
    check('held mask shows slot 3', bench.wait(lambda: int(bench.jnt()[2]) == 1 << 3, 2.0), f'held=0x{int(bench.jnt()[2]):04x}')
    r = bench.service(12, 3, timeout=3.0)
    check('centring the same slot twice is refused', r is not None and r['ok'] == 0, r)
    alive = True
    for _ in range(8):      # 4 s: longer than the autopilot's 2 s session timeout
        r = bench.service(13)
        alive = alive and r is not None and r['ok'] == 1
        bench.pump(0.5)
    check('session kept alive by heartbeats for 4 s', alive and mcu.snapshot()['host_mode'] == 1)
    r = bench.service(11)
    check('calibration session closes', r is not None and r['ok'] == 1 and bench.wait(lambda: mcu.snapshot()['host_mode'] == 0, 2.0))
    check('balance pose after calibration', bench.wait(at_home, 6.0))
    r = bench.service(10)
    check('session abandoned by the ground station is closed', r is not None and r['ok'] == 1
          and bench.wait(lambda: mcu.snapshot()['host_mode'] == 1, 2.0)
          and bench.wait(lambda: mcu.snapshot()['host_mode'] == 0, 4.0)
          and bench.wait(at_home, 6.0), [e for e in mcu.events[-4:]])

    # ---- servo setup session
    print('servo setup session', flush=True)
    r = bench.service(26, 10)
    check('servo operation outside a session is refused', r is not None and r['ok'] == 0 and r['source'] == 1, r)
    r = bench.service(20)
    check('servo session opens', r is not None and r['ok'] == 1 and bench.wait(lambda: mcu.snapshot()['host_mode'] == 2, 2.0), r)
    r = bench.service(26, 10)
    check('ping id 10', r is not None and r['ok'] == 1 and r['id'] == 10 and r['source'] == 0, r)
    r = bench.service(26, 77)
    check('ping of a missing id fails', r is not None and r['ok'] == 0, r)
    r = bench.service(24, 11, 9, 33)
    check('set gains on id 11', r is not None and r['ok'] == 1 and r['kp'] == 9 and r['kd'] == 33, r)
    r = bench.service(25, 11)
    check('read id 11 returns the new gains', r is not None and r['ok'] == 1 and (r['kp'], r['kd']) == (9, 33) and r['center'] == 2047, r)
    r = bench.service(23, 14, 1)
    check('change id 14 -> 1', r is not None and r['ok'] == 1 and mcu.snapshot()['ids'][4] == 1, r)
    r = bench.service(23, 1, 14)
    check('change id 1 -> 14', r is not None and r['ok'] == 1 and mcu.snapshot()['ids'][4] == 14, r)
    r = bench.service(28, 0, 0)
    check('overload protection off', r is not None and r['ok'] == 1 and r['protection'] == 0 and mcu.protection == 0x04, r)
    r = bench.service(28, 0, 1)
    check('overload protection on', r is not None and r['ok'] == 1 and r['protection'] == 1 and mcu.protection == 0x0D, r)
    r = bench.service(29, 0, 200)
    check('protection current', r is not None and r['ok'] == 1 and r['current'] == 200 and mcu.protection_current == 200, r)
    r = bench.service(27, 12)
    check('reset centre of id 12', r is not None and r['ok'] == 1, r)
    r = bench.service(21)
    check('servo session closes', r is not None and r['ok'] == 1 and bench.wait(lambda: mcu.snapshot()['host_mode'] == 0, 2.0))
    check('balance pose after servo setup', bench.wait(at_home, 6.0))

    # ---- mouth
    bench.array('NNM_AUX', [20.0])
    bench.pump(0.3)

    # ---- armed: the policy drives the servos
    print('armed', flush=True)
    mcu.reset_step_watch()
    mode_ok = bench.command(mavutil.mavlink.MAV_CMD_DO_SET_MODE, 1.0, 0.0)     # MANUAL
    r = bench.arm(True)
    check('force arm in MANUAL', r == 0 and bench.armed, f'mode ack={mode_ok} arm ack={r}')
    bench.pump(2.0)
    s = mcu.snapshot()
    check('servos stay powered through arming', s['enabled'] and s['last_mode'] == POLICY)
    check('no policy failure', bench.named.get('PPO_FAIL') == 0.0, f"PPO_FAIL={bench.named.get('PPO_FAIL')}")
    check('walk policy in use, XDuck chain', bench.policy() == 0 and int(bench.jnt()[55]) == 1 and int(bench.jnt()[57]) == 1,
          f'policy={bench.policy()} chain={bench.jnt()[55]}')
    check('inference time reported', 0.0 < bench.named.get('PPO_MS', 0.0) < 20.0, f"{bench.named.get('PPO_MS')} ms")
    check('targets finite and bounded', all(math.isfinite(t) and abs(t) < 120.0 for t in s['target']),
          ' '.join(f'{t:.0f}' for t in s['target']))
    check('hand-over to the policy without steps', s['max_target_step'] < 6.0, f"largest step {s['max_target_step']:.2f} deg")
    check('mouth follows NNM_AUX', abs(s['target'][14] - 20.0) < 0.01, f"{s['target'][14]:.2f} deg")
    r = bench.service(1)
    check('torque-off refused while armed', r is not None and r['ok'] == 0, r)
    r = bench.service(20)
    check('servo session refused while armed', r is not None and r['ok'] == 0 and mcu.snapshot()['host_mode'] == 0, r)

    mcu.command_timing()
    bench.timing.clear()
    bench.pump(4.0)
    t = mcu.command_timing()
    print('    autopilot task timing:', {k: [round(x, 1) for x in v] for k, v in bench.timing.items()}, flush=True)
    check('policy commands at 50 Hz', 47.0 <= t['hz'] <= 53.0 and t['p99_ms'] < 35.0,
          f"{t['hz']:.1f} Hz, median {t['p50_ms']:.1f} ms, 99% {t['p99_ms']:.1f} ms, longest {t['max_ms']:.1f} ms over {t['n']} commands")
    if mcu.long_gaps:
        print('    long gaps (t s, gap ms, seq step, emulator tick ms):', mcu.long_gaps[-12:], flush=True)
    check('state packets at 100 Hz seen by the autopilot', 95.0 <= bench.jnt()[49] <= 105.0, f'{bench.jnt()[49]:.1f} Hz')

    # twist from RC override
    before = list(mcu.snapshot()['target'])
    spread = [0.0] * 15
    bench.rc = [65535, 1800, 65535, 65535, 65535, 65535, 65535, 65535]
    end = time.monotonic() + 3.0
    lo, hi = list(before), list(before)
    while time.monotonic() < end:
        bench.pump(0.02)
        t = mcu.snapshot()['target']
        lo = [min(a, b) for a, b in zip(lo, t)]
        hi = [max(a, b) for a, b in zip(hi, t)]
    spread = [h - l for h, l in zip(hi, lo)]
    check('forward command reaches the policy', bench.named.get('PPO_VX', 0.0) > 0.05, f"PPO_VX={bench.named.get('PPO_VX')}")
    check('leg targets move with a forward command', max(spread[:10]) > 3.0, 'range ' + ' '.join(f'{v:.1f}' for v in spread[:10]))
    bench.rc = [65535, 1500, 65535, 65535, 65535, 65535, 65535, 65535]
    bench.pump(1.0)
    bench.array('NNM_POSE', [0.5, 0.0, 0.0, 0.0])
    bench.pump(0.1)

    # ---- skills
    print('skills', flush=True)
    user1 = mavutil.mavlink.MAV_CMD_USER_1
    check('unknown skill refused', bench.command(user1, 99.0) == mavutil.mavlink.MAV_RESULT_DENIED)
    check('stand refused when not sitting', bench.command(user1, 4.0) == mavutil.mavlink.MAV_RESULT_DENIED)
    mark = len(bench.history)
    check('kick left accepted', bench.command(user1, 5.0) == mavutil.mavlink.MAV_RESULT_ACCEPTED)
    ran = bench.wait(lambda: any(h[1] == 6 and h[2] == 5 for h in bench.history[mark:]), 3.0)
    back = bench.wait(lambda: bench.policy() == 0 and bench.skill() == 0, 4.0)
    check('kick runs policy 6 and returns to walk', ran and back, [h[1:] for h in bench.history[mark:]])
    mark = len(bench.history)
    check('sit accepted', bench.command(user1, 3.0) == mavutil.mavlink.MAV_RESULT_ACCEPTED)
    check('sit holds policy 5', bench.wait(lambda: bench.policy() == 5 and bench.skill() == 3, 3.0) and
          not bench.wait(lambda: bench.skill() != 3, 2.0), [h[1:] for h in bench.history[mark:]])
    check('kick refused while sitting', bench.command(user1, 5.0) == mavutil.mavlink.MAV_RESULT_DENIED)
    check('stand accepted', bench.command(user1, 4.0) == mavutil.mavlink.MAV_RESULT_ACCEPTED)
    check('stand ends back on walk', bench.wait(lambda: bench.skill() == 4, 2.0) and
          bench.wait(lambda: bench.policy() == 0 and bench.skill() == 0, 5.0), [h[1:] for h in bench.history[mark:]])
    mark = len(bench.history)
    check('pick accepted', bench.command(user1, 1.0) == mavutil.mavlink.MAV_RESULT_ACCEPTED)
    opened = bench.wait(lambda: abs(mcu.snapshot()['target'][14] - 30.0) < 0.01, 3.0)
    done = bench.wait(lambda: bench.policy() == 0 and bench.skill() == 0, 7.0)
    check('pick opens the mouth and returns to walk', opened and done, [h[1:] for h in bench.history[mark:]])
    check('no policy failure after the skills', bench.named.get('PPO_FAIL') == 0.0, f"PPO_FAIL={bench.named.get('PPO_FAIL')}")

    # ---- roller policy by parameter
    print('policy by parameter', flush=True)
    bench.param_set('NNM_POLICY', 2)
    check('NNM_POLICY 2 loads the roller', bench.wait(lambda: bench.policy() == 2, 5.0), f'policy={bench.policy()}')
    bench.param_set('NNM_POLICY', 0)
    check('NNM_POLICY 0 back to walk', bench.wait(lambda: bench.policy() == 0, 5.0), f'policy={bench.policy()}')

    # ---- fall and get-up
    print('fall and get-up', flush=True)
    if sitl:
        skip('fall, get-up and return to walk', 'SITL supplies the IMU, the emulator cannot tilt it')
    else:
        mark = len(bench.history)
        mcu.set_tilt(80.0)
        fell = bench.wait(lambda: bench.getup() == 1, 3.0)
        check('fall detected, pre-hold on the default pose', fell, [h[1:] for h in bench.history[mark:]])
        bench.pump(0.5)
        s = mcu.snapshot()
        check('default pose held while falling', max(abs(t - h) for t, h in zip(s['target'][:14], home_slot_deg[:14])) < 1.0,
              ' '.join(f'{t:.1f}' for t in s['target'][:14]))
        check('get-up policy takes over', bench.wait(lambda: bench.getup() == 2 and bench.policy() == 1, 4.0),
              [h[1:] for h in bench.history[mark:]])
        check('skill refused during get-up', bench.command(user1, 5.0) == mavutil.mavlink.MAV_RESULT_DENIED)
        bench.pump(1.5)
        mcu.set_tilt(0.0)
        check('upright again: back to walk', bench.wait(lambda: bench.policy() == 0 and bench.getup() == 0, 8.0),
              [h[1:] for h in bench.history[mark:]])
        check('still armed and powered after the get-up', bench.armed and mcu.snapshot()['enabled'])

    # ---- disarm
    print('disarm', flush=True)
    bench.rc = None
    bench.pump(1.5)
    quiet = len(mcu.step_log)
    mcu.reset_step_watch()
    t_disarm = time.perf_counter() - mcu.t0
    bench.arm(False)
    check('disarmed', not bench.armed)
    check('balance pose after disarm, torque kept', bench.wait(at_home, 6.0))
    big = [x for x in mcu.step_log[quiet:] if x[1] > 3.0]
    check('return to the pose without steps', mcu.snapshot()['max_target_step'] < 3.0,
          f"{mcu.snapshot()['max_target_step']:.2f} deg; disarm sent at {t_disarm:.2f} s, steps above 3 deg (t, deg): {big[:8]}")

    # ---- deployment chain
    print('deployment chain', flush=True)
    bench.param_set('NNM_DEPLOY', 0)
    check('NNM_DEPLOY 0 selects the standard chain', bench.wait(lambda: int(bench.jnt()[55]) == 0, 3.0))
    bench.arm(True)
    bench.pump(2.0)
    check('standard chain runs armed', bench.armed and mcu.snapshot()['last_mode'] == POLICY and bench.named.get('PPO_FAIL') == 0.0,
          f"PPO_FAIL={bench.named.get('PPO_FAIL')}")
    bench.param_set('NNM_DEPLOY', 1)
    bench.pump(1.0)
    check('chain not changed while armed', int(bench.jnt()[55]) == 0)
    bench.arm(False)
    check('chain follows the parameter once disarmed', bench.wait(lambda: int(bench.jnt()[55]) == 1, 3.0))

    # ---- IMU loss on the MCU
    print('faults', flush=True)
    bench.wait(at_home, 6.0)
    mcu.set_imu_ok(False)
    check('MCU drops torque without IMU, reported', bench.wait(lambda: not bench.status() & (ST_IMU | ST_TORQUE)
                                                           and bench.status() & ST_LINK, 3.0), f'status={bench.status()}')
    mcu.set_imu_ok(True)
    check('torque and pose back when the IMU returns', bench.wait(at_home, 8.0))
    s = mcu.snapshot()
    check('no command refused, no CRC error over the whole run', s['invalid'] == 0 and s['crc_errors'] == 0
          and bench.jnt()[50] == 0 and bench.jnt()[51] == 0,
          f"refused: mcu={s['invalid']} autopilot={bench.jnt()[50]:.0f}; crc: mcu={s['crc_errors']} autopilot={bench.jnt()[51]:.0f}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--binary', required=True)
    ap.add_argument('--sitl', action='store_true', help='the binary is the SITL build')
    ap.add_argument('--robot-dir', default=str(ROOT / 'robots' / 'xgoduck'))
    ap.add_argument('--defaults', default=str(ROOT / 'ardupilot/libraries/AP_HAL_Linux/boards/unoq/defaults.parm'))
    ap.add_argument('--bus-port', type=int, default=9003)
    ap.add_argument('--gcs-port', type=int, default=14650)
    ap.add_argument('--report', help='write the results as JSON')
    ap.add_argument('--keep', action='store_true', help='keep the working directory')
    args = ap.parse_args()

    robot = Path(args.robot_dir)
    profile = json.loads((robot / 'robot' / 'profile.json').read_text())
    home_slot_deg = [0.0] * 15
    for j, q in enumerate(profile['q0']):
        home_slot_deg[JOINT_SLOT[j]] = math.degrees(q)

    work = Path(tempfile.mkdtemp(prefix='unoq_bench_'))
    data = work / 'APM' / 'nnm' / profile['robot_id']
    (data / 'policies').mkdir(parents=True)
    for name in ('robot.bin', 'skills.bin'):
        shutil.copy(robot / 'robot' / name, data / name)
    policies = sorted((robot / 'policies').glob('*.nnm'))
    if not policies:
        sys.exit(f'no .nnm under {robot / "policies"}: run tools/robots/import_xgoduck.py first')
    for p in policies:
        shutil.copy(p, data / 'policies' / p.name)

    binary = str(Path(args.binary).resolve())
    bus = f'127.0.0.1:{args.bus_port}'
    gcs = f'127.0.0.1:{args.gcs_port}'
    if args.sitl:
        cmd = [binary, '--model', 'rover', '--speedup', '1', '-I0', '-w', '--defaults', str(Path(args.defaults).resolve()),
               '--sim-address=127.0.0.1', '--serial0', f'udpclient:{gcs}', '--serial2', f'udpclient:{bus}']
    else:
        cmd = [binary, '--serial0', f'udp:{gcs}', '--serial2', f'udp:{bus}']

    mcu = FakeMcu('127.0.0.1', args.bus_port, rest_deg=[REST_DEG] * 15).start()
    bench = Bench(mcu, args.gcs_port)
    log = open(work / 'ardurover.log', 'w')
    proc = subprocess.Popen(cmd, cwd=work, stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL)
    report = []
    print(' '.join(cmd), f'\n(cwd {work})', flush=True)
    try:
        run(bench, mcu, home_slot_deg, args.sitl, report)
    finally:
        alive = proc.poll() is None
        if alive:
            proc.send_signal(signal.SIGTERM)
            try:
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                proc.kill()
        mcu.stop()
        log.close()
    report.append(dict(name='autopilot still running at the end', ok=alive, detail=f'exit code {proc.returncode}' if not alive else ''))
    if not alive:
        print(f'  [FAIL] autopilot exited during the test (code {proc.returncode})')

    text = (work / 'ardurover.log').read_text(errors='replace')
    print('\n--- autopilot console (last lines) ---')
    print('\n'.join(text.splitlines()[-25:]))
    print('\n--- status texts ---')
    print('\n'.join(dict.fromkeys(bench.texts)))
    print('\n--- MCU events ---')
    print('\n'.join(f'{t:7.2f}s {e}' for t, e in mcu.events[:60]))
    passed = sum(1 for r in report if r['ok'] is True)
    failed = [r for r in report if r['ok'] is False]
    skipped = sum(1 for r in report if r['ok'] is None)
    print(f'\n{passed} passed, {len(failed)} failed, {skipped} skipped')
    for r in failed:
        print(f"  FAILED: {r['name']}  {r['detail']}")
    if args.report:
        Path(args.report).write_text(json.dumps(dict(binary=args.binary, sitl=args.sitl, passed=passed, failed=len(failed),
                                                     skipped=skipped, checks=report), indent=1) + '\n')
    if args.keep:
        print(f'working directory kept: {work}')
    else:
        shutil.rmtree(work, ignore_errors=True)
    sys.exit(1 if failed else 0)


if __name__ == '__main__':
    main()
