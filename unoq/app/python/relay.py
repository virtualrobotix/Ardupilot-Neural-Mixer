# Author: Roberto Navoni, member of the ArduPilot Dev Team
# Contact: r.navoni74@gmail.com
# Developed by Roberto Navoni — DelphyAI LAB
# For information: r.navoni74@gmail.com
"""Relay between the sketch (Arduino Bridge RPC) and ArduPilot's joint-bus port (UDP).

    STM32 sketch  <-- Bridge RPC over the router -->  Relay  <-- UDP 127.0.0.1:9003 -->  ardurover

Packets are not interpreted, only framed: one RPC value on the Bridge side, packet + CRC16 on the
UDP side (frames.py). The one thing the relay decides by itself: the sketch keeps the last command
for as long as the Bridge is up, so when ardurover stops talking the relay sends DISARM in its
place and the servos are released.
"""
from __future__ import annotations

import socket
import struct
import threading
import time

import frames

COMMAND = struct.Struct('<4sIIIBBH15f')
STATE_HEAD = struct.Struct('<4sBBH10I')
DIAGNOSTIC_FMT = struct.Struct('<4s16I')
SCAN_MAX = 6
MODE_DISARM = 0
HOST_TIMEOUT_S = 0.3
IMU_KIND = {0: 'none', 1: 'QMI8658 (I2C)', 2: 'MPU6000/6050 (I2C)', 3: 'MPU6000 (SPI)'}
SERVO_IDS = [10, 11, 12, 13, 14, 20, 21, 22, 23, 24, 30, 31, 32, 33, 34]


class Relay:
    def __init__(self, bridge, host='127.0.0.1', port=9003):
        self.bridge = bridge
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((host, port))
        self.sock.settimeout(0.1)
        self.peer = None
        self.deframer = frames.Deframer(accept=frames.RPC_TO_MCU)
        self.lock = threading.Lock()
        self.count = {name: 0 for name in frames.RPC_FROM_MCU}
        self.count.update(command=0, cal=0, servo=0, disarm_sent=0, send_errors=0)
        self.state = None               # last state packet
        self.state_t = 0.0
        self.host_t = 0.0               # last packet from ardurover
        self.last_seq = 0
        self.last_mode = None
        self.imu = None                 # (kind, address, who_am_i at the MPU addresses)
        self.spi = None                 # (WHO_AM_I read on the SPI header, CS pin)
        self.i2c_scan = None            # what the sketch found on the I2C buses when no IMU answered
        self.diagnostics = None
        self.voltage = None
        self.rate = dict(state=0.0, command=0.0)
        self._rate_mark = (time.monotonic(), 0, 0)
        for name in frames.RPC_FROM_MCU:
            bridge.provide(name, self._from_mcu(name))
        bridge.provide('duck_imu', self._imu)
        bridge.provide('duck_boot_pd', lambda payload: None)

    # ---------------------------------------------------------------- MCU -> ardurover
    def _from_mcu(self, name):
        magic = frames.RPC_FROM_MCU[name]

        def handler(payload):
            packet = bytes(payload)
            if packet[:4] != magic or len(packet) != frames.LENGTH[magic]:
                return
            self.count[name] += 1
            if magic == frames.STATE:
                self.state, self.state_t = packet, time.monotonic()
            elif magic == frames.VOLTAGE:
                self.voltage = packet
            elif magic == frames.DIAGNOSTIC:
                self._diagnostics(packet)
            peer = self.peer
            if peer is not None:
                try:
                    self.sock.sendto(frames.frame(packet), peer)
                except OSError:
                    self.count['send_errors'] += 1
        return handler

    def _imu(self, payload):
        b = bytes(payload)
        if len(b) < 3:
            return
        self.imu = (b[0], b[1], b[2])
        if len(b) >= 6 + 2 * (1 + SCAN_MAX):
            self.spi = (b[4 + 2 * (1 + SCAN_MAX)], b[5 + 2 * (1 + SCAN_MAX)])
        if len(b) >= 4 + 2 * (1 + SCAN_MAX):
            scan = {}
            for bus, name in enumerate(('wire_d20_d21', 'wire1_qwiic')):
                bits = b[3] >> (2 * bus) & 3
                if not bits & 1:
                    continue
                base = 4 + bus * (1 + SCAN_MAX)
                n = b[base]
                scan[name] = dict(usable=not bits & 2, devices=n,
                                  addresses=[f'0x{a:02x}' for a in b[base + 1:base + 1 + min(n, SCAN_MAX)]])
            self.i2c_scan = scan

    def _diagnostics(self, packet):
        v = DIAGNOSTIC_FMT.unpack(packet)[1:]
        self.diagnostics = dict(zip(('bridge_state', 'bridge_attempts', 'bridge_failures', 'notify_drops', 'imu_seq',
                                     'imu_us', 'imu_period_us', 'imu_work_us', 'imu_overruns', 'imu_errors',
                                     'imu_init_attempts', 'servo_writes', 'servo_reads', 'servo_wait_us',
                                     'reconnects', 'bridge_ready_us'), v))

    # ---------------------------------------------------------------- ardurover -> MCU
    def _to_mcu(self, packet):
        magic = packet[:4]
        if magic == frames.COMMAND:
            _, seq, _, _, mode, *_ = COMMAND.unpack(packet)
            self.last_seq, self.last_mode = seq, mode
            self.count['command'] += 1
        elif magic == frames.CAL:
            self.count['cal'] += 1
        else:
            self.count['servo'] += 1
        self.bridge.notify(frames.RPC_TO_MCU[magic], packet)

    def _disarm(self):
        """ardurover is silent: release the servos in its place."""
        self.last_seq = (self.last_seq + 1) & 0xFFFFFFFF
        target = [0.0] * 15
        if self.state is not None:
            target = list(struct.unpack_from('<15f', self.state, 60))     # measured angles
        self.bridge.notify('duck_command', COMMAND.pack(frames.COMMAND, self.last_seq, 0, 0, MODE_DISARM, 0, 0, *target))
        self.last_mode = MODE_DISARM
        self.count['disarm_sent'] += 1

    def run(self):
        last_disarm = 0.0
        while True:
            try:
                data, peer = self.sock.recvfrom(4096)
            except socket.timeout:
                data = None
            except OSError:
                time.sleep(0.05)
                continue
            now = time.monotonic()
            if data:
                self.peer = peer
                self.host_t = now
                for packet in self.deframer.feed(data):
                    try:
                        self._to_mcu(packet)
                    except Exception:
                        self.count['send_errors'] += 1
            elif now - self.host_t > HOST_TIMEOUT_S and now - last_disarm > 0.1:
                # also at start-up, before ardurover has said anything
                if self.last_mode != MODE_DISARM or now - last_disarm > 1.0:
                    try:
                        self._disarm()
                    except Exception:
                        self.count['send_errors'] += 1
                    last_disarm = now
            t0, s0, c0 = self._rate_mark
            if now - t0 >= 1.0:
                s1, c1 = self.count['duck_state'], self.count['command']
                self.rate = dict(state=(s1 - s0) / (now - t0), command=(c1 - c0) / (now - t0))
                self._rate_mark = (now, s1, c1)

    def start(self):
        threading.Thread(target=self.run, name='relay', daemon=True).start()
        return self

    # ---------------------------------------------------------------- status
    def status(self) -> dict:
        now = time.monotonic()
        out = dict(state_hz=round(self.rate['state'], 1), command_hz=round(self.rate['command'], 1),
                   mcu_link=self.state is not None and now - self.state_t < 0.3,
                   autopilot_link=now - self.host_t < HOST_TIMEOUT_S,
                   last_command_mode={None: None, 0: 'DISARM', 1: 'ARM', 2: 'POLICY', 3: 'HOLD'}.get(self.last_mode),
                   crc_errors=self.deframer.crc_errors, counts=dict(self.count))
        if self.imu is not None:
            kind, addr, who = self.imu
            out['imu'] = dict(kind=IMU_KIND.get(kind, str(kind)),
                              address=None if not addr else f'CS D{addr}' if kind == 3 else f'0x{addr:02x}',
                              i2c_who_am_i=f'0x{who:02x}' if who else None)
            if self.spi is not None:
                # 0x68 is an MPU6000; 0xff is the pull-up on MISO, nothing is driving the line
                out['imu'].update(spi_who_am_i=f'0x{self.spi[0]:02x}', spi_cs=f'D{self.spi[1]}')
        if self.i2c_scan is not None and (self.imu is None or self.imu[0] == 0):
            out['i2c_scan'] = self.i2c_scan
        if self.diagnostics is not None:
            out['diagnostics'] = self.diagnostics
        if self.state is not None:
            (_, _, flags, mask, _, _, _, _, _, period_us, work_us, overruns, report_drops,
             invalid) = STATE_HEAD.unpack_from(self.state)
            out['control'] = dict(period_us=period_us, work_us=work_us, overruns=overruns,
                                  report_drops=report_drops, invalid_commands=invalid)
            acc = struct.unpack_from('<3h', self.state, 48)
            gyro = struct.unpack_from('<3h', self.state, 54)
            angle = struct.unpack_from('<15f', self.state, 60)
            out.update(imu_ok=bool(flags & 1), torque=bool(flags & 2), timed_out=bool(flags & 4),
                       calibration=bool(flags & 8), servo_setup=bool(flags & 16),
                       servo_mask=f'0x{mask:04x}',
                       servos_answering=[i for k, i in enumerate(SERVO_IDS) if mask >> k & 1],
                       servos_missing=[i for k, i in enumerate(SERVO_IDS) if not mask >> k & 1],
                       acc_g=[round(a / 16384.0, 3) for a in acc],
                       gyro_dps=[round(g / 32.0, 2) for g in gyro],
                       angle_deg=[round(a, 1) for a in angle])
        if self.voltage is not None and len(self.voltage) == 29:
            out['voltage'] = [round(v / 10.0, 1) for v in self.voltage[14:29]]
        return out
