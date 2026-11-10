#!/usr/bin/env python3
# Author: Roberto Navoni, member of the ArduPilot Dev Team
# Contact: r.navoni74@gmail.com
# Developed by Roberto Navoni — DelphyAI LAB
# For information: r.navoni74@gmail.com
"""What the sketch on the Uno Q microcontroller reports, without commanding anything.

Connects to arduino-router, subscribes to the packets the XDuck sketch publishes (state, voltage,
servo boot gains) and prints what they say about the IMU and the servo bus. It sends nothing to the
microcontroller: torque stays as it is. Standard library only.

On the board the router socket belongs to the group arduino-router, which the apps get from their
container:

    docker run --rm --group-add 107 --user 1000:1000 -v /var/run/arduino-router.sock:/r.sock \
        -v $PWD:/t --entrypoint python3 ghcr.io/arduino/app-bricks/python-apps-base:0.13.2 \
        /t/mcu_probe.py --socket /r.sock
"""
import argparse
import math
import socket
import struct
import time

IDS = [10, 11, 12, 13, 14, 20, 21, 22, 23, 24, 30, 31, 32, 33, 34]
STATE = struct.Struct('<4sBBH10I6h45f')
VOLTAGE = struct.Struct('<4sIIH15B')


class Incomplete(Exception):
    pass


def unpack(b, i=0):
    """Minimal MessagePack decoder: (value, next index)."""
    def need(n):
        if len(b) - i < n:
            raise Incomplete
    need(1)
    t = b[i]
    i += 1

    def raw(n):
        nonlocal i
        need(n)
        v = bytes(b[i:i + n])
        i += n
        return v

    def seq(n, pairs=False):
        nonlocal i
        out = []
        for _ in range(n * (2 if pairs else 1)):
            v, i = unpack(b, i)
            out.append(v)
        return dict(zip(out[::2], out[1::2])) if pairs else out
    if t <= 0x7f:
        return t, i
    if t >= 0xe0:
        return t - 256, i
    if 0x80 <= t <= 0x8f:
        return seq(t & 15, True), i
    if 0x90 <= t <= 0x9f:
        return seq(t & 15), i
    if 0xa0 <= t <= 0xbf:
        return raw(t & 31).decode('utf-8', 'replace'), i
    if t == 0xc0:
        return None, i
    if t in (0xc2, 0xc3):
        return t == 0xc3, i
    if t in (0xc4, 0xc5, 0xc6):
        n = int.from_bytes(raw(1 << (t - 0xc4)), 'big')
        return raw(n), i
    if t in (0xd9, 0xda, 0xdb):
        n = int.from_bytes(raw(1 << (t - 0xd9)), 'big')
        return raw(n).decode('utf-8', 'replace'), i
    if t == 0xca:
        return struct.unpack('>f', raw(4))[0], i
    if t == 0xcb:
        return struct.unpack('>d', raw(8))[0], i
    if 0xcc <= t <= 0xcf:
        return int.from_bytes(raw(1 << (t - 0xcc)), 'big'), i
    if 0xd0 <= t <= 0xd3:
        return int.from_bytes(raw(1 << (t - 0xd0)), 'big', signed=True), i
    if t in (0xdc, 0xdd):
        return seq(int.from_bytes(raw(2 if t == 0xdc else 4), 'big')), i
    if t in (0xde, 0xdf):
        return seq(int.from_bytes(raw(2 if t == 0xde else 4), 'big'), True), i
    raise ValueError(f'unsupported MessagePack type 0x{t:02x}')


def pack_str(s):
    e = s.encode()
    return (bytes([0xa0 | len(e)]) if len(e) < 32 else b'\xd9' + bytes([len(e)])) + e


def register(sock, msgid, name):
    sock.sendall(b'\x94\x00' + bytes([msgid]) + pack_str('$/register') + b'\x91' + pack_str(name))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--socket', default='/var/run/arduino-router.sock')
    ap.add_argument('--seconds', type=float, default=4.0)
    args = ap.parse_args()

    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    s.connect(args.socket)
    names = ['duck_state', 'duck_voltage', 'duck_boot_pd', 'duck_diagnostics']
    for k, name in enumerate(names):
        register(s, k + 1, name)
    s.settimeout(0.2)
    buf = bytearray()
    got = {n: [] for n in names}
    replies = []
    end = time.monotonic() + args.seconds
    while time.monotonic() < end:
        try:
            data = s.recv(65536)
        except socket.timeout:
            continue
        if not data:
            break
        buf += data
        while buf:
            try:
                msg, used = unpack(buf)
            except Incomplete:
                break
            del buf[:used]
            if isinstance(msg, list) and msg and msg[0] == 2 and msg[1] in got:
                got[msg[1]].append((time.monotonic(), msg[2][0] if msg[2] else b''))
            elif isinstance(msg, list) and msg and msg[0] == 1:
                replies.append(msg[1:])
    s.close()

    print('router replies to the subscriptions (id, error, result):', replies)
    states = [(t, STATE.unpack(p)) for t, p in got['duck_state'] if len(p) == STATE.size and p[:4] == b'DQS1']
    print(f"packets in {args.seconds:.0f} s: " + ', '.join(f'{n} {len(v)}' for n, v in got.items()))
    if not states:
        print('NO STATE PACKET: the sketch on the microcontroller is not the XDuck one, or it is not running')
        return
    span = states[-1][0] - states[0][0]
    last = states[-1][1]
    flags, mask = last[2], last[3]
    print(f'state rate {(len(states) - 1) / span:.1f} Hz, sketch tick {last[9]} us, work {last[10]} us, overruns {last[11]}')
    print(f'flags 0x{flags:02x}: imu_ok={flags & 1} torque={flags >> 1 & 1} timed_out={flags >> 2 & 1} '
          f'calibration={flags >> 3 & 1} servo_setup={flags >> 4 & 1}')
    imu_ok = sum(1 for _, x in states if x[2] & 1)
    print(f'IMU ok in {imu_ok} of {len(states)} packets')
    acc = [[x[14 + k] * 9.80665 / 16384 for _, x in states] for k in range(3)]
    gyr = [[x[17 + k] / 32.0 for _, x in states] for k in range(3)]

    def stat(v):
        m = sum(v) / len(v)
        return m, math.sqrt(sum((a - m) ** 2 for a in v) / len(v))
    a = [stat(v) for v in acc]
    g = [stat(v) for v in gyr]
    print('acc  m/s2  mean ' + ' '.join(f'{m:8.3f}' for m, _ in a) + '   std ' + ' '.join(f'{d:6.3f}' for _, d in a)
          + f'   |mean| {math.sqrt(sum(m * m for m, _ in a)):.3f}')
    print('gyro deg/s mean ' + ' '.join(f'{m:8.3f}' for m, _ in g) + '   std ' + ' '.join(f'{d:6.3f}' for _, d in g))
    imu_times = len({x[6] for _, x in states})
    print(f'distinct IMU sample times: {imu_times} of {len(states)} packets')
    always = mask
    ever = 0
    for _, x in states:
        always &= x[3]
        ever |= x[3]
    print(f'servo mask 0x{mask:04x}: answering now ' + (' '.join(str(IDS[i]) for i in range(15) if mask >> i & 1) or 'none'))
    print('  answering in every packet: ' + (' '.join(str(IDS[i]) for i in range(15) if always >> i & 1) or 'none'))
    print('  never answering: ' + (' '.join(str(IDS[i]) for i in range(15) if not ever >> i & 1) or 'none'))
    print('angle deg  ' + ' '.join(f'{v:7.1f}' for v in last[20:35]))
    print('target deg ' + ' '.join(f'{v:7.1f}' for v in last[50:65]))
    for _, p in got['duck_voltage'][-1:]:
        if len(p) == VOLTAGE.size:
            v = VOLTAGE.unpack(p)
            print(f'voltage mask 0x{v[3]:04x}  V ' + ' '.join(f'{d / 10:.1f}' for d in v[4:]))
    for _, p in got['duck_boot_pd'][-1:]:
        if len(p) == 32:
            print(f'boot gains mask 0x{p[0] | p[1] << 8:04x}  P {list(p[2:17])}  D {list(p[17:32])}')


if __name__ == '__main__':
    main()
