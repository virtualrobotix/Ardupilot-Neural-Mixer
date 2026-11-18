#!/usr/bin/env python3
# Author: Roberto Navoni, member of the ArduPilot Dev Team
# Contact: r.navoni74@gmail.com
# Developed by Roberto Navoni — DelphyAI LAB
# For information: r.navoni74@gmail.com
"""Read the KP/KD that the 15 bus servos hold in EEPROM, through the Uno Q web API.

Needs the robot on the bench, disarmed, with torque released: the setup session releases every
servo (servo_enter), reads each slot in MCU order and closes the session (servo_exit). Nothing is
written. The output is the servo_gains block of robots/xgoduck/robot/profile.json, ready to paste
into tools/robots/catalog.py (ROBOTS["xgoduck"]["servo_gains"]); a servo that does not answer is null.

    tools/unoq/read_servo_gains.py --url http://<uno-q-address>:9527
    tools/unoq/read_servo_gains.py --url http://<uno-q-address>:9527 --json servo_gains.json --yes

Only the permanent (EEPROM) pair is read: the SERVO_READ operation returns it. The live pair
(registers 50/51) that the sketch writes at ARM is not read back by this tool.
"""
from __future__ import annotations

import argparse
import json
import sys
import urllib.error
import urllib.request

# bus slots in MCU order (sketch/duck_config.h MOTOR_IDS), with the joint each one drives
SLOTS = [
    (10, 'left_hip_yaw'), (11, 'left_hip_roll'), (12, 'left_hip_pitch'), (13, 'left_knee'), (14, 'left_ankle'),
    (20, 'right_hip_yaw'), (21, 'right_hip_roll'), (22, 'right_hip_pitch'), (23, 'right_knee'), (24, 'right_ankle'),
    (30, 'neck_pitch'), (31, 'head_pitch'), (32, 'head_yaw'), (33, 'head_roll'), (34, 'mouth'),
]


def post(base: str, action: str, timeout: float = 6.0, **body):
    req = urllib.request.Request(f"{base}/api/service", data=json.dumps({'action': action, **body}).encode(),
                                 headers={'Content-Type': 'application/json'}, method='POST')
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return json.loads(r.read() or b'{}')
    except urllib.error.HTTPError as e:
        raise SystemExit(f"{action}: {e.code} {e.read().decode(errors='replace')}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--url', required=True, help='base URL of the Uno Q web page, e.g. http://192.168.1.50:9527')
    ap.add_argument('--json', help='also write the servo_gains block to this file')
    ap.add_argument('--yes', action='store_true', help='do not ask before releasing the servos')
    args = ap.parse_args()
    base = args.url.rstrip('/')

    if not args.yes:
        answer = input('I servo vengono rilasciati per la lettura. Robot a terra e disarmato? [y/N] ')
        if answer.strip().lower() not in ('y', 's', 'si', 'sì'):
            return 1

    post(base, 'servo_enter')
    rows = []
    try:
        for ident, name in SLOTS:
            post(base, 'heartbeat')
            r = post(base, 'read', id=ident)
            ok = bool(r.get('ok'))
            rows.append(dict(id=ident, name=name, ok=ok, kp=r.get('kp') if ok else None,
                             kd=r.get('kd') if ok else None, raw=r.get('raw_pos') if ok else None))
    finally:
        post(base, 'servo_exit')

    print(f"{'ID':>3}  {'giunto':<16} {'risposta':<9} {'Kp':>4} {'Kd':>4} {'encoder':>7}")
    for row in rows:
        print(f"{row['id']:>3}  {row['name']:<16} {'si' if row['ok'] else 'NO':<9} "
              f"{'-' if row['kp'] is None else row['kp']:>4} {'-' if row['kd'] is None else row['kd']:>4} "
              f"{'-' if row['raw'] is None else row['raw']:>7}")

    block = [dict(id=row['id'], kp=row['kp'], kd=row['kd']) for row in rows]
    text = json.dumps(block, indent=2)
    print('\nservo_gains (EEPROM letti; null = non risponde):')
    print(text)
    if args.json:
        with open(args.json, 'w', encoding='utf-8') as f:
            f.write(text + '\n')
        print(f'scritto {args.json}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
