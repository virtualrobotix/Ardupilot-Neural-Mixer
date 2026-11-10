#!/usr/bin/env python3
# Author: Roberto Navoni, member of the ArduPilot Dev Team
# Contact: r.navoni74@gmail.com
# Developed by Roberto Navoni — DelphyAI LAB
# For information: r.navoni74@gmail.com
"""Test of the web gateway and API (unoq/app/python) against ardurover and the emulated microcontroller.

Calls the same functions the pages reach through /api/* and checks the effect on the autopilot and
on the emulated servos. The relay and the supervisor of the app are replaced by stand-ins: here
ardurover talks to the emulator directly.

    unoq/build_ardurover.sh gateway-test
"""
from __future__ import annotations

import argparse
import json
import math
import shutil
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(ROOT / 'unoq' / 'app' / 'python'))
from fake_mcu import FakeMcu  # noqa: E402
from api import Api  # noqa: E402
from gateway import Gateway, GatewayError  # noqa: E402

JOINT_SLOT = [0, 1, 2, 3, 4, 10, 11, 12, 13, 5, 6, 7, 8, 9]
SERVO_IDS = [10, 11, 12, 13, 14, 20, 21, 22, 23, 24, 30, 31, 32, 33, 34]


class RelayStandIn:
    def __init__(self, mcu):
        self.mcu = mcu

    def status(self):
        s = self.mcu.snapshot()
        return dict(mcu_link=True, imu_ok=True, torque=s['enabled'], state_hz=100.0, command_hz=50.0,
                    servos_answering=list(SERVO_IDS), servos_missing=[], acc_g=[0.0, 0.0, -1.0])


class SupervisorStandIn:
    def status(self):
        return dict(running=True, starts=1, last_exit=None, error=None)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--binary', required=True)
    ap.add_argument('--robot-dir', default=str(ROOT / 'robots' / 'xgoduck'))
    args = ap.parse_args()

    robot = Path(args.robot_dir)
    profile = json.loads((robot / 'robot' / 'profile.json').read_text())
    home = [0.0] * 15
    for j, q in enumerate(profile['q0']):
        home[JOINT_SLOT[j]] = math.degrees(q)
    work = Path(tempfile.mkdtemp(prefix='unoq_gateway_'))
    data = work / 'APM' / 'nnm' / profile['robot_id']
    (data / 'policies').mkdir(parents=True)
    for name in ('robot.bin', 'skills.bin'):
        shutil.copy(robot / 'robot' / name, data / name)
    for p in sorted((robot / 'policies').glob('*.nnm')):
        shutil.copy(p, data / 'policies' / p.name)

    mcu = FakeMcu('127.0.0.1', 9003, rest_deg=[12.0] * 15).start()
    gateway = Gateway('udpin:127.0.0.1:14550').start()
    api = Api(gateway, RelayStandIn(mcu), SupervisorStandIn())
    log = open(work / 'ardurover.log', 'w')
    proc = subprocess.Popen([str(Path(args.binary).resolve()), '--serial0', 'udp:127.0.0.1:14550', '--serial2', 'udp:127.0.0.1:9003'],
                            cwd=work, stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL)
    results = []

    def check(name, ok, detail=''):
        results.append(bool(ok))
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f'  — {detail}' if detail != '' else ''), flush=True)
        return ok

    def wait(cond, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            try:
                if cond():
                    return True
            except (KeyError, TypeError):
                pass
            time.sleep(0.05)
        return False

    def st():
        return api.status()['autopilot']

    def refused(fn):
        try:
            fn()
        except GatewayError as e:
            return str(e)
        return None

    def at_home():
        s = mcu.snapshot()
        return s['enabled'] and max(abs(a - h) for a, h in zip(s['angle'][:14], home[:14])) < 1.0

    try:
        print('status', flush=True)
        if not check('gateway hears the autopilot', wait(lambda: st()['connected'], 40.0)):
            return
        check('joint telemetry in the status', wait(lambda: len(st()['joints']) == 15, 15.0))
        check('parameters read', wait(lambda: all(k in st()['params'] for k in
                                                   ('NNM_DEPLOY', 'NNM_POLICY', 'NNM_DIS_POSE', 'NNM_VX_MAX', 'NNM_RC_VX')), 10.0),
              st()['params'])
        s = api.status()
        check('status serialises to JSON', bool(json.dumps(s)))
        a = s['autopilot']
        check('disarmed, XDuck chain, walk policy', not a['armed'] and a['robot']['chain'] == 'xduck'
              and a['robot']['policy_name'] == 'walk', a['robot'])
        check('all servos present with their ids', [j['id'] for j in a['joints'] if j['present']] == SERVO_IDS)
        check('tilt computed', s.get('tilt_deg') == 0.0, s.get('tilt_deg'))
        check('balance pose reached while disarmed', wait(at_home, 10.0))
        check('voltages reported', wait(lambda: all(j['voltage'] for j in st()['joints']), 5.0),
              [j['voltage'] for j in st()['joints']][:3])

        print('torque', flush=True)
        r = api.torque(dict(on=False))
        check('servos released', r['ok'] and wait(lambda: not mcu.snapshot()['enabled'], 2.0)
              and wait(lambda: 'torque_released' in st()['bus']['status'], 2.0))
        r = api.torque(dict(on=True))
        check('balance pose again', r['ok'] and wait(at_home, 6.0))

        print('parameters', flush=True)
        check('parameter outside the list refused', refused(lambda: api.param(dict(name='ARMING_SKIPCHK', value=0))))
        check('value out of range refused', refused(lambda: api.param(dict(name='NNM_DEPLOY', value=5))))
        r = api.param(dict(name='NNM_DEPLOY', value=0))
        check('standard chain selected', r['value'] == 0 and wait(lambda: st()['robot']['chain'] == 'standard', 3.0))
        api.param(dict(name='NNM_DEPLOY', value=1))
        check('XDuck chain selected', wait(lambda: st()['robot']['chain'] == 'xduck', 3.0))
        r = api.param(dict(name='NNM_DIS_POSE', value=0))
        check('NNM_DIS_POSE 0 releases the servos', r['value'] == 0 and wait(lambda: not mcu.snapshot()['enabled'], 3.0))
        api.param(dict(name='NNM_DIS_POSE', value=1))
        check('NNM_DIS_POSE 1 brings the pose back', wait(at_home, 6.0))

        print('armed', flush=True)
        check('skill refused while disarmed', refused(lambda: api.skill(dict(name='kick_left'))))
        r = api.arm(dict(armed=True))
        check('armed', r['armed'] and r['note'] is None and st()['armed'] and st()['mode'] == 'MANUAL', r)
        check('policy running', wait(lambda: st()['policy_task']['fail'] == 0 and mcu.snapshot()['last_mode'] == 2, 4.0),
              st()['policy_task'])
        check('torque-off refused while armed', refused(lambda: api.torque(dict(on=False))))
        for _ in range(25):
            api.motion(dict(vx=1.0))
            time.sleep(0.1)
        vx_max = st()['params']['NNM_VX_MAX']
        check('forward command reaches the policy', abs(st()['policy_task']['vx'] - vx_max) < 0.02,
              f"PPO_VX={st()['policy_task']['vx']} limit={vx_max}")
        check('motion stops by itself when the page is silent', wait(lambda: abs(st()['policy_task']['vx']) < 0.01, 3.0),
              f"PPO_VX={st()['policy_task']['vx']}")
        check('motion out of range refused', refused(lambda: api.motion(dict(vx=1.5))))
        api.head(dict(mouth=20))
        check('mouth command reaches the servo', wait(lambda: abs(mcu.snapshot()['target'][14] - 20.0) < 0.01, 2.0))
        api.head(dict(head=[0.5, 0.0, 0.0, 0.0], mouth=0))
        check('head command reaches the policy', wait(lambda: gateway.named.get('PPO_POSE0', 0.0) > 0.4, 2.0),
              gateway.named.get('PPO_POSE0'))
        api.head(dict(head=[0.0] * 4))

        print('skills', flush=True)
        check('unknown skill refused', refused(lambda: api.skill(dict(name='dance'))))
        api.skill(dict(name='kick_left'))
        check('kick runs and returns to walk', wait(lambda: st()['robot']['skill'] == 'kick_left', 3.0)
              and wait(lambda: st()['robot']['skill'] is None and st()['robot']['policy_name'] == 'walk', 5.0))
        api.skill(dict(name='sit'))
        check('sit', wait(lambda: st()['robot']['skill'] == 'sit' and st()['robot']['policy_name'] == 'sitstand', 3.0))
        check('kick refused while sitting', refused(lambda: api.skill(dict(name='kick_right'))))
        api.skill(dict(name='stand'))
        check('stand returns to walk', wait(lambda: st()['robot']['skill'] is None and st()['robot']['policy_name'] == 'walk', 6.0))
        api.param(dict(name='NNM_POLICY', value=2))
        check('roller policy selected', wait(lambda: st()['robot']['policy_name'] == 'roller', 5.0))
        api.param(dict(name='NNM_POLICY', value=0))
        check('walk policy selected', wait(lambda: st()['robot']['policy_name'] == 'walk', 5.0))
        check('sessions refused while armed', not api.service(dict(action='cal_enter'))['ok'])

        print('stop', flush=True)
        r = api.stop()
        check('stop disarms and releases the servos', not st()['armed'] and wait(lambda: not mcu.snapshot()['enabled'], 2.0), r)
        r = api.arm(dict(armed=True))
        check('arming again after a stop powers the servos', r['armed'] and wait(lambda: mcu.snapshot()['enabled'], 4.0))
        api.arm(dict(armed=False))
        check('disarm returns to the balance pose', not st()['armed'] and wait(at_home, 6.0))

        print('calibration session', flush=True)
        r = api.service(dict(action='cal_enter'))
        check('session opens, servos free', r['ok'] and wait(lambda: 'calibration' in st()['bus']['status'], 2.0)
              and not mcu.snapshot()['enabled'])
        check('raw encoders shown', wait(lambda: all(j['raw'] > 0 for j in st()['joints']), 3.0), [j['raw'] for j in st()['joints']][:4])
        r = api.service(dict(action='cal_center', index=3))
        check('centre one joint', r['ok'] and wait(lambda: st()['joints'][3]['held'], 2.0), r)
        alive = True
        for _ in range(6):
            alive = alive and api.service(dict(action='heartbeat'))['ok']
            time.sleep(0.6)
        check('heartbeats keep the session open', alive and 'calibration' in st()['bus']['status'])
        r = api.service(dict(action='cal_exit'))
        check('session closes, pose back', r['ok'] and wait(at_home, 6.0))

        print('servo session', flush=True)
        check('servo operation outside a session refused', not api.service(dict(action='ping', id=10))['ok'])
        r = api.service(dict(action='servo_enter'))
        check('session opens', r['ok'] and wait(lambda: 'servo_setup' in st()['bus']['status'], 2.0))
        r = api.service(dict(action='ping', id=10))
        check('ping id 10', r['ok'] and r['source'] == 'servo', r)
        check('ping of a missing id fails', not api.service(dict(action='ping', id=77))['ok'])
        r = api.service(dict(action='set_gains', id=11, kp=9, kd=33))
        check('gains written', r['ok'] and (r['kp'], r['kd']) == (9, 33), r)
        r = api.service(dict(action='read', id=11))
        check('servo read back', r['ok'] and (r['kp'], r['kd'], r['center']) == (9, 33, 2047), r)
        r = api.service(dict(action='set_id', id=14, new_id=1))
        check('id changed', r['ok'] and mcu.snapshot()['ids'][4] == 1, r)
        api.service(dict(action='set_id', id=1, new_id=14))
        r = api.service(dict(action='set_protection_current', current_a=1.3))
        check('protection current in amperes', r['ok'] and r['current'] == 200 and r['current_a'] == 1.3, r)
        r = api.service(dict(action='set_protection', enabled=False))
        check('protection off', r['ok'] and r['protection'] == 0, r)
        api.service(dict(action='set_protection', enabled=True))
        check('bad servo id refused', refused(lambda: api.service(dict(action='ping', id=300))))
        check('unknown operation refused', refused(lambda: api.service(dict(action='explode'))))
        r = api.service(dict(action='servo_exit'))
        check('session closes, pose back', r['ok'] and wait(at_home, 6.0))
        s = mcu.snapshot()
        check('no command refused by the MCU, no CRC error', s['invalid'] == 0 and s['crc_errors'] == 0
              and st()['bus']['refused'] == 0 and st()['bus']['crc_errors'] == 0)
        check('no gateway error', gateway.error is None, gateway.error)
    finally:
        alive = proc.poll() is None
        if alive:
            proc.send_signal(signal.SIGTERM)
            try:
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                proc.kill()
        gateway.stop()
        mcu.stop()
        log.close()
        check('autopilot still running at the end', alive)
        print('\n--- status texts ---')
        print('\n'.join(t for _, t in gateway.texts))
        shutil.rmtree(work, ignore_errors=True)
        print(f'\n{sum(results)} passed, {len(results) - sum(results)} failed')
    sys.exit(0 if all(results) else 1)


if __name__ == '__main__':
    main()
