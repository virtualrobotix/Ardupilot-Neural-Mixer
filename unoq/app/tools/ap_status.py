#!/usr/bin/env python3
# Author: Roberto Navoni, member of the ArduPilot Dev Team
# Contact: r.navoni74@gmail.com
# Developed by Roberto Navoni — DelphyAI LAB
# For information: r.navoni74@gmail.com
"""What ArduPilot on the Uno Q sees of the robot, read over MAVLink. Read only: no parameter is
written and no command is sent.

On the board, inside the running app container (ardurover sends its telemetry to 127.0.0.1:14550):

    docker exec arduhumanoid-main-1 /app/.cache/.venv/bin/python /app/tools/ap_status.py
"""
import argparse
import time

from pymavlink import mavutil

STATUS_BITS = ['link', 'imu_ok', 'torque', 'timed_out', 'calibration', 'servo_setup', 'torque_off_requested', 'raw_valid']
PARAMS = ['NNM_ENABLE', 'NNM_DEPLOY', 'NNM_DIS_POSE', 'NNM_POLICY', 'SCHED_LOOP_RATE', 'SERIAL2_PROTOCOL']


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--listen', default='udpin:127.0.0.1:14550')
    ap.add_argument('--seconds', type=float, default=8.0)
    args = ap.parse_args()
    m = mavutil.mavlink_connection(args.listen, source_system=255, source_component=190)
    if m.wait_heartbeat(timeout=15) is None:
        raise SystemExit('no MAVLink heartbeat from the autopilot')
    named, arrays, texts, params, count = {}, {}, [], {}, {}
    hb = None
    end = time.monotonic() + args.seconds
    asked = 0.0
    while time.monotonic() < end:
        now = time.monotonic()
        if now - asked > 1.0:
            asked = now
            m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            for name in PARAMS:
                if name not in params:
                    m.mav.param_request_read_send(m.target_system, m.target_component, name.encode(), -1)
        msg = m.recv_match(blocking=True, timeout=0.1)
        if msg is None:
            continue
        kind = msg.get_type()
        count[kind] = count.get(kind, 0) + 1
        if kind == 'HEARTBEAT' and msg.get_srcComponent() == 1:
            hb = msg
        elif kind == 'NAMED_VALUE_FLOAT':
            named[msg.name] = msg.value
        elif kind == 'DEBUG_FLOAT_ARRAY':
            arrays[msg.name] = list(msg.data)
            count[msg.name] = count.get(msg.name, 0) + 1
        elif kind == 'PARAM_VALUE':
            params[msg.param_id] = msg.param_value
        elif kind == 'STATUSTEXT':
            texts.append(msg.text)
        elif kind in ('ATTITUDE', 'RAW_IMU', 'SCALED_IMU', 'SYS_STATUS'):
            arrays[kind] = msg

    print(f'heartbeat: system {m.target_system}, type {hb.type}, mode {hb.custom_mode}, '
          f'armed {bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)}')
    print('parameters:', ', '.join(f'{k}={params[k]:g}' for k in PARAMS if k in params))
    for t in texts:
        print('  text:', t)
    j = arrays.get('NNM_JNT')
    if j is None:
        print('NNM_JNT: not received')
    else:
        status = int(j[0])
        print(f"NNM_JNT at {count['NNM_JNT'] / args.seconds:.1f} Hz: status "
              f"{[n for i, n in enumerate(STATUS_BITS) if status >> i & 1]}")
        n = int(j[48])
        present = int(j[1])
        print(f'  slots {n}, servos answering {bin(present).count("1")}/{n} (mask 0x{present:04x}), '
              f'state packets {j[49]:.1f} Hz, commands refused {int(j[50])}, CRC errors {int(j[51])}')
        print('  angle deg :', ' '.join(f'{v:6.1f}' for v in j[3:3 + n]))
        print('  target deg:', ' '.join(f'{v:6.1f}' for v in j[18:18 + n]))
        print(f'  policy {int(j[52])}, skill {int(j[53])}, get-up {int(j[54])}, chain {int(j[55])}, armed {int(j[57])}')
    v = arrays.get('NNM_VLT')
    print('NNM_VLT:', 'not received (no servo has reported a voltage)' if v is None
          else ' '.join(f'{x:.1f}' for x in v[1:16]) + f' V, {v[16]:.1f} s old')
    a = arrays.get('ATTITUDE')
    if a is not None:
        print(f'attitude: roll {a.roll * 57.2958:.1f} pitch {a.pitch * 57.2958:.1f} yaw {a.yaw * 57.2958:.1f} deg')
    r = arrays.get('RAW_IMU') or arrays.get('SCALED_IMU')
    if r is not None:
        print(f'accelerometer mg: {r.xacc} {r.yacc} {r.zacc}   gyro mrad/s: {r.xgyro} {r.ygyro} {r.zgyro}')
    print('timing:', ', '.join(f'{k}={named[k]:.1f}' for k in ('PPO_HZ', 'PPO_GAP', 'PPO_LHZ', 'PPO_LGAP', 'PPO_FAIL') if k in named))
    print('messages:', ', '.join(f'{k} {v}' for k, v in sorted(count.items())))


if __name__ == '__main__':
    main()
