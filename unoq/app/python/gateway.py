# Author: Roberto Navoni, member of the ArduPilot Dev Team
# Contact: r.navoni74@gmail.com
# Developed by Roberto Navoni — DelphyAI LAB
# For information: r.navoni74@gmail.com
"""MAVLink gateway between the web pages and ArduPilot.

Everything the pages show or ask for goes through MAVLink, the same way a ground station would:

    telemetry   HEARTBEAT, ATTITUDE, STATUSTEXT, NAMED_VALUE_FLOAT (PPO_*),
                DEBUG_FLOAT_ARRAY NNM_JNT (joints, bus state) and NNM_VLT (servo voltages)
    motion      RC_CHANNELS_OVERRIDE on the channels of NNM_RC_VX / NNM_RC_VY / NNM_RC_WZ
    head, mouth DEBUG_FLOAT_ARRAY NNM_POSE and NNM_AUX
    skills      MAV_CMD_USER_1, param1 = skill id
    setup       DEBUG_FLOAT_ARRAY NNM_SRV [seq, op, id, a0, a1], answered by NNM_SRVR
    settings    PARAM_SET / PARAM_REQUEST_READ

ardurover sends its telemetry to 127.0.0.1:14550, where this gateway listens.
"""
from __future__ import annotations

import math
import os
import threading
import time

os.environ.setdefault('MAVLINK20', '1')     # DEBUG_FLOAT_ARRAY is a MAVLink 2 message
from pymavlink import mavutil  # noqa: E402

MAV = mavutil.mavlink

POLICY_NAMES = ['walk', 'getup', 'roller', 'pick', 'roulade', 'sitstand', 'kick_left', 'kick_right']
SKILLS = {'pick': 1, 'roulade': 2, 'sit': 3, 'stand': 4, 'kick_left': 5, 'kick_right': 6}
SKILL_NAMES = {v: k for k, v in SKILLS.items()}
GETUP_NAMES = ['idle', 'falling', 'active', 'recovered']
CHAIN_NAMES = ['standard', 'xduck']
STATUS_BITS = ['link', 'imu_ok', 'torque', 'timed_out', 'calibration', 'servo_setup', 'torque_released', 'raw_valid']

# actuator slots of the bus, in the order of NNM_JNT
SLOT_IDS = [10, 11, 12, 13, 14, 20, 21, 22, 23, 24, 30, 31, 32, 33, 34]
SLOT_NAMES = ['left_hip_yaw', 'left_hip_roll', 'left_hip_pitch', 'left_knee', 'left_ankle',
              'right_hip_yaw', 'right_hip_roll', 'right_hip_pitch', 'right_knee', 'right_ankle',
              'neck_pitch', 'head_pitch', 'head_yaw', 'head_roll', 'mouth']

# NNM_SRV operations
OPS = dict(torque_off=1, default_pose=2, cal_enter=10, cal_exit=11, cal_center=12, heartbeat=13,
           servo_enter=20, servo_exit=21, unlock=22, set_id=23, set_gains=24, read=25, ping=26,
           reset_mid=27, set_protection=28, set_protection_current=29)
PROTECTION_CURRENT_UNIT_A = 0.0065

# parameters the pages may read and change: name -> (min, max)
PARAMS = {
    'NNM_DEPLOY': (0, 1),           # 0 standard chain, 1 XDuck chain
    'NNM_POLICY': (0, 7),           # policy while walking: 0 walk, 2 roller
    'NNM_DIS_POSE': (0, 1),         # 1: balance pose (servos powered) while disarmed
    'NNM_VX_MAX': (0.0, 1.0),
    'NNM_VY_MAX': (0.0, 1.0),
    'NNM_WZ_MAX': (0.0, 3.0),
}
READ_ONLY_PARAMS = ['NNM_ENABLE', 'NNM_ROBOT', 'NNM_GETUP_IDX', 'SCHED_LOOP_RATE', 'NNM_RC_VX', 'NNM_RC_VY', 'NNM_RC_WZ']

MOTION_TIMEOUT_S = 0.5      # a motion command older than this counts as "stop"
HEAD_LIMIT = 1.0
MOUTH_MAX_DEG = 30.0


class GatewayError(Exception):
    """A request the autopilot refused or did not answer; the text is shown on the page."""


class Gateway:
    def __init__(self, listen='udpin:127.0.0.1:14550'):
        self.listen = listen
        self.m = None
        self.tx = threading.Lock()              # one sender at a time
        self.cv = threading.Condition()         # state below, and the replies requests wait for
        self.error = None
        self.heartbeat_t = 0.0
        self.heartbeats = 0
        self.armed = False
        self.mode = None
        self.named = {}
        self.jnt = None
        self.jnt_t = 0.0
        self.vlt = None
        self.attitude = None
        self.params = {}
        self.texts = []                         # (wall time, text)
        self.acks = {}
        self.replies = {}
        self.seq = int(time.time()) % 30000 + 100
        self.twist = [0.0, 0.0, 0.0]            # normalised -1..1: forward, left, turn left
        self.twist_t = 0.0
        self.head = [0.0] * 4
        self.mouth = 0.0
        self._stop = threading.Event()
        self._t = dict(hb=0.0, rc=0.0, pose=0.0, aux=0.0, param=0.0)
        self._param_i = 0
        self._aux_sent = None

    # ------------------------------------------------------------------ thread
    def start(self):
        threading.Thread(target=self.run, name='mavlink', daemon=True).start()
        return self

    def stop(self):
        self._stop.set()

    def run(self):
        while not self._stop.is_set():
            try:
                if self.m is None:
                    self.m = mavutil.mavlink_connection(self.listen, source_system=255, source_component=190)
                msg = self.m.recv_match(blocking=True, timeout=0.02)
                if msg is not None:
                    self._handle(msg)
                self._periodic(time.monotonic())
                self.error = None
            except Exception as e:      # the link must survive anything a packet can do
                self.error = f'{type(e).__name__}: {e}'
                time.sleep(0.2)

    def _handle(self, msg):
        kind = msg.get_type()
        with self.cv:
            if kind == 'HEARTBEAT':
                if msg.get_srcComponent() != 1:
                    return
                self.m.target_system, self.m.target_component = msg.get_srcSystem(), 1
                self.heartbeats += 1
                self.heartbeat_t = time.monotonic()
                self.armed = bool(msg.base_mode & MAV.MAV_MODE_FLAG_SAFETY_ARMED)
                self.mode = msg.custom_mode
            elif kind == 'NAMED_VALUE_FLOAT':
                self.named[msg.name] = msg.value
            elif kind == 'DEBUG_FLOAT_ARRAY':
                if msg.name == 'NNM_JNT':
                    self.jnt, self.jnt_t = list(msg.data), time.monotonic()
                elif msg.name == 'NNM_VLT':
                    self.vlt = list(msg.data)
                elif msg.name == 'NNM_SRVR':
                    d = msg.data
                    self.replies[int(d[0])] = dict(op=int(d[1]), ok=bool(int(d[2])), id=int(d[3]), raw_pos=int(d[4]),
                                                   center=int(d[5]), kp=int(d[6]), kd=int(d[7]), protection=int(d[8]),
                                                   current=int(d[9]), source='autopilot' if int(d[10]) else 'servo')
                    self.cv.notify_all()
            elif kind == 'ATTITUDE':
                self.attitude = (math.degrees(msg.roll), math.degrees(msg.pitch), math.degrees(msg.yaw))
            elif kind == 'COMMAND_ACK':
                self.acks[msg.command] = msg.result
                self.cv.notify_all()
            elif kind == 'PARAM_VALUE':
                if msg.param_id in PARAMS or msg.param_id in READ_ONLY_PARAMS:
                    self.params[msg.param_id] = msg.param_value
                self.cv.notify_all()
            elif kind == 'STATUSTEXT':
                if not self.texts or self.texts[-1][1] != msg.text:
                    self.texts.append((time.time(), msg.text))
                    del self.texts[:-30]

    def _periodic(self, now):
        if not self.heartbeats:
            return                      # nobody to talk to yet: the port learns the peer from its first packet
        t = self._t
        if now - t['hb'] >= 1.0:
            t['hb'] = now
            with self.tx:
                self.m.mav.heartbeat_send(MAV.MAV_TYPE_GCS, MAV.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        if self.armed and now - t['rc'] >= 0.05:
            # sticks: sent for as long as the robot is armed, centred when the page is silent
            t['rc'] = now
            vx, vy, wz = self.twist if now - self.twist_t < MOTION_TIMEOUT_S else (0.0, 0.0, 0.0)
            ch = [65535] * 8
            for name, default, value in (('NNM_RC_VX', 2, vx), ('NNM_RC_VY', 1, vy), ('NNM_RC_WZ', 4, wz)):
                n = int(self.params.get(name, default))
                if 1 <= n <= 8:
                    # full stick = 1000 / 2000 us, the range of the RCn_MIN / RCn_MAX defaults
                    ch[n - 1] = int(round(1500 + 500 * max(-1.0, min(1.0, value))))
            with self.tx:
                self.m.mav.rc_channels_override_send(self.m.target_system, self.m.target_component, *ch)
        if self.armed and any(self.head) and now - t['pose'] >= 0.1:
            # the autopilot eases the head back to rest when this stream stops
            t['pose'] = now
            self._array('NNM_POSE', self.head)
        if self._aux_sent != self.mouth or now - t['aux'] >= 2.0:
            t['aux'] = now
            self._aux_sent = self.mouth
            self._array('NNM_AUX', [self.mouth])
        names = list(PARAMS) + READ_ONLY_PARAMS
        missing = [n for n in names if n not in self.params]
        if now - t['param'] >= (0.15 if missing else 0.5):
            # the ones not known yet first, then all of them in turn: another ground station may change them
            t['param'] = now
            self._param_i = (self._param_i + 1) % len(names)
            name = missing[0] if missing else names[self._param_i]
            with self.tx:
                self.m.mav.param_request_read_send(self.m.target_system, self.m.target_component, name.encode(), -1)

    def _array(self, name, data):
        data = [float(v) for v in data]
        with self.tx:
            self.m.mav.debug_float_array_send(int(time.time() * 1e6), name.encode(), 0, data + [0.0] * (58 - len(data)))

    # ------------------------------------------------------------------ requests
    def connected(self):
        return self.heartbeats > 0 and time.monotonic() - self.heartbeat_t < 3.0

    def _require_link(self):
        if not self.connected():
            raise GatewayError("l'autopilota non risponde su MAVLink")

    def command(self, cmd, *p, timeout=3.0):
        """COMMAND_LONG; the MAV_RESULT of its acknowledgement."""
        self._require_link()
        p = [float(v) for v in p] + [0.0] * (7 - len(p))
        with self.cv:
            self.acks.pop(cmd, None)
        with self.tx:
            self.m.mav.command_long_send(self.m.target_system, self.m.target_component, cmd, 0, *p)
        with self.cv:
            if not self.cv.wait_for(lambda: cmd in self.acks, timeout):
                raise GatewayError("l'autopilota non ha risposto al comando")
            return self.acks[cmd]

    def service(self, op, ident=0, a0=0.0, a1=0.0, timeout=2.5):
        """NNM_SRV request; the NNM_SRVR reply."""
        self._require_link()
        with self.cv:
            self.seq = self.seq + 1 if self.seq < 65000 else 100
            seq = self.seq
            self.replies.pop(seq, None)
        self._array('NNM_SRV', [seq, OPS[op], ident, a0, a1])
        with self.cv:
            if not self.cv.wait_for(lambda: seq in self.replies, timeout):
                raise GatewayError(f"nessuna risposta alla richiesta '{op}'")
            return dict(self.replies.pop(seq), action=op)

    def param_set(self, name, value, timeout=3.0):
        if name not in PARAMS:
            raise GatewayError(f'parametro {name} non modificabile da qui')
        lo, hi = PARAMS[name]
        value = float(value)
        if not math.isfinite(value) or not lo <= value <= hi:
            raise GatewayError(f'{name} deve stare tra {lo} e {hi}')
        self._require_link()
        with self.cv:
            self.params.pop(name, None)
        with self.tx:
            self.m.mav.param_set_send(self.m.target_system, self.m.target_component, name.encode(), value,
                                      MAV.MAV_PARAM_TYPE_REAL32)
        with self.cv:
            if not self.cv.wait_for(lambda: name in self.params, timeout):
                raise GatewayError(f"l'autopilota non ha confermato {name}")
            return self.params[name]

    def arm(self, on: bool):
        if on:
            self.command(MAV.MAV_CMD_DO_SET_MODE, 1.0, 0.0)             # MANUAL: the sticks drive the policy
        result = self.command(MAV.MAV_CMD_COMPONENT_ARM_DISARM, 1.0 if on else 0.0, 21196.0)
        if result != MAV.MAV_RESULT_ACCEPTED:
            raise GatewayError(('armamento' if on else 'disarmo') + f' rifiutato (MAV_RESULT {result})')
        end = time.monotonic() + 3.0
        while self.armed != on and time.monotonic() < end:
            time.sleep(0.05)
        if not on:
            self.set_motion(0.0, 0.0, 0.0)
            self.head = [0.0] * 4
        return self.armed

    def skill(self, name):
        if name not in SKILLS:
            raise GatewayError(f'skill sconosciuta: {name}')
        result = self.command(MAV.MAV_CMD_USER_1, SKILLS[name])
        if result != MAV.MAV_RESULT_ACCEPTED:
            raise GatewayError(f"skill '{name}' rifiutata: serve il robot armato, in piedi e senza altre skill in corso")
        return True

    def set_motion(self, vx, vy, wz):
        v = [float(vx), float(vy), float(wz)]
        if any(not math.isfinite(x) or abs(x) > 1.0 for x in v):
            raise GatewayError('comando di moto fuori scala (-1..1)')
        self.twist, self.twist_t = v, time.monotonic()

    def set_head(self, head=None, mouth=None):
        if head is not None:
            h = [float(x) for x in head]
            if len(h) != 4 or any(not math.isfinite(x) or abs(x) > HEAD_LIMIT for x in h):
                raise GatewayError('comando testa fuori scala')
            self.head = h
        if mouth is not None:
            m = float(mouth)
            if not math.isfinite(m) or not 0.0 <= m <= MOUTH_MAX_DEG:
                raise GatewayError('apertura bocca fuori scala')
            self.mouth = m

    # ------------------------------------------------------------------ view
    def status(self) -> dict:
        now = time.monotonic()
        with self.cv:
            out = dict(connected=self.connected(), error=self.error, armed=self.armed,
                       mode={0: 'MANUAL', 4: 'HOLD'}.get(self.mode, None if self.mode is None else str(self.mode)),
                       params={k: round(v, 4) for k, v in self.params.items()},
                       texts=[dict(time=time.strftime('%H:%M:%S', time.localtime(t)), text=x) for t, x in self.texts[-12:]],
                       command=dict(vx=self.twist[0], vy=self.twist[1], wz=self.twist[2], head=list(self.head),
                                    mouth=self.mouth, active=now - self.twist_t < MOTION_TIMEOUT_S))
            n = self.named
            def value(key, digits):
                v = n.get(key)
                return round(v, digits) if v is not None and math.isfinite(v) else None
            out['policy_task'] = dict(hz=value('PPO_HZ', 1), longest_gap_ms=value('PPO_GAP', 1),
                                      inference_ms=value('PPO_MS', 2), fail=value('PPO_FAIL', 0), vx=value('PPO_VX', 3))
            if self.attitude is not None:
                out['attitude'] = dict(zip(('roll', 'pitch', 'yaw'), (round(v, 1) for v in self.attitude)))
            j = self.jnt
            if j is not None and now - self.jnt_t < 2.0:
                bits = int(j[0])
                present, held, slots = int(j[1]), int(j[2]), int(j[48])
                policy, skill, getup, chain = (int(j[i]) for i in (52, 53, 54, 55))
                out['bus'] = dict(status=[s for i, s in enumerate(STATUS_BITS) if bits >> i & 1],
                                  state_hz=round(j[49], 1), refused=int(j[50]), crc_errors=int(j[51]))
                out['robot'] = dict(policy=policy, policy_name=POLICY_NAMES[policy] if 0 <= policy < len(POLICY_NAMES) else '—',
                                    skill=SKILL_NAMES.get(skill), getup=GETUP_NAMES[getup] if getup < len(GETUP_NAMES) else str(getup),
                                    chain=CHAIN_NAMES[chain] if chain < len(CHAIN_NAMES) else str(chain),
                                    mouth_deg=round(j[56], 1))
                volt = self.vlt
                out['joints'] = [dict(slot=i, id=SLOT_IDS[i], name=SLOT_NAMES[i], present=bool(present >> i & 1),
                                      held=bool(held >> i & 1), angle=round(j[3 + i], 1), target=round(j[18 + i], 1),
                                      raw=int(j[33 + i]),
                                      voltage=None if volt is None or not int(volt[0]) >> i & 1 else round(volt[1 + i], 1))
                                 for i in range(min(slots, len(SLOT_IDS)))]
            return out
