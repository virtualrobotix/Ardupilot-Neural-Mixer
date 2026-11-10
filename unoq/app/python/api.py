# Author: Roberto Navoni, member of the ArduPilot Dev Team
# Contact: r.navoni74@gmail.com
# Developed by Roberto Navoni — DelphyAI LAB
# For information: r.navoni74@gmail.com
"""What the web pages can ask for. Every action becomes MAVLink in gateway.py.

    GET  /api/status
    POST /api/arm       {"armed": true|false}
    POST /api/stop      {}                               disarm and release the servos
    POST /api/torque    {"on": true|false}               disarmed: balance pose / servos released
    POST /api/motion    {"vx": -1..1, "vy": -1..1, "wz": -1..1}   repeat at 5 Hz or more while moving
    POST /api/head      {"head": [4 x -1..1], "mouth": 0..30}
    POST /api/skill     {"name": "pick"|"roulade"|"sit"|"stand"|"kick_left"|"kick_right"}
    POST /api/param     {"name": "NNM_DEPLOY", "value": 1}
    POST /api/service   {"action": "cal_enter"|"cal_center"|"ping"|..., "id": .., ...}
"""
from __future__ import annotations

import math
import time

from gateway import PROTECTION_CURRENT_UNIT_A, SLOT_IDS, GatewayError


class Api:
    def __init__(self, gateway, relay, supervisor):
        self.gateway, self.relay, self.supervisor = gateway, relay, supervisor

    def status(self):
        link = self.relay.status()
        out = dict(time=time.strftime('%H:%M:%S'), process=self.supervisor.status(), link=link,
                   autopilot=self.gateway.status())
        acc = link.get('acc_g')
        if acc and link.get('imu_ok'):
            norm = math.sqrt(sum(a * a for a in acc))
            if norm > 0.2:
                # standing level reads (0, 0, -1) g in the sketch's axes
                out['tilt_deg'] = round(math.degrees(math.acos(max(-1.0, min(1.0, -acc[2] / norm)))), 1)
        return out

    def arm(self, body):
        on = bool(body['armed'])
        note = None
        if on:
            link = self.relay.status()
            if not link.get('mcu_link'):
                raise GatewayError('lo sketch sul microcontrollore non risponde')
            missing = [i for i in link.get('servos_missing', []) if i != 34]
            if not link.get('imu_ok'):
                note = "IMU non valida: la policy gira ma il microcontrollore non dà coppia"
            elif missing:
                note = f"servo assenti ({len(missing)}): la policy gira ma il microcontrollore non dà coppia"
        return dict(armed=self.gateway.arm(on), note=note)

    def stop(self, body=None):
        """Emergency: disarm, then release the servos."""
        errors = []
        for step in (lambda: self.gateway.arm(False), lambda: self.gateway.service('torque_off')):
            try:
                step()
            except GatewayError as e:
                errors.append(str(e))
        if errors:
            raise GatewayError('; '.join(errors))
        return dict(armed=False, torque=False)

    def torque(self, body):
        r = self.gateway.service('default_pose' if body['on'] else 'torque_off')
        if not r['ok']:
            raise GatewayError('rifiutato: si può fare solo da disarmato')
        return r

    def motion(self, body):
        self.gateway.set_motion(body.get('vx', 0.0), body.get('vy', 0.0), body.get('wz', 0.0))
        return dict(ok=True)

    def head(self, body):
        self.gateway.set_head(body.get('head'), body.get('mouth'))
        return dict(ok=True)

    def skill(self, body):
        return dict(ok=self.gateway.skill(body['name']))

    def param(self, body):
        return dict(name=body['name'], value=self.gateway.param_set(body['name'], body['value']))

    def service(self, body):
        action = body['action']
        g = self.gateway
        if action in ('cal_enter', 'cal_exit', 'servo_enter', 'servo_exit', 'heartbeat'):
            r = g.service(action)
        elif action == 'cal_center':
            index = int(body['index'])
            if not 0 <= index < len(SLOT_IDS):
                raise GatewayError('giunto inesistente')
            r = g.service(action, index, timeout=4.0)
        elif action in ('ping', 'read', 'unlock', 'reset_mid'):
            r = g.service(action, self._servo_id(body['id']), timeout=4.0)
        elif action == 'set_id':
            r = g.service(action, self._servo_id(body['id']), self._servo_id(body['new_id']), timeout=4.0)
        elif action == 'set_gains':
            kp, kd = int(body['kp']), int(body['kd'])
            if not (0 <= kp <= 255 and 0 <= kd <= 255):
                raise GatewayError('guadagni fuori scala (0..255)')
            r = g.service(action, self._servo_id(body['id']), kp, kd, timeout=4.0)
        elif action == 'set_protection':
            r = g.service(action, 0, 1 if body['enabled'] else 0, timeout=4.0)
        elif action == 'set_protection_current':
            amps = float(body['current_a'])
            if not math.isfinite(amps) or not 0.0 <= amps <= 6.0:
                raise GatewayError('corrente di protezione fuori scala (0..6 A)')
            r = g.service(action, 0, round(amps / PROTECTION_CURRENT_UNIT_A), timeout=4.0)
        else:
            raise GatewayError(f'operazione sconosciuta: {action}')
        r['current_a'] = round(r['current'] * PROTECTION_CURRENT_UNIT_A, 2)
        return r

    @staticmethod
    def _servo_id(value):
        ident = int(value)
        if not 0 <= ident <= 253:
            raise GatewayError('ID servo fuori scala (0..253)')
        return ident


def register(ui, api: Api):
    """Publish the API on the App Lab WebUI brick (FastAPI underneath)."""
    from fastapi import HTTPException

    def post(fn):
        def handler(body: dict = None):
            try:
                return fn(body or {})
            except GatewayError as e:
                raise HTTPException(400, str(e))
            except (KeyError, TypeError, ValueError) as e:
                raise HTTPException(400, f'richiesta non valida: {e}')
        return handler

    ui.expose_api('GET', '/api/status', api.status)
    for name in ('arm', 'stop', 'torque', 'motion', 'head', 'skill', 'param', 'service'):
        ui.expose_api('POST', f'/api/{name}', post(getattr(api, name)))
