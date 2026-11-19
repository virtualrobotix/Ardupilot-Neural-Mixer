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
                        goto {"id", "raw": 0..4095}; run_gains {} (sends the robot's KP/KD table)
    POST /api/sim       {"mode": "off"|"simulate"|"debug", "reset": false}
    GET  /api/network                                    Wi-Fi or access point, address (no login)
    POST /api/network   {"user", "password", "action": "status"|"scan"|"connect" {"ssid", "psk"}|"forget"|"mode" {"mode": "auto"|"ap"}|
                        "ap_password"|"set_password", ...}   see network.py; the only API with a login
"""
from __future__ import annotations

import math
import time

from gateway import PROTECTION_CURRENT_UNIT_A, SLOT_IDS, GatewayError
from network import LoginError, NetworkError
from sim_link import SimError


class Api:
    def __init__(self, gateway, relay, supervisor, sim=None, network=None):
        self.gateway, self.relay, self.supervisor, self.simlink = gateway, relay, supervisor, sim
        self.net = network

    def status(self):
        link = self.relay.status()
        sim = self.simlink.status() if self.simlink is not None else dict(mode='off', available=False, error=None)
        if self.relay.remote_active():
            sim = dict(sim, mode='simulate', available=True, engine='MuJoCo sul Mac', error=None,
                       powered=bool(link.get('torque')), gravity_g=link.get('acc_g'),
                       joints_deg=link.get('angle_deg'))
        out = dict(time=time.strftime('%H:%M:%S'), process=self.supervisor.status(), link=link,
                   autopilot=self.gateway.status(), sim=sim)
        acc = link.get('acc_g')
        if acc and link.get('imu_ok'):
            norm = math.sqrt(sum(a * a for a in acc))
            if norm > 0.2:
                # standing level reads (0, 0, -1) g in the sketch's axes
                out['tilt_deg'] = round(math.degrees(math.acos(max(-1.0, min(1.0, -acc[2] / norm)))), 1)
                if self.relay.remote_active():
                    sim['tilt_deg'] = out['tilt_deg']
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

    def sim(self, body):
        if self.simlink is None:
            raise GatewayError('simulazione non disponibile in questo processo')
        mode = body.get('mode', self.simlink.mode)
        remote = self.relay.remote_active()
        leaving = mode != 'simulate' and (self.simlink.mode == 'simulate' or remote)
        if leaving and self.gateway.armed:
            raise GatewayError('disarma prima di uscire dalla simulazione: i comandi tornerebbero ai motori reali')
        if mode != 'simulate':
            self.relay.drop_remote()
        elif remote:
            if body.get('reset'):
                self.relay.drop_remote()
            return self.simlink.status()
        try:
            return self.simlink.set_mode(mode, reset=bool(body.get('reset')))
        except SimError as e:
            raise GatewayError(str(e))

    def network_status(self):
        return self.net.public() if self.net is not None else dict(service=False)

    def network(self, body):
        if self.net is None:
            raise NetworkError('rete non disponibile in questo processo')
        return self.net.handle(body)

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
        elif action == 'goto':
            raw = int(body['raw'])
            if not 0 <= raw <= 4095:
                raise GatewayError('posizione fuori scala (0..4095)')
            r = g.service(action, self._servo_id(body['id']), raw, timeout=4.0)
        elif action == 'run_gains':
            r = g.service(action, timeout=4.0)
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
            except LoginError as e:
                raise HTTPException(401, str(e))
            except (GatewayError, NetworkError) as e:
                raise HTTPException(400, str(e))
            except (KeyError, TypeError, ValueError) as e:
                raise HTTPException(400, f'richiesta non valida: {e}')
        return handler

    ui.expose_api('GET', '/api/status', api.status)
    ui.expose_api('GET', '/api/network', api.network_status)
    for name in ('arm', 'stop', 'torque', 'motion', 'head', 'skill', 'param', 'service', 'sim', 'network'):
        ui.expose_api('POST', f'/api/{name}', post(getattr(api, name)))
