# Author: Roberto Navoni, member of the ArduPilot Dev Team
# Contact: r.navoni74@gmail.com
"""Network page: the app side.

The container cannot reach NetworkManager. tools/arduduck_network.py runs on the board outside the
container and does the work; the two meet in data/network/ (status.json, request-*.json,
reply-*.json). This module also keeps the login of the Network page: user "arduduck", initial
password "runrun", stored salted and hashed in data/network/login.json once it is changed.
"""
from __future__ import annotations

import hashlib
import hmac
import json
import os
import time
import uuid
from pathlib import Path

USER = 'arduduck'
INITIAL_PASSWORD = 'runrun'
STATUS_MAX_AGE_S = 15.0


class NetworkError(Exception):
    pass


class LoginError(NetworkError):
    pass


def _digest(password, salt):
    return hashlib.pbkdf2_hmac('sha256', password.encode(), salt, 100_000).hex()


class Network:
    def __init__(self, app_dir):
        self.folder = Path(app_dir) / 'data' / 'network'
        self.folder.mkdir(parents=True, exist_ok=True)

    # ── login
    def _login(self):
        try:
            return json.loads((self.folder / 'login.json').read_text())
        except (OSError, ValueError):
            salt = b'arduduck-initial'
            return dict(user=USER, salt=salt.hex(), hash=_digest(INITIAL_PASSWORD, salt), initial=True)

    def check(self, body):
        login = self._login()
        user, password = str(body.get('user', '')), str(body.get('password', ''))
        good = hmac.compare_digest(_digest(password, bytes.fromhex(login['salt'])), login['hash'])
        if not (good and hmac.compare_digest(user, login['user'])):
            time.sleep(1.0)     # slows down guessing
            raise LoginError('accesso negato: utente o password errati')
        return login

    def set_password(self, password):
        if not 6 <= len(password) <= 64:
            raise NetworkError('la password della pagina deve avere da 6 a 64 caratteri')
        salt = os.urandom(16)
        path = self.folder / 'login.json'
        tmp = path.with_suffix('.tmp')
        tmp.write_text(json.dumps(dict(user=USER, salt=salt.hex(), hash=_digest(password, salt))) + '\n')
        os.chmod(tmp, 0o600)
        tmp.replace(path)

    # ── helper on the board
    def _status(self):
        path = self.folder / 'status.json'
        try:
            if time.time() - path.stat().st_mtime > STATUS_MAX_AGE_S:
                return None
            return json.loads(path.read_text())
        except (OSError, ValueError):
            return None

    def public(self):
        """What anyone on the page may see: no network names other than the one in use."""
        s = self._status()
        if s is None:
            return dict(service=False)
        return dict(service=True, mode=s.get('mode', 'auto'), wifi=s['wifi'], ap=s['ap'], fallback_s=s['fallback_s'],
                    last=s['last'])

    def _request(self, timeout, **request):
        if self._status() is None:
            raise NetworkError('il servizio di rete non è attivo sulla scheda')
        ident = uuid.uuid4().hex[:12]
        path = self.folder / f'request-{ident}.json'
        tmp = path.with_suffix('.tmp')
        tmp.write_text(json.dumps(dict(id=ident, **request)))
        os.chmod(tmp, 0o600)
        tmp.replace(path)
        reply = self.folder / f'reply-{ident}.json'
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                data = json.loads(reply.read_text())
            except (OSError, ValueError):
                time.sleep(0.1)
                continue
            reply.unlink(missing_ok=True)
            if not data.get('ok'):
                raise NetworkError(data.get('message') or 'operazione non riuscita')
            return data
        path.unlink(missing_ok=True)
        raise NetworkError('il servizio di rete non risponde')

    def handle(self, body):
        login = self.check(body)
        action = body.get('action', 'status')
        if action == 'scan':
            self._request(40.0, action='scan')
        elif action == 'connect':
            ssid, password = str(body.get('ssid', '')).strip(), str(body.get('psk', ''))   # 'password' is the login
            if not 1 <= len(ssid.encode()) <= 32:
                raise NetworkError('nome della rete non valido')
            if password and not 8 <= len(password) <= 63:
                raise NetworkError('la password Wi-Fi deve avere da 8 a 63 caratteri')
            self._request(8.0, action='connect', ssid=ssid, password=password)
        elif action == 'forget':
            self._request(15.0, action='forget', name=str(body.get('name', '')))
        elif action == 'mode':
            if body.get('mode') not in ('auto', 'ap'):
                raise NetworkError('modalità non valida')
            self._request(8.0, action='mode', mode=body['mode'])
        elif action == 'ap_password':
            password = str(body.get('new_password', ''))
            if not 8 <= len(password) <= 63:
                raise NetworkError('la password Wi-Fi deve avere da 8 a 63 caratteri')
            self._request(8.0, action='ap_password', password=password)
        elif action == 'set_password':
            self.set_password(str(body.get('new_password', '')))
            login = self._login()
        elif action != 'status':
            raise NetworkError(f'operazione sconosciuta: {action}')
        s = self._status()
        out = self.public()
        out.update(user=login['user'], initial_password=bool(login.get('initial')),
                   saved=s['saved'] if s else [], networks=s['networks'] if s else [],
                   scan_time=s.get('scan_time') if s else None)
        return out
