#!/usr/bin/env python3
# Author: Roberto Navoni, member of the ArduPilot Dev Team
# Contact: r.navoni74@gmail.com
"""ArduDuck network helper. Runs on the Linux side of the Uno Q, outside the app container.

The app container cannot reach NetworkManager, so this small service does it with nmcli:

  * at start it waits FALLBACK_S for the Wi-Fi to connect; if it does not, it turns the board
    into the access point "ArduDuck" (10.42.0.1), so the pages stay reachable;
  * it carries out what the Network page asks for: scan, connect, forget, access point password,
    and the mode: "auto" (saved Wi-Fi networks, access point as fallback) or "ap" (access point
    only, also after a reboot). The mode is kept in config.json.

It talks to the app through files in <app>/data/network/, which the container sees as
/app/data/network/:

    status.json            written here every few seconds
    config.json            {"mode": "auto"|"ap", "previous": last network joined}, written here
    request-<id>.json      written by the app (python/network.py), removed here
    reply-<id>.json        written here, removed by the app

Installed as a systemd user service by tools/network-setup.sh. No root needed: on the Uno Q the
arduino user is allowed to control NetworkManager.
"""
import json
import subprocess
import sys
import time
from pathlib import Path

AP_PROFILE = 'ArduDuck-AP'
AP_SSID = 'ArduDuck'
AP_PASSWORD = 'runrunrun'      # initial Wi-Fi password; WPA2 needs 8 characters or more
AP_ADDRESS = '10.42.0.1'       # NetworkManager's address in shared mode
FALLBACK_S = 45.0
STATUS_PERIOD_S = 3.0


def nmcli(*args, timeout=45.0):
    """(ok, text): stdout when it worked, the error otherwise."""
    try:
        r = subprocess.run(['nmcli', *args], capture_output=True, text=True, timeout=timeout)
    except (OSError, subprocess.TimeoutExpired) as e:
        return False, str(e)
    return r.returncode == 0, (r.stdout if r.returncode == 0 else (r.stderr or r.stdout)).strip()


def fields(line):
    """Split one line of `nmcli -t`: ':' separates, '\\:' is a colon inside a value."""
    out, current, escaped = [], '', False
    for ch in line:
        if escaped:
            current, escaped = current + ch, False
        elif ch == '\\':
            escaped = True
        elif ch == ':':
            out.append(current)
            current = ''
        else:
            current += ch
    out.append(current)
    return out


class Wifi:
    def __init__(self):
        self.device = self._device()
        self.networks = []          # last scan, kept while the access point holds the radio
        self.scan_time = None
        self._ssid = {}
        self.previous = None        # last network joined: the first to try when leaving the access point

    @staticmethod
    def _device():
        ok, text = nmcli('-t', '-f', 'DEVICE,TYPE', 'device')
        for line in text.splitlines() if ok else []:
            name, kind = (fields(line) + [''])[:2]
            if kind == 'wifi':
                return name
        return 'wlan0'

    def link(self):
        """state: 'client', 'ap', 'connecting' or 'disconnected'."""
        ok, text = nmcli('-t', '-f', 'GENERAL.STATE,GENERAL.CONNECTION,IP4.ADDRESS', 'device', 'show', self.device)
        out = dict(state='disconnected', connection=None, ssid=None, ip=None)
        if not ok:
            return out
        code = 0
        for line in text.splitlines():
            key, _, value = line.partition(':')
            value = ':'.join(fields(value))
            if key == 'GENERAL.STATE':
                code = int(value.split()[0]) if value.split() and value.split()[0].isdigit() else 0
            elif key == 'GENERAL.CONNECTION' and value:
                out['connection'] = value
            elif key.startswith('IP4.ADDRESS') and out['ip'] is None:
                out['ip'] = value.split('/')[0]
        if code == 100 and out['connection']:
            out['state'] = 'ap' if out['connection'] == AP_PROFILE else 'client'
            out['ssid'] = AP_SSID if out['state'] == 'ap' else self._profile_ssid(out['connection'])
            if out['state'] == 'client':
                self.previous = out['connection']
        elif 40 <= code < 100:
            out['state'] = 'connecting'
        return out

    def _profile_ssid(self, name):
        if name not in self._ssid:
            ok, text = nmcli('-g', '802-11-wireless.ssid', 'connection', 'show', name)
            if ok and text:
                self._ssid[name] = text
        return self._ssid.get(name, name)

    def saved(self):
        ok, text = nmcli('-t', '-f', 'NAME,TYPE', 'connection', 'show')
        names = []
        for line in text.splitlines() if ok else []:
            name, kind = (fields(line) + [''])[:2]
            if kind == '802-11-wireless' and name != AP_PROFILE:
                names.append(name)
        return names

    def scan(self, rescan=True):
        """The networks in range, strongest first, one row per name."""
        args = ('-t', '-f', 'SSID,SIGNAL,SECURITY,IN-USE', 'device', 'wifi', 'list', 'ifname', self.device)
        ok, text = nmcli(*args, '--rescan', 'yes' if rescan else 'no', timeout=30.0)
        if not ok and rescan:
            ok, text = nmcli(*args, '--rescan', 'no')
        if not ok:
            return False, text
        best = {}
        for line in text.splitlines():
            ssid, signal, security, in_use = (fields(line) + ['', '', '', ''])[:4]
            if not ssid or ssid == AP_SSID:
                continue
            row = dict(ssid=ssid, signal=int(signal) if signal.isdigit() else 0,
                       secure=security not in ('', '--'), in_use=in_use == '*')
            if ssid not in best or row['in_use'] or (row['signal'] > best[ssid]['signal'] and not best[ssid]['in_use']):
                best[ssid] = row
        if best or not self.networks:
            self.networks = sorted(best.values(), key=lambda r: -r['signal'])
            self.scan_time = time.strftime('%H:%M:%S')
        return True, ''

    # ── access point
    def ap_profile(self):
        ok, _ = nmcli('-g', 'connection.id', 'connection', 'show', AP_PROFILE)
        if ok:
            return True, ''
        return nmcli('connection', 'add', 'type', 'wifi', 'ifname', self.device, 'con-name', AP_PROFILE,
                     'autoconnect', 'no', 'ssid', AP_SSID, '802-11-wireless.mode', 'ap', '802-11-wireless.band', 'bg',
                     'ipv4.method', 'shared', 'ipv6.method', 'ignore',
                     'wifi-sec.key-mgmt', 'wpa-psk', 'wifi-sec.proto', 'rsn', 'wifi-sec.pairwise', 'ccmp',
                     'wifi-sec.group', 'ccmp', 'wifi-sec.psk', AP_PASSWORD)

    def ap_up(self):
        ok, text = self.ap_profile()
        if not ok:
            return False, text
        self.scan(rescan=True)      # while the radio can still listen
        return nmcli('--wait', '20', 'connection', 'up', 'id', AP_PROFILE, 'ifname', self.device)

    def ap_down(self):
        return nmcli('--wait', '10', 'connection', 'down', 'id', AP_PROFILE)

    def ap_password(self, password):
        ok, text = self.ap_profile()
        if not ok:
            return False, text
        ok, text = nmcli('connection', 'modify', 'id', AP_PROFILE, 'wifi-sec.psk', password)
        if ok and self.link()['state'] == 'ap':
            ok, text = nmcli('--wait', '20', 'connection', 'up', 'id', AP_PROFILE, 'ifname', self.device)
        return ok, text

    # ── client
    def connect(self, ssid, password):
        existed = ssid in self.saved()
        if self.link()['state'] == 'ap':
            self.ap_down()
        self.scan(rescan=True)      # NetworkManager must have seen the network to connect to it
        if existed:
            ok, text = (True, '')
            if password:
                ok, text = nmcli('connection', 'modify', 'id', ssid, 'wifi-sec.key-mgmt', 'wpa-psk', 'wifi-sec.psk', password)
            if ok:
                ok, text = nmcli('--wait', '30', 'connection', 'up', 'id', ssid, 'ifname', self.device)
        else:
            args = ['--wait', '30', 'device', 'wifi', 'connect', ssid, 'ifname', self.device]
            if password:
                args += ['password', password]
            ok, text = nmcli(*args)
            if not ok and ssid in self.saved():
                nmcli('connection', 'delete', 'id', ssid)     # a profile that never worked
        self._ssid.pop(ssid, None)
        return ok, text

    def join_saved(self):
        """Back to a saved network that is in range: the last one joined, then the strongest."""
        self.scan(rescan=True)
        saved = {self._profile_ssid(name): name for name in self.saved()}
        in_range = [saved[n['ssid']] for n in self.networks if n['ssid'] in saved]
        for name in sorted(in_range, key=lambda name: name != self.previous):
            if nmcli('--wait', '30', 'connection', 'up', 'id', name, 'ifname', self.device)[0]:
                return True
        return False

    def forget(self, name):
        if name == AP_PROFILE or name not in self.saved():
            return False, 'rete non salvata'
        self._ssid.pop(name, None)
        return nmcli('connection', 'delete', 'id', name)


class Service:
    def __init__(self, folder):
        self.folder = Path(folder)
        self.folder.mkdir(parents=True, exist_ok=True)
        self.wifi = Wifi()
        self.last = None
        try:
            config = json.loads((self.folder / 'config.json').read_text())
        except (OSError, ValueError):
            config = {}
        self.mode = 'ap' if config.get('mode') == 'ap' else 'auto'
        self.wifi.previous = self.saved_previous = config.get('previous')
        # None once a network or the access point is up. In "ap" mode there is nothing to wait for.
        self.fallback_at = time.monotonic() + (FALLBACK_S if self.mode == 'auto' else 0.0)
        self.status_at = 0.0

    def write(self, name, data):
        tmp = self.folder / (name + '.tmp')
        tmp.write_text(json.dumps(data, indent=1) + '\n')
        tmp.replace(self.folder / name)

    def note(self, action, ok, text, **more):
        self.last = dict(action=action, ok=bool(ok), message='' if ok else text, time=time.strftime('%H:%M:%S'), **more)
        print(f'arduduck-network: {action} {more or ""} -> {"ok" if ok else text}', flush=True)

    def set_mode(self, mode):
        self.mode = mode
        self.saved_previous = self.wifi.previous
        self.write('config.json', dict(mode=mode, previous=self.wifi.previous))

    def status(self):
        link = self.wifi.link()
        due = self.fallback_at is not None and time.monotonic() >= self.fallback_at
        if self.mode == 'ap' and link['state'] != 'ap':
            # access point only: also when NetworkManager joined a saved network by itself at boot
            if due or self.fallback_at is None:
                ok, text = self.wifi.ap_up()
                self.note('ap_only', ok, text)
                self.fallback_at = None if ok else time.monotonic() + 10.0
                link = self.wifi.link()
        elif link['state'] in ('client', 'ap'):
            self.fallback_at = None
        elif link['state'] == 'disconnected' and due:
            ok, text = self.wifi.ap_up()
            self.note('fallback_ap', ok, text)
            self.fallback_at = None if ok else time.monotonic() + 10.0
            link = self.wifi.link()
        if self.wifi.previous != self.saved_previous:
            self.set_mode(self.mode)
        self.write('status.json', dict(
            time=time.strftime('%H:%M:%S'), device=self.wifi.device, mode=self.mode, wifi=link,
            ap=dict(ssid=AP_SSID, address=AP_ADDRESS, active=link['state'] == 'ap'),
            saved=self.wifi.saved(), networks=self.wifi.networks, scan_time=self.wifi.scan_time,
            fallback_s=FALLBACK_S, last=self.last))
        self.status_at = time.monotonic()

    def handle(self, request):
        action, ident = request.get('action'), request['id']
        reply = lambda **data: self.write(f'reply-{ident}.json', dict(id=ident, **data))
        if action == 'scan':
            ok, text = self.wifi.scan(rescan=True)
            reply(ok=ok, message=text)
        elif action == 'connect':
            ssid, password = str(request.get('ssid', '')), str(request.get('password', ''))
            reply(ok=True)          # the answer is in status.json: the page may lose the link meanwhile
            ok, text = self.wifi.connect(ssid, password)
            self.note('connect', ok, text, ssid=ssid)
            if ok:
                self.set_mode('auto')
            elif self.mode == 'ap' or not self.wifi.join_saved():    # back where it was, or the access point
                self.fallback_at = time.monotonic() + 3.0
        elif action == 'forget':
            ok, text = self.wifi.forget(str(request.get('name', '')))
            self.note('forget', ok, text, ssid=request.get('name'))
            reply(ok=ok, message=text)
            if self.wifi.link()['state'] == 'disconnected':
                self.fallback_at = time.monotonic() + 5.0
        elif action == 'mode':
            reply(ok=True)
            self.set_mode('ap' if request.get('mode') == 'ap' else 'auto')
            if self.mode == 'ap':
                ok, text = self.wifi.ap_up()
                self.note('ap_only', ok, text)
                self.fallback_at = None if ok else time.monotonic() + 10.0
            else:
                ok, text = self.wifi.ap_down() if self.wifi.link()['state'] == 'ap' else (True, '')
                self.note('wifi_mode', ok, text)
                if not self.wifi.join_saved():
                    self.fallback_at = time.monotonic() + FALLBACK_S  # back to the access point if nothing connects
        elif action == 'ap_password':
            reply(ok=True)
            ok, text = self.wifi.ap_password(str(request.get('password', '')))
            self.note('ap_password', ok, text)
        else:
            reply(ok=False, message=f'unknown action {action}')

    def run(self):
        print(f'arduduck-network: started, device {self.wifi.device}, folder {self.folder}', flush=True)
        self.wifi.scan(rescan=False)
        while True:
            for path in sorted(self.folder.glob('request-*.json'), key=lambda p: p.stat().st_mtime):
                try:
                    request = json.loads(path.read_text())
                    path.unlink()
                    self.handle(request)
                except Exception as e:      # a bad request must not stop the service
                    print(f'arduduck-network: {path.name}: {type(e).__name__}: {e}', flush=True)
                    path.unlink(missing_ok=True)
                self.status_at = 0.0
            for path in self.folder.glob('reply-*.json'):
                if time.time() - path.stat().st_mtime > 120:
                    path.unlink(missing_ok=True)
            if time.monotonic() - self.status_at >= STATUS_PERIOD_S:
                try:
                    self.status()
                except Exception as e:
                    print(f'arduduck-network: status: {type(e).__name__}: {e}', flush=True)
                    self.status_at = time.monotonic()
            time.sleep(0.3)


if __name__ == '__main__':
    default = Path(__file__).resolve().parents[1] / 'data' / 'network'
    Service(sys.argv[1] if len(sys.argv) > 1 else default).run()
