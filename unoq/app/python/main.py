# Author: Roberto Navoni, member of the ArduPilot Dev Team
# Contact: r.navoni74@gmail.com
# Developed by Roberto Navoni — DelphyAI LAB
# For information: r.navoni74@gmail.com
"""ArduHumanoid on Arduino Uno Q.

    sketch (STM32)  <-- Bridge -->  relay  <-- UDP -->  ardurover  <-- MAVLink -->  gateway  <-- HTTP -->  pages

relay.py       passes the sketch packets to ArduPilot's joint-bus port and back
supervisor.py  runs ardurover and starts it again if it exits
gateway.py     MAVLink ground station for the pages
api.py         what the pages can ask for, on port 9527 with the pages themselves (assets/)

The status is also written once a second to data/status.json and logged every 10 s.
"""
import json
import os
import sys
import threading
import time
from pathlib import Path

sys.dont_write_bytecode = True

from arduino.app_bricks.web_ui import WebUI
from arduino.app_utils import App, Bridge

from api import Api, register
from gateway import Gateway
from relay import Relay
from supervisor import Supervisor

APP_DIR = Path(os.environ.get('APP_HOME_IN_CONTAINER', Path(__file__).resolve().parents[1]))

ui = WebUI(port=9527, cors_origins='')
relay = Relay(Bridge).start()
supervisor = Supervisor(APP_DIR).start()
gateway = Gateway().start()
api = Api(gateway, relay, supervisor)
register(ui, api)


def report():
    path = APP_DIR / 'data' / 'status.json'
    n = 0
    while True:
        time.sleep(1.0)
        try:
            status = api.status()
            path.parent.mkdir(parents=True, exist_ok=True)
            tmp = path.with_suffix('.tmp')
            tmp.write_text(json.dumps(status, indent=1) + '\n')
            tmp.replace(path)
        except Exception as e:
            print(f'ArduHumanoid: status not written: {e}', flush=True)
            continue
        n += 1
        if n % 10 == 1:
            link, ap = status['link'], status['autopilot']
            print(f"ArduHumanoid: ardurover running={status['process']['running']} mavlink={ap['connected']} "
                  f"armed={ap['armed']} mcu_link={link['mcu_link']} state={link['state_hz']} Hz "
                  f"command={link['command_hz']} Hz imu={link.get('imu', {}).get('kind')} imu_ok={link.get('imu_ok')} "
                  f"servos={len(link.get('servos_answering', []))}/15 torque={link.get('torque')}", flush=True)


threading.Thread(target=report, name='report', daemon=True).start()
try:
    App.run()
finally:
    gateway.stop()
    supervisor.stop()
