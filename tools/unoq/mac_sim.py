#!/usr/bin/env python3
# Author: Roberto Navoni, member of the ArduPilot Dev Team
# Contact: r.navoni74@gmail.com
# Developed by Roberto Navoni — DelphyAI LAB
# For information: r.navoni74@gmail.com
"""MuJoCo window on the host computer (macOS, Windows or Linux), driven by the autopilot on the Uno Q.

    .venv/bin/mjpython tools/unoq/mac_sim.py --host 192.168.1.50     macOS, board on the network
    python tools/unoq/mac_sim.py --host 192.168.1.50                 Windows and Linux
    .venv/bin/mjpython tools/unoq/mac_sim.py                         over USB, after: adb forward tcp:7000 tcp:7000

The board listens on tcp:7000. --host is its address on the Wi-Fi network (10.42.0.1 when it is the
ArduDuck access point); without it the window connects to 127.0.0.1, which is the board through
adb forward. This window sends the simulated IMU and joint angles and applies the joint commands.
On macOS the window opens only under mjpython. Closing the window returns the board to the sketch.

The robot is the XGO Duck of the training stack (xgoduck_scene.py): its body, the electronics
backpack, the gear play and the HLS1910 voltage model. It needs third_party/xgoduck_rl.
"""
from __future__ import annotations

import argparse
import socket
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'unoq' / 'app' / 'python'))

import frames  # noqa: E402
from sim_link import COMMAND, SimError  # noqa: E402
from xgoduck_scene import KP_FW, VIN, XgoDuck  # noqa: E402

HOST = '127.0.0.1'
PORT = 7000
TICK_S = 0.01
POWERED = {1, 2, 3}


class Bus:
    """Network only. Physics stays on the viewer thread: MuJoCo is not safe to step elsewhere."""

    def __init__(self, duck: XgoDuck, host: str = HOST, port: int = PORT):
        self.duck = duck
        self.host, self.port = host, port
        self.lock = threading.Lock()
        self.q_target = None
        self.mouth = 0.0
        self.powered = False
        self.stop = threading.Event()
        self._sock = None
        self._send_lock = threading.Lock()

    def apply(self, packet: bytes):
        if packet[:4] != b'DQC1' or len(packet) != COMMAND.size:
            return
        values = COMMAND.unpack(packet)
        mode, targets = values[4], values[7:]
        with self.lock:
            self.powered = mode in POWERED
            self.duck.targets_deg = [float(value) for value in targets]
            self.mouth = float(targets[14])
            self.q_target = self.duck.slot_deg_to_q(targets) if self.powered else None

    def connect_loop(self):
        while not self.stop.is_set():
            try:
                sock = socket.create_connection((self.host, self.port), timeout=5)
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)     # 100 Hz of small packets
            except OSError as error:
                print(f'[mac-sim] in attesa della Uno Q su {self.host}:{self.port} ({error})', flush=True)
                if self.stop.wait(1.0):
                    return
                continue
            print('[mac-sim] collegato: l\'autopilota usa questa finestra', flush=True)
            sock.settimeout(0.5)
            with self._send_lock:
                self._sock = sock
            with self.lock:
                self.duck.reset()
                self.q_target = None
                self.powered = False
            self._recv(sock)
            with self._send_lock:
                if self._sock is sock:
                    self._sock = None
            try:
                sock.close()
            except OSError:
                pass
            if not self.stop.is_set():
                print('[mac-sim] collegamento chiuso, riprovo', flush=True)

    def send(self, packets):
        with self._send_lock:
            sock = self._sock
        if sock is None:
            return
        try:
            for packet in packets:
                sock.sendall(frames.frame(packet))
        except OSError:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass

    def _recv(self, sock):
        deframer = frames.Deframer()
        while not self.stop.is_set():
            try:
                data = sock.recv(8192)
            except socket.timeout:
                continue
            except OSError:
                break
            if not data:
                break
            for packet in deframer.feed(data):
                self.apply(packet)


def main():
    parser = argparse.ArgumentParser(description='MuJoCo in place of the robot, for the autopilot on the Uno Q.')
    parser.add_argument('--host', default=HOST, help='address of the board (default 127.0.0.1: adb forward tcp:7000 tcp:7000)')
    parser.add_argument('--port', type=int, default=PORT)
    args = parser.parse_args()
    import mujoco.viewer
    try:
        duck = XgoDuck()
    except SimError as error:
        sys.exit(str(error))
    print(f'[mac-sim] XGO Duck di addestramento: {sum(duck.model.body_mass):.3f} kg, servo HLS1910 '
          f'(BAM, guadagno {KP_FW:g}, {VIN:g} V), gioco ±1,5°, zaino elettronica', flush=True)
    bus = Bus(duck, args.host, args.port)
    threading.Thread(target=bus.connect_loop, name='bus', daemon=True).start()
    with mujoco.viewer.launch_passive(duck.model, duck.data, show_left_ui=False, show_right_ui=False) as viewer:
        # follow the trunk: a walking duck leaves a fixed frame in a few seconds
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        viewer.cam.trackbodyid = duck.trunk_id
        viewer.cam.distance = 0.75
        viewer.cam.azimuth = 140
        viewer.cam.elevation = -18
        # floor (group 0) and visual meshes (group 2); the collision boxes (group 3) stay hidden
        viewer.opt.geomgroup[:] = [1, 1, 1, 0, 0, 0]
        nxt = time.perf_counter()
        while viewer.is_running():
            nxt += TICK_S
            with bus.lock:
                duck.step(bus.q_target, bus.mouth)
                packets = duck.packets(bus.powered)
                viewer.sync()
            bus.send(packets)
            delay = nxt - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
            else:
                nxt = time.perf_counter()
    bus.stop.set()


if __name__ == '__main__':
    main()
