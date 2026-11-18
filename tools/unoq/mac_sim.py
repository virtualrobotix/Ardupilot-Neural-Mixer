#!/usr/bin/env python3
# Author: Roberto Navoni, member of the ArduPilot Dev Team
# Contact: r.navoni74@gmail.com
# Developed by Roberto Navoni — DelphyAI LAB
# For information: r.navoni74@gmail.com
"""MuJoCo window on the Mac, driven by the autopilot on the Uno Q.

    .venv/bin/mjpython tools/unoq/mac_sim.py

The board listens on tcp:7000 (adb forward). This window sends the simulated IMU and joint
angles and applies the joint commands. On macOS the window opens only under mjpython.
Closing the window returns the board to the sketch.
"""
from __future__ import annotations

import socket
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'unoq' / 'app' / 'python'))

import frames  # noqa: E402
from sim_link import COMMAND, Duck, model_path  # noqa: E402

HOST = '127.0.0.1'
PORT = 7000
TICK_S = 0.01
POWERED = {1, 2, 3}


# Walk stance, policy joint order. Hips are python/stance.py BASELINE_HOME_DEG. The ankle is not:
# the robot stands at 18 degrees, this MicroDuck body falls on its beak there and stands at 25.95.
STANCE_DEG = (0.0, -5.0, -26.24, -0.28, 25.95, 20.0, 20.0, 0.0, 0.0, 0.0, 5.0, 26.24, 0.28, -25.95)
# HD-1910-C001 peak stall torque at 7.4 V is 15 kg·cm. The scene file still carries the XL330 limit.
HD1910_STALL_NM = 15.0 * 0.0980665
# N·m/rad. The get-up policy of the getup-hd1910 branch, run alone on this scene, stands
# from every start between 1.25 and 2.0. At the scene value 0.55 it settles crouched at
# 40 degrees of tilt, and from 3.0 up it falls again.
HD1910_KP = 1.5


def use_hd1910(duck: Duck):
    """Position servos: HD-1910 stall torque, scene stiffness. bam has no HD-1910 voltage model."""
    m = duck.model
    m.actuator_forcerange[:] = (-HD1910_STALL_NM, HD1910_STALL_NM)
    m.actuator_gainprm[:, 0] = HD1910_KP
    m.actuator_biasprm[:, 1] = -HD1910_KP
    m.actuator_biasprm[:, 2] = 0.0


def stand_on_stance(duck: Duck):
    """Put the duck on the measured stance, which is the walk and get-up home."""
    import math
    duck.reset()
    for joint, deg in enumerate(STANCE_DEG):
        q = math.radians(deg)
        duck.data.qpos[duck.qpos_idx[joint]] = q
        duck.data.ctrl[joint] = q
    duck.mujoco.mj_forward(duck.model, duck.data)
    duck.targets_deg = duck._angles_deg()


class Bus:
    """Network only. Physics stays on the viewer thread: MuJoCo is not safe to step elsewhere."""

    def __init__(self, duck: Duck):
        self.duck = duck
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
                sock = socket.create_connection((HOST, PORT), timeout=5)
            except OSError as error:
                print(f'[mac-sim] in attesa della Uno Q su {HOST}:{PORT} ({error})', flush=True)
                if self.stop.wait(1.0):
                    return
                continue
            print('[mac-sim] collegato: l\'autopilota usa questa finestra', flush=True)
            sock.settimeout(0.5)
            with self._send_lock:
                self._sock = sock
            with self.lock:
                stand_on_stance(self.duck)
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
    xml = model_path()
    if xml is None:
        sys.exit('scena MuJoCo non trovata')
    import mujoco.viewer
    duck = Duck(xml)
    use_hd1910(duck)
    stand_on_stance(duck)
    print('[mac-sim] servo HD-1910, kp 1.5 N·m/rad, posa della scena (anca ±26°, caviglia ±26°)',
          flush=True)
    bus = Bus(duck)
    threading.Thread(target=bus.connect_loop, name='bus', daemon=True).start()
    with mujoco.viewer.launch_passive(duck.model, duck.data, show_left_ui=False, show_right_ui=False) as viewer:
        viewer.cam.lookat[:] = (0.0, 0.0, 0.08)
        viewer.cam.distance = 0.75
        viewer.cam.azimuth = 140
        viewer.cam.elevation = -18
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
