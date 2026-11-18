# Author: Roberto Navoni, member of the ArduPilot Dev Team
# Contact: r.navoni74@gmail.com
# Developed by Roberto Navoni — DelphyAI LAB
# For information: r.navoni74@gmail.com
"""MuJoCo in place of the real servos and IMU, switched from the web page.

off        the sketch is the only source, as before
simulate   IMU and joint angles sent to the autopilot come from the scene. Commands are not
           forwarded to the sketch, and the real servos are released. This is how a policy is
           tried when the robot is not connected.
debug      the real robot stays in the loop. The same joint commands are repeated in the scene,
           and the page shows the simulated angle next to the measured one.

The scene is the MicroDuck model (same 14 joint names). Actuators are the position servos of
that file, not the XL330 voltage model the policies were trained with.
"""
from __future__ import annotations

import math
import os
import struct
import threading
import time
from pathlib import Path

JOINT_NAMES = (
    'left_hip_yaw', 'left_hip_roll', 'left_hip_pitch', 'left_knee', 'left_ankle',
    'neck_pitch', 'head_pitch', 'head_yaw', 'head_roll',
    'right_hip_yaw', 'right_hip_roll', 'right_hip_pitch', 'right_knee', 'right_ankle',
)
# policy order -> bus slot. Slot 14 is the mouth, which is not a MuJoCo joint.
POLICY_TO_SLOT = (0, 1, 2, 3, 4, 10, 11, 12, 13, 5, 6, 7, 8, 9)
MODES = ('off', 'simulate', 'debug')
TICK_S = 0.01
PHYSICS_S = 0.005
ACC_PER_G = 16384
GYRO_COUNTS_PER_DEG = 32

STATE = struct.Struct('<4sBBH10I6h45f')
VOLTAGE = struct.Struct('<4sIIH15B')
DIAGNOSTIC = struct.Struct('<4s16I')
COMMAND = struct.Struct('<4sIIIBBH15f')


class SimError(Exception):
    pass


def model_path() -> Path | None:
    env = os.environ.get('NNMIXER_MJCF')
    if env and Path(env).is_file():
        return Path(env)
    here = Path(__file__).resolve()
    # On the Uno Q the app lives at /app, so the repository root is not a parent.
    candidates = []
    if len(here.parents) > 1:
        candidates.append(here.parents[1] / 'sim' / 'microduck' / 'scene.xml')
    if len(here.parents) > 3:
        candidates.append(here.parents[3] / 'third_party' / 'microduck' / 'src'
                          / 'mjlab_microduck' / 'robot' / 'microduck' / 'scene.xml')
    for path in candidates:
        if path.is_file():
            return path
    return None


def _i16(value: float) -> int:
    return max(-32767, min(32767, int(round(value))))


class Duck:
    """One MicroDuck scene. q_target None holds the pose the robot is already in."""

    def __init__(self, xml: Path):
        import mujoco
        self.mujoco = mujoco
        self.model = mujoco.MjModel.from_xml_path(str(xml))
        self.model.opt.timestep = PHYSICS_S
        self.data = mujoco.MjData(self.model)
        m = self.model
        if m.nu != 14:
            raise SimError(f'la scena ha {m.nu} attuatori, ne servono 14')
        self.joint_ids = [int(m.actuator_trnid[i, 0]) for i in range(m.nu)]
        names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j) for j in self.joint_ids]
        if tuple(names) != JOINT_NAMES:
            raise SimError(f'ordine dei giunti diverso da quello della policy: {names}')
        self.qpos_idx = [int(m.jnt_qposadr[j]) for j in self.joint_ids]
        self.qvel_idx = [int(m.jnt_dofadr[j]) for j in self.joint_ids]
        self.trunk_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'trunk_base')
        if self.trunk_id < 0:
            raise SimError('la scena non ha il corpo trunk_base')
        self.gyro_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, 'imu_ang_vel')
        self.gyro_adr = int(m.sensor_adr[self.gyro_id]) if self.gyro_id >= 0 else None
        self.stand_key = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, 'STAND')
        self.t = 0.0
        self.fallen_since = None
        self.seq = 0
        self.mouth_deg = 0.0
        self.targets_deg = [0.0] * 15
        self.reset()

    def reset(self):
        m, d = self.model, self.data
        if self.stand_key >= 0:
            self.mujoco.mj_resetDataKeyframe(m, d, self.stand_key)
        else:
            self.mujoco.mj_resetData(m, d)
        self.mujoco.mj_forward(m, d)
        self.t = 0.0
        self.fallen_since = None
        self.mouth_deg = 0.0
        self.targets_deg = self._angles_deg()

    def slot_deg_to_q(self, targets_deg) -> list[float]:
        q = [0.0] * 14
        for joint, slot in enumerate(POLICY_TO_SLOT):
            q[joint] = math.radians(float(targets_deg[slot]))
        return q

    def step(self, q_target, mouth_deg: float):
        d = self.data
        if q_target is None:
            # These position actuators saturate below the torque that holds the body,
            # so integrating while disarmed drops the duck before a policy starts.
            # The pose stays where reset or the last command left it.
            self.mujoco.mj_forward(self.model, d)
            self.t += TICK_S
        else:
            d.ctrl[:] = q_target
            steps = max(1, round(TICK_S / self.model.opt.timestep))
            for _ in range(steps):
                self.mujoco.mj_step(self.model, d)
                self.t += self.model.opt.timestep
        self.mouth_deg += (mouth_deg - self.mouth_deg) * 0.3
        tilt = self.tilt_deg()
        if tilt > 70.0:
            if self.fallen_since is None:
                self.fallen_since = self.t
            elif self.t - self.fallen_since > 3.0:
                self.reset()
        else:
            self.fallen_since = None

    def tilt_deg(self) -> float:
        w, x, y, z = self.data.xquat[self.trunk_id]
        return math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (x * x + y * y)))))

    def gravity_flu(self):
        """Body +z expressed in the world. Negated, it is gravity in the trunk frame: upright is (0, 0, -1)."""
        w, x, y, z = self.data.xquat[self.trunk_id]
        return (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y))

    def _angles_deg(self) -> list[float]:
        deg = [0.0] * 15
        for joint, slot in enumerate(POLICY_TO_SLOT):
            deg[slot] = math.degrees(float(self.data.qpos[self.qpos_idx[joint]]))
        deg[14] = self.mouth_deg
        return deg

    def _velocity_deg(self) -> list[float]:
        deg = [0.0] * 15
        for joint, slot in enumerate(POLICY_TO_SLOT):
            deg[slot] = math.degrees(float(self.data.qvel[self.qvel_idx[joint]]))
        return deg

    def read(self) -> dict:
        g = self.gravity_flu()
        if self.gyro_adr is None:
            gyro = (0.0, 0.0, 0.0)
        else:
            raw = self.data.sensordata[self.gyro_adr:self.gyro_adr + 3]
            gyro = tuple(math.degrees(float(v)) for v in raw)
        return dict(tilt_deg=round(self.tilt_deg(), 2), sim_time=round(self.t, 3),
                    gravity_g=[round(v, 3) for v in (-g[0], -g[1], -g[2])],
                    gyro_dps=[round(v, 2) for v in gyro],
                    joints_deg=[round(v, 2) for v in self._angles_deg()])

    def packets(self, powered: bool) -> list[bytes]:
        self.seq = (self.seq + 1) & 0xFFFFFFFF
        angles = self._angles_deg()
        velocity = self._velocity_deg()
        g = self.gravity_flu()
        # The sketch reports minus the specific force, which at rest is this gravity vector.
        acc = [_i16(-c * ACC_PER_G) for c in g]
        gyro = self.read()['gyro_dps']
        gyro_counts = [_i16(v * GYRO_COUNTS_PER_DEG) for v in gyro]
        flags = 1 | (2 if powered else 0)
        sample = self.seq * 10000
        packets = [STATE.pack(b'DQS1', 1, flags, 0x7FFF, self.seq, sample, sample, 0,
                              0, 10000, 500, 0, 0, 0, *acc, *gyro_counts, *angles, *velocity, *self.targets_deg)]
        if self.seq % 100 == 0:
            packets.append(VOLTAGE.pack(b'DQV1', self.seq, sample, 0x7FFF, *([74] * 15)))
            packets.append(DIAGNOSTIC.pack(b'DQD1', 3, 1, 0, 0, self.seq, sample, 10000, 200, 0, 0, 0,
                                           0, 0, 0, 0, sample))
        return packets


class SimLink:
    def __init__(self):
        self.mode = 'off'
        self.error = None
        self.available = False
        self.model = None
        self.engine = None
        self.powered = False
        self._probed = False
        self._duck = None
        self._q_target = None
        self._mouth = 0.0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._relay = None
        self._read = None
        self.probe()

    def attach(self, relay):
        self._relay = relay

    def replaces_mcu(self) -> bool:
        return self.mode == 'simulate'

    def probe(self):
        if self._probed:
            return
        self._probed = True
        self.model = model_path()
        try:
            import mujoco
        except Exception as e:
            self.error = f'MuJoCo non è installato in questa app ({e}).'
            return
        self.engine = f'MuJoCo {mujoco.__version__}'
        if self.model is None:
            self.error = 'Modello MuJoCo non trovato.'
            return
        self.available = True
        self.error = None

    def set_mode(self, mode: str, reset: bool = False) -> dict:
        if mode not in MODES:
            raise SimError('modalità sconosciuta: spento, simulazione o debug')
        if mode == 'off' and not reset:
            self._halt()
            return self.status()
        self.probe()
        if not self.available:
            raise SimError(self.error or 'MuJoCo non è disponibile')
        with self._lock:
            if self._duck is None:
                self._duck = Duck(self.model)
            if reset:
                self._duck.reset()
                self._q_target = None
                self.powered = False
            self.mode = mode
            self._read = self._duck.read()
        if mode == 'simulate' and self._relay is not None:
            self._relay.release_real_servos()
        self._ensure_thread()
        return self.status()

    def take_command(self, packet: bytes) -> bool:
        """Record joint targets. True means the sketch must not receive this packet."""
        with self._lock:
            if self.mode == 'off' or self._duck is None:
                return False
            if packet[:4] == b'DQC1' and len(packet) == COMMAND.size:
                mode = COMMAND.unpack(packet)[4]
                targets = COMMAND.unpack(packet)[7:]
                self.powered = mode in (1, 2, 3)
                self._duck.targets_deg = [float(v) for v in targets]
                self._mouth = float(targets[14])
                self._q_target = self._duck.slot_deg_to_q(targets) if self.powered else None
            return self.mode == 'simulate'

    def status(self) -> dict:
        with self._lock:
            read = dict(self._read) if self._read else None
            model = None if self.model is None else self.model.name
        return dict(mode=self.mode, available=self.available, engine=self.engine, model=model,
                    error=self.error, powered=self.powered, **(read or {}))

    def _ensure_thread(self):
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, name='mujoco', daemon=True)
        self._thread.start()

    def _halt(self):
        self._stop.set()
        thread = self._thread
        if thread is not None:
            thread.join(timeout=1.0)
        self._thread = None
        with self._lock:
            self.mode = 'off'
            self.powered = False

    def _loop(self):
        nxt = time.perf_counter()
        while not self._stop.is_set():
            nxt += TICK_S
            packets = None
            with self._lock:
                if self.mode == 'off' or self._duck is None:
                    break
                self._duck.step(self._q_target, self._mouth)
                self._read = self._duck.read()
                if self.mode == 'simulate':
                    packets = self._duck.packets(self.powered)
            if packets is not None and self._relay is not None:
                self._relay.publish_sim(packets)
            delay = nxt - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
            else:
                nxt = time.perf_counter()
