#!/usr/bin/env python3
# Author: Roberto Navoni, member of the ArduPilot Dev Team
# Contact: r.navoni74@gmail.com
# Developed by Roberto Navoni — DelphyAI LAB
# For information: r.navoni74@gmail.com
"""XGO Duck as the policies of the getup-hd1910 branch were trained on it.

Source: github.com/ZXerniXZ/xgoduck_rl, branch hd1910-standup, cloned to third_party/xgoduck_rl
(Apache-2.0). This file rebuilds, without mjlab, what that training stack assembles at run time:

  body        robot_allcollisions_backlash.xml: SolidWorks masses, ±1.5° of gear play per servo
  backpack    the electronics stack on the back, 55 g moved out of the trunk (xgoduck_constants.py)
  contacts    the get-up collision table: every link meets the floor, head meets trunk and soles
  servos      BAM voltage model of the HLS1910 (params/1910_m6.json), firmware gain 5
  encoder     on the output side of the play: a joint angle is servo + play, as the robot reads it
  physics     5 ms step, 30 solver iterations, 50 line-search iterations

Training randomizes supply voltage (7.4 to 8.0 V), sole friction (0.2 to 0.8) and command delay
(0 to 30 ms). This scene takes one value of each.
"""
from __future__ import annotations

import math
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'unoq' / 'app' / 'python'))

from sim_link import PHYSICS_S, TICK_S, Duck, SimError  # noqa: E402

RL_ROOT = ROOT / 'third_party' / 'xgoduck_rl'
ROBOT_DIR = Path('src') / 'mjlab_microduck' / 'robot' / 'xgoduck'

# xgoduck_constants.py, electronics stack. Trunk frame: x forward, y left, z up. Metres, kg.
STACK_SIZE = (0.0270, 0.0533, 0.0691)
STACK_INNER_X = -0.036
STACK_CENTER_Z = -0.030
STACK_MASS = 0.055

# XGODUCK_GETUP_COLLISION. Bit 0 floor, bit 1 head and trunk, bit 2 head and the two soles.
# First matching pattern wins.
CONTYPE = ((r'head_collision', 7), (r'trunk_collision', 3), (r'(left|right)_foot_collision', 5), (r'.*_collision', 1))
CONAFFINITY = ((r'head_collision', 6), (r'trunk_collision', 2), (r'(left|right)_foot_collision', 4), (r'.*_collision', 0))
FOOT = r'(left|right)_foot_collision'

# Policy joint order. Hips and ankles: the standing pose the get-up was trained to reach, which is
# also the walk stance of the runtime (stance.py BASELINE_HOME_DEG). STAND_Z puts the soles 1 mm
# above the floor in that pose (XGODUCK_STANDUP_STAND_Z).
STANCE_DEG = (0.0, -5.0, -26.2, -0.28, 18.0, 20.0, 20.0, 0.0, 0.0, 0.0, 5.0, 26.2, 0.28, -18.0)
STAND_Z = 0.141762

VIN = 7.7
VIN_MIN = 7.0
KP_FW = 5.0
SOLE_FRICTION = 0.5
STIFF_SOLREF_FRICTION = (-5.0e4, -2.0e2)
STIFF_SOLIMP_FRICTION = (0.99, 0.9999, 0.001, 0.5, 2.0)


def _first(table, name, default):
    for pattern, value in table:
        if re.fullmatch(pattern, name):
            return value
    return default


def build_spec(rl_root: Path = RL_ROOT, sole_friction: float = SOLE_FRICTION):
    import mujoco
    xml = rl_root / ROBOT_DIR / 'robot_allcollisions_backlash.xml'
    if not xml.is_file():
        raise SimError(f'modello XGO Duck non trovato: {xml}\n'
                       'git clone -b hd1910-standup https://github.com/ZXerniXZ/xgoduck_rl third_party/xgoduck_rl')
    spec = mujoco.MjSpec.from_file(str(xml))

    for geom in spec.geoms:
        name = geom.name or ''
        if not name.endswith('_collision'):
            # mjlab looks geoms up by name, so an unnamed collision mesh would leak contacts
            geom.contype = 0
            geom.conaffinity = 0
            continue
        geom.contype = _first(CONTYPE, name, 1)
        geom.conaffinity = _first(CONAFFINITY, name, 0)
        if re.fullmatch(FOOT, name):
            geom.condim = 3
            geom.priority = 1
            friction = list(geom.friction)
            friction[0] = sole_friction
            geom.friction = friction
        else:
            geom.condim = 1

    trunk = spec.body('trunk_base')
    mass = float(trunk.mass)
    if mass <= STACK_MASS:
        raise SimError(f'massa del tronco {mass}: serve un <inertial> esplicito')
    scale = (mass - STACK_MASS) / mass
    trunk.mass = mass - STACK_MASS
    trunk.inertia = [v * scale for v in trunk.inertia]
    if any(abs(v) > 0 for v in trunk.fullinertia):
        trunk.fullinertia = [v * scale for v in trunk.fullinertia]
    length, width, height = STACK_SIZE
    stack = trunk.add_body(name='electronics_stack', pos=[STACK_INNER_X - length / 2, 0.0, STACK_CENTER_Z])
    stack.mass = STACK_MASS
    stack.ipos = [0.0, 0.0, 0.0]
    stack.inertia = [STACK_MASS / 12 * (width ** 2 + height ** 2),
                     STACK_MASS / 12 * (length ** 2 + height ** 2),
                     STACK_MASS / 12 * (length ** 2 + width ** 2)]
    stack.explicitinertial = True
    hull = spec.add_mesh(name='backpack_hull', file='backpack_hull.stl')
    hull.maxhullvert = 64
    stack.add_geom(name='stack_collision', type=mujoco.mjtGeom.mjGEOM_MESH, meshname='backpack_hull',
                   group=3, density=0, contype=1, conaffinity=0, condim=1, rgba=[1.0, 0.55, 0.1, 0.9])
    visual = spec.add_mesh(name='backpack_visual', file='backpack_visual.stl')
    visual.inertia = mujoco.mjtMeshInertia.mjMESH_INERTIA_SHELL
    stack.add_geom(type=mujoco.mjtGeom.mjGEOM_MESH, meshname='backpack_visual', group=2,
                   contype=0, conaffinity=0, density=0, rgba=[0.25, 0.25, 0.28, 1.0])

    spec.add_texture(name='skybox', type=mujoco.mjtTexture.mjTEXTURE_SKYBOX, builtin=mujoco.mjtBuiltin.mjBUILTIN_GRADIENT,
                     rgb1=[0.3, 0.5, 0.7], rgb2=[0, 0, 0], width=512, height=3072)
    spec.add_texture(name='groundplane', type=mujoco.mjtTexture.mjTEXTURE_2D, builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER,
                     mark=mujoco.mjtMark.mjMARK_EDGE, rgb1=[0.2, 0.3, 0.4], rgb2=[0.1, 0.2, 0.3],
                     markrgb=[0.8, 0.8, 0.8], width=300, height=300)
    ground = spec.add_material(name='groundplane', texuniform=True, texrepeat=[5, 5], reflectance=0.2)
    ground.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = 'groundplane'
    spec.worldbody.add_light(pos=[0, 0, 3.5], dir=[0, 0, -1], type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL)
    spec.worldbody.add_geom(name='floor', type=mujoco.mjtGeom.mjGEOM_PLANE, size=[0, 0, 0.05], material='groundplane')
    return spec


class XgoDuck(Duck):
    """The trained body and servos behind the interface of sim_link.Duck."""

    def __init__(self, rl_root: Path = RL_ROOT, vin: float = VIN, kp_fw: float = KP_FW,
                 sole_friction: float = SOLE_FRICTION):
        import mujoco
        import numpy as np
        from bam.model import load_model
        self.mujoco = mujoco
        self.np = np
        spec = build_spec(rl_root, sole_friction)
        self.bam = load_model(str(rl_root / ROBOT_DIR / 'params' / '1910_m6.json'))
        servo = self.bam.actuator
        servo.kp = kp_fw
        servo.vin = vin
        # ceiling at the top of the trained voltage range, as bam.mjlab does
        force = 8.0 * self.bam.kt.value / self.bam.R.value
        names = []
        for act in spec.actuators:
            joint = spec.joint(act.target)
            act.set_to_motor()
            act.forcelimited = True
            act.forcerange = (-force, force)
            act.ctrllimited = False
            act.gear = [1.0, 0, 0, 0, 0, 0]
            joint.armature = float(servo.get_extra_inertia())
            joint.damping = np.zeros((3, 1))
            joint.frictionloss = 0.0
            joint.solref_friction = STIFF_SOLREF_FRICTION
            joint.solimp_friction = STIFF_SOLIMP_FRICTION
            names.append(act.name)
        self.model = m = spec.compile()
        m.opt.timestep = PHYSICS_S
        m.opt.iterations = 30
        m.opt.ls_iterations = 50
        self.data = mujoco.MjData(m)
        self.vin, self.vin_min = vin, VIN_MIN

        self.joint_ids = [int(m.actuator_trnid[i, 0]) for i in range(m.nu)]
        self.qpos_idx = [int(m.jnt_qposadr[j]) for j in self.joint_ids]
        self.qvel_idx = [int(m.jnt_dofadr[j]) for j in self.joint_ids]
        play = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f'passive_{name}_backlash') for name in names]
        if min(play) < 0:
            raise SimError('il modello non ha i giunti di gioco passive_*_backlash')
        self.play_qpos = [int(m.jnt_qposadr[j]) for j in play]
        self.play_qvel = [int(m.jnt_dofadr[j]) for j in play]
        self.trunk_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'trunk_base')
        self.gyro_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, 'imu_ang_vel')
        self.gyro_adr = int(m.sensor_adr[self.gyro_id]) if self.gyro_id >= 0 else None
        self.stand_key = -1
        self.t = 0.0
        self.fallen_since = None
        self.seq = 0
        self.mouth_deg = 0.0
        self.targets_deg = [0.0] * 15
        self.reset()

    # joint angles as the servo encoder reads them, through the gear play
    def encoder(self):
        d = self.data
        return self.np.array([d.qpos[a] + d.qpos[b] for a, b in zip(self.qpos_idx, self.play_qpos)])

    def encoder_velocity(self):
        d = self.data
        return self.np.array([d.qvel[a] + d.qvel[b] for a, b in zip(self.qvel_idx, self.play_qvel)])

    def reset(self, pose_deg=STANCE_DEG, z: float = STAND_Z):
        m, d = self.model, self.data
        self.mujoco.mj_resetData(m, d)
        d.qpos[2] = z
        for joint, deg in enumerate(pose_deg):
            d.qpos[self.qpos_idx[joint]] = math.radians(deg)
        self.mujoco.mj_forward(m, d)
        self.bam.actuator.q_target_smooth = self.encoder()
        self.t = 0.0
        self.fallen_since = None
        self.mouth_deg = 0.0
        self.targets_deg = self._angles_deg()

    def _servo_step(self, q_target):
        """bam.mujoco.MujocoController.update, with the position loop closed through the play."""
        m, d, np, bam = self.model, self.data, self.np, self.bam
        act = bam.actuator
        dof = self.qvel_idx
        dq = d.qvel[dof]
        control = act.compute_control(np.asarray(q_target, dtype=float), self.encoder(), dq, m.opt.timestep)
        d.ctrl[:] = act.compute_torque(control, True, self.encoder(), dq)
        external = -d.qfrc_bias[dof] + d.qfrc_constraint[dof]
        if d.nefc:
            rows = (d.efc_type == self.mujoco.mjtConstraint.mjCNSTR_FRICTION_DOF.value)
            for i, joint in enumerate(self.joint_ids):
                external[i] -= d.efc_force[rows & (d.efc_id == joint)].sum()
        frictionloss, damping = bam.compute_frictions(d.qfrc_actuator[dof], external, dq)
        m.dof_frictionloss[dof] = frictionloss
        m.dof_damping[dof] = damping

    def step(self, q_target, mouth_deg: float):
        m, d = self.model, self.data
        if q_target is None:
            # Torque off. The pose stays where reset or the last command left it, as in sim_link.Duck:
            # the page arms from a standing robot, not from a heap.
            self.mujoco.mj_forward(m, d)
            self.bam.actuator.q_target_smooth = self.encoder()
            self.t += TICK_S
        else:
            for _ in range(max(1, round(TICK_S / m.opt.timestep))):
                self._servo_step(q_target)
                self.mujoco.mj_step(m, d)
                self.t += m.opt.timestep
        self.mouth_deg += (mouth_deg - self.mouth_deg) * 0.3
        if self.tilt_deg() > 70.0:
            if self.fallen_since is None:
                self.fallen_since = self.t
        else:
            self.fallen_since = None

    def _angles_deg(self):
        from sim_link import POLICY_TO_SLOT
        deg = [0.0] * 15
        for value, slot in zip(self.encoder(), POLICY_TO_SLOT):
            deg[slot] = math.degrees(float(value))
        deg[14] = self.mouth_deg
        return deg

    def _velocity_deg(self):
        from sim_link import POLICY_TO_SLOT
        deg = [0.0] * 15
        for value, slot in zip(self.encoder_velocity(), POLICY_TO_SLOT):
            deg[slot] = math.degrees(float(value))
        return deg
