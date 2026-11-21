#!/usr/bin/env python3
# Author: Roberto Navoni, member of the ArduPilot Dev Team
# Contact: r.navoni74@gmail.com
# Developed by Roberto Navoni — DelphyAI LAB
# For information: r.navoni74@gmail.com
"""Build the MuJoCo model of the Q8bot (Yufeng Wu, UCLA RoMeLa) with Feetech HD-1910M servos.

    python tools/robots/q8bot_mjcf.py                       # writes robots/q8bot/robot/q8bot.xml and checks it
    python tools/robots/q8bot_mjcf.py --render /tmp/q8bot.png
    python tools/robots/q8bot_mjcf.py --demo --video docs/media/q8bot_model_demo.mp4
    mjpython tools/robots/q8bot_mjcf.py --view              # interactive viewer, the demo in a loop

Upstream publishes no MJCF or URDF. The geometry is the CAD of the hardware release (hw-v2.5.zip of
https://github.com/EricYufengWu/q8bot/releases, Q8bot_rev2_5_FDM.STEP), exported once by
tools/robots/q8bot_step_export.py to robots/q8bot/robot/meshes/: one STL per part in its own frame plus
assembly.json with the pose of every instance. This script places those meshes on the bodies and adds the
dynamics:

  kinematics   python-tools/q8bot/kinematics_solver.py: d = 19.5 mm between the two motors of a leg, upper links
               25 mm, lower links 40 mm. The CAD agrees: shafts at x = +/-30.25 and +/-49.75 mm, legs in the
               planes at +/-26.4 mm.
  closed chain each leg is a planar five-bar: motor A (q1, front) and motor B (q2, rear) drive the upper links,
               the two lower links meet at the foot and are closed with one `equality connect`.
  joints       theta = q - 90 deg of the firmware angle q (from +x toward the ground): the contract sends
               targets as PWM clamped at +/-2.1 rad around zero and q2 = 140.58 deg at the stand would not fit.
  servos       Feetech HD-1910M (HD-1910-C001), the MicroDuck / XGO Duck servo, instead of the XL330-M077-T:
               34 x 20 x 23 mm, 21 g, 1.47 N.m and 113 rpm at 7.4 V. The dynamics are the BAM voltage model
               identified on that servo (hd1910_m6.json, from xgoduck_rl, Apache 2.0) with firmware gain 5.
               The <position> actuators in the XML are the plain-MuJoCo companion of that model.
               The servo is drawn with the CAD body of the XL330: same 34 x 20 mm face, 3 mm deeper.
  IMU          the robot has none: the trunk carries a virtual one (site `imu`), as the autopilot would.
  mass         221 g published (18 g motors) + 8 x 3 g for the HD-1910M = 245 g. PLA is the remainder of the
               221 g spread over the CAD volumes (the infill is not published).

Collision: the trunk box, the link capsules and a sphere on each foot pivot touch the floor; nothing collides
with the robot itself.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import REPO_ROOT, robot_dir  # noqa: E402

ROBOT_DIR = robot_dir("q8bot") / "robot"
MESH_DIR = ROBOT_DIR / "meshes"
BAM_PARAMS = ROBOT_DIR / "hd1910_m6.json"

# python-tools/q8bot/operate.py, mm in the IK frame (x forward, y toward the ground)
D, L1, L2 = 19.5, 25.0, 40.0
# gait_manager.GAITS: x0, y0, xrange, yrange, yrange2, s1_count, s2_count
GAITS = {
    "TROT": (9.75, 43.36, 40.0, 20.0, 0.0, 15, 30),
    "TROT_LOW": (9.75, 25.0, 20.0, 10.0, 0.0, 15, 30),
}
STAND = GAITS["TROT"][:2]
FOOT_R = 5.0                               # mm, rounded end of the lower link around the foot pivot (CAD)
THETA0 = math.pi / 2                       # contract joint theta = q - 90 deg
JOINT_RANGE = (-2.1, 2.1)

# kg
M_SERVO = 0.021                            # HD-1910M, 21 +/- 2 g
M_XL330 = 0.018
M_CELL, M_PCB = 0.021, 0.008
M_UPSTREAM = 0.221                         # Q8bot_Weight.jpg, with the XL330
M_TOTAL = M_UPSTREAM + 8 * (M_SERVO - M_XL330)

# HD-1910M at 7.4 V (two 14500 cells in series; the servo takes 4-8.4 V). BAM: kt 0.624 N.m/A, R 4.91 ohm.
SERVO_VIN = 7.4
SERVO_KP_FW = 5.0
# companion of the BAM model for plain MuJoCo (xgoduck_rl robot_walk.xml)
SERVO_KP, SERVO_TORQUE = 0.84, 1.04
JOINT_DAMPING, JOINT_FRICTION, ARMATURE = 0.109, 0.012, 0.0018

LEGS = (("FL", +1, +1), ("FR", +1, -1), ("BL", -1, +1), ("BR", -1, -1))   # name, front, left
FOOT_ORDER = tuple(n for n, _, _ in LEGS)

# CAD frame (x forward, y up, z right) -> trunk frame (x forward, y left, z up)
C = np.array([[1.0, 0, 0], [0, 0, -1.0], [0, 1.0, 0]])

MATERIAL = (("Q8_Main_Frame", "pla"), ("Q8_Upper_Leg", "pla"), ("Q8_Lower_Leg", "pla"), ("XL330", "servo"),
            ("Q8_PCB", "pcb"), ("Batt_KeepPower", "battery"), ("Batt_Clip", "steel"), ("692ZZ", "steel"),
            ("M2_Tapping", "screw"))


# ------------------------------------------------------------------ kinematics
def ik(x: float, y: float) -> tuple[float, float]:
    """kinematics_solver.ik_solve(): foot (x, y) mm -> firmware angles (q1, q2) rad."""
    c1 = math.hypot(x - D, y)
    c2 = math.hypot(x, y)
    a1 = math.acos((c1 ** 2 + D ** 2 - c2 ** 2) / (2 * c1 * D))
    a2 = math.acos((c2 ** 2 + D ** 2 - c1 ** 2) / (2 * c2 * D))
    b1 = math.acos((c1 ** 2 + L1 ** 2 - L2 ** 2) / (2 * c1 * L1))
    b2 = math.acos((c2 ** 2 + L1 ** 2 - L2 ** 2) / (2 * c2 * L1))
    return math.pi - a1 - b1, a2 + b2


def elbows(q1: float, q2: float):
    return (D + L1 * math.cos(q1), L1 * math.sin(q1)), (L1 * math.cos(q2), L1 * math.sin(q2))


def fk(q1: float, q2: float) -> tuple[float, float]:
    """Foot of the closed chain: the lower intersection of the two 40 mm circles around the elbows."""
    (xa, ya), (xb, yb) = elbows(q1, q2)
    dx, dy = xb - xa, yb - ya
    c = math.hypot(dx, dy)
    if c > 2 * L2 or c < 1e-9:
        raise ValueError(f"no closure at q1={math.degrees(q1):.1f} q2={math.degrees(q2):.1f} deg")
    h = math.sqrt(L2 ** 2 - (c / 2) ** 2)
    mx, my = xa + dx / 2, ya + dy / 2
    return max([(mx + h * dy / c, my - h * dx / c), (mx - h * dy / c, my + h * dx / c)], key=lambda p: p[1])


def stand_q0() -> list[float]:
    """Contract joints (theta = q - 90 deg) of the stand, the same on all four legs."""
    q1, q2 = ik(*STAND)
    return [q1 - THETA0, q2 - THETA0] * 4


def home_z() -> float:
    """Trunk origin (the shaft line) above the floor with the foot spheres on it."""
    return round((fk(*ik(*STAND))[1] + FOOT_R) * 1e-3, 4)


# ------------------------------------------------------------------ CAD assembly
def _rot_y(a: float) -> np.ndarray:
    """Rotation about +y: turns +x toward the ground (-z) by a."""
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _quat(R: np.ndarray) -> np.ndarray:
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1) * 2
        q = [s / 4, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s]
    else:
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = math.sqrt(1 + R[i, i] - R[j, j] - R[k, k]) * 2
        q = [0.0] * 4
        q[0] = (R[k, j] - R[j, k]) / s
        q[1 + i] = s / 4
        q[1 + j] = (R[j, i] + R[i, j]) / s
        q[1 + k] = (R[k, i] + R[i, k]) / s
    return np.array(q)


def _f(v) -> str:
    return " ".join(f"{x:.6g}" if abs(x) >= 1e-9 else "0" for x in v)


def _material(part: str) -> str:
    return next(m for key, m in MATERIAL if part.startswith(key))


class Assembly:
    """assembly.json in the trunk frame (metres): trunk instances and, per leg, the instances of each link with
    the link frame they had in the CAD pose."""

    def __init__(self):
        asm = json.loads((MESH_DIR / "assembly.json").read_text())
        self.parts = asm["parts"]
        self.trunk, legs = [], {}
        for ins in asm["instances"]:
            T = np.array(ins["T"])
            R, p = C @ T[:3, :3], C @ T[:3, 3] * 1e-3
            assert np.linalg.det(R) > 0.99, f"mirrored instance of {ins['part']}"
            item = (ins["part"], R, p)
            if "Leg_SubAssm" in ins["path"]:
                legs.setdefault(ins["path"], []).append(item)
            else:
                self.trunk.append(item)
        # the four leg sub-assemblies share one path: split them in order, 14 instances each
        # (2 upper links, 2 lower links, 3 bearings, 7 screws)
        items = [it for group in legs.values() for it in group]
        assert len(items) == 56, f"{len(items)} leg instances, 56 expected"
        self.legs = {}
        for k in range(4):
            group = items[14 * k:14 * k + 14]
            ups = [it for it in group if it[0].startswith("Q8_Upper_Leg")]
            front = +1 if np.mean([u[2][0] for u in ups]) > 0 else -1
            left = +1 if ups[0][2][1] > 0 else -1
            name = next(n for n, f, l in LEGS if (f, l) == (front, left))
            self.legs[name] = self._leg(group, ups)

    @staticmethod
    def _leg(group, ups):
        xz = lambda p: np.array([p[0], p[2]])
        up_a, up_b = sorted(ups, key=lambda u: -u[2][0])           # motor A is the front one
        s_a, s_b, y = up_a[2], up_b[2], up_a[2][1]
        assert abs((s_a[0] - s_b[0]) * 1e3 - D) < 0.01 and abs(s_a[2]) < 1e-6, "CAD shafts are not 19.5 mm apart"
        bearings = [it[2] for it in group if it[0].startswith("692ZZ")]
        near = lambda s: next(b for b in bearings if abs(np.linalg.norm(xz(b) - xz(s)) * 1e3 - L1) < 0.05)
        e_a, e_b = near(s_a), near(s_b)
        foot = next(b for b in bearings if b is not e_a and b is not e_b)
        assert abs(np.linalg.norm(xz(foot) - xz(e_a)) * 1e3 - L2) < 0.05
        ang = lambda a, b: math.atan2(-(b[2] - a[2]), b[0] - a[0])
        at = lambda p: np.array([p[0], y, p[2]])
        frames = {"upA": (_rot_y(ang(s_a, e_a)), at(s_a)), "upB": (_rot_y(ang(s_b, e_b)), at(s_b)),
                  "lowA": (_rot_y(ang(e_a, foot)), at(e_a)), "lowB": (_rot_y(ang(e_b, foot)), at(e_b))}
        anchors = (("upA", s_a), ("upB", s_b), ("lowA", e_a), ("lowB", e_b), ("lowA", foot))
        out = {b: [] for b in frames}
        out["upper_part"] = {"upA": up_a[0], "upB": up_b[0]}
        for part, R, p in group:
            if part.startswith("Q8_Lower_Leg"):
                # the part runs 40 mm along its -y from its origin: it belongs to the elbow at either end
                ends = (p, p + R @ np.array([0, -L2 * 1e-3, 0]))
                body = next(b for b, e in (("lowA", e_a), ("lowB", e_b))
                            if min(np.linalg.norm(xz(q) - xz(e)) for q in ends) < 1e-4)
            else:
                body = min(anchors, key=lambda a: np.linalg.norm(xz(p) - xz(a[1])))[0]
            Rb, pb = frames[body]
            out[body].append((part, Rb.T @ R, Rb.T @ (p - pb)))
        out["y"] = y
        out["shaft_x"] = {"A": s_a[0], "B": s_b[0]}
        return out


def _mesh_geom(part: str, R: np.ndarray, p: np.ndarray) -> str:
    return (f'<geom class="visual" mesh="{Path(ASM.parts[part]["file"]).stem}" material="{_material(part)}" '
            f'pos="{_f(p)}" quat="{_f(_quat(R))}"/>')


def _box_of(part: str, R: np.ndarray, p: np.ndarray, mass: float, shrink=(1.0, 1.0, 1.0)) -> str:
    """Mass box on the bounding box of a part instance."""
    bb = np.array(ASM.parts[part]["bbox"]) * 1e-3
    centre, half = (bb[:3] + bb[3:]) / 2, (bb[3:] - bb[:3]) / 2 * np.array(shrink)
    return (f'<geom class="mass" type="box" size="{_f(half)}" pos="{_f(p + R @ centre)}" quat="{_f(_quat(R))}" '
            f'mass="{mass:.6g}"/>')


def _pla_density() -> float:
    """kg per mm3 of printed part: what is left of the upstream 221 g after motors, cells and PCB."""
    v = ASM.parts
    vol = (2 * v["Q8_Main_Frame_LATEST_Reduced_PLA"]["volume_mm3"]
           + 4 * (v["Q8_Upper_Leg_FDM_Short"]["volume_mm3"] + v["Q8_Upper_Leg_FDM_Tall"]["volume_mm3"])
           + 8 * v["Q8_Lower_Leg_FDM_Left_40mm"]["volume_mm3"])
    return (M_UPSTREAM - 8 * M_XL330 - 2 * M_CELL - M_PCB) / vol


def pla_mass(part: str) -> float:
    return ASM.parts[part]["volume_mm3"] * _pla_density()


# ------------------------------------------------------------------ MJCF
def leg_xml(name: str, front: int, left: int) -> str:
    leg = ASM.legs[name]
    q1, q2 = ik(*STAND)
    (xa, ya), (xb, yb) = elbows(q1, q2)
    fx, fy = fk(q1, q2)
    ta, tb = math.atan2(fy - ya, fx - xa), math.atan2(fy - yb, fx - xb)
    ua, ll, y = L1 * 1e-3, L2 * 1e-3, leg["y"]
    foot_y = left * 0.00425                 # between the two lower links, which sit side by side

    def arm(tag: str, q: float, t: float, joint: str, foot: bool) -> str:
        up, low = f"up{tag}", f"low{tag}"
        ind = "\n          "
        up_vis = ind.join(_mesh_geom(*g) for g in leg[up])
        low_vis = (ind + "  ").join(_mesh_geom(*g) for g in leg[low])
        foot_xml = f"""
            <geom name="{name}_foot" class="foot" pos="{ll:.6g} {foot_y:.6g} 0"/>
            <site name="{name}_foot" pos="{ll:.6g} {foot_y:.6g} 0" size="0.002"/>
            <site name="{name}_touch" type="sphere" pos="{ll:.6g} {foot_y:.6g} 0" size="{FOOT_R * 1e-3 + 0.0015:.6g}" group="4"/>""" if foot else ""
        return f"""
        <body name="{name}_{up}" pos="{_f((leg['shaft_x'][tag], y, 0))}" quat="{_f(_quat(_rot_y(q)))}">
          <joint name="{joint}" class="motor" ref="{q - THETA0:.9g}"/>
          <geom class="link" type="capsule" size="0.003" fromto="0 0 0 {ua:.6g} 0 0" mass="{pla_mass(leg['upper_part'][up]):.6g}"/>
          {up_vis}
          <body name="{name}_{low}" pos="{ua:.6g} 0 0" quat="{_f(_quat(_rot_y(t - q)))}">
            <joint name="{name}_elbow{tag}" class="passive"/>
            <geom class="link" type="capsule" size="0.0025" fromto="0 0 0 {ll:.6g} 0 0" mass="{pla_mass('Q8_Lower_Leg_FDM_Left_40mm'):.6g}"/>
            {low_vis}{foot_xml}
          </body>
        </body>"""

    return arm("A", q1, ta, f"{name}_q1", True) + arm("B", q2, tb, f"{name}_q2", False)


def build_xml() -> str:
    z0 = home_z()
    q1, q2 = ik(*STAND)
    q0 = stand_q0()
    ind = "\n      "
    trunk_vis = ind.join(_mesh_geom(*g) for g in ASM.trunk)
    masses = []
    for part, R, p in ASM.trunk:
        if part.startswith("XL330"):
            masses.append(_box_of(part, R, p, M_SERVO))
        elif part.startswith("Batt_KeepPower"):
            masses.append(_box_of(part, R, p, M_CELL, shrink=(1.0, 0.8, 0.8)))
        elif part.startswith("Q8_PCB"):
            masses.append(_box_of(part, R, p, M_PCB, shrink=(1.0, 1.0, 0.2)))
        elif part.startswith("Q8_Main_Frame"):
            masses.append(_box_of(part, R, p, pla_mass(part)))
    masses = ind.join(masses)
    legs = "".join(leg_xml(*leg) for leg in LEGS)
    joints = [f"{n}_{j}" for n, _, _ in LEGS for j in ("q1", "q2")]
    actuators = "\n".join(f'    <position name="{j}" joint="{j}"/>' for j in joints)
    touch = "\n".join(f'    <touch name="{n}_touch" site="{n}_touch"/>' for n, _, _ in LEGS)
    loops = "\n".join(
        f'    <connect name="{n}_loop" body1="{n}_lowA" body2="{n}_lowB" anchor="{L2 * 1e-3:.6g} 0 0"/>'
        for n, _, _ in LEGS)
    meshes = "\n".join(f'    <mesh name="{Path(p["file"]).stem}" file="{p["file"]}" scale="0.001 0.001 0.001"/>'
                       for p in ASM.parts.values())
    key_q = " ".join(f"{q1 - THETA0:.9g} 0 {q2 - THETA0:.9g} 0" for _ in LEGS)
    return f"""<?xml version="1.0" encoding="utf-8"?>
<!--
  Q8bot (Yufeng Wu, UCLA RoMeLa) with Feetech HD-1910M servos, MuJoCo model for training and simulation.
  Generated by tools/robots/q8bot_mjcf.py: edit that script, not this file.
  Geometry: the CAD of the hardware release hw-v2.5 (meshes/, exported by tools/robots/q8bot_step_export.py).
  Frame: trunk origin on the motor shaft line, x forward, y left, z up. Four planar five-bar legs: motor A (q1)
  {D / 2:.2f} mm in front of the leg centre, motor B (q2) {D / 2:.2f} mm behind; upper links {L1:.0f} mm, lower links
  {L2:.0f} mm, closed at the foot with an equality connect (one per leg).
    *_q1, *_q2   axis +y, theta = q - 90 deg of kinematics_solver.py: zero = link pointing down, positive = back
    *_elbow*     passive, not in the contract
  Stand q0 = IK of the foot at ({STAND[0]}, {STAND[1]}) mm: q {math.degrees(q1):.2f} and {math.degrees(q2):.2f} deg from +x,
  theta {math.degrees(q1 - THETA0):.2f} and {math.degrees(q2 - THETA0):+.2f} deg, all legs.
  Servos: HD-1910M (21 g, 7.4 V). Training and the demo use the BAM model hd1910_m6.json (firmware gain {SERVO_KP_FW:g});
  the <position> actuators below are its plain-MuJoCo companion. Drawn with the CAD body of the XL330.
  Mass {M_TOTAL * 1e3:.0f} g = {M_UPSTREAM * 1e3:.0f} g published + 8 x 3 g of servo.
  IMU: virtual, on the trunk (the robot has none); the autopilot provides gravity and angular rate.
  Groups: 2 visual (CAD meshes), 3 collision and mass, 4 touch sites.
-->
<mujoco model="q8bot">
  <compiler angle="radian" autolimits="true" meshdir="meshes"/>
  <option timestep="0.005" integrator="implicitfast" cone="elliptic" impratio="100"/>
  <statistic center="0 0 0.03" extent="0.25"/>

  <visual>
    <global offwidth="1920" offheight="1080" azimuth="130" elevation="-18"/>
    <quality shadowsize="8192" offsamples="8"/>
    <map znear="0.02" shadowclip="0.5"/>
    <headlight ambient="0.3 0.3 0.3" diffuse="0.45 0.45 0.45" specular="0.2 0.2 0.2"/>
    <rgba haze="0.93 0.94 0.95 1"/>
  </visual>

  <default>
    <geom condim="3" friction="0.9 0.02 0.001"/>
    <default class="motor">
      <joint type="hinge" axis="0 1 0" limited="true" range="{JOINT_RANGE[0]} {JOINT_RANGE[1]}" armature="{ARMATURE}" damping="{JOINT_DAMPING}" frictionloss="{JOINT_FRICTION}"/>
    </default>
    <default class="passive">
      <joint type="hinge" axis="0 1 0" armature="1e-6" damping="1e-5"/>
    </default>
    <position kp="{SERVO_KP}" forcerange="{-SERVO_TORQUE} {SERVO_TORQUE}" ctrlrange="{JOINT_RANGE[0]} {JOINT_RANGE[1]}"/>
    <site group="5"/>
    <equality solref="0.01 1" solimp="0.99 0.999 0.0005"/>
    <!-- contype 2 / conaffinity 1: collide with the floor (contype 1) and nothing else -->
    <default class="link">
      <geom group="3" contype="2" conaffinity="1" rgba="0.8 0.2 0.2 0.4"/>
    </default>
    <default class="mass">
      <geom group="3" contype="0" conaffinity="0" rgba="0.2 0.2 0.8 0.4"/>
    </default>
    <default class="foot">
      <geom type="sphere" size="{FOOT_R * 1e-3:.6g}" contype="2" conaffinity="1" friction="1 0.02 0.001" group="3" mass="1e-5" rgba="0.8 0.2 0.2 0.4"/>
    </default>
    <default class="visual">
      <geom type="mesh" group="2" contype="0" conaffinity="0" density="0"/>
    </default>
  </default>

  <asset>
    <texture name="sky" type="skybox" builtin="gradient" rgb1="0.96 0.96 0.95" rgb2="0.8 0.82 0.84" width="512" height="512"/>
    <texture name="mat" type="2d" builtin="checker" rgb1="0.8 0.81 0.8" rgb2="0.72 0.73 0.72" width="512" height="512" mark="edge" markrgb="0.88 0.88 0.88"/>
    <material name="floor_mat" texture="mat" texrepeat="80 80" reflectance="0.12" specular="0.3" shininess="0.3"/>
    <material name="pla" rgba="0.075 0.075 0.08 1" specular="0.25" shininess="0.25"/>
    <material name="servo" rgba="0.13 0.13 0.14 1" specular="0.5" shininess="0.6"/>
    <material name="pcb" rgba="0.04 0.07 0.05 1" specular="0.4" shininess="0.5"/>
    <material name="battery" rgba="0.05 0.05 0.06 1" specular="0.8" shininess="0.9" reflectance="0.1"/>
    <material name="steel" rgba="0.78 0.78 0.8 1" specular="0.9" shininess="0.9" reflectance="0.25"/>
    <material name="screw" rgba="0.2 0.2 0.21 1" specular="0.7" shininess="0.8"/>
{meshes}
  </asset>

  <worldbody>
    <light name="key" pos="0.4 -0.5 0.9" dir="-0.4 0.5 -0.9" directional="true" castshadow="true" diffuse="0.75 0.75 0.72" specular="0.4 0.4 0.4"/>
    <light name="fill" pos="-0.5 0.4 0.6" dir="0.5 -0.4 -0.6" directional="true" castshadow="false" diffuse="0.3 0.31 0.33" specular="0.1 0.1 0.1"/>
    <geom name="floor" type="plane" size="10 10 0.1" material="floor_mat"/>

    <body name="trunk" pos="0 0 {z0}">
      <freejoint name="floating_base"/>
      <camera name="track" mode="trackcom" pos="0.17 -0.2 0.07" xyaxes="0.762 0.648 0 -0.17 0.2 0.965"/>
      <camera name="side" mode="trackcom" pos="0 -0.28 0.0" xyaxes="1 0 0 0 0 1"/>
      <camera name="front" mode="trackcom" pos="0.28 0 0.03" xyaxes="0 1 0 -0.1 0 1"/>
      <site name="imu" pos="0 0 0.01" size="0.003"/>

      <geom name="trunk" class="link" type="box" size="0.06 0.0264 0.018" pos="0 0 0.008" mass="1e-5"/>
      {masses}
      {trunk_vis}
{legs}
    </body>
  </worldbody>

  <equality>
{loops}
  </equality>

  <actuator>
{actuators}
  </actuator>

  <sensor>
    <gyro name="imu_gyro" site="imu"/>
    <accelerometer name="imu_acc" site="imu"/>
    <framequat name="imu_quat" objtype="site" objname="imu"/>
{touch}
  </sensor>

  <keyframe>
    <key name="home" qpos="0 0 {z0} 1 0 0 0 {key_q}" ctrl="{_f(q0)}"/>
  </keyframe>
</mujoco>
"""


# ------------------------------------------------------------------ upstream gaits
SAMPLE_HZ = 200.0                          # operate.py plays one IK sample per 200 Hz tick


def leg_cycle(gait, stride: float) -> list[tuple[float, float]]:
    """gait_generator._generate_base_trajectories(): one foot path (x, y) mm, swing forward then stance back."""
    x0, y0, xr, yr, yr2, s1, s2 = gait
    x = x0 - xr * stride / 2
    out = []
    for i in range(s1 + s2):
        if i < s1:
            x += xr * stride / s1
            y = y0 - math.sin((i + 1) * math.pi / s1) * yr
        else:
            x -= xr * stride / s2
            y = y0 + math.sin((i - s1 + 1) * math.pi / s2) * yr2
        out.append((x, y))
    return out


def gait_targets(move: str, gait_name: str = "TROT") -> np.ndarray:
    """generate_trot_trajectories(): contract joints, FL FR BL BR, one row per upstream sample.
    move: f forward, b backward, l turn left, r turn right (the four straight entries of the upstream table)."""
    gait = GAITS[gait_name]
    theta = lambda cyc: [(a - THETA0, b - THETA0) for a, b in (ik(x, y) for x, y in cyc)]
    fwd, bwd = theta(leg_cycle(gait, 1.0)), theta(leg_cycle(gait, -1.0))
    shift = int((gait[5] + gait[6]) / 2)
    ph = lambda t: t[shift:] + t[:shift]
    pf, nb = ph(fwd), ph(bwd)
    table = {"f": (fwd, pf, pf, fwd), "b": (bwd, nb, nb, bwd), "l": (bwd, pf, nb, fwd), "r": (fwd, nb, pf, bwd)}
    legs = table[move]
    out = np.array([[v for leg in legs for v in leg[i]] for i in range(len(fwd))])
    assert np.abs(out).max() < JOINT_RANGE[1], f"{gait_name} {move}: joint beyond the wire range"
    return out


def stand_of(gait_name: str) -> np.ndarray:
    q1, q2 = ik(*GAITS[gait_name][:2])
    return np.array([q1 - THETA0, q2 - THETA0] * 4)


# The demo: forward, backward, turn at the default height, then the same on the low stance (TROT_LOW).
DEMO = (
    ("in piedi", None, "TROT", 1.0),
    ("avanti", "f", "TROT", 4.0),
    ("indietro", "b", "TROT", 4.0),
    ("rotazione", "l", "TROT", 4.0),
    ("si abbassa", None, "TROT_LOW", 1.5),
    ("avanti, basso", "f", "TROT_LOW", 4.0),
    ("indietro, basso", "b", "TROT_LOW", 4.0),
    ("rotazione, basso", "l", "TROT_LOW", 4.0),
    ("si rialza", None, "TROT", 1.5),
)
BLEND_S = 0.4                              # cross-fade of the joint targets between two segments


# ------------------------------------------------------------------ simulation helpers
def _joint_names():
    return [f"{n}_{j}" for n in FOOT_ORDER for j in ("q1", "q2")]


class Sim:
    """The model with its servos: BAM HD-1910M when `bam` is installed, else the <position> companion."""

    def __init__(self, path: Path, use_bam: bool = True):
        import mujoco
        self.mj = mujoco
        self.bam = None
        if use_bam:
            try:
                sys.path.insert(0, str(REPO_ROOT / "plant"))
                from mujoco_json_plant import load_with_bam
                self.model, self.data, self.bam = load_with_bam(path, SERVO_VIN, SERVO_KP_FW, params=BAM_PARAMS)
            except ImportError:
                print("bam not installed: plain <position> actuators")
        if self.bam is None:
            self.model = mujoco.MjModel.from_xml_path(str(path))
            self.data = mujoco.MjData(self.model)
        m = self.model
        m.opt.timestep = 0.005
        self.act = [m.actuator(n).id for n in _joint_names()]
        self.qadr = [int(m.jnt_qposadr[m.joint(n).id]) for n in _joint_names()]
        self.reset()

    def reset(self):
        self.mj.mj_resetDataKeyframe(self.model, self.data, 0)
        self.mj.mj_forward(self.model, self.data)
        if self.bam is not None:
            self.bam.last_ts = self.data.time
            # Feetech actuator of BAM: the slew-limited target starts on the encoder
            self.bam.model.actuator.q_target_smooth = self.data.qpos[self.bam.qpos_indexes].copy()
        self.target = np.array(stand_q0())

    def step(self, target=None):
        if target is not None:
            self.target = np.asarray(target, dtype=float)
        if self.bam is not None:
            for n, v in zip(_joint_names(), self.target):
                self.bam.set_q_target(n, v)
            self.bam.update()
        else:
            self.data.ctrl[self.act] = self.target
        self.mj.mj_step(self.model, self.data)

    def loop_error(self) -> float:
        m, d = self.model, self.data
        worst = 0.0
        for n in FOOT_ORDER:
            pts = [d.xpos[m.body(f"{n}_low{t}").id] + d.xmat[m.body(f"{n}_low{t}").id].reshape(3, 3) @ [L2 * 1e-3, 0, 0]
                   for t in ("A", "B")]
            worst = max(worst, float(np.linalg.norm(pts[0] - pts[1])))
        return worst

    def tilt_deg(self) -> float:
        qx, qy = self.data.qpos[4], self.data.qpos[5]
        return math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * float(qx * qx + qy * qy)))))

    def yaw(self) -> float:
        w, qx, qy, qz = self.data.qpos[3:7]
        return math.atan2(2 * (w * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))


def check(path: Path) -> None:
    """Catalog values, closure at the stand, mass, foot positions against the IK, a 3 s stand on the servos."""
    import mujoco

    from catalog import ROBOTS

    cat = ROBOTS["q8bot"]
    assert np.allclose(cat["q0"], stand_q0(), atol=1e-5), f"catalog q0 {cat['q0']} != IK stand {stand_q0()}"
    assert cat["sim"]["home_z"] == home_z(), f"catalog sim.home_z != {home_z()}"
    sim = Sim(path)
    m, d = sim.model, sim.data
    total = float(m.body_subtreemass[m.body("trunk").id])
    assert abs(total - M_TOTAL) < 2e-4, f"mass {total} kg != {M_TOTAL}"
    trunk = d.xpos[m.body("trunk").id]
    fx, fy = fk(*ik(*STAND))
    for name, front, left in LEGS:
        foot = d.site_xpos[m.site(f"{name}_foot").id] - trunk
        want = np.array([ASM.legs[name]["shaft_x"]["B"] + fx * 1e-3, foot[1], -fy * 1e-3])
        assert np.abs(foot - want).max() < 1e-5, f"{name}: foot off by {(foot - want) * 1e3} mm"
    assert sim.loop_error() < 1e-6, f"loop open by {sim.loop_error() * 1e3} mm at the stand"
    for _ in range(600):
        sim.step()
    touch = [float(d.sensor(f"{n}_touch").data[0]) for n in FOOT_ORDER]
    servo = "BAM HD-1910M" if sim.bam is not None else "<position> companion"
    print(f"check ({servo}): IK stand and closure match the CAD; mass {total * 1e3:.0f} g; after 3 s z "
          f"{d.qpos[2] * 1e3:.1f} mm (home {home_z() * 1e3:.1f}), tilt {sim.tilt_deg():.2f} deg, foot load "
          f"{np.round(touch, 2).tolist()} N, loop {sim.loop_error() * 1e3:.3f} mm, max servo torque "
          f"{np.abs(d.qfrc_actuator[[int(m.jnt_dofadr[m.joint(n).id]) for n in _joint_names()]]).max():.3f} N.m")


def demo_stream(rate_hz: float, dt: float):
    """Yields (label, joint targets) every dt for the DEMO sequence, cross-fading between segments."""
    last = stand_of("TROT")
    for label, move, gait, seconds in DEMO:
        if move is None:
            fn = (lambda g: lambda t: stand_of(g))(gait)
        else:
            tab = gait_targets(move, gait)

            def fn(t, tab=tab):
                s = t * rate_hz
                i = int(s) % len(tab)
                return tab[i] + (tab[(i + 1) % len(tab)] - tab[i]) * (s - int(s))
        blend = seconds if move is None else BLEND_S
        t, tgt = 0.0, last
        while t < seconds - 1e-9:
            a = min(1.0, t / blend)
            a = a * a * (3 - 2 * a)
            tgt = (1 - a) * last + a * fn(t)
            yield label, tgt
            t += dt
        last = tgt


def run_demo(path: Path, rate_hz: float, video: Path | None = None) -> None:
    """Play DEMO on the model and print what each segment did. With `video`, also write an mp4 with labels."""
    import subprocess

    import mujoco

    sim = Sim(path)
    m, d = sim.model, sim.data
    fps = 50
    sub = int(round(1 / fps / m.opt.timestep))
    proc = r = cam = None
    if video:
        from PIL import Image, ImageDraw, ImageFont
        r = mujoco.Renderer(m, 720, 1280)
        cam = mujoco.MjvCamera()
        cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        cam.trackbodyid = m.body("trunk").id
        cam.distance, cam.azimuth, cam.elevation = 0.36, 125.0, -16.0
        try:
            font = ImageFont.truetype("/System/Library/Fonts/Helvetica.ttc", 34)
        except OSError:
            font = ImageFont.load_default()
        video.parent.mkdir(parents=True, exist_ok=True)
        proc = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
                                 "-s", "1280x720", "-r", str(fps), "-i", "-", "-pix_fmt", "yuv420p",
                                 "-vcodec", "libx264", "-crf", "20", str(video)], stdin=subprocess.PIPE)
    seg, k = None, 0
    stats = {}

    def close(label):
        if label is None:
            return
        s = stats[label]
        dx, dy = d.qpos[0] - s["p"][0], d.qpos[1] - s["p"][1]
        along = dx * math.cos(s["yaw0"]) + dy * math.sin(s["yaw0"])
        print(f"  {label:18s} {s['t']:4.1f} s  avanzamento {100 * along:+6.1f} cm ({100 * along / s['t']:+5.1f} cm/s)  "
              f"imbardata {math.degrees(s['yaw']):+6.1f} deg  inclinazione max {s['tilt']:4.1f} deg  "
              f"tronco {s['zmin'] * 1e3:4.1f}-{s['zmax'] * 1e3:4.1f} mm  errore giunti {math.degrees(s['err'] / max(s['n'], 1)):4.1f} deg  "
              f"pentagono {s['loop'] * 1e3:4.2f} mm")

    for label, tgt in demo_stream(rate_hz, m.opt.timestep):
        if label != seg:
            close(seg)
            seg = label
            stats[label] = {"p": d.qpos[:2].copy(), "yaw0": sim.yaw(), "yaw": 0.0, "yaw_prev": sim.yaw(), "t": 0.0,
                            "tilt": 0.0, "zmin": 1.0, "zmax": 0.0, "err": 0.0, "n": 0, "loop": 0.0}
        sim.step(tgt)
        s = stats[label]
        y = sim.yaw()
        s["yaw"] += math.atan2(math.sin(y - s["yaw_prev"]), math.cos(y - s["yaw_prev"]))
        s["yaw_prev"] = y
        s["t"] += m.opt.timestep
        s["tilt"] = max(s["tilt"], sim.tilt_deg())
        s["zmin"], s["zmax"] = min(s["zmin"], d.qpos[2]), max(s["zmax"], d.qpos[2])
        s["err"] += float(np.abs(d.qpos[sim.qadr] - tgt).mean())
        s["n"] += 1
        if k % 4 == 0:
            s["loop"] = max(s["loop"], sim.loop_error())
        if proc and k % sub == 0:
            r.update_scene(d, camera=cam)
            img = Image.fromarray(r.render())
            ImageDraw.Draw(img).text((40, 30), f"Q8bot + HD-1910M   {label}", fill=(20, 20, 20), font=font)
            proc.stdin.write(img.tobytes())
        k += 1
    close(seg)
    if proc:
        proc.stdin.close()
        proc.wait()
        print(f"wrote {video}")


def view(path: Path, rate_hz: float) -> None:
    """Interactive viewer (run with mjpython on macOS): the demo in a loop, in real time."""
    import time

    import mujoco
    import mujoco.viewer

    sim = Sim(path)
    m, d = sim.model, sim.data
    with mujoco.viewer.launch_passive(m, d) as v:
        v.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        v.cam.trackbodyid = m.body("trunk").id
        v.cam.distance, v.cam.azimuth, v.cam.elevation = 0.36, 125.0, -16.0
        v.opt.geomgroup[3] = 0
        while v.is_running():
            sim.reset()
            shown = None
            for label, tgt in demo_stream(rate_hz, m.opt.timestep):
                t0 = time.perf_counter()
                sim.step(tgt)
                if label != shown:
                    shown = label
                    print(label, flush=True)
                    try:
                        v.set_texts((mujoco.mjtFontScale.mjFONTSCALE_200, mujoco.mjtGridPos.mjGRID_TOPLEFT,
                                     "Q8bot + HD-1910M", label))
                    except (AttributeError, TypeError):
                        pass
                v.sync()
                if not v.is_running():
                    return
                time.sleep(max(0.0, m.opt.timestep - (time.perf_counter() - t0)))


def render(path: Path, out: Path, camera: str) -> None:
    import mujoco
    from PIL import Image

    m = mujoco.MjModel.from_xml_path(str(path))
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, 0)
    mujoco.mj_forward(m, d)
    r = mujoco.Renderer(m, 720, 1280)
    r.update_scene(d, camera=camera)
    Image.fromarray(r.render()).save(out)
    print(f"wrote {out}")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", type=Path, default=ROBOT_DIR / "q8bot.xml")
    ap.add_argument("--render", type=Path, help="PNG of the stand pose")
    ap.add_argument("--camera", default="track")
    ap.add_argument("--demo", action="store_true", help="forward, backward, turn; then the same on the low stance")
    ap.add_argument("--video", type=Path, help="mp4 of the demo")
    ap.add_argument("--view", action="store_true", help="interactive viewer with the demo in a loop (mjpython)")
    ap.add_argument("--rate", type=float, default=SAMPLE_HZ, help="gait samples per second (upstream 200)")
    args = ap.parse_args()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(build_xml())
    print(f"wrote {args.out} (stand q0 {[round(a, 6) for a in stand_q0()[:2]]} x4, theta = q - 90 deg, "
          f"home_z {home_z()})")
    check(args.out)
    if args.render:
        render(args.out, args.render, args.camera)
    if args.demo or args.video:
        run_demo(args.out, args.rate, video=args.video)
    if args.view:
        view(args.out, args.rate)


ASM = Assembly()

if __name__ == "__main__":
    main()
