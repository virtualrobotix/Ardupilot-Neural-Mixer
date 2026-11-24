#!/usr/bin/env python3
# Author: Roberto Navoni, member of the ArduPilot Dev Team
# Contact: r.navoni74@gmail.com
# Developed by Roberto Navoni — DelphyAI LAB
# For information: r.navoni74@gmail.com
"""Flying ArduDuck: the ArduDuck (XGO Duck) with the flight pack of the CMU MicroDuck fork, in MuJoCo.

Source of the idea: https://github.com/qilinxiaoxiang/microduck_rl, `cmu/finetune/flight`. That folder holds the
fine-tune pipeline for a *flight pack* (extension `flight-20261110` of the CMU robot: 936 g, four EDFs, two flight
batteries, a wing plate; the robot goes from 0.96 to 1.90 kg). The pack CAD and the `flightpack-*.json` manifests are
in another repository (`microduck-sim`) and are not in the fork, so this file rebuilds the pack from the facts the
fork states and from stated assumptions, on the ArduDuck body of `tools/unoq/xgoduck_scene.py`:

  from the fork   pack 936 g = 2 x 151 g flight batteries + 4 EDFs + wing plate, trays and flight controller;
                  trunk-mounted, in the saddle position; legs keep the 14-joint contract of the walk policies
  assumed here    EDF 50 mm class: 85 g each; thrust 6.4 N each (about 0.65 kgf, 1.5 x the weight of the
                  1.73 kg robot); thrust follows the throttle linearly through a 50 ms first-order lag;
                  reaction torque 0.02 N m per N, alternating spin; geometry of plate, tray and nacelles
                  (the real CAD is not available)

  Replace the assumptions with the measured thrust curve and the real manifest when they are available.

    .venv/bin/python tools/robots/flying_arduduck_mjcf.py --write     # robots/flying_arduduck/robot/*.xml + meshes
    .venv/bin/python tools/robots/flying_arduduck_mjcf.py --demo      # hover flight, numbers and plots
    .venv/bin/mjpython tools/robots/flying_arduduck_mjcf.py --view    # interactive viewer, the flight in loop
    .venv/bin/python tools/robots/flying_arduduck_mjcf.py --render    # PNG views + MP4 of the flight
"""

from __future__ import annotations

import argparse
import math
import shutil
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(ROOT / "tools" / "unoq"))

ROBOT_DIR = ROOT / "robots" / "flying_arduduck" / "robot"
XML_PATH = ROBOT_DIR / "flying_arduduck.xml"
RL_ASSETS = ROOT / "third_party" / "xgoduck_rl" / "src" / "mjlab_microduck" / "robot" / "xgoduck" / "assets"
G = 9.81

# ---- flight pack (see the module docstring for what is stated and what is assumed) -----------------------------
PACK_KG = 0.936                        # fork: flight_worker.py
BATTERY_KG = 0.151                     # fork: payload.py, 2 x 151 g
EDF_KG = 0.085                         # assumed
EDF_RADIUS, EDF_LEN = 0.031, 0.040     # assumed, 50 mm class with the nacelle
EDF_THRUST_N = 6.4                     # assumed, max thrust of one EDF
EDF_TAU_S = 0.05                       # assumed, spool lag
EDF_YAW_K = 0.02                       # assumed, N m of reaction torque per N of thrust
EDF_XY = 0.075, 0.125                  # half spacing of the X layout (x, y) from the centre of mass
PLATE_Z = 0.200                        # plate height in the trunk frame (the trunk top is at 0.174)
KP_POS, KD_POS = 0.9, 2.2             # horizontal position loop (m/s^2 per m, per m/s)
LEG_KP = 3.0                           # N m/rad, leg servos (torque limit 1.04 N m unchanged)
KP_ATT, KD_ATT, KI_ATT = 36.0, 14.0, 40.0   # attitude loop (s^-2, s^-1, s^-3 times the inertia)
EDFS = (("FL", +1, +1, +1), ("FR", +1, -1, -1), ("RL", -1, +1, -1), ("RR", -1, -1, +1))   # name, sx, sy, spin

JOINTS = ("left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle",
          "neck_pitch", "head_pitch", "head_yaw", "head_roll",
          "right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle")


def _box_inertia(m: float, half) -> list[float]:
    x, y, z = (2 * h for h in half)
    return [m / 12 * (y * y + z * z), m / 12 * (x * x + z * z), m / 12 * (x * x + y * y)]


def _add_part(trunk, name: str, pos, mass: float, inertia, rgba, geoms, collide: bool = True):
    """Child body of the trunk with an explicit inertial; geoms = [(type, size, pos)] (visual + a hidden collider)."""
    import mujoco
    b = trunk.add_body(name=name, pos=list(pos))
    b.mass = mass
    b.ipos = [0.0, 0.0, 0.0]
    b.inertia = list(inertia)
    b.explicitinertial = True
    for i, (gtype, size, gpos) in enumerate(geoms):
        b.add_geom(type=gtype, size=list(size), pos=list(gpos), group=2, density=0, contype=0, conaffinity=0,
                   rgba=list(rgba))
        if collide:
            b.add_geom(name=f"{name}_collision{i}", type=gtype, size=list(size), pos=list(gpos), group=3, density=0,
                       contype=1, conaffinity=0, condim=1, rgba=[1, 0, 0, 0.3])
    return b


def build_spec(xc: float | None = None, xt: float | None = None):
    """The ArduDuck spec of xgoduck_scene with the flight pack and the four EDF thrust actuators."""
    import mujoco
    import xgoduck_scene as xs

    spec = xs.build_spec()
    # the model of xgoduck_rl is a bare-viewing PD (kp 0.84 N m/rad): under the 1.7 kg of the robot with the pack the
    # legs sag. The real HLS1910 at firmware gain 5 is much stiffer up to its 1.04 N m limit, kept here.
    for act in spec.actuators:
        if act.name in JOINTS:
            act.gainprm[0] = LEG_KP
            act.biasprm[1] = -LEG_KP
    if xc is None:
        xc = base_com_x()
    if xt is None:
        xt = xc            # x of the centre of the EDF square (the thrust centroid)
    trunk = spec.body("trunk_base")
    box, cyl = mujoco.mjtGeom.mjGEOM_BOX, mujoco.mjtGeom.mjGEOM_CYLINDER
    ax, ay = EDF_XY
    fixed_kg = PACK_KG - 2 * BATTERY_KG - 4 * EDF_KG                  # plate, trays, flight controller, wiring
    m_wing, m_rear, m_fc = 0.080, fixed_kg - 2 * 0.080 - 0.054, 0.054

    carbon = [0.10, 0.10, 0.12, 1.0]
    # wing plate: two side wings clear of the head (y 0.055..0.160) and a rear bar behind the neck
    for side, sy in (("L", 1), ("R", -1)):
        half = (0.085, 0.0525, 0.003)
        _add_part(trunk, f"wing_{side}", (xc, sy * 0.1075, PLATE_Z), m_wing, _box_inertia(m_wing, half), carbon,
                  [(box, half, (0, 0, 0))])
    half = (0.020, 0.055, 0.003)
    _add_part(trunk, "wing_rear", (xc - 0.065, 0.0, PLATE_Z), m_rear, _box_inertia(m_rear, half), carbon,
              [(box, half, (0, 0, 0))])
    # flight batteries on the back of the trunk, under the plate
    half = (0.037, 0.017, 0.010)
    for side, sy in (("L", 1), ("R", -1)):
        _add_part(trunk, f"battery_{side}", (xc - 0.005, sy * 0.019, 0.186), BATTERY_KG,
                  _box_inertia(BATTERY_KG, half), [0.15, 0.30, 0.75, 1.0], [(box, half, (0, 0, 0))])
    half = (0.020, 0.020, 0.004)
    _add_part(trunk, "flight_controller", (xc - 0.065, 0.0, PLATE_Z + 0.007), m_fc, _box_inertia(m_fc, half),
              [0.10, 0.55, 0.20, 1.0], [(box, half, (0, 0, 0))], collide=False)
    # four EDFs at the corners of the X layout, thrust along +z
    ixx = EDF_KG / 12 * (3 * EDF_RADIUS ** 2 + EDF_LEN ** 2)
    izz = EDF_KG / 2 * EDF_RADIUS ** 2
    for name, sx, sy, spin in EDFS:
        pos = (xt + sx * ax, sy * ay, PLATE_Z + 0.003 + EDF_LEN / 2)
        b = _add_part(trunk, f"edf_{name}", pos, EDF_KG, [ixx, ixx, izz], [0.85, 0.45, 0.10, 1.0],
                      [(cyl, (EDF_RADIUS, EDF_LEN / 2, 0), (0, 0, 0))])
        b.add_geom(type=cyl, size=[EDF_RADIUS * 0.72, 0.0006, 0], pos=[0, 0, EDF_LEN / 2], group=2, density=0,
                   contype=0, conaffinity=0, rgba=[0.05, 0.05, 0.05, 1.0])        # intake grille
        b.add_site(name=f"thrust_{name}", pos=[0, 0, 0], size=[0.004, 0, 0], group=4)
        a = spec.add_actuator(name=f"edf_{name}", trntype=mujoco.mjtTrn.mjTRN_SITE, target=f"thrust_{name}",
                              gear=[0, 0, 1, 0, 0, spin * EDF_YAW_K], ctrllimited=True, ctrlrange=[0.0, 1.0],
                              gaintype=mujoco.mjtGain.mjGAIN_FIXED, gainprm=[EDF_THRUST_N] + [0.0] * 9,
                              biastype=mujoco.mjtBias.mjBIAS_NONE, dyntype=mujoco.mjtDyn.mjDYN_FILTER,
                              dynprm=[EDF_TAU_S] + [0.0] * 9)
    return spec


def aligned_xt(xc: float, iterations: int = 6) -> float:
    """x of the EDF square that puts the thrust centroid under the centre of mass of the whole robot in the
    stance pose. The plate, the batteries and the flight controller are fixed at xc; the pack is not symmetric
    (rear bar and controller), so the EDFs move until the two coincide."""
    import mujoco

    xt = xc
    for _ in range(iterations):
        m = build_spec(xc, xt).compile()
        d = mujoco.MjData(m)
        set_stance(m, d)
        mujoco.mj_forward(m, d)
        new = float(d.subtree_com[1][0])
        if abs(new - xt) < 1e-5:
            return new
        xt = new
    return xt


def base_com_x() -> float:
    """x of the ArduDuck centre of mass in the stance pose: the pack is built symmetric about it."""
    import mujoco
    import xgoduck_scene as xs

    spec = xs.build_spec()
    m = spec.compile()
    d = mujoco.MjData(m)
    set_stance(m, d)
    mujoco.mj_forward(m, d)
    return float(d.subtree_com[1][0])


def set_stance(m, d) -> None:
    import mujoco
    import xgoduck_scene as xs

    for n, deg in zip(JOINTS, xs.STANCE_DEG):
        d.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)]] = math.radians(deg)
    d.qpos[2] = xs.STAND_Z


def stance_ctrl() -> np.ndarray:
    import xgoduck_scene as xs
    return np.radians(np.array(xs.STANCE_DEG))


def write_model() -> Path:
    """Compile the spec, write the XML with the meshes next to it, so the scene is versioned in this repo."""
    import mujoco

    if not RL_ASSETS.is_dir():
        sys.exit(f"{RL_ASSETS} not found: git clone -b hd1910-standup https://github.com/ZXerniXZ/xgoduck_rl "
                 "third_party/xgoduck_rl")
    xc = base_com_x()
    spec = build_spec(xc, aligned_xt(xc))
    spec.compile()
    mesh_dir = ROBOT_DIR / "meshes"
    mesh_dir.mkdir(parents=True, exist_ok=True)
    for me in spec.meshes:
        src = RL_ASSETS / Path(me.file).name
        shutil.copy2(src, mesh_dir / src.name)
    spec.meshdir = str(mesh_dir)             # to_xml re-reads the meshes: point it at the copies, then relativize
    xml = spec.to_xml().replace(str(mesh_dir), "meshes")
    XML_PATH.write_text(xml)
    return XML_PATH


def load():
    """Model and data of the versioned scene (written first when missing)."""
    import mujoco

    if not XML_PATH.is_file():
        write_model()
    m = mujoco.MjModel.from_xml_path(str(XML_PATH))
    return m, mujoco.MjData(m)


class Hover:
    """Cascade flight controller on the four EDFs (what an ArduCopter-style mixer does on the autopilot):
    altitude PD with weight feed-forward, position PD to a tilt reference, attitude PD to torques, X mixer."""

    def __init__(self, m, d):
        import mujoco
        self.m = m
        self.mass = float(m.body_subtreemass[1])
        mujoco.mj_forward(m, d)
        self.com = d.subtree_com[1].copy()
        self.act = np.array([mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, f"edf_{n}") for n, *_ in EDFS])
        self.leg = np.array([mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in JOINTS])
        self.sx = np.array([s[1] for s in EDFS], float)
        self.sy = np.array([s[2] for s in EDFS], float)
        self.spin = np.array([s[3] for s in EDFS], float)
        self.ax, self.ay = EDF_XY
        # rotational inertia of the whole robot about its centre of mass (stance pose, trunk level)
        I = np.zeros((3, 3))
        for b in range(1, m.nbody):
            r = d.xipos[b] - self.com
            R = d.ximat[b].reshape(3, 3)
            I += R @ np.diag(m.body_inertia[b]) @ R.T + m.body_mass[b] * (r @ r * np.eye(3) - np.outer(r, r))
        self.inertia = np.diag(I).copy()
        self.q0 = stance_ctrl()
        self.i_att = np.zeros(2)

    def command(self, d, ref) -> tuple[np.ndarray, dict]:
        """ref = dict(p=[x,y,z], yaw). Returns the 4 throttle values in [0, 1] and the logged terms."""
        p, v = d.qpos[0:3], d.qvel[0:3]
        w = d.qvel[3:6]                                           # body-frame angular velocity
        R = d.xmat[1].reshape(3, 3)                               # body -> world
        yaw = math.atan2(R[1, 0], R[0, 0])
        c, s = math.cos(yaw), math.sin(yaw)
        arm = float(np.clip((d.time - 0.2) / 1.0, 0.0, 1.0))      # spool-up: EDFs idle at 40 % of the weight
        airborne = float(np.clip((p[2] - 0.18) / 0.10, 0.0, 1.0))  # horizontal loops act only off the ground
        # horizontal position PD in the heading frame -> tilt reference
        e = np.asarray(ref["p"], float) - p
        a_w = (KP_POS * e[:2] - KD_POS * v[:2]) * airborne
        a_w = np.clip(a_w, -2.0, 2.0)
        a_x, a_y = c * a_w[0] + s * a_w[1], -s * a_w[0] + c * a_w[1]
        theta_ref = float(np.clip(a_x / G, -0.30, 0.30))          # nose down (positive) accelerates forward
        phi_ref = float(np.clip(-a_y / G, -0.30, 0.30))
        u_b = R.T @ np.array([0.0, 0.0, 1.0])                     # world up in the body frame
        phi, theta = u_b[1], -u_b[0]
        I = self.inertia
        # integral trim on the attitude: the centre of mass sits a few mm off the thrust centroid (legs, head)
        dt_c = 0.005
        self.i_att = np.clip(self.i_att + np.array([phi_ref - phi, theta_ref - theta]) * dt_c * airborne,
                             -0.10, 0.10)
        tau = np.array([I[0] * (KP_ATT * (phi_ref - phi) - KD_ATT * w[0] + KI_ATT * self.i_att[0]),
                        I[1] * (KP_ATT * (theta_ref - theta) - KD_ATT * w[1] + KI_ATT * self.i_att[1]),
                        0.0])
        dyaw = math.atan2(math.sin(ref.get("yaw", 0.0) - yaw), math.cos(ref.get("yaw", 0.0) - yaw))
        tau[2] = I[2] * (14.0 * dyaw - 5.0 * w[2])
        T = self.mass * G * (0.4 + 0.6 * arm) + arm * self.mass * (28.0 * e[2] - 8.0 * v[2])
        T = max(T, 0.0) / max(u_b[2], 0.6)
        T_i = (T / 4 + tau[0] / (4 * self.ay) * self.sy - tau[1] / (4 * self.ax) * self.sx
               + tau[2] / (4 * EDF_YAW_K) * self.spin)
        u = np.clip(T_i / EDF_THRUST_N, 0.0, 1.0)
        return u, {"phi": phi, "theta": theta, "yaw": yaw, "T": T}


# (label, seconds, target x, y, z, yaw deg); z is the trunk height, 0.142 standing
PROFILE = [
    ("a terra", 1.0, 0.0, 0.0, 0.142, 0.0),
    ("decollo", 3.0, 0.0, 0.0, 0.60, 0.0),
    ("hover", 3.0, 0.0, 0.0, 0.60, 0.0),
    ("avanti 0,6 m", 3.0, 0.6, 0.0, 0.60, 0.0),
    ("laterale 0,4 m", 3.0, 0.6, 0.4, 0.60, 0.0),
    ("imbardata 90°", 3.0, 0.6, 0.4, 0.60, 90.0),
    ("ritorno e sali", 4.0, 0.0, 0.0, 0.85, 90.0),
    ("atterraggio", 5.0, 0.0, 0.0, 0.142, 90.0),
]


def ref_at(t: float, prev: dict) -> tuple[str, dict, dict]:
    """Smooth (minimum-jerk) reference between the profile waypoints; returns label, ref, new prev."""
    t0 = 0.0
    p0 = np.array([0.0, 0.0, 0.142])
    y0 = 0.0
    for label, dur, x, y, z, yaw in PROFILE:
        p1, y1 = np.array([x, y, z]), math.radians(yaw)
        if t < t0 + dur:
            s = (t - t0) / dur
            k = 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5
            return label, {"p": p0 + (p1 - p0) * k, "yaw": y0 + (y1 - y0) * k}, prev
        t0 += dur
        p0, y0 = p1, y1
    return PROFILE[-1][0], {"p": p0, "yaw": y0}, prev


def flight_seconds() -> float:
    return sum(p[1] for p in PROFILE) + 2.0


def run_flight(model=None, data=None, hooks=None, seconds: float | None = None, realtime: bool = False):
    """Fly the profile; hooks(t, label, m, d) is called every 20 ms. Returns the log as a dict of arrays."""
    import mujoco

    m, d = (model, data) if model is not None else load()
    mujoco.mj_resetData(m, d)
    set_stance(m, d)
    mujoco.mj_forward(m, d)
    hv = Hover(m, d)
    d.ctrl[hv.leg] = hv.q0
    dt = m.opt.timestep
    n_ctl = max(1, int(round(0.005 / dt)))                        # controller at 200 Hz
    seconds = seconds or flight_seconds()
    log = {k: [] for k in ("t", "x", "y", "z", "roll", "pitch", "yaw", "u0", "u1", "u2", "u3", "zr", "label")}
    t_wall = time.perf_counter()
    k = 0
    prev: dict = {}
    u = np.zeros(4)
    n = int(seconds / dt)
    for k in range(n):
        t = k * dt
        label, ref, prev = ref_at(t, prev)
        if k % n_ctl == 0:
            u, info = hv.command(d, ref)
            d.ctrl[hv.act] = u
            d.ctrl[hv.leg] = hv.q0
        mujoco.mj_step(m, d)
        if k % int(0.02 / dt) == 0:
            R = d.xmat[1].reshape(3, 3)
            log["t"].append(t); log["x"].append(d.qpos[0]); log["y"].append(d.qpos[1]); log["z"].append(d.qpos[2])
            log["roll"].append(math.degrees(info["phi"])); log["pitch"].append(math.degrees(info["theta"]))
            log["yaw"].append(math.degrees(math.atan2(R[1, 0], R[0, 0])))
            for i in range(4):
                log[f"u{i}"].append(float(u[i]))
            log["zr"].append(float(ref["p"][2])); log["label"].append(label)
            if hooks is not None:
                if hooks(t, label, m, d) is False:
                    break
            if realtime:
                lag = t_wall + t - time.perf_counter()
                if lag > 0:
                    time.sleep(lag)
    return {k: (np.array(v) if k != "label" else v) for k, v in log.items()}


def summarize(m, log) -> dict:
    t, z = log["t"], log["z"]
    air = z > 0.142 + 0.03
    return {
        "mass_kg": float(m.body_subtreemass[1]),
        "weight_N": float(m.body_subtreemass[1]) * G,
        "thrust_max_N": 4 * EDF_THRUST_N,
        "twr": 4 * EDF_THRUST_N / (float(m.body_subtreemass[1]) * G),
        "z_max": float(z.max()),
        "hover_z_err_mm": float(1000 * np.abs(z[(t > 4.2) & (t < 7.0)] - 0.60).max()),
        "max_tilt_deg": float(np.max(np.hypot(log["roll"], log["pitch"]))),
        "airborne_s": float(air.sum() * 0.02),
        "final_z": float(z[-1]),
        "final_xy_mm": float(1000 * math.hypot(log["x"][-1], log["y"][-1])),
        "min_throttle": float(min(log[f"u{i}"][air].min() for i in range(4))) if air.any() else 0.0,
        "max_throttle": float(max(log[f"u{i}"].max() for i in range(4))),
    }


def plot_flight(log, path: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = log["t"]
    fig, ax = plt.subplots(4, 1, figsize=(10, 9), sharex=True)
    ax[0].plot(t, log["z"], label="z tronco"); ax[0].plot(t, log["zr"], "--", label="z riferimento")
    ax[0].plot(t, log["x"], label="x"); ax[0].plot(t, log["y"], label="y")
    ax[0].set_ylabel("m"); ax[0].legend(ncol=4, fontsize=8); ax[0].set_title("Flying ArduDuck: volo in MuJoCo")
    ax[1].plot(t, log["roll"], label="rollio"); ax[1].plot(t, log["pitch"], label="beccheggio")
    ax[1].set_ylabel("gradi"); ax[1].legend(ncol=2, fontsize=8)
    ax[2].plot(t, log["yaw"]); ax[2].set_ylabel("imbardata (°)")
    for i, nm in enumerate(("FL", "FR", "RL", "RR")):
        ax[3].plot(t, log[f"u{i}"], label=f"EDF {nm}")
    ax[3].set_ylabel("manetta"); ax[3].set_xlabel("s"); ax[3].legend(ncol=4, fontsize=8)
    # phase bands
    lbl, edges = log["label"], []
    for i, l in enumerate(lbl):
        if i == 0 or l != lbl[i - 1]:
            edges.append((t[i], l))
    for a in ax:
        for tt, _ in edges:
            a.axvline(tt, color="k", lw=0.3, alpha=0.4)
    for tt, l in edges:
        ax[0].text(tt + 0.05, ax[0].get_ylim()[1] * 0.97, l, fontsize=7, va="top", rotation=90)
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=130)
    plt.close(fig)


def render(out_dir: Path, media_dir: Path, video: bool = True) -> None:
    import imageio.v2 as imageio
    import mujoco
    from PIL import Image, ImageDraw, ImageFont

    m, d = load()
    m.vis.global_.offwidth, m.vis.global_.offheight = 1280, 720
    r = mujoco.Renderer(m, 720, 1280)
    out_dir.mkdir(parents=True, exist_ok=True)

    def font(sz):
        for f in ("/System/Library/Fonts/Menlo.ttc", "/System/Library/Fonts/Monaco.ttf"):
            if Path(f).exists():
                return ImageFont.truetype(f, sz)
        return ImageFont.load_default()

    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE

    def frame(d, label, t, lookat, dist, az, el):
        cam.lookat[:] = lookat
        cam.distance, cam.azimuth, cam.elevation = dist, az, el
        r.update_scene(d, camera=cam)
        img = Image.fromarray(r.render())
        dr = ImageDraw.Draw(img, "RGBA")
        dr.rectangle([0, 0, img.width, 40], fill=(10, 20, 35, 200))
        dr.text((12, 10), f"Flying ArduDuck   t={t:5.1f}s   {label}   z={d.qpos[2]:.2f} m", fill=(255, 255, 255, 255),
                font=font(18))
        dr.text((12, img.height - 26), "Realizzati da Roberto Navoni — DelphyAI LAB", fill=(225, 225, 225, 255),
                font=font(14))
        return np.asarray(img)

    # stills: standing with the pack, and in flight
    mujoco.mj_resetData(m, d); set_stance(m, d); mujoco.mj_forward(m, d)
    for view, (az, el, dist) in {"3q": (140, -18, 0.95), "side": (90, -5, 0.95), "top": (90, -88, 0.9)}.items():
        Image.fromarray(frame(d, "a terra con il pacco di volo", 0.0, d.subtree_com[1], dist, az, el)).save(
            out_dir / f"flying_arduduck_{view}.png")
    frames: list[np.ndarray] = []
    shot = {"done": False}
    state = {"az": 140.0}

    def hook(t, label, m_, d_):
        state["az"] = 140.0 + 12.0 * t
        if video and int(round(t / 0.02)) % 2 == 0:
            frames.append(frame(d_, label, t, d_.qpos[0:3] + np.array([0, 0, 0.0]), 1.5, state["az"], -15))
        if label == "hover" and t > 5.4 and not shot["done"]:
            shot["done"] = True
            Image.fromarray(frame(d_, "hover", t, d_.qpos[0:3], 1.1, 140, -10)).save(out_dir / "flying_arduduck_hover.png")
        return True

    log = run_flight(m, d, hooks=hook)
    plot_flight(log, out_dir / "flying_arduduck_flight.png")
    if video and frames:
        media_dir.mkdir(parents=True, exist_ok=True)
        w = imageio.get_writer(str(media_dir / "flying_arduduck_hover.mp4"), fps=25, codec="libx264",
                               quality=7, macro_block_size=1, pixelformat="yuv420p")
        for f in frames:
            w.append_data(f)
        w.close()
        print(f"video: {media_dir / 'flying_arduduck_hover.mp4'} ({len(frames)} frames)")
    print(f"png: {out_dir}")


def view() -> None:
    import mujoco
    import mujoco.viewer

    m, d = load()
    state = {"restart": False}
    viewer = mujoco.viewer.launch_passive(m, d, key_callback=lambda k: state.update(restart=True) if chr(k).lower() == "r" else None)
    viewer.cam.distance, viewer.cam.elevation, viewer.cam.azimuth = 1.4, -15, 140
    print("viewer: R riavvia il volo, chiudi la finestra per uscire")
    while viewer.is_running():
        state["restart"] = False

        def hook(t, label, m_, d_):
            if not viewer.is_running() or state["restart"]:
                return False
            with viewer.lock():
                viewer.cam.lookat[:] = d_.qpos[0:3]
            viewer.set_texts((mujoco.mjtFontScale.mjFONTSCALE_150, mujoco.mjtGridPos.mjGRID_TOPLEFT,
                              f"Flying ArduDuck  t={t:4.1f}s  {label}", f"z={d_.qpos[2]:.2f} m"))
            viewer.sync()
            return True

        run_flight(m, d, hooks=hook, realtime=True)
        t_end = time.perf_counter() + 2.0
        while viewer.is_running() and not state["restart"] and time.perf_counter() < t_end:
            time.sleep(0.05)
    viewer.close()


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--write", action="store_true", help="write robots/flying_arduduck/robot/flying_arduduck.xml")
    ap.add_argument("--demo", action="store_true", help="fly the profile headless, print the numbers, save the plot")
    ap.add_argument("--render", action="store_true", help="PNG views and the MP4 of the flight")
    ap.add_argument("--view", action="store_true", help="interactive MuJoCo viewer (use mjpython)")
    ap.add_argument("--img-dir", type=Path, default=ROOT / "docs" / "robots" / "img")
    ap.add_argument("--media-dir", type=Path, default=ROOT / "docs" / "media")
    args = ap.parse_args()
    if args.write or not XML_PATH.is_file():
        print(f"model: {write_model()}")
    if args.demo:
        m, d = load()
        log = run_flight(m, d)
        for k, v in summarize(m, log).items():
            print(f"  {k:16s} {v:9.3f}")
        plot_flight(log, args.img_dir / "flying_arduduck_flight.png")
        print(f"plot: {args.img_dir / 'flying_arduduck_flight.png'}")
    if args.render:
        render(args.img_dir, args.media_dir)
    if args.view:
        view()


if __name__ == "__main__":
    main()
