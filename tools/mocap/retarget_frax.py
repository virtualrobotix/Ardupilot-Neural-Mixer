#!/usr/bin/env python3
# Author: Roberto Navoni, member of the ArduPilot Dev Team
# Contact: r.navoni74@gmail.com
# Developed by Roberto Navoni — DelphyAI LAB
"""Offline human -> robot motion retargeting with frax.ik (StanfordASL/frax, MIT).

Turns a human recording into a dense joint clip for the gesture imitation training
(train_velocity.py --gesture motion --motion <clip.npz>):

  MediaPipe world landmarks (.npz, landmarks (T, 33, 3), fps)  ─┐
  BVH mocap (LAFAN1 bone names)                                 ─┼─> keypoints ─> frax DiffIK ─> q (T, n)
                                                                                    (joint limits ∩ q0 ± act_max,
                                                                                     servo speed limit)

The robot model is the training MJCF (common.resolve_mjcf), converted to a minimal URDF
(kinematic tree, inertias, joint limits): frax reads URDF only. Each frame is one
differential IK step warm-started from the previous one, solved by ElastiQP, which stays
feasible when a target is out of reach (the l1 relaxation gives up on the target, never
on a limit).

The clip is retargeted in place: heading and horizontal travel are removed, the feet stay
on their q0 stance unless --free-feet, the trunk follows the human trunk (height from the
hip-ankle distance, lean and sway), the arms follow the human elbow and wrist directions
scaled to the robot's limbs.

Needs the optional environment (JAX >= 0.9.1, frax 0.2):
    uv venv .venv-frax --python 3.12 && uv pip install -p .venv-frax/bin/python "frax[ik]==0.2.0" mujoco scipy

Usage:
    .venv-frax/bin/python tools/mocap/retarget_frax.py --robot microban \
        --mediapipe recordings/wave.npz --name wave_frax --loop-blend 0.3
    .venv-frax/bin/python tools/mocap/retarget_frax.py --robot microban \
        --bvh data/lafan1/dance1_subject1.bvh --start 10 --end 16 --name dance_lafan
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import tempfile
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

os.environ.setdefault("JAX_PLATFORMS", "cpu")
os.environ.setdefault("JAX_ENABLE_X64", "1")
os.environ.setdefault("XLA_FLAGS", "--xla_cpu_multi_thread_eigen=false")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")

import numpy as np  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "robots"))
from common import load_profile, resolve_mjcf, robot_dir  # noqa: E402

DEFAULT_ACT_MAX = 2.0  # NNM_ACT_MAX, deploy_contract.DEFAULT_ACT_MAX

# Human keypoints used for the retarget, in this order
KEYPOINTS = ("hip_r", "hip_l", "shoulder_r", "shoulder_l", "elbow_r", "elbow_l",
             "wrist_r", "wrist_l", "knee_r", "knee_l", "ankle_r", "ankle_l")
K = {n: i for i, n in enumerate(KEYPOINTS)}

# MediaPipe PoseLandmark indices for KEYPOINTS
MEDIAPIPE_INDEX = (24, 23, 12, 11, 14, 13, 16, 15, 26, 25, 28, 27)
# MediaPipe world (x image right, y down, z away from the camera, subject facing it)
# -> x forward, y left, z up
MEDIAPIPE_TO_FLU = np.array([[0.0, 0.0, -1.0], [1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])

# LAFAN1 / CMU-style BVH bone names for KEYPOINTS
BVH_BONES = ("RightUpLeg", "LeftUpLeg", "RightArm", "LeftArm", "RightForeArm", "LeftForeArm",
             "RightHand", "LeftHand", "RightLeg", "LeftLeg", "RightFoot", "LeftFoot")
BVH_Y_UP_TO_Z_UP = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], dtype=float)


# --------------------------------------------------------------------------- MJCF -> URDF

def _rpy(quat_wxyz: np.ndarray) -> str:
    import warnings

    from scipy.spatial.transform import Rotation

    w, x, y, z = quat_wxyz
    with warnings.catch_warnings():
        # at ±90° pitch the split between roll and yaw is arbitrary; the rotation is still exact
        warnings.simplefilter("ignore", UserWarning)
        r, p, yaw = Rotation.from_quat([x, y, z, w]).as_euler("xyz")
    return f"{r:.9g} {p:.9g} {yaw:.9g}"


def _vec(v) -> str:
    return " ".join(f"{float(a):.9g}" for a in v)


def mjcf_to_urdf(mjcf: Path, joint_names: list[str], vel_max: float) -> tuple[str, str]:
    """Minimal URDF of the robot in `mjcf` (hinge joints only). Returns (urdf, root link name).

    Bodies without joints are attached with fixed joints (frax fuses their inertia into the
    parent). The free joint is dropped: frax adds its own floating base."""
    import mujoco

    m = mujoco.MjModel.from_xml_path(str(mjcf))
    name = lambda t, i: mujoco.mj_id2name(m, t, i)  # noqa: E731
    roots = [b for b in range(1, m.nbody) if m.body_parentid[b] == 0 and m.body_jntnum[b] > 0
             and m.jnt_type[m.body_jntadr[b]] == mujoco.mjtJoint.mjJNT_FREE]
    if len(roots) != 1:
        raise SystemExit(f"{mjcf}: expected one floating root body, found {len(roots)}")
    root = roots[0]
    wanted = set(joint_names)
    robot = ET.Element("robot", name=mjcf.stem)

    def link(lname: str, b: int | None) -> None:
        el = ET.SubElement(robot, "link", name=lname)
        inertial = ET.SubElement(el, "inertial")
        if b is None or m.body_mass[b] <= 0.0:
            ET.SubElement(inertial, "origin", xyz="0 0 0", rpy="0 0 0")
            ET.SubElement(inertial, "mass", value="0")
            ET.SubElement(inertial, "inertia", ixx="0", ixy="0", ixz="0", iyy="0", iyz="0", izz="0")
            return
        ET.SubElement(inertial, "origin", xyz=_vec(m.body_ipos[b]), rpy=_rpy(m.body_iquat[b]))
        ET.SubElement(inertial, "mass", value=f"{m.body_mass[b]:.9g}")
        ix, iy, iz = m.body_inertia[b]
        ET.SubElement(inertial, "inertia", ixx=f"{ix:.9g}", ixy="0", ixz="0",
                      iyy=f"{iy:.9g}", iyz="0", izz=f"{iz:.9g}")

    link(name(mujoco.mjtObj.mjOBJ_BODY, root), root)
    for b in range(1, m.nbody):
        if b == root or not _under(m, b, root):
            continue
        bname = name(mujoco.mjtObj.mjOBJ_BODY, b)
        parent = name(mujoco.mjtObj.mjOBJ_BODY, int(m.body_parentid[b]))
        # joints outside the policy (e.g. a passive head) are locked at their zero position
        hinges = [m.body_jntadr[b] + k for k in range(m.body_jntnum[b])
                  if name(mujoco.mjtObj.mjOBJ_JOINT, m.body_jntadr[b] + k) in wanted]
        for j in hinges:
            if m.jnt_type[j] != mujoco.mjtJoint.mjJNT_HINGE:
                raise SystemExit(f"{bname}: only hinge joints are supported (joint {j})")
            if np.linalg.norm(m.jnt_pos[j]) > 1e-9:
                raise SystemExit(f"{bname}: joint anchor away from the body origin is not supported")
        if not hinges:
            j_el = ET.SubElement(robot, "joint", name=f"{bname}_fixed", type="fixed")
            ET.SubElement(j_el, "parent", link=parent)
            ET.SubElement(j_el, "child", link=bname)
            ET.SubElement(j_el, "origin", xyz=_vec(m.body_pos[b]), rpy=_rpy(m.body_quat[b]))
            link(bname, b)
            continue
        prev = parent
        for k, j in enumerate(hinges):
            jname = name(mujoco.mjtObj.mjOBJ_JOINT, j)
            child = bname if k == len(hinges) - 1 else f"{bname}__{jname}"
            j_el = ET.SubElement(robot, "joint", name=jname, type="revolute")
            ET.SubElement(j_el, "parent", link=prev)
            ET.SubElement(j_el, "child", link=child)
            if k == 0:
                ET.SubElement(j_el, "origin", xyz=_vec(m.body_pos[b]), rpy=_rpy(m.body_quat[b]))
            else:
                ET.SubElement(j_el, "origin", xyz="0 0 0", rpy="0 0 0")
            ET.SubElement(j_el, "axis", xyz=_vec(m.jnt_axis[j]))
            lo, hi = (m.jnt_range[j] if m.jnt_limited[j] else (-np.pi, np.pi))
            ET.SubElement(j_el, "limit", lower=f"{lo:.9g}", upper=f"{hi:.9g}",
                          effort="10", velocity=f"{vel_max:.9g}")
            link(child, b if child == bname else None)
            prev = child
    return ET.tostring(robot, encoding="unicode"), name(mujoco.mjtObj.mjOBJ_BODY, root)


def _under(m, b: int, root: int) -> bool:
    while b > 0:
        if b == root:
            return True
        b = int(m.body_parentid[b])
    return False


# --------------------------------------------------------------------------- human input

def load_mediapipe(path: Path) -> tuple[np.ndarray, float]:
    """(T, 12, 3) keypoints in x-forward / y-left / z-up, and the frame rate."""
    data = np.load(path, allow_pickle=False)
    lm = np.asarray(data["landmarks"], dtype=np.float64)
    if lm.ndim != 3 or lm.shape[1] < 33 or lm.shape[2] < 3:
        raise SystemExit(f"{path}: landmarks must be (T, 33, 3), got {lm.shape}")
    fps = float(data["fps"]) if "fps" in data.files else 30.0
    kp = lm[:, MEDIAPIPE_INDEX, :3] @ MEDIAPIPE_TO_FLU.T
    return kp, fps


def load_bvh_keypoints(path: Path) -> tuple[np.ndarray, float]:
    from frax.ik import load_bvh

    poses, names = load_bvh(path, BVH_Y_UP_TO_Z_UP)
    missing = [b for b in BVH_BONES if b not in names]
    if missing:
        raise SystemExit(f"{path}: BVH lacks bones {missing}")
    kp = poses[:, [names.index(b) for b in BVH_BONES], :3]
    if np.median(np.linalg.norm(kp[:, K["hip_r"]] - kp[:, K["ankle_r"]], axis=-1)) > 5.0:
        kp = kp * 0.01  # centimetre BVH (LAFAN1)
    fps = 30.0
    for line in Path(path).read_text(errors="ignore").splitlines():
        if line.strip().startswith("Frame Time:"):
            fps = 1.0 / float(line.split(":")[1])
            break
    return kp, fps


def smooth(kp: np.ndarray, fps: float, sigma_s: float) -> np.ndarray:
    if sigma_s <= 0.0:
        return kp
    from scipy.ndimage import gaussian_filter1d

    return gaussian_filter1d(kp, sigma=sigma_s * fps, axis=0, mode="nearest")


def trunk_frames(kp: np.ndarray) -> np.ndarray:
    """(T, 3, 3) human trunk rotation (columns: forward, left, up) in the keypoint world."""
    hips = 0.5 * (kp[:, K["hip_r"]] + kp[:, K["hip_l"]])
    sh = 0.5 * (kp[:, K["shoulder_r"]] + kp[:, K["shoulder_l"]])
    up = _unit(sh - hips)
    left = (kp[:, K["hip_l"]] - kp[:, K["hip_r"]]) + (kp[:, K["shoulder_l"]] - kp[:, K["shoulder_r"]])
    left = _unit(left - np.sum(left * up, axis=-1, keepdims=True) * up)
    fwd = np.cross(left, up)
    return np.stack([fwd, left, up], axis=-1)


def _unit(v: np.ndarray) -> np.ndarray:
    return v / np.maximum(np.linalg.norm(v, axis=-1, keepdims=True), 1e-9)


def remove_heading(kp: np.ndarray) -> np.ndarray:
    """In place: hip midpoint over the origin, heading of the trunk along +x, every frame."""
    R = trunk_frames(kp)
    yaw = np.arctan2(R[:, 1, 0], R[:, 0, 0])
    c, s = np.cos(-yaw), np.sin(-yaw)
    Rz = np.zeros((len(kp), 3, 3))
    Rz[:, 0, 0], Rz[:, 0, 1], Rz[:, 1, 0], Rz[:, 1, 1], Rz[:, 2, 2] = c, -s, s, c, 1.0
    hips = 0.5 * (kp[:, K["hip_r"]] + kp[:, K["hip_l"]])
    centre = hips.copy()
    centre[:, 2] = 0.0
    return np.einsum("tij,tkj->tki", Rz, kp - centre[:, None, :])


# --------------------------------------------------------------------------- robot side

@dataclass
class RobotMap:
    """frax joint indices of the tracked frames and the robot's own dimensions at q0."""
    names: list[str]
    hip_r: str
    hip_l: str
    shoulder_r: str
    shoulder_l: str
    elbow_r: str
    elbow_l: str
    knee_r: str
    knee_l: str
    ankle_r: str
    ankle_l: str


def humanoid_map(joint_names: list[str]) -> RobotMap:
    def first(*cands: str) -> str:
        for c in cands:
            if c in joint_names:
                return c
        raise SystemExit(f"robot has none of {cands}: retarget_frax needs a humanoid joint layout")

    sides = {}
    for s, side in (("r", "right"), ("l", "left")):
        sides[f"hip_{s}"] = first(f"{side}_hip_roll", f"{side}_hip_pitch", f"{side}_hip_yaw")
        sides[f"shoulder_{s}"] = first(f"{side}_shoulder_roll", f"{side}_shoulder_pitch")
        sides[f"elbow_{s}"] = first(f"{side}_elbow")
        sides[f"knee_{s}"] = first(f"{side}_knee")
        sides[f"ankle_{s}"] = first(f"{side}_ankle_roll", f"{side}_ankle_pitch")
    return RobotMap(names=list(joint_names), **sides)


def hand_offsets(mjcf: Path, elbow_joints: tuple[str, str], q0: dict[str, float]) -> np.ndarray:
    """(2, 4, 4) hand tip in each elbow joint frame: the forearm geometry point farthest from
    the elbow, at q0."""
    import mujoco

    m = mujoco.MjModel.from_xml_path(str(mjcf))
    d = mujoco.MjData(m)
    for jn, v in q0.items():
        jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, jn)
        if jid >= 0:
            d.qpos[m.jnt_qposadr[jid]] = v
    mujoco.mj_kinematics(m, d)
    out = np.tile(np.eye(4), (2, 1, 1))
    for k, jn in enumerate(elbow_joints):
        jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, jn)
        body = int(m.jnt_bodyid[jid])
        Rb, pb = d.xmat[body].reshape(3, 3), d.xpos[body]
        best, best_d = pb.copy(), 0.0
        for g in range(m.ngeom):
            if not _under(m, int(m.geom_bodyid[g]), body):
                continue
            c, h = m.geom_aabb[g, :3], m.geom_aabb[g, 3:]
            Rg, pg = d.geom_xmat[g].reshape(3, 3), d.geom_xpos[g]
            for sx in (-1, 1):
                for sy in (-1, 1):
                    for sz in (-1, 1):
                        p = pg + Rg @ (c + h * np.array([sx, sy, sz]))
                        dist = float(np.linalg.norm(p - pb))
                        if dist > best_d:
                            best, best_d = p, dist
        out[k, :3, 3] = Rb.T @ (best - pb)
    return out


# --------------------------------------------------------------------------- retarget

def swing(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """(T, 3, 3) shortest rotations taking unit vector a (3,) onto each b (T, 3). Near the
    antipode the axis of the previous frame is kept, so an arm passing overhead does not flip."""
    out = np.zeros((len(b), 3, 3))
    prev_axis = np.cross(a, [1.0, 0.0, 0.0])
    if np.linalg.norm(prev_axis) < 1e-6:
        prev_axis = np.cross(a, [0.0, 1.0, 0.0])
    prev_axis = prev_axis / np.linalg.norm(prev_axis)
    for t, bt in enumerate(_unit(b)):
        axis = np.cross(a, bt)
        s, c = float(np.linalg.norm(axis)), float(np.clip(np.dot(a, bt), -1.0, 1.0))
        if s > 1e-3:
            axis = axis / s
            prev_axis = axis
        else:
            axis = prev_axis
        ang = math.atan2(s, c)
        x, y, z = axis
        Kx = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
        out[t] = np.eye(3) + math.sin(ang) * Kx + (1.0 - math.cos(ang)) * (Kx @ Kx)
    return out


def build_targets(kp: np.ndarray, fk0: dict[str, np.ndarray], R_trunk0: np.ndarray,
                  free_feet: bool) -> tuple[np.ndarray, np.ndarray]:
    """Per-frame world targets for the tracked robot frames.

    Returns positions (T, 7, 3) and rotations (T, 7, 3, 3) for, in order:
    root, elbow_r, elbow_l, hand_r, hand_l, ankle_r, ankle_l.

    Arms map rotations, not positions: each human segment's swing away from hanging straight
    down (in the trunk frame) is applied to the robot's own segment at q0. A small robot rarely
    rests like a human (Microban's upper arm leans 40° out at q0), and absolute position
    targets would pull it onto the wrong IK branch."""
    T = len(kp)
    Rh = trunk_frames(kp)
    h_leg = np.median(np.linalg.norm(kp[:, K["hip_r"]] - kp[:, K["knee_r"]], axis=-1)
                      + np.linalg.norm(kp[:, K["knee_r"]] - kp[:, K["ankle_r"]], axis=-1))
    r_leg = (np.linalg.norm(fk0["hip_r"] - fk0["knee_r"]) + np.linalg.norm(fk0["knee_r"] - fk0["ankle_r"]))
    leg_s = r_leg / max(h_leg, 1e-6)

    hips_h = 0.5 * (kp[:, K["hip_r"]] + kp[:, K["hip_l"]])
    ank_h = 0.5 * (kp[:, K["ankle_r"]] + kp[:, K["ankle_l"]])
    hips_r0 = 0.5 * (fk0["hip_r"] + fk0["hip_l"])
    ank_r0 = 0.5 * (fk0["ankle_r"] + fk0["ankle_l"])
    # human stance height (hip over ankles) at full extension maps to the robot's q0 stance
    stand_h = np.percentile(hips_h[:, 2] - ank_h[:, 2], 95)
    hips_t = np.zeros((T, 3))
    # sway and lean are the human's departures from their own mean stance, on top of q0's
    rel_xy = hips_h[:, :2] - ank_h[:, :2]
    hips_t[:, :2] = hips_r0[:2] + leg_s * (rel_xy - np.median(rel_xy, axis=0))
    hips_t[:, 2] = (hips_r0[2] - ank_r0[2]) + leg_s * ((hips_h[:, 2] - ank_h[:, 2]) - stand_h) + ank_r0[2]

    # trunk orientation: the human trunk relative to its own mean upright pose, applied on q0's
    R_root = np.einsum("tij,jk->tik", Rh, R_trunk0)
    root_off = R_trunk0.T @ (fk0["root"] - hips_r0)
    root_t = hips_t + np.einsum("tij,j->ti", R_root, root_off)

    def implied(point: str) -> np.ndarray:
        """Robot frame position given the root target (rigid trunk at its q0 offset)."""
        off = R_trunk0.T @ (fk0[point] - fk0["root"])
        return root_t + np.einsum("tij,j->ti", R_root, off)

    pos = np.zeros((T, 7, 3))
    rot = np.tile(np.eye(3), (T, 7, 1, 1))
    pos[:, 0], rot[:, 0] = root_t, R_root
    down = np.array([0.0, 0.0, -1.0])
    for s, ie, ih in (("r", 1, 3), ("l", 2, 4)):
        sh = implied(f"shoulder_{s}")
        # human segment directions in the human trunk frame; robot segments at q0 in its trunk frame
        upper = np.einsum("tji,tj->ti", Rh, kp[:, K[f"elbow_{s}"]] - kp[:, K[f"shoulder_{s}"]])
        lower = np.einsum("tji,tj->ti", Rh, kp[:, K[f"wrist_{s}"]] - kp[:, K[f"elbow_{s}"]])
        r_upper = R_trunk0.T @ (fk0[f"elbow_{s}"] - fk0[f"shoulder_{s}"])
        r_lower = R_trunk0.T @ (fk0[f"hand_{s}"] - fk0[f"elbow_{s}"])
        up_tr = np.einsum("tij,j->ti", swing(down, upper), r_upper)
        lo_tr = np.einsum("tij,j->ti", swing(down, lower), r_lower)
        pos[:, ie] = sh + np.einsum("tij,tj->ti", R_root, up_tr)
        pos[:, ih] = pos[:, ie] + np.einsum("tij,tj->ti", R_root, lo_tr)
    for s, ia in (("r", 5), ("l", 6)):
        if free_feet:
            ank = ank_r0 + leg_s * (kp[:, K[f"ankle_{s}"]] - ank_h) + (fk0[f"ankle_{s}"] - ank_r0)
            ank[:, 2] = np.maximum(fk0[f"ankle_{s}"][2] + leg_s * (kp[:, K[f"ankle_{s}"], 2] - ank_h[:, 2]),
                                   fk0[f"ankle_{s}"][2])
        else:
            ank = np.broadcast_to(fk0[f"ankle_{s}"], (T, 3)).copy()
        pos[:, ia] = ank
        rot[:, ia] = fk0[f"ankle_{s}_rot"]
    print(f"leg scale {leg_s:.3f} (robot / human); arms follow segment rotations")
    return pos, rot


def retarget(args) -> Path:
    import jax
    import jax.numpy as jnp
    import frax
    from frax.ik import DiffIKSolver, FrameTask, JointLimits, JointTask, Retargeter
    from frax.utils.rotation_utils import rmat_to_quat_wxyz

    profile = load_profile(args.robot)
    names = list(profile["joint_names"])
    q0 = np.asarray(profile["q0"], dtype=np.float64)
    mjcf = Path(args.mjcf) if args.mjcf else resolve_mjcf(args.robot, profile)
    if mjcf is None or not mjcf.is_file():
        raise SystemExit(f"no MJCF for {args.robot}: run tools/robots/fetch_upstream.py --robot {args.robot}")
    import yaml

    ppo_path = robot_dir(args.robot) / "robot" / "ppo.yaml"
    ppo = yaml.safe_load(ppo_path.read_text()) if ppo_path.is_file() else {}
    act_max = float((ppo or {}).get("env", {}).get("act_max", DEFAULT_ACT_MAX))

    urdf, root_link = mjcf_to_urdf(mjcf, names, args.vel_max)
    with tempfile.NamedTemporaryFile("w", suffix=".urdf", delete=False) as f:
        f.write(urdf)
        urdf_path = f.name
    robot = frax.Robot(urdf_path, floating_base="quaternion")
    act_names = list(robot.joint_names[robot.nv_floating:])
    if sorted(act_names) != sorted(names):
        raise SystemExit(f"URDF joints {act_names} differ from the profile {names}")
    perm = np.array([act_names.index(n) for n in names])  # profile order -> frax column
    rmap = humanoid_map(names)

    # robot dimensions at q0, standing with the root at the origin
    q_init = np.array(robot.default_configuration, dtype=np.float64)
    q_init[:7] = [0, 0, 0, 1, 0, 0, 0]
    q_init[7 + perm] = q0
    fk_jit = jax.jit(robot.joint_to_world_transforms)
    tfs = np.asarray(fk_jit(jnp.asarray(q_init)))
    jidx = robot.joint_name_to_index
    # the trunk frame is the last of frax's six virtual floating-base joints
    root_j = robot.nv_floating - 1
    hand_off = hand_offsets(mjcf, (rmap.elbow_r, rmap.elbow_l), dict(zip(names, q0)))
    fk0 = {"root": tfs[root_j][:3, 3]}
    for key in ("hip_r", "hip_l", "shoulder_r", "shoulder_l", "elbow_r", "elbow_l",
                "knee_r", "knee_l", "ankle_r", "ankle_l"):
        fk0[key] = tfs[jidx[getattr(rmap, key)]][:3, 3]
    for s, k in (("r", 0), ("l", 1)):
        fk0[f"hand_{s}"] = (tfs[jidx[getattr(rmap, f"elbow_{s}")]] @ hand_off[k])[:3, 3]
        fk0[f"ankle_{s}_rot"] = tfs[jidx[getattr(rmap, f"ankle_{s}")]][:3, :3]
    R_trunk0 = tfs[root_j][:3, :3]

    if args.mediapipe:
        kp, fps = load_mediapipe(Path(args.mediapipe))
    else:
        kp, fps = load_bvh_keypoints(Path(args.bvh))
    a = int(round(args.start * fps)) if args.start else 0
    b = int(round(args.end * fps)) if args.end else len(kp)
    kp = kp[a:b]
    if len(kp) < 4:
        raise SystemExit(f"clip too short: {len(kp)} frames")
    kp = smooth(remove_heading(kp), fps, args.smooth)
    if args.loop_blend > 0.0:
        # blend the human end into its start before the IK, so the closing frames also respect
        # the joint and speed limits
        n = min(int(round(args.loop_blend * fps)), len(kp) // 2)
        w = (0.5 - 0.5 * np.cos(np.linspace(0.0, np.pi, n)))[:, None, None]
        kp[-n:] = (1.0 - w) * kp[-n:] + w * kp[0]
    pos, rot = build_targets(kp, fk0, R_trunk0, args.free_feet)

    joints = [root_j, jidx[rmap.elbow_r], jidx[rmap.elbow_l], jidx[rmap.elbow_r], jidx[rmap.elbow_l],
              jidx[rmap.ankle_r], jidx[rmap.ankle_l]]
    offsets = np.tile(np.eye(4), (7, 1, 1))
    offsets[3], offsets[4] = hand_off[0], hand_off[1]
    w_pos = np.array([args.w_root, args.w_hand, args.w_hand, args.w_hand, args.w_hand, 100.0, 100.0])
    w_rot = np.array([args.w_root, 0.0, 0.0, 0.0, 0.0, 10.0, 10.0])
    gain = fps
    task = FrameTask(robot, joints, gain_pos=gain, gain_rot=gain, weight_pos=w_pos, weight_rot=w_rot,
                     offsets=offsets)
    q0_frax = q0[np.argsort(perm)]
    # weak pull toward q0 for the directions no target fixes (hip yaw, arm twist)
    posture = JointTask(robot, np.arange(robot.nv_floating, robot.nv), "q_posture",
                        gain=gain, weight=args.w_posture)
    lo = np.array(robot.actuated_joint_lower_limits, dtype=np.float64)
    hi = np.array(robot.actuated_joint_upper_limits, dtype=np.float64)
    lo = np.maximum(lo, q0_frax - act_max)
    hi = np.minimum(hi, q0_frax + act_max)
    if not args.allow_hyperextension:
        for kn, ank in ((rmap.knee_r, rmap.ankle_r), (rmap.knee_l, rmap.ankle_l)):
            i = act_names.index(kn)
            # bending the knee raises the hanging foot backward (-x of the trunk), like a human's
            dq = np.zeros(robot.nq)
            dq[7 + i] = 1e-3
            back = -(np.asarray(fk_jit(jnp.asarray(q_init + dq)))[jidx[ank], :3, 3]
                     - tfs[jidx[ank]][:3, 3]) @ R_trunk0[:, 0]
            if back > 0:
                lo[i] = max(lo[i], q0_frax[i] - 0.05)
            else:
                hi[i] = min(hi[i], q0_frax[i] + 0.05)
    limits = JointLimits(robot, q_min=lo, q_max=hi, gain=gain)
    solver = DiffIKSolver(robot, [task, posture], [limits], regularization=np.full(robot.nv, args.damping))

    def initial_q(frame):
        return jnp.asarray(q_init).at[:3].set(frame["frame_pos"][0]).at[3:7].set(
            rmat_to_quat_wxyz(frame["frame_rmat"][0]))

    rt = Retargeter(solver, preprocess=lambda frame: frame, initial_q=initial_q)
    T = len(pos)
    periods = 2 if args.loop_blend > 0.0 else 1
    # a periodic clip is solved twice in a row and the second pass kept: the IK lag behind a
    # speed-limited target has settled when the loop closes
    frames = {"frame_pos": jnp.asarray(np.tile(pos, (periods, 1, 1))),
              "frame_rmat": jnp.asarray(np.tile(rot, (periods, 1, 1, 1))),
              "q_posture": jnp.broadcast_to(jnp.asarray(q0_frax), (periods * T, len(q0_frax)))}
    t0 = time.perf_counter()
    run = jax.jit(lambda fr: rt.retarget_clip(fr, 1.0 / fps)[0])
    qs = np.asarray(run(frames))
    t_first = time.perf_counter() - t0
    t0 = time.perf_counter()
    qs = np.asarray(run(frames))
    t_solve = time.perf_counter() - t0
    print(f"retargeted {len(qs)} frames at {fps:.0f} fps: compile+solve {t_first:.1f} s, "
          f"solve {t_solve * 1e3:.1f} ms ({len(qs) / max(t_solve, 1e-9):,.0f} frames/s)")
    qs = qs[-T:]

    q = qs[:, 7:][:, perm]  # profile joint order, absolute radians
    root_quat = qs[:, 3:7]
    root_z = qs[:, 2]
    err = _tracking_error(robot, qs, pos, joints, offsets)
    lim = np.max(np.abs(q - q0)) if len(q) else 0.0
    print(f"tracking error (median / 95%): hands {err['hand'][0] * 1e3:.1f} / {err['hand'][1] * 1e3:.1f} mm, "
          f"feet {err['ankle'][0] * 1e3:.1f} / {err['ankle'][1] * 1e3:.1f} mm; max |q - q0| {lim:.2f} rad "
          f"(act_max {act_max})")
    vel = np.max(np.abs(np.diff(q, axis=0))) * fps if len(q) > 1 else 0.0
    print(f"peak joint speed {vel:.2f} rad/s (limit {args.vel_max})")

    out = Path(args.out) if args.out else robot_dir(args.robot) / "motions" / f"{args.name}.npz"
    out.parent.mkdir(parents=True, exist_ok=True)
    np.savez(out, q=q.astype(np.float32), joint_names=np.array(names), fps=float(fps),
             duration_s=float(len(q) / fps), phase=np.linspace(0.0, 1.0, len(q)),
             root_quat=root_quat.astype(np.float32), root_z=root_z.astype(np.float32),
             oneshot=bool(args.oneshot), source=str(args.mediapipe or args.bvh), retarget="frax.ik")
    print(f"wrote {out}")
    os.unlink(urdf_path)
    return out


def _tracking_error(robot, qs, pos, joints, offsets) -> dict[str, tuple[float, float]]:
    import jax

    tfs = np.asarray(jax.jit(jax.vmap(robot.joint_to_world_transforms))(qs))
    j = np.asarray(joints)
    T = tfs[:, j] @ offsets[None]
    e = np.linalg.norm(T[:, :, :3, 3] - pos, axis=-1)
    stats = lambda x: (float(np.median(x)), float(np.percentile(x, 95)))  # noqa: E731
    return {"hand": stats(e[:, 3:5]), "ankle": stats(e[:, 5:7])}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--robot", required=True)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--mediapipe", help=".npz with landmarks (T, 33, 3) world landmarks and fps")
    src.add_argument("--bvh", help="BVH mocap file with LAFAN1/CMU bone names")
    ap.add_argument("--name", default="motion_frax", help="clip name: robots/<id>/motions/<name>.npz")
    ap.add_argument("--out", default=None, help="output .npz (overrides --name)")
    ap.add_argument("--mjcf", default=None, help="robot MJCF (default: the training model)")
    ap.add_argument("--start", type=float, default=0.0, help="clip start (s)")
    ap.add_argument("--end", type=float, default=0.0, help="clip end (s, 0 = end of file)")
    ap.add_argument("--smooth", type=float, default=0.05, help="zero-lag Gaussian smoothing sigma (s)")
    ap.add_argument("--free-feet", action="store_true", help="let the feet follow the human (default: q0 stance)")
    ap.add_argument("--loop-blend", type=float, default=0.0,
                    help="blend the last N seconds into the first frame, for a periodic clip")
    ap.add_argument("--oneshot", action="store_true", help="mark the clip one-shot (clock θ = π·t/T)")
    ap.add_argument("--vel-max", type=float, default=6.0, help="servo speed limit (rad/s)")
    ap.add_argument("--damping", type=float, default=1e-3, help="velocity regularisation")
    ap.add_argument("--w-root", type=float, default=100.0, help="trunk pose weight")
    ap.add_argument("--allow-hyperextension", action="store_true",
                    help="let the knees straighten past q0 (default: at most 0.05 rad)")
    ap.add_argument("--w-hand", type=float, default=10.0, help="elbow and hand position weight")
    ap.add_argument("--w-posture", type=float, default=0.05, help="pull toward q0 (redundant joints)")
    retarget(ap.parse_args())


if __name__ == "__main__":
    main()
