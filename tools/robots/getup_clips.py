# Author: Roberto Navoni, member of the ArduPilot Dev Team
# Contact: r.navoni74@gmail.com
# Developed by Roberto Navoni — DelphyAI LAB
"""Keyframe clips for Microban get-up (prone and supine → stand at q0).

One-shot clips: phase in [0, 1] advances once over CLIP_DURATION_S. The last
keyframe of every joint is exactly q0 so the handoff to walk / pose_cmd is
seamless. Flat-foot squat ratios match pose_cmd / dance (POSE_KNEE_HIP,
POSE_KNEE_ANKLE). Root (quat, z) keyframes drive Reference State Init and the
clip-check visualiser; the policy only sees joints + gravity + the oneshot clock.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

import numpy as np

# Duration of one get-up attempt (firmware NNM_GETUP_S, training episode clock).
CLIP_DURATION_S = 4.0
# Episode timeout after the clip finishes (standing hold).
EPISODE_S = 6.0
# Trunk height when lying flat (settled on the Microban MJCF).
LIE_Z = 0.038
HOME_Z = 0.168

# Flat-foot ballet ratios reused from nnm_env / pose_cmd.
POSE_KNEE_HIP = -0.48
POSE_KNEE_ANKLE = -0.56

# Microban q0 (must match robots/microban/robot/profile.json).
Q0 = {
    "right_shoulder_pitch": 0.0,
    "right_shoulder_roll": -0.174533,
    "right_elbow": -0.349066,
    "right_hip_yaw": 0.0,
    "right_hip_roll": -0.087266,
    "right_hip_pitch": -0.174533,
    "right_knee": 0.0,
    "right_ankle_pitch": 0.0,
    "right_ankle_roll": 0.087266,
    "left_shoulder_pitch": 0.0,
    "left_shoulder_roll": 0.174533,
    "left_elbow": -0.349066,
    "left_hip_yaw": 0.0,
    "left_hip_roll": 0.087266,
    "left_hip_pitch": -0.174533,
    "left_knee": 0.0,
    "left_ankle_pitch": 0.0,
    "left_ankle_roll": -0.087266,
}

JOINT_NAMES = list(Q0.keys())

# Soft action budget: |q - q0| must stay within act_max (firmware NNM_ACT_MAX).
ACT_MAX = 2.0

# Joint hard ranges from the Microban MJCF (right/left mirrored where needed).
JOINT_LIMITS = {
    "right_shoulder_pitch": (-3.14, 3.14),
    "right_shoulder_roll": (-3.14, 0.0),
    "right_elbow": (-2.18, 2.18),
    "right_hip_yaw": (-4.19, 1.05),
    "right_hip_roll": (-0.61, 0.61),
    "right_hip_pitch": (-1.57, 1.57),
    "right_knee": (-0.79, 2.36),
    "right_ankle_pitch": (-1.57, 0.61),
    "right_ankle_roll": (-0.61, 0.61),
    "left_shoulder_pitch": (-3.14, 3.14),
    "left_shoulder_roll": (0.0, 3.14),
    "left_elbow": (-2.18, 2.18),
    "left_hip_yaw": (-1.05, 4.19),
    "left_hip_roll": (-0.61, 0.61),
    "left_hip_pitch": (-1.57, 1.57),
    "left_knee": (-0.79, 2.36),
    "left_ankle_pitch": (-1.57, 0.61),
    "left_ankle_roll": (-0.61, 0.61),
}


def _squat(knee: float) -> tuple[float, float, float]:
    """knee, hip_pitch, ankle_pitch for a flat-foot squat around q0."""
    return (
        knee,
        Q0["right_hip_pitch"] + POSE_KNEE_HIP * knee,
        Q0["right_ankle_pitch"] + POSE_KNEE_ANKLE * knee,
    )


def _legs(knee: float, hip_pitch: float | None = None, ankle: float | None = None,
          hip_roll_r: float | None = None, hip_yaw: float = 0.0) -> dict[str, list[tuple[float, float]]]:
    """Symmetric leg keyframes at a single phase (filled later by the builder)."""
    hp = Q0["right_hip_pitch"] + POSE_KNEE_HIP * knee if hip_pitch is None else hip_pitch
    ap = Q0["right_ankle_pitch"] + POSE_KNEE_ANKLE * knee if ankle is None else ankle
    hr_r = Q0["right_hip_roll"] if hip_roll_r is None else hip_roll_r
    hr_l = -hr_r if hip_roll_r is not None else Q0["left_hip_roll"]
    ar_r = Q0["right_ankle_roll"] if hip_roll_r is None else -hr_r
    ar_l = Q0["left_ankle_roll"] if hip_roll_r is None else hr_r
    return {
        "right_hip_yaw": hip_yaw, "left_hip_yaw": -hip_yaw if hip_yaw else 0.0,
        "right_hip_roll": hr_r, "left_hip_roll": hr_l if hip_roll_r is not None else Q0["left_hip_roll"],
        "right_hip_pitch": hp, "left_hip_pitch": hp,
        "right_knee": knee, "left_knee": knee,
        "right_ankle_pitch": ap, "left_ankle_pitch": ap,
        "right_ankle_roll": ar_r if hip_roll_r is not None else Q0["right_ankle_roll"],
        "left_ankle_roll": ar_l if hip_roll_r is not None else Q0["left_ankle_roll"],
    }


def _arms(sp: float, sr_r: float, el: float) -> dict[str, float]:
    return {
        "right_shoulder_pitch": sp, "left_shoulder_pitch": sp,
        "right_shoulder_roll": sr_r, "left_shoulder_roll": -sr_r,
        "right_elbow": el, "left_elbow": el,
    }


def _pose_at(phase: float, arms: dict[str, float], legs: dict[str, float]) -> dict[str, float]:
    out = {**Q0}
    out.update(arms)
    out.update(legs)
    return out


def _build_keys(phases: list[tuple[float, dict[str, float]]]) -> dict[str, list[tuple[float, float]]]:
    keys: dict[str, list[tuple[float, float]]] = {n: [] for n in JOINT_NAMES}
    for ph, pose in phases:
        for n in JOINT_NAMES:
            keys[n].append((ph, float(pose[n])))
    return keys


# ---------------------------------------------------------------------------
# Prone: belly-down push → tuck → flat-foot squat → stand
# Root: pitch +90° (FLU), g_body ≈ [+1,0,0]
# ---------------------------------------------------------------------------
_k_lie, _hp_lie, _ap_lie = _squat(0.15)
_k_plant = 0.35
_k_push = 1.55
_k_tuck = 1.95
_k_squat = 1.25
_k_rise = 0.45

_prone_phases = [
    # resting on belly, arms along the body
    (0.00, _pose_at(0.0,
                    _arms(0.05, -0.25, -0.40),
                    {**_legs(0.15), "right_hip_pitch": -0.35, "left_hip_pitch": -0.35,
                     "right_ankle_pitch": -0.05, "left_ankle_pitch": -0.05})),
    # hands under the shoulders
    (0.12, _pose_at(0.12,
                    _arms(-1.35, -0.35, -1.55),
                    {**_legs(0.40), "right_hip_pitch": -0.55, "left_hip_pitch": -0.55,
                     "right_ankle_pitch": -0.20, "left_ankle_pitch": -0.20})),
    # push-up: arms extend, hips stay low
    (0.28, _pose_at(0.28,
                    _arms(-1.15, -0.30, -0.45),
                    {**_legs(1.40), "right_hip_pitch": -1.05, "left_hip_pitch": -1.05,
                     "right_ankle_pitch": -0.70, "left_ankle_pitch": -0.70})),
    # knees under hips
    (0.45, _pose_at(0.45,
                    _arms(-0.85, -0.25, -0.55),
                    {**_legs(_k_tuck), "right_hip_pitch": -1.25, "left_hip_pitch": -1.25,
                     "right_ankle_pitch": -0.95, "left_ankle_pitch": -0.95})),
    # weight on the feet, flat-foot squat, arms for balance
    (0.60, _pose_at(0.60,
                    _arms(-0.55, -0.20, -0.65),
                    {**_legs(_k_squat)})),
    # rising
    (0.80, _pose_at(0.80,
                    _arms(-0.20, -0.18, -0.45),
                    {**_legs(_k_rise)})),
    # stand = q0
    (1.00, dict(Q0)),
]

# Root trajectory for prone RSI / visualisation: (phase, quat_wxyz, z)
# pitch +π/2 → [cos(π/4), 0, sin(π/4), 0] = belly down
_PRONE_ROOT = [
    (0.00, (0.7071, 0.0, 0.7071, 0.0), LIE_Z),
    (0.12, (0.7071, 0.0, 0.7071, 0.0), LIE_Z + 0.01),
    (0.28, (0.7934, 0.0, 0.6088, 0.0), 0.06),   # ~pitch 75°
    (0.45, (0.8870, 0.0, 0.4617, 0.0), 0.09),   # ~pitch 55°
    (0.60, (0.9659, 0.0, 0.2588, 0.0), 0.11),   # ~pitch 30°
    (0.80, (0.9914, 0.0, 0.1305, 0.0), 0.14),   # ~pitch 15°
    (1.00, (1.0, 0.0, 0.0, 0.0), HOME_Z),
]

# ---------------------------------------------------------------------------
# Supine: on the back → rock with arms as counterweight → squat → stand
# (XL330 torque is limited; the rock uses the arms and a tucked posture.
#  If dynamics fail the clip-check, a roll-to-prone fallback is used.)
# Root: pitch −π/2 → [cos(π/4), 0, −sin(π/4), 0], g_body ≈ [−1,0,0]
# ---------------------------------------------------------------------------
_supine_phases = [
    # on the back, knees soft, arms ready
    (0.00, _pose_at(0.0,
                    _arms(0.10, -0.20, -0.30),
                    {**_legs(0.35), "right_hip_pitch": -0.45, "left_hip_pitch": -0.45,
                     "right_ankle_pitch": -0.15, "left_ankle_pitch": -0.15})),
    # arms swing forward as counterweight, knees pull in
    (0.18, _pose_at(0.18,
                    _arms(-1.45, -0.40, -0.90),
                    {**_legs(1.70), "right_hip_pitch": -1.20, "left_hip_pitch": -1.20,
                     "right_ankle_pitch": -0.85, "left_ankle_pitch": -0.85})),
    # rock: trunk pitches up (root keyframes), feet coming under
    (0.35, _pose_at(0.35,
                    _arms(-1.20, -0.35, -0.70),
                    {**_legs(1.85), "right_hip_pitch": -1.30, "left_hip_pitch": -1.30,
                     "right_ankle_pitch": -0.95, "left_ankle_pitch": -0.95})),
    # weight on the feet
    (0.50, _pose_at(0.50,
                    _arms(-0.70, -0.25, -0.60),
                    {**_legs(_k_squat + 0.15)})),
    (0.65, _pose_at(0.65,
                    _arms(-0.45, -0.20, -0.55),
                    {**_legs(_k_squat)})),
    (0.82, _pose_at(0.82,
                    _arms(-0.15, -0.18, -0.42),
                    {**_legs(_k_rise)})),
    (1.00, dict(Q0)),
]

_SUPINE_ROOT = [
    (0.00, (0.7071, 0.0, -0.7071, 0.0), LIE_Z),
    (0.18, (0.7934, 0.0, -0.6088, 0.0), 0.05),  # ~pitch -75°
    (0.35, (0.8870, 0.0, -0.4617, 0.0), 0.08),
    (0.50, (0.9659, 0.0, -0.2588, 0.0), 0.11),
    (0.65, (0.9808, 0.0, -0.1951, 0.0), 0.12),
    (0.82, (0.9914, 0.0, -0.1305, 0.0), 0.145),
    (1.00, (1.0, 0.0, 0.0, 0.0), HOME_Z),
]

# Roll-to-prone fallback for supine when the sit-up is not feasible with XL330.
# Asymmetric roll onto the right side, then the prone push sequence.
_supine_roll_phases = [
    (0.00, _pose_at(0.0,
                    {"right_shoulder_pitch": 0.10, "left_shoulder_pitch": 0.10,
                     "right_shoulder_roll": -0.15, "left_shoulder_roll": 0.55,
                     "right_elbow": -0.30, "left_elbow": -0.80},
                    {**_legs(0.40, hip_yaw=0.25)})),
    # roll onto the right side
    (0.18, _pose_at(0.18,
                    {"right_shoulder_pitch": -0.40, "left_shoulder_pitch": -1.10,
                     "right_shoulder_roll": -0.10, "left_shoulder_roll": 1.10,
                     "right_elbow": -0.50, "left_elbow": -1.20},
                    {**_legs(0.80, hip_yaw=0.55, hip_roll_r=-0.35)})),
    # arrive prone
    (0.32, _pose_at(0.32,
                    _arms(-1.30, -0.35, -1.50),
                    {**_legs(0.50), "right_hip_pitch": -0.60, "left_hip_pitch": -0.60,
                     "right_ankle_pitch": -0.25, "left_ankle_pitch": -0.25})),
    # from here: same as prone push → stand (phases remapped into [0.32, 1])
    (0.45, _pose_at(0.45,
                    _arms(-1.15, -0.30, -0.45),
                    {**_legs(1.40), "right_hip_pitch": -1.05, "left_hip_pitch": -1.05,
                     "right_ankle_pitch": -0.70, "left_ankle_pitch": -0.70})),
    (0.58, _pose_at(0.58,
                    _arms(-0.85, -0.25, -0.55),
                    {**_legs(_k_tuck), "right_hip_pitch": -1.25, "left_hip_pitch": -1.25,
                     "right_ankle_pitch": -0.95, "left_ankle_pitch": -0.95})),
    (0.72, _pose_at(0.72,
                    _arms(-0.55, -0.20, -0.65),
                    {**_legs(_k_squat)})),
    (0.88, _pose_at(0.88,
                    _arms(-0.20, -0.18, -0.45),
                    {**_legs(_k_rise)})),
    (1.00, dict(Q0)),
]

_SUPINE_ROLL_ROOT = [
    (0.00, (0.7071, 0.0, -0.7071, 0.0), LIE_Z),
    # roll about X toward prone: interpolate via side-lying (roll ≈ 90°)
    (0.18, (0.5, 0.5, -0.5, 0.5), LIE_Z + 0.005),   # approx side
    (0.32, (0.7071, 0.0, 0.7071, 0.0), LIE_Z + 0.01),  # prone
    (0.45, (0.7934, 0.0, 0.6088, 0.0), 0.06),
    (0.58, (0.8870, 0.0, 0.4617, 0.0), 0.09),
    (0.72, (0.9659, 0.0, 0.2588, 0.0), 0.11),
    (0.88, (0.9914, 0.0, 0.1305, 0.0), 0.14),
    (1.00, (1.0, 0.0, 0.0, 0.0), HOME_Z),
]


# ---------------------------------------------------------------------------
# Physically stable waypoints (settled 2.5 s on the BAM XL330 model, root pitch in rad,
# positive = leaning forward). Used for Reference State Initialization: the policy
# learns each stage from a state it can actually hold. `progress` feeds the one-shot
# clock so the phase channel stays consistent with the stage of the get-up.
# ---------------------------------------------------------------------------
def _leg(h: float, k: float, a: float) -> dict[str, float]:
    return {"right_hip_pitch": h, "right_knee": k, "right_ankle_pitch": a,
            "left_hip_pitch": h, "left_knee": k, "left_ankle_pitch": a}


def _arm(sp: float, el: float, sr: float = -0.17) -> dict[str, float]:
    return {"right_shoulder_pitch": sp, "right_elbow": el, "right_shoulder_roll": sr,
            "left_shoulder_pitch": sp, "left_elbow": el, "left_shoulder_roll": -sr}


def _wp(name: str, side: str, pitch: float, progress: float, *parts: dict[str, float],
        weight: float = 1.0) -> dict[str, Any]:
    pose = dict(Q0)
    for p in parts:
        pose.update(p)
    return {"name": name, "side": side, "pitch": pitch, "progress": progress, "pose": pose,
            "weight": weight}


# weight: RSI sampling weight. The stages the policy has not linked yet (seated on heels -> squat,
# frog -> seated, the supine sit-up) are drawn more often than the squats it stands from already.
WAYPOINTS: list[dict[str, Any]] = [
    # prone chain
    _wp("prone_flat", "prone", 1.57, 0.00, _leg(0.0, 0.0, 0.0), _arm(-1.2, -1.4)),
    _wp("pushup", "prone", 1.0, 0.20, _leg(0.55, 0.0, 0.0), _arm(-1.0, -0.9), weight=1.5),
    _wp("frog", "prone", 0.9, 0.40, _leg(-1.57, 1.9, -1.23), _arm(-1.3, -0.3), weight=2.5),
    # supine chain
    _wp("supine_flat", "supine", -1.57, 0.00, _leg(-0.2, 0.3, 0.0), _arm(0.1, -0.3)),
    _wp("supine_knees", "supine", -1.57, 0.15, _leg(-1.2, 2.2, 0.57), _arm(0.1, -0.3), weight=2.0),
    _wp("recline_prop", "supine", -0.8, 0.35, _leg(-1.2, 2.0, 0.0), _arm(0.8, -0.1), weight=2.5),
    # shared: seated on heels -> squats -> stand
    _wp("seat_heels", "both", 0.1, 0.60, _leg(-1.1, 2.3, -1.3), _arm(-0.6, -0.6), weight=3.0),
    _wp("squat_deep", "both", 0.1, 0.72, _leg(-0.86, 1.8, -1.04), _arm(-0.5, -0.6)),
    _wp("squat_mid", "both", 0.3, 0.84, _leg(-0.78, 1.2, -0.72), _arm(-0.3, -0.5), weight=0.5),
    _wp("squat_low", "both", 0.1, 0.93, _leg(-0.19, 0.6, -0.51), _arm(-0.2, -0.4), weight=0.5),
    _wp("stand", "both", 0.0, 1.00),
]

LYING = {"prone": WAYPOINTS[0], "supine": WAYPOINTS[3]}


def waypoint_vector(wp: dict[str, Any]) -> list[float]:
    return [float(wp["pose"][n]) for n in JOINT_NAMES]


def getup_presets(use_supine_roll: bool = True) -> dict[str, Any]:
    """Return prone / supine clip specs for nnm_env and the clip checker."""
    supine_phases = _supine_roll_phases if use_supine_roll else _supine_phases
    supine_root = _SUPINE_ROLL_ROOT if use_supine_roll else _SUPINE_ROOT
    return {
        "getup_prone": {
            "hz": 1.0 / CLIP_DURATION_S,
            "oneshot": True,
            "keys": _build_keys(_prone_phases),
            "root": _PRONE_ROOT,
            "lie_quat": (0.7071, 0.0, 0.7071, 0.0),
            "lie_z": LIE_Z,
        },
        "getup_supine": {
            "hz": 1.0 / CLIP_DURATION_S,
            "oneshot": True,
            "keys": _build_keys(supine_phases),
            "root": supine_root,
            "lie_quat": (0.7071, 0.0, -0.7071, 0.0),
            "lie_z": LIE_Z,
        },
    }


def clip_value(keys: list[tuple[float, float]], phase: float, oneshot: bool = True) -> float:
    """Cosine interpolation; oneshot clamps at the ends instead of wrapping."""
    if len(keys) == 1:
        return keys[0][1]
    phase = float(phase)
    if oneshot:
        if phase <= keys[0][0]:
            return keys[0][1]
        if phase >= keys[-1][0]:
            return keys[-1][1]
        for k in range(len(keys) - 1):
            p0, v0 = keys[k]
            p1, v1 = keys[k + 1]
            if p0 <= phase <= p1:
                span = (p1 - p0) or 1e-9
                t = (phase - p0) / span
                s = 0.5 - 0.5 * math.cos(math.pi * t)
                return v0 + (v1 - v0) * s
        return keys[-1][1]
    # periodic (same convention as nnm_env._clip_value)
    n = len(keys)
    for k in range(n):
        p0, v0 = keys[k]
        p1, v1 = keys[(k + 1) % n]
        span = (p1 - p0) % 1.0 or 1.0
        t = (phase - p0) % 1.0
        if t <= span:
            s = 0.5 - 0.5 * math.cos(math.pi * t / span)
            return v0 + (v1 - v0) * s
    return keys[-1][1]


def root_at(root_keys: list[tuple[float, tuple, float]], phase: float
            ) -> tuple[tuple[float, float, float, float], float]:
    """Interpolate root quaternion (normalised lerp) and height."""
    phase = min(max(float(phase), 0.0), 1.0)
    if phase <= root_keys[0][0]:
        return root_keys[0][1], root_keys[0][2]
    if phase >= root_keys[-1][0]:
        return root_keys[-1][1], root_keys[-1][2]
    for k in range(len(root_keys) - 1):
        p0, q0, z0 = root_keys[k]
        p1, q1, z1 = root_keys[k + 1]
        if p0 <= phase <= p1:
            span = (p1 - p0) or 1e-9
            t = (phase - p0) / span
            s = 0.5 - 0.5 * math.cos(math.pi * t)
            # nlerp with hemisphere fix
            dot = sum(a * b for a, b in zip(q0, q1))
            qb = tuple(-x for x in q1) if dot < 0 else q1
            q = tuple(a + (b - a) * s for a, b in zip(q0, qb))
            n = math.sqrt(sum(x * x for x in q)) or 1.0
            q = tuple(x / n for x in q)
            z = z0 + (z1 - z0) * s
            return q, z
    return root_keys[-1][1], root_keys[-1][2]


def joint_vector(keys: dict[str, list[tuple[float, float]]], phase: float,
                 oneshot: bool = True) -> list[float]:
    return [clip_value(keys[n], phase, oneshot=oneshot) for n in JOINT_NAMES]


def validate_clips(presets: dict[str, Any] | None = None) -> list[str]:
    """Return a list of human-readable problems (empty = ok)."""
    presets = presets or getup_presets()
    problems: list[str] = []
    for name, spec in presets.items():
        keys = spec["keys"]
        for n in JOINT_NAMES:
            if n not in keys:
                problems.append(f"{name}: missing joint {n}")
                continue
            lo, hi = JOINT_LIMITS[n]
            q0 = Q0[n]
            for ph, val in keys[n]:
                if val < lo - 1e-3 or val > hi + 1e-3:
                    problems.append(f"{name}.{n}@{ph:.2f}={val:.3f} outside [{lo:.2f},{hi:.2f}]")
                if abs(val - q0) > ACT_MAX + 1e-3:
                    problems.append(f"{name}.{n}@{ph:.2f} |q-q0|={abs(val-q0):.3f} > act_max {ACT_MAX}")
            # last keyframe must be q0
            if abs(keys[n][-1][1] - q0) > 1e-4:
                problems.append(f"{name}.{n}: last keyframe {keys[n][-1][1]:.4f} != q0 {q0:.4f}")
    return problems


# ---------------------------------------------------------------------------
# Recorded Stage-I motions (self-discovered, slowed) for Stage II imitation
# ---------------------------------------------------------------------------

_MOTION_DIR = Path(__file__).resolve().parents[2] / "robots" / "microban" / "motions"


def load_recorded_motions(motion_dir: Path | None = None) -> dict[str, Any] | None:
    """Load getup_prone.npz / getup_supine_roll.npz if present. Returns None until Stage I records them."""
    root = Path(motion_dir) if motion_dir else _MOTION_DIR
    out: dict[str, Any] = {}
    for side, fname in (("prone", "getup_prone.npz"), ("supine", "getup_supine_roll.npz")):
        path = root / fname
        if not path.is_file():
            continue
        data = np.load(path, allow_pickle=False)
        q = np.asarray(data["q"], dtype=np.float64)          # (T, 18)
        phase = np.asarray(data["phase"], dtype=np.float64)  # (T,)
        duration_s = float(data["duration_s"]) if "duration_s" in data.files else CLIP_DURATION_S
        root_q = np.asarray(data["root_quat"], dtype=np.float64) if "root_quat" in data.files else None
        root_z = np.asarray(data["root_z"], dtype=np.float64) if "root_z" in data.files else None
        out[side] = {
            "q": q, "phase": phase, "duration_s": duration_s,
            "root_quat": root_q, "root_z": root_z, "path": str(path),
        }
    if not out:
        return None
    # shared duration for episode clock: take the longer of the two
    out["duration_s"] = max(float(v["duration_s"]) for v in out.values() if isinstance(v, dict) and "duration_s" in v)
    return out


def sample_recorded(traj: dict[str, Any], phase: float) -> list[float]:
    """Linear interpolate joint angles of a recorded trajectory at phase ∈ [0, 1]."""
    q = traj["q"]
    ph = traj["phase"]
    phase = float(np.clip(phase, 0.0, 1.0))
    if phase <= float(ph[0]):
        return q[0].tolist()
    if phase >= float(ph[-1]):
        return q[-1].tolist()
    i = int(np.searchsorted(ph, phase, side="right") - 1)
    i = max(0, min(i, len(ph) - 2))
    t0, t1 = float(ph[i]), float(ph[i + 1])
    s = 0.0 if t1 <= t0 else (phase - t0) / (t1 - t0)
    return ((1.0 - s) * q[i] + s * q[i + 1]).tolist()


def sample_recorded_root(traj: dict[str, Any], phase: float) -> tuple[np.ndarray, float]:
    """Recorded trunk quaternion (w, x, y, z; nearest frame) and height (interpolated) at phase."""
    ph = traj["phase"]
    phase = float(np.clip(phase, 0.0, 1.0))
    k = int(np.clip(np.searchsorted(ph, phase), 0, len(ph) - 1))
    z = float(np.interp(phase, ph, traj["root_z"]))
    return np.asarray(traj["root_quat"][k], dtype=np.float64), z


if __name__ == "__main__":
    probs = validate_clips()
    if probs:
        print("FAIL")
        for p in probs:
            print(" ", p)
        raise SystemExit(1)
    print("OK: getup_prone / getup_supine within joint limits and act_max; end at q0")
    motions = load_recorded_motions()
    if motions:
        print(f"recorded motions: {list(k for k in motions if k in ('prone', 'supine'))} "
              f"duration={motions['duration_s']:.2f}s")
    else:
        print("no recorded motions yet (Stage I)")
