#!/usr/bin/env python3
# Author: Roberto Navoni, member of the ArduPilot Dev Team
# Contact: r.navoni74@gmail.com
# Developed by Roberto Navoni — DelphyAI LAB
"""Retarget MediaPipe Pose (33 world landmarks, metres) to Microban's 8 pose channels.

Channels (offsets from q0, matching nnm_env pose_cmd / firmware NNM_POSE):
  0 right_shoulder_pitch, 1 right_shoulder_roll, 2 right_elbow,
  3 left_shoulder_pitch,  4 left_shoulder_roll,  5 left_elbow,
  6 knee_bend (0..0.55),  7 sway (±0.15).

Microban MJCF signs: negative shoulder pitch raises the hand forward; right shoulder
roll opens outward for negative angles, left for positive. Default is mirror-off
(user right arm -> robot right arm); pass mirror=True to flip left/right.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

# MediaPipe PoseLandmark indices
L_SHOULDER, R_SHOULDER = 11, 12
L_ELBOW, R_ELBOW = 13, 14
L_WRIST, R_WRIST = 15, 16
L_HIP, R_HIP = 23, 24

POSE_RANGES = np.array([
    [-1.55, 0.15], [-1.15, 0.05], [-1.35, -0.05],
    [-1.55, 0.15], [-0.05, 1.15], [-1.35, -0.05],
    [0.0, 0.55], [-0.15, 0.15],
], np.float32)


def _v(landmarks, i: int) -> np.ndarray:
    lm = landmarks[i]
    return np.array([lm.x, lm.y, lm.z], dtype=np.float64)


def _unit(v: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    return v / n if n > 1e-6 else v


@dataclass
class MicrobanRetarget:
    mirror: bool = False
    ema: float = 0.35
    max_rate: float = 4.0  # rad/s per channel
    rate_hz: float = 30.0
    _filt: np.ndarray = field(default_factory=lambda: np.zeros(8, np.float32))
    _out: np.ndarray = field(default_factory=lambda: np.zeros(8, np.float32))
    _stand_hip_y: float | None = None
    _calib_ready: bool = False

    def calibrate_stand(self, landmarks) -> None:
        """Call once while the user stands arms at sides; sets the hip height baseline."""
        mid_hip = 0.5 * (_v(landmarks, L_HIP) + _v(landmarks, R_HIP))
        # MediaPipe world: y is up
        self._stand_hip_y = float(mid_hip[1])
        self._calib_ready = True
        self._filt[:] = 0.0

    def retarget(self, landmarks) -> np.ndarray:
        """landmarks: sequence of 33 pose_world_landmarks with .x/.y/.z."""
        raw = self._raw_pose(landmarks)
        # EMA against detector jitter, then a per-tick rate limit on the emitted command
        a = float(np.clip(self.ema, 0.05, 1.0))
        self._filt = (1.0 - a) * self._filt + a * raw
        step = self.max_rate / max(self.rate_hz, 1.0)
        delta = self._filt - self._out
        nrm = float(np.max(np.abs(delta)))
        if nrm > step:
            self._out = self._out + delta * (step / nrm)
        else:
            self._out = self._filt.copy()
        return np.clip(self._out, POSE_RANGES[:, 0], POSE_RANGES[:, 1]).astype(np.float32)

    def _raw_pose(self, landmarks) -> np.ndarray:
        ls, rs = _v(landmarks, L_SHOULDER), _v(landmarks, R_SHOULDER)
        le, re = _v(landmarks, L_ELBOW), _v(landmarks, R_ELBOW)
        lw, rw = _v(landmarks, L_WRIST), _v(landmarks, R_WRIST)
        lh, rh = _v(landmarks, L_HIP), _v(landmarks, R_HIP)

        mid_sh = 0.5 * (ls + rs)
        mid_hip = 0.5 * (lh + rh)
        # torso frame: x right, y up, z forward (approx from MediaPipe world)
        x_axis = _unit(rs - ls)          # shoulder right
        y_up = _unit(mid_sh - mid_hip)
        z_axis = _unit(np.cross(x_axis, y_up))
        x_axis = _unit(np.cross(y_up, z_axis))
        R = np.stack([x_axis, y_up, z_axis], axis=0)  # world -> torso

        def arm(shoulder, elbow, wrist, right: bool) -> tuple[float, float, float]:
            # torso frame: x person's right, y up, z forward. The rest pose (q0, offset 0) is the
            # arm hanging straight down, so every angle is measured from the -y direction.
            upper = R @ (elbow - shoulder)
            lower = R @ (wrist - elbow)
            down = max(-float(upper[1]), 1e-4)
            # pitch: forward elevation in the sagittal plane; Microban negative raises the hand forward
            pitch = -math.atan2(float(upper[2]), down)
            # roll: abduction in the frontal plane. Right opens outward (+x) for negative angles,
            # left opens outward (-x) for positive angles: one formula covers both sides
            roll = -math.atan2(float(upper[0]), down)
            # elbow flexion = angle between upper and lower arm; straight 0, Microban bent is negative
            ang = math.acos(float(np.clip(np.dot(_unit(upper), _unit(lower)), -1.0, 1.0)))
            elbow_q = -ang
            return pitch, roll, elbow_q

        rp, rr, re_ = arm(rs, re, rw, True)
        lp, lr, le_ = arm(ls, le, lw, False)

        # knee_bend from drop of hip height vs stand calibration
        knee = 0.0
        if self._calib_ready and self._stand_hip_y is not None:
            drop = max(0.0, self._stand_hip_y - float(mid_hip[1]))
            knee = float(np.clip(drop / 0.12 * 0.55, 0.0, 0.55))  # ~12 cm -> full bend

        # sway: lateral lean of the torso from the vertical (world y up), positive toward the
        # person's right, same sign as the robot's sway channel (trunk leans toward the raised arm
        # in the dance clip). Measured against the horizontal shoulder direction, not the torso
        # frame, whose up axis is the torso itself.
        x_h = rs - ls
        x_h[1] = 0.0
        x_h = _unit(x_h)
        torso = mid_sh - mid_hip
        sway = float(np.clip(math.atan2(float(np.dot(torso, x_h)), max(float(torso[1]), 1e-4)),
                             -0.15, 0.15))

        out = np.array([rp, rr, re_, lp, lr, le_, knee, sway], np.float32)
        if self.mirror:
            # swap arms and flip roll/sway signs
            out = np.array([lp, -lr, le_, rp, -rr, re_, knee, -sway], np.float32)
        return np.clip(out, POSE_RANGES[:, 0], POSE_RANGES[:, 1]).astype(np.float32)


# Clip streams reused by mocap_gcs --source clip (same numbers as nnm_env).
CLIP_KEYS = {
    "wave_right": {
        "hz": 0.5,
        "arms": {
            "right_shoulder_pitch": [(0.0, -1.10)],
            "right_elbow": [(0.0, -1.00)],
            "right_shoulder_roll": [(0.0, -0.25), (0.5, -1.05)],
        },
    },
    "dance": {
        "hz": 0.625,
        "arms": {
            "right_shoulder_pitch": [(0.0, -1.40), (0.5, -0.20)],
            "left_shoulder_pitch": [(0.0, -0.20), (0.5, -1.40)],
            "right_shoulder_roll": [(0.0, -0.75), (0.5, -0.25)],
            "left_shoulder_roll": [(0.0, 0.25), (0.5, 0.75)],
            "right_elbow": [(0.0, -1.20), (0.5, -0.45)],
            "left_elbow": [(0.0, -0.45), (0.5, -1.20)],
        },
        "knee": [(0.0, 0.15), (0.25, 0.55), (0.5, 0.15), (0.75, 0.55)],
        "sway": [(0.0, 0.05), (0.5, -0.05)],
    },
}

ARM_ORDER = (
    "right_shoulder_pitch", "right_shoulder_roll", "right_elbow",
    "left_shoulder_pitch", "left_shoulder_roll", "left_elbow",
)


def _clip_value(keys: list[tuple[float, float]], phase: float) -> float:
    if len(keys) == 1:
        return keys[0][1]
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


def pose_from_clip(name: str, phase: float, q0_arms: np.ndarray | None = None) -> np.ndarray:
    """Absolute clip angles converted to q0-offsets (q0_arms default 0)."""
    spec = CLIP_KEYS[name]
    q0 = np.zeros(6, np.float64) if q0_arms is None else np.asarray(q0_arms, dtype=np.float64)
    out = np.zeros(8, np.float32)
    arms = spec["arms"]
    for i, n in enumerate(ARM_ORDER):
        if n in arms:
            out[i] = _clip_value(arms[n], phase) - q0[i]
    if "knee" in spec:
        out[6] = _clip_value(spec["knee"], phase)
    if "sway" in spec:
        out[7] = _clip_value(spec["sway"], phase)
    return np.clip(out, POSE_RANGES[:, 0], POSE_RANGES[:, 1]).astype(np.float32)
