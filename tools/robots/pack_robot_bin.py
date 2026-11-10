#!/usr/bin/env python3
# Author: Roberto Navoni, member of the ArduPilot Dev Team
# Contact: r.navoni74@gmail.com
# Developed by Roberto Navoni — DelphyAI LAB
# For information: r.navoni74@gmail.com
"""Pack robots/<id>/robot/profile.json into robot.bin for the microSD layout.

Layout (little-endian):
  magic[4] = b'NNMR'
  robot_id[16]
  n_joints u16, obs_dim u16, rate_hz u16, servo_fn0 u16
  q0[n_joints] f32
  joint_names as length-prefixed ASCII strings (u8 len + bytes), n_joints times

A profile with deployment extras (pose_limits, aux_max, deploy, skills) also gets skills.bin
next to robot.bin; the firmware reads it when present (nnm_parse_skills_bin in AP_NNMixer_Policy.h).

skills.bin layout (little-endian), version 1:
  magic[4] = b'NNMS', version u8, n_policy u8, n_skill u8, reserved u8
  head_first u8, head_count u8, reserved u16      joints filtered with alpha_head (NNM_DEPLOY 1)
  alpha_leg f32, alpha_head f32                   default target filter (NNM_DEPLOY 1)
  pose_limit f32[8]                               symmetric limit per NNM_POSE channel, 0 = unused
  aux_max f32                                     auxiliary actuator range, degrees
  n_policy x { policy u8, flags u8, reserved u16, alpha_leg f32, alpha_head f32,
               heading_kp f32, wz_max f32 }       flags: 1 zero pose command, 2 no accel-x trim
  n_skill x  { id u8, policy u8, flags u8, requires u8, duration_s f32, cmd f32[3],
               aux_deg f32, aux_until f32 }       flags: 1 circle command, 2 zero pose command,
                                                  4 get-up suspended, 8 keep history, 16 drive aux

Usage:
    python tools/robots/pack_robot_bin.py --robot microduck \\
        --out robots/microduck/robot/robot.bin
"""

from __future__ import annotations

import argparse
import struct
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import ROBOT_INDEX, load_profile, robot_dir  # noqa: E402


def pack(profile: dict) -> bytes:
    robot_id = profile["robot_id"]
    n_joints = int(profile["n_joints"])
    obs_dim = int(profile.get("obs_dim") or 0)
    rate_hz = int(profile.get("rate_hz") or 50)
    servo_fn0 = int(profile.get("servo_fn0") or 94)
    q0 = np.array(profile.get("q0") or [0.0] * n_joints, dtype=np.float32)
    names = list(profile.get("joint_names") or [f"j{i}" for i in range(n_joints)])
    if q0.size != n_joints:
        raise SystemExit(f"q0 length {q0.size} != n_joints {n_joints}")
    if len(names) != n_joints:
        raise SystemExit(f"joint_names length {len(names)} != n_joints {n_joints}")

    rid = robot_id.encode("ascii")[:16]
    rid = rid + b"\0" * (16 - len(rid))
    buf = bytearray()
    buf += b"NNMR"
    buf += rid
    buf += struct.pack("<HHHH", n_joints, obs_dim, rate_hz, servo_fn0)
    buf += q0.tobytes()
    for name in names:
        b = name.encode("ascii")
        if len(b) > 255:
            raise SystemExit(f"joint name too long: {name}")
        buf += bytes([len(b)]) + b
    return bytes(buf)


SKILL_KEYS = ("pose_limits", "aux_max", "deploy", "skills")
POLICY_ZERO_POSE, POLICY_NO_ACCX = 1, 2
SKILL_CIRCLE, SKILL_ZERO_POSE, SKILL_NO_GETUP, SKILL_KEEP_HISTORY, SKILL_AUX = 1, 2, 4, 8, 16


def pack_skills(profile: dict) -> bytes | None:
    """skills.bin for a profile with deployment extras, None when it has none."""
    if not any(k in profile for k in SKILL_KEYS):
        return None
    deploy = profile.get("deploy") or {}
    policies = list(deploy.get("policies") or [])
    skills = list(profile.get("skills") or [])
    n_joints = int(profile["n_joints"])
    head_first, head_count = deploy.get("head_joints") or (0, 0)
    if head_first + head_count > n_joints:
        raise SystemExit(f"head_joints {head_first}+{head_count} past n_joints {n_joints}")
    alpha_leg = float(deploy.get("alpha_leg", 0.0))
    alpha_head = float(deploy.get("alpha_head", alpha_leg))
    limits = [float(v) for v in profile.get("pose_limits") or []]
    if len(limits) > 8:
        raise SystemExit("pose_limits holds at most 8 channels")
    limits += [0.0] * (8 - len(limits))
    ids = [int(s["id"]) for s in skills]
    if len(set(ids)) != len(ids) or any(not 1 <= i <= 255 for i in ids):
        raise SystemExit(f"skill ids must be unique and within 1..255: {ids}")
    buf = bytearray()
    buf += struct.pack("<4sBBBB", b"NNMS", 1, len(policies), len(skills), 0)
    buf += struct.pack("<BBH", int(head_first), int(head_count), 0)
    buf += struct.pack("<ff", alpha_leg, alpha_head)
    buf += struct.pack("<8f", *limits)
    buf += struct.pack("<f", float(profile.get("aux_max", 0.0)))
    for p in policies:
        flags = (POLICY_ZERO_POSE if p.get("zero_pose") else 0) | (POLICY_NO_ACCX if p.get("no_accx") else 0)
        buf += struct.pack("<BBHffff", int(p["policy"]), flags, 0,
                           float(p.get("alpha_leg", alpha_leg)), float(p.get("alpha_head", alpha_head)),
                           float(p.get("heading_kp", 0.0)), float(p.get("wz_max", 0.0)))
    for s in skills:
        cmd = s.get("cmd", [0.0, 0.0, 0.0])
        flags = ((SKILL_CIRCLE if cmd == "circle" else 0) | (SKILL_ZERO_POSE if s.get("zero_pose") else 0)
                 | (SKILL_NO_GETUP if s.get("no_getup") else 0)
                 | (SKILL_KEEP_HISTORY if s.get("keep_history") else 0)
                 | (SKILL_AUX if "aux_deg" in s else 0))
        requires = int(s.get("requires", 0))
        if requires and requires not in ids:
            raise SystemExit(f"skill {s['id']} requires unknown skill {requires}")
        twist = [0.0, 0.0, 0.0] if cmd == "circle" else [float(v) for v in cmd]
        if len(twist) != 3:
            raise SystemExit(f"skill {s['id']}: cmd must be 'circle' or three values")
        buf += struct.pack("<BBBBf3fff", int(s["id"]), int(s["policy"]), flags, requires,
                           float(s.get("duration_s", 0.0)), *twist,
                           float(s.get("aux_deg", 0.0)), float(s.get("aux_until", 0.0)))
    return bytes(buf)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--robot", required=True, choices=sorted(ROBOT_INDEX))
    ap.add_argument("--out", type=Path, default=None)
    args = ap.parse_args()
    profile = load_profile(args.robot)
    if int(profile.get("n_joints") or 0) <= 0:
        raise SystemExit(f"{args.robot}: n_joints not set in profile yet")
    out = args.out or (robot_dir(args.robot) / "robot" / "robot.bin")
    out.parent.mkdir(parents=True, exist_ok=True)
    blob = pack(profile)
    out.write_bytes(blob)
    print(f"wrote {out} ({len(blob)} bytes) robot_id={args.robot}")
    skills = pack_skills(profile)
    if skills is not None:
        sk_out = out.with_name("skills.bin")
        sk_out.write_bytes(skills)
        print(f"wrote {sk_out} ({len(skills)} bytes): "
              f"{len(profile.get('skills') or [])} skills")


if __name__ == "__main__":
    main()
