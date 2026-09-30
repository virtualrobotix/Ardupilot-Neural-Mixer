#!/usr/bin/env python3
# Author: Roberto Navoni, member of the ArduPilot Dev Team
# Contact: r.navoni74@gmail.com
# Developed by Roberto Navoni — DelphyAI LAB
"""Validate Microban get-up clips: joint limits, act_max, kinematic replay, BAM dynamics.

    .venv/bin/python tools/robots/getup_clip_check.py
    .venv/bin/python tools/robots/getup_clip_check.py --video docs/media/microban_getup_clip_check.mp4
    .venv/bin/python tools/robots/getup_clip_check.py --dynamics   # drive BAM along the clip (no policy)
"""

from __future__ import annotations

import argparse
import math
import subprocess
import sys
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import getup_clips as getup  # noqa: E402
from common import load_profile, resolve_mjcf  # noqa: E402
from nnm_env import NNMixerEnv, load_ppo_config  # noqa: E402


def kinematic_replay(side: str, n_frames: int = 100) -> list[dict]:
    """Place the robot at each clip phase with mj_kinematics (no physics)."""
    presets = getup.getup_presets()
    key = "getup_prone" if side == "prone" else "getup_supine"
    spec = presets[key]
    profile = load_profile("microban")
    path = resolve_mjcf("microban", profile)
    m = mujoco.MjModel.from_xml_path(str(path))
    d = mujoco.MjData(m)
    names = profile["joint_names"]
    jid = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n) for n in names]
    qpos_idx = np.array([m.jnt_qposadr[j] for j in jid])
    fj = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "trunk_freejoint")
    fq = int(m.jnt_qposadr[fj])
    tid = int(m.jnt_bodyid[fj])
    frames = []
    for i in range(n_frames):
        phase = i / (n_frames - 1)
        q = np.array(getup.joint_vector(spec["keys"], phase), dtype=np.float64)
        quat, z = getup.root_at(spec["root"], phase)
        d.qpos[:] = 0
        d.qpos[fq:fq + 3] = [0.0, 0.0, z]
        d.qpos[fq + 3:fq + 7] = quat
        d.qpos[qpos_idx] = q
        mujoco.mj_kinematics(m, d)
        frames.append({
            "phase": phase, "z": float(d.xpos[tid, 2]), "q": q.copy(),
            "quat": quat, "tilt_deg": _tilt(d.xquat[tid]),
        })
    return frames


def _tilt(quat) -> float:
    w, x, y, z = quat
    return math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (x * x + y * y)))))


def dynamics_replay(side: str, rate_hz: float = 50.0) -> dict:
    """Drive the BAM actuators along the clip reference (open-loop); report peak torque / final pose."""
    cfg = load_ppo_config("microban")
    cfg.setdefault("env", {})["gesture"] = "getup"
    env = NNMixerEnv("microban", cfg, seed=0)
    env.getup_side = side
    env.clip_phase = 0.0
    key = "getup_prone" if side == "prone" else "getup_supine"
    env.gesture_spec = {
        "mode": "getup", "hz": 1.0 / getup.CLIP_DURATION_S, "oneshot": True,
        "keys": env.getup_presets[key]["keys"],
    }
    env._place_getup(side, 0.0, settle=True)
    env.gravity = __import__("deploy_contract", fromlist=["GravityFilter"]).GravityFilter(env.contract.att_tau)
    peak_tau = 0.0
    steps = int(getup.CLIP_DURATION_S * rate_hz) + int(1.0 * rate_hz)
    for k in range(steps):
        phase = min(1.0, k / (getup.CLIP_DURATION_S * rate_hz))
        env.clip_phase = phase
        q_ref = np.array(getup.joint_vector(env.gesture_spec["keys"], phase), dtype=np.float64)
        action = np.clip(q_ref - env.contract.q0, -env.contract.act_max, env.contract.act_max)
        obs, _, fell, timeout, info = env.step(action.astype(np.float32))
        tau = float(np.max(np.abs(env.data.actuator_force[env.act_idx])))
        peak_tau = max(peak_tau, tau)
        if timeout:
            break
    return {
        "side": side,
        "final_tilt_deg": env.tilt_deg(),
        "final_z": float(env.data.qpos[env.free_qpos + 2]),
        "peak_torque_nm": peak_tau,
        "force_limit_nm": float(env.force_limit[0]),
        "q_err": float(np.mean(np.abs(env.data.qpos[env.qpos_idx] - env.contract.q0))),
        "clip_phase": env.clip_phase,
    }


def write_video(out: Path, sides: tuple[str, ...] = ("prone", "supine")) -> None:
    import mujoco

    profile = load_profile("microban")
    path = resolve_mjcf("microban", profile)
    m = mujoco.MjModel.from_xml_path(str(path))
    d = mujoco.MjData(m)
    m.vis.global_.offwidth = max(m.vis.global_.offwidth, 960)
    m.vis.global_.offheight = max(m.vis.global_.offheight, 540)
    renderer = mujoco.Renderer(m, 540, 960)
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.distance, cam.elevation, cam.azimuth = 1.0, -15, 120
    names = profile["joint_names"]
    jid = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n) for n in names]
    qpos_idx = np.array([m.jnt_qposadr[j] for j in jid])
    fj = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "trunk_freejoint")
    fq = int(m.jnt_qposadr[fj])
    presets = getup.getup_presets()
    proc = subprocess.Popen(
        ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
         "-s", "960x540", "-r", "25", "-i", "-", "-pix_fmt", "yuv420p", str(out)],
        stdin=subprocess.PIPE,
    )
    for side in sides:
        key = "getup_prone" if side == "prone" else "getup_supine"
        spec = presets[key]
        for i in range(100):
            phase = i / 99.0
            q = np.array(getup.joint_vector(spec["keys"], phase), dtype=np.float64)
            quat, z = getup.root_at(spec["root"], phase)
            d.qpos[:] = 0
            d.qpos[fq:fq + 3] = [0.0, 0.0, z]
            d.qpos[fq + 3:fq + 7] = quat
            d.qpos[qpos_idx] = q
            mujoco.mj_forward(m, d)
            cam.lookat[:] = d.qpos[fq:fq + 3]
            renderer.update_scene(d, camera=cam)
            frame = renderer.render()
            # tiny caption via raw bytes is skipped; phase encoded in look
            proc.stdin.write(frame.tobytes())
    proc.stdin.close()
    proc.wait()
    print(f"video: {out}")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--video", type=Path, default=None)
    ap.add_argument("--dynamics", action="store_true", help="open-loop BAM replay of both clips")
    args = ap.parse_args()

    problems = getup.validate_clips()
    if problems:
        print("LIMITS FAIL")
        for p in problems:
            print(" ", p)
        raise SystemExit(1)
    print("limits: OK (joint ranges, act_max, end at q0)")

    for side in ("prone", "supine"):
        frames = kinematic_replay(side)
        z0, z1 = frames[0]["z"], frames[-1]["z"]
        t0, t1 = frames[0]["tilt_deg"], frames[-1]["tilt_deg"]
        print(f"kinematic {side:6s}: z {z0:.3f}→{z1:.3f} m  tilt {t0:.0f}→{t1:.0f}°")

    if args.dynamics:
        for side in ("prone", "supine"):
            r = dynamics_replay(side)
            ok = r["final_tilt_deg"] < 25 and r["final_z"] > 0.10
            print(f"dynamics {side:6s}: tilt={r['final_tilt_deg']:.1f}° z={r['final_z']:.3f} "
                  f"peak_τ={r['peak_torque_nm']:.2f}/{r['force_limit_nm']:.2f} Nm "
                  f"q_err={r['q_err']:.3f}  {'OK' if ok else 'WEAK (policy must recover)'}")

    if args.video:
        args.video.parent.mkdir(parents=True, exist_ok=True)
        write_video(args.video)


if __name__ == "__main__":
    main()
