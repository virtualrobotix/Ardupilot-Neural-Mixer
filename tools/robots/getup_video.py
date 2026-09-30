#!/usr/bin/env python3
# Author: Roberto Navoni, member of the ArduPilot Dev Team
# Contact: r.navoni74@gmail.com
# Developed by Roberto Navoni — DelphyAI LAB
"""Video of a Microban get-up policy: prone then supine start, deployed int8 network.

    .venv/bin/python tools/robots/getup_video.py --nnm robots/microban/policies/getup_run/getup_it0400.nnm \
        --video docs/media/microban_getup_it0400.mp4
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import deploy_contract as dc  # noqa: E402
import getup_clips as getup  # noqa: E402
from export_nnm import unpack_nnm  # noqa: E402
from nnm_env import NNMixerEnv, load_ppo_config  # noqa: E402
from play_policy import _overlay  # noqa: E402


def start_lying(env: NNMixerEnv, start: str) -> np.ndarray:
    """start: 'prone' | 'supine' (lying) or the name of a getup_clips waypoint (RSI stage)."""
    wp = next((w for w in getup.WAYPOINTS if w["name"] == start), None)
    side = start if wp is None else ("supine" if wp["side"] == "supine" else "prone")
    env.getup_side = side
    env.gesture_spec = {"mode": "getup", "hz": 1.0 / getup.CLIP_DURATION_S, "oneshot": True,
                        "keys": env.getup_presets[f"getup_{side}"]["keys"]}
    if wp is None:
        env.clip_phase = 0.0
        env._place_getup(side, 0.0, settle=True)
    else:
        env.clip_phase = float(wp["progress"])
        env._place_waypoint(wp)
    env.gravity = dc.GravityFilter(env.contract.att_tau)
    for _ in range(dc.AUTOPILOT_LOOP_HZ // 10):
        env._apply_target(env.q_wire)
        mujoco.mj_step(env.model, env.data)
        g, a = env._imu_frd()
        env.gravity.update(g, a, 1.0 / dc.AUTOPILOT_LOOP_HZ)
    env.steps = 0
    return env._observe()


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--robot", default="microban")
    ap.add_argument("--nnm", type=Path, required=True)
    ap.add_argument("--video", type=Path, required=True)
    ap.add_argument("--seconds", type=float, default=7.0)
    ap.add_argument("--sides", default="prone,supine",
                    help="comma list of starts: prone, supine, or waypoint names (seat_heels, squat_deep, ...)")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--gesture", default="getup",
                    choices=("getup", "getup_discover"),
                    help="env gesture mode (discover for Stage I policies)")
    ap.add_argument("--assist", type=float, default=0.0,
                    help="upward trunk assist force in N (Stage I curriculum)")
    ap.add_argument("--no-posture-clip", action="store_true",
                    help="disable the discovery hip-yaw/arm action limits (older Stage I policies)")
    args = ap.parse_args()

    pk = unpack_nnm(args.nnm.read_bytes())
    cfg = load_ppo_config(args.robot)
    cfg.setdefault("env", {})["gesture"] = args.gesture
    args.video.parent.mkdir(parents=True, exist_ok=True)
    proc = subprocess.Popen(
        ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
         "-s", "960x540", "-r", "25", "-i", "-", "-pix_fmt", "yuv420p", str(args.video)],
        stdin=subprocess.PIPE)
    renderer = None
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.distance, cam.elevation, cam.azimuth = 1.0, -18, 125
    assist_tag = f" assist={args.assist:.1f}N" if args.assist else ""
    title = f"Microban {args.nnm.stem}{assist_tag} · "
    for i, side in enumerate(args.sides.split(",")):
        env = NNMixerEnv(args.robot, cfg, seed=args.seed + i)
        env.max_steps = 10 ** 9
        if args.no_posture_clip:
            env.getup_clip_ids = np.zeros(0, dtype=int)
        env.getup_p_rsi = env.getup_p_stand = 0.0
        env.getup_assist_n = float(args.assist)
        env.reset()
        env.getup_assist_n = float(args.assist)
        obs = start_lying(env, side)
        env.getup_assist_n = float(args.assist)
        rate = env.contract.rate_hz
        env.model.vis.global_.offwidth = max(env.model.vis.global_.offwidth, 960)
        env.model.vis.global_.offheight = max(env.model.vis.global_.offheight, 540)
        renderer = mujoco.Renderer(env.model, 540, 960)
        stood = None
        for k in range(int(args.seconds * rate)):
            t = k / rate
            obs, _, _, _, info = env.step(dc.int8_forward(pk["layers"], pk["mean"], pk["std"], obs))
            z = float(env.data.qpos[env.free_qpos + 2])
            if stood is None and env.tilt_deg() < 12 and z > 0.85 * env.home_z:
                stood = t
            if k % max(1, rate // 25) == 0:
                cam.lookat[:] = env.data.qpos[env.free_qpos:env.free_qpos + 3]
                renderer.update_scene(env.data, camera=cam)
                label = f"getup {side}  tilt={env.tilt_deg():.0f}°  z={z:.2f}{assist_tag}"
                proc.stdin.write(_overlay(renderer.render(), label, t, info, np.zeros(3, np.float32),
                                          title + side).tobytes())
        print(f"{side}: tilt={env.tilt_deg():.1f} z={float(env.data.qpos[env.free_qpos + 2]):.3f}"
              f"  {'in piedi a %.1fs' % stood if stood is not None else 'non si alza'}")
    proc.stdin.close()
    proc.wait()
    print(f"video: {args.video}")


if __name__ == "__main__":
    main()
