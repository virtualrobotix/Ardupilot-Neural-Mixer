#!/usr/bin/env python3
# Author: Roberto Navoni, member of the ArduPilot Dev Team
# Contact: r.navoni74@gmail.com
# Developed by Roberto Navoni — DelphyAI LAB
"""Multi-policy Microban demo: getup → walk → teleop, with simulated firmware blend.

Mirrors AP_NNMixer dual-slot switch + NNM_BLEND_MS cross-fade and the automatic
get-up state machine (tilt trigger → getup.nnm → return to previous policy).

    .venv/bin/python tools/robots/play_getup_mix.py \\
        --getup robots/microban/policies/getup.nnm \\
        --walk  robots/microban/policies/walk_md.nnm \\
        --teleop robots/microban/policies/pose_cmd.nnm \\
        --video docs/media/microban_getup_walk_teleop.mp4
"""

from __future__ import annotations

import argparse
import math
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import deploy_contract as dc  # noqa: E402
import getup_clips as getup  # noqa: E402
from export_nnm import unpack_nnm  # noqa: E402
from nnm_env import NNMixerEnv, load_ppo_config  # noqa: E402
from play_policy import _overlay, _font, SIGNATURE  # noqa: E402


def yaw_of(q) -> float:
    w, x, y, z = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


class PolicyBlend:
    """Cross-fade between two unpacked .nnm policies (firmware NNM_BLEND_MS)."""

    def __init__(self, blend_ms: float = 500.0, rate_hz: float = 50.0):
        self.blend_s = blend_ms / 1000.0
        self.rate_hz = rate_hz
        self.pk = None
        self.pk_from = None
        self.a_from = None
        self.blend_left = 0
        self.name = ""

    def set(self, pk: dict, name: str, blend: bool = True):
        if blend and self.pk is not None:
            self.pk_from = self.pk
            self.a_from = None  # filled on next act()
            self.blend_left = int(self.blend_s * self.rate_hz)
        else:
            self.pk_from = None
            self.blend_left = 0
        self.pk = pk
        self.name = name

    def act(self, obs: np.ndarray) -> np.ndarray:
        a = dc.int8_forward(self.pk["layers"], self.pk["mean"], self.pk["std"], obs)
        if self.blend_left > 0 and self.pk_from is not None:
            if self.a_from is None:
                self.a_from = dc.int8_forward(
                    self.pk_from["layers"], self.pk_from["mean"], self.pk_from["std"], obs)
            total = max(1, int(self.blend_s * self.rate_hz))
            alpha = 1.0 - self.blend_left / total
            a = self.a_from * (1.0 - alpha) + a * alpha
            self.blend_left -= 1
            if self.blend_left <= 0:
                self.pk_from = None
                self.a_from = None
        return a.astype(np.float32)


def run_mix(getup_nnm: Path, walk_nnm: Path, teleop_nnm: Path, video: Path | None,
            blend_ms: float = 500.0, push_at_s: float = 8.0, getup_tilt: float = 55.0,
            getup_ok: float = 20.0, getup_hold_s: float = 1.5, getup_s: float = 4.0,
            start_side: str = "prone") -> dict:
    cfg = load_ppo_config("microban")
    cfg.setdefault("env", {})["gesture"] = "getup"
    env = NNMixerEnv("microban", cfg, seed=0)
    env.max_steps = 10 ** 9
    env.getup_p_rsi = 0.0
    env.getup_p_stand = 0.0
    rate = env.contract.rate_hz
    blend = PolicyBlend(blend_ms, rate)

    pk_getup = unpack_nnm(getup_nnm.read_bytes())
    pk_walk = unpack_nnm(walk_nnm.read_bytes())
    pk_teleop = unpack_nnm(teleop_nnm.read_bytes())
    # firmware NNM_GETUP_S = recorded clip duration (Stage II clock), else the waypoint clip
    clip_s = float(env.getup_motion["duration_s"]) if env.getup_motion else getup.CLIP_DURATION_S
    getup_s = max(getup_s, clip_s)

    # start lying
    env.reset()
    env.getup_side = start_side
    key = "getup_prone" if start_side == "prone" else "getup_supine"
    env.gesture_spec = {
        "mode": "getup", "hz": 1.0 / clip_s, "oneshot": True,
        "keys": env.getup_presets[key]["keys"],
    }
    env.clip_phase = 0.0
    env._place_getup(start_side, 0.0, settle=True)
    env.gravity = dc.GravityFilter(env.contract.att_tau)
    for _ in range(dc.AUTOPILOT_LOOP_HZ // 10):
        env._apply_target(env.q_wire)
        import mujoco
        mujoco.mj_step(env.model, env.data)
        g, a = env._imu_frd()
        env.gravity.update(g, a, 1.0 / dc.AUTOPILOT_LOOP_HZ)
    obs = env._observe()

    blend.set(pk_getup, "getup", blend=False)
    mode = "getup"          # getup | walk | teleop
    prev_mode = "walk"      # where to return after auto-getup
    getup_hold = 0
    phase_t0 = 0.0
    label = f"getup ({start_side})"
    d = env.data
    k = 0
    fell_flag = False
    log = []
    pushed = False

    renderer = proc = cam = None
    if video is not None:
        import mujoco
        env.model.vis.global_.offwidth = max(env.model.vis.global_.offwidth, 960)
        env.model.vis.global_.offheight = max(env.model.vis.global_.offheight, 540)
        renderer = mujoco.Renderer(env.model, 540, 960)
        cam = mujoco.MjvCamera()
        cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        cam.distance, cam.elevation, cam.azimuth = 1.2, -20, 135
        video.parent.mkdir(parents=True, exist_ok=True)
        proc = subprocess.Popen(
            ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
             "-s", "960x540", "-r", "25", "-i", "-", "-pix_fmt", "yuv420p", str(video)],
            stdin=subprocess.PIPE,
        )

    # timeline (seconds of wall-sim time after first successful stand)
    # getup → walk 5s → push → auto getup → walk 3s → teleop wave 6s
    walk_after_stand_s = 5.0
    teleop_s = 6.0
    total_budget_s = 45.0
    stood_once = False
    walk_t0 = None
    teleop_t0 = None
    auto_getup_count = 0

    while k / rate < total_budget_s:
        t = k / rate
        tilt = env.tilt_deg()

        # --- automatic get-up trigger (firmware NNM_GETUP_*) ---
        if mode != "getup" and tilt > getup_tilt:
            prev_mode = mode
            mode = "getup"
            label = "auto-getup"
            env.getup = True
            # choose side from gravity x sign
            g = env._gravity_obs()
            side = "prone" if g[0] > 0 else "supine"
            env.getup_side = side
            key = "getup_prone" if side == "prone" else "getup_supine"
            env.gesture_spec = {
                "mode": "getup", "hz": 1.0 / clip_s, "oneshot": True,
                "keys": env.getup_presets[key]["keys"],
            }
            env.clip_phase = 0.0
            blend.set(pk_getup, "getup", blend=True)
            getup_hold = 0
            phase_t0 = t
            auto_getup_count += 1
            cfg_g = cfg  # keep env in getup reward/clock mode
            _ = cfg_g

        # --- command / gesture domain per mode ---
        twist = np.zeros(3, np.float32)
        if mode == "getup":
            env.getup = True
            # oneshot clock already advanced by env.step when gesture getup
            if env.clip_phase >= 0.95 and tilt < getup_ok and float(env.data.qpos[env.free_qpos + 2]) > 0.12:
                getup_hold += 1
            else:
                getup_hold = 0
            if getup_hold >= int(getup_hold_s * rate) and (t - phase_t0) >= getup_s * 0.5:
                stood_once = True
                # return to previous locomotion / teleop
                target = prev_mode if prev_mode != "getup" else "walk"
                mode = target
                getup_hold = 0
                phase_t0 = t
                if target == "walk":
                    label = "walk"
                    env.getup = False
                    env.gesture_spec = None
                    # leave gesture clock off; walk_md has no clock requirement
                    cfg["env"].pop("gesture", None)
                    blend.set(pk_walk, "walk", blend=True)
                    walk_t0 = t
                else:
                    label = "teleop pose_cmd"
                    env.getup = False
                    # switch env to pose_cmd domain
                    env._setup_pose_cmd()
                    env.pose_stream = "clip"
                    env.pose_clip_name = "wave_right"
                    env.clip_phase = 0.0
                    env.pose_cmd_cur = env._pose_from_clip("wave_right", 0.0)
                    env.pose_cmd_tgt = env.pose_cmd_cur.copy()
                    env.pose_cmd_filt = env.pose_cmd_cur.copy()
                    env.pose_delay_buf = [env.pose_cmd_filt.copy()]
                    blend.set(pk_teleop, "teleop", blend=True)
                    teleop_t0 = t
        elif mode == "walk":
            # forward then turn
            elapsed = t - (walk_t0 or t)
            if elapsed < 3.0:
                twist = np.array([0.25, 0.0, 0.0], np.float32)
                label = "walk avanti"
            elif elapsed < walk_after_stand_s:
                twist = np.array([0.0, 0.0, 0.6], np.float32)
                label = "walk destra"
            elif elapsed < walk_after_stand_s + 1.0:
                # stop before handing over (switching policies mid-turn dropped the robot)
                twist = np.zeros(3, np.float32)
                label = "walk fermo"
            else:
                # hand off to teleop
                mode = "teleop"
                label = "teleop pose_cmd"
                env._setup_pose_cmd()
                env.pose_stream = "clip"
                env.pose_clip_name = "wave_right"
                env.clip_phase = 0.0
                env.pose_cmd_cur = env._pose_from_clip("wave_right", 0.0)
                env.pose_cmd_tgt = env.pose_cmd_cur.copy()
                env.pose_cmd_filt = env.pose_cmd_cur.copy()
                env.pose_delay_buf = [env.pose_cmd_filt.copy()]
                blend.set(pk_teleop, "teleop", blend=True)
                teleop_t0 = t
            # scheduled push to demonstrate auto-getup
            if not pushed and stood_once and walk_t0 is not None and (t - walk_t0) > 2.0 and auto_getup_count == 0:
                env.data.qvel[env.free_qvel:env.free_qvel + 2] += np.array([1.2, 0.3])
                pushed = True
                label = "spinta!"
        elif mode == "teleop":
            label = "teleop wave"
            if teleop_t0 is not None and (t - teleop_t0) > teleop_s:
                break

        env.command = twist
        # patch twist into obs before forward (firmware does the same)
        obs = obs.copy()
        obs[env.contract.twist_offset:env.contract.twist_offset + 3] = twist
        action = blend.act(obs)
        obs, _, fell, _, info = env.step(action)
        if fell:
            fell_flag = True

        if not log or log[-1][0] != label:
            log.append([label, t, t, blend.name])
        log[-1][2] = t + 1 / rate

        if renderer is not None and k % max(1, rate // 25) == 0:
            cam.lookat[:] = d.qpos[env.free_qpos:env.free_qpos + 3]
            renderer.update_scene(d, camera=cam)
            frame = renderer.render()
            title = f"Microban mix · policy={blend.name}"
            proc.stdin.write(_overlay(frame, label, t, info, twist, title).tobytes())

        k += 1

    if proc is not None:
        proc.stdin.close()
        proc.wait()
        print(f"video: {video}")

    return {
        "fell": fell_flag, "t_end": k / rate, "auto_getups": auto_getup_count,
        "phases": [(a, round(b, 2), round(c, 2), p) for a, b, c, p in log],
    }


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--getup", type=Path, required=True)
    ap.add_argument("--walk", type=Path, required=True)
    ap.add_argument("--teleop", type=Path, required=True)
    ap.add_argument("--video", type=Path, default=None)
    ap.add_argument("--side", choices=("prone", "supine"), default="prone")
    ap.add_argument("--blend-ms", type=float, default=500.0)
    args = ap.parse_args()
    for p in (args.getup, args.walk, args.teleop):
        if not p.is_file():
            raise SystemExit(f"missing policy: {p}")
    res = run_mix(args.getup, args.walk, args.teleop, args.video,
                  blend_ms=args.blend_ms, start_side=args.side)
    print(f"{'CADUTO' if res['fell'] else 'ok'} t={res['t_end']:.1f}s  auto-getup×{res['auto_getups']}")
    for lbl, a, b, pol in res["phases"]:
        print(f"    {lbl:22s} {a:5.1f}-{b:5.1f}s  [{pol}]")


if __name__ == "__main__":
    main()
