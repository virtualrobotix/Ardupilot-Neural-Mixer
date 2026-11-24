#!/usr/bin/env python3
# Author: Roberto Navoni, member of the ArduPilot Dev Team
# Contact: r.navoni74@gmail.com
# Developed by Roberto Navoni — DelphyAI LAB
"""Interactive MuJoCo viewer of the training phases of a .nnm policy (ArduPilot-contract env).

A training run folder holds one checkpoint per evaluation (`<name>_itNNNN.nnm`) plus `<name>_final.nnm`.
The viewer plays the selected checkpoint through a command schedule (stand, forward, backward, turn) with the
firmware int8 forward, so you can step through the phases of learning:

    ]  next checkpoint        [  previous checkpoint        R  restart        F  follow camera

With --low-dir the viewer also loads a second network trained for the low stance and crossfades the two
actions (a = (1-mix) * a_high + mix * a_low) along a switch schedule: walk high, switch to low, walk low,
switch back. Both networks share the contract q0, so the blend is what a mixer switch would do in the loop.

    .venv/bin/mjpython tools/robots/policy_viewer.py --robot q8bot \\
        --run-dir robots/q8bot/policies/walk_v1_run --low-dir robots/q8bot/policies/walk_low_run
"""

from __future__ import annotations

import argparse
import re
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import deploy_contract as dc  # noqa: E402
from export_nnm import unpack_nnm  # noqa: E402
from nnm_env import NNMixerEnv, apply_stance, load_ppo_config  # noqa: E402

# (label, (vx, vy, wz), seconds, mix at start, mix at end); mix 0 = high network, 1 = low network
DEMO = [
    ("in piedi", (0.0, 0.0, 0.0), 2.0, 0.0, 0.0),
    ("avanti", (0.08, 0.0, 0.0), 5.0, 0.0, 0.0),
    ("indietro", (-0.08, 0.0, 0.0), 5.0, 0.0, 0.0),
    ("rotazione sinistra", (0.0, 0.0, 0.6), 4.0, 0.0, 0.0),
    ("rotazione destra", (0.0, 0.0, -0.6), 4.0, 0.0, 0.0),
]
SWITCH = [
    ("alto: avanti", (0.08, 0.0, 0.0), 4.0, 0.0, 0.0),
    ("switch -> basso", (0.0, 0.0, 0.0), 2.5, 0.0, 1.0),
    ("basso: avanti", (0.07, 0.0, 0.0), 4.0, 1.0, 1.0),
    ("basso: indietro", (-0.06, 0.0, 0.0), 4.0, 1.0, 1.0),
    ("basso: rotazione", (0.0, 0.0, 0.6), 4.0, 1.0, 1.0),
    ("switch -> alto", (0.0, 0.0, 0.0), 2.5, 1.0, 0.0),
    ("alto: indietro", (-0.08, 0.0, 0.0), 4.0, 0.0, 0.0),
    ("alto: rotazione", (0.0, 0.0, 0.6), 4.0, 0.0, 0.0),
]


def scan(run_dir: Path) -> list[Path]:
    """Checkpoints of a run in training order: itNNNN ascending, then the final one.
    A single .nnm file is accepted too (one checkpoint)."""
    if run_dir.is_file():
        return [run_dir]
    its = sorted((p for p in run_dir.glob("*_it*.nnm") if re.search(r"_it\d+\.nnm$", p.name)),
                 key=lambda p: int(re.search(r"_it(\d+)\.nnm$", p.name).group(1)))
    return its + sorted(run_dir.glob("*_final.nnm"))


def segment_at(schedule, t: float):
    """(index, label, twist, mix) of the schedule at time t; the schedule loops."""
    total = sum(s[2] for s in schedule)
    t = t % total
    for i, (label, twist, dur, m0, m1) in enumerate(schedule):
        if t < dur:
            return i, label, np.array(twist, np.float32), m0 + (m1 - m0) * (t / dur)
        t -= dur
    label, twist, _, _, m1 = schedule[-1]
    return len(schedule) - 1, label, np.array(twist, np.float32), m1


def ckpt_label(p: Path) -> str:
    m = re.search(r"_it(\d+)\.nnm$", p.name)
    return f"iterazione {int(m.group(1))}" if m else p.stem.replace("_", " ")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--robot", required=True)
    ap.add_argument("--run-dir", type=Path, required=True, help="training run folder (high / default stance)")
    ap.add_argument("--low-dir", type=Path, default=None, help="run folder of the low-stance network (enables the switch)")
    ap.add_argument("--speed", type=float, default=1.0, help="playback speed (1 = real time)")
    ap.add_argument("--start", type=int, default=0, help="first checkpoint index (-1 = final)")
    ap.add_argument("--seconds", type=float, default=0.0, help="stop after this many seconds (0 = until closed)")
    args = ap.parse_args()

    ckpts = scan(args.run_dir)
    if not ckpts:
        sys.exit(f"no .nnm checkpoints in {args.run_dir}")
    low_pk = None
    if args.low_dir is not None:
        low = scan(args.low_dir)
        if not low:
            sys.exit(f"no .nnm checkpoints in {args.low_dir}")
        low_pk = unpack_nnm(low[-1].read_bytes())
        print(f"rete bassa: {low[-1].name}")
    schedule = SWITCH if low_pk is not None else DEMO

    cfg = load_ppo_config(args.robot)
    if low_pk is not None:
        apply_stance(cfg, "low")
    env = NNMixerEnv(args.robot, cfg, seed=0)
    env.max_steps = 10 ** 9
    rate = env.contract.rate_hz

    state = {"idx": args.start % len(ckpts), "restart": True, "follow": True}
    state["pk"] = unpack_nnm(ckpts[state["idx"]].read_bytes())

    def select(i: int) -> None:
        state["idx"] = i % len(ckpts)
        state["pk"] = unpack_nnm(ckpts[state["idx"]].read_bytes())
        state["restart"] = True
        print(f"[{state['idx'] + 1}/{len(ckpts)}] {ckpts[state['idx']].name}  ({ckpt_label(ckpts[state['idx']])})")

    def on_key(keycode: int) -> None:
        ch = chr(keycode).lower() if 0 < keycode < 256 else ""
        if ch == "]":
            select(state["idx"] + 1)
        elif ch == "[":
            select(state["idx"] - 1)
        elif ch == "r":
            state["restart"] = True
        elif ch == "f":
            state["follow"] = not state["follow"]

    import mujoco.viewer
    viewer = mujoco.viewer.launch_passive(env.model, env.data, key_callback=on_key,
                                          show_left_ui=False, show_right_ui=False)
    viewer.cam.distance, viewer.cam.elevation, viewer.cam.azimuth = 0.45, -20, 135
    print(f"{len(ckpts)} checkpoint in {args.run_dir}: ] avanti  [ indietro  R riavvia  F camera")
    print(f"[{state['idx'] + 1}/{len(ckpts)}] {ckpts[state['idx']].name}")

    obs = env.reset()
    k = 0
    last_seg = -1
    t_wall = time.perf_counter()
    d = env.data
    while viewer.is_running():
        if state["restart"]:
            obs = env.reset()
            k = 0
            last_seg = -1
            t_wall = time.perf_counter()
            state["restart"] = False
        t = k / rate
        seg, label, twist, mix = segment_at(schedule, t)
        if seg != last_seg:
            last_seg = seg
            print(f"  t={t:5.1f}s  {label}")
        if getattr(env, "gait_clock", False):
            hz = float(np.mean(env.gait_hz_range))
            env.set_gait_clock(hz if float(np.linalg.norm(twist)) > 1e-6 else 0.0)
            env.gait_hz_range = (hz, hz)
        env.command = twist
        obs[env.contract.twist_offset:env.contract.twist_offset + 3] = twist
        pk = state["pk"]
        a = dc.int8_forward(pk["layers"], pk["mean"], pk["std"], obs)
        if low_pk is not None and mix > 0.0:
            a_lo = dc.int8_forward(low_pk["layers"], low_pk["mean"], low_pk["std"], obs)
            a = (1.0 - mix) * a + mix * a_lo
        obs, _, fell, _, _ = env.step(a)
        if fell:
            print(f"  caduto a t={t:.1f}s: riavvio")
            state["restart"] = True
        with viewer.lock():
            if state["follow"]:
                viewer.cam.lookat[:] = d.qpos[env.free_qpos:env.free_qpos + 3]
        if hasattr(viewer, "set_texts"):
            import mujoco
            viewer.set_texts((mujoco.mjtFontScale.mjFONTSCALE_150, mujoco.mjtGridPos.mjGRID_TOPLEFT,
                              f"{ckpt_label(ckpts[state['idx']])}   {label}",
                              f"cmd vx={twist[0]:+.2f} wz={twist[2]:+.2f}   mix basso={mix:.2f}"))
        viewer.sync()
        k += 1
        if args.seconds and k / rate >= args.seconds:
            break
        dt = t_wall + (k / rate) / max(args.speed, 1e-3) - time.perf_counter()
        if dt > 0:
            time.sleep(dt)
    viewer.close()


if __name__ == "__main__":
    main()
