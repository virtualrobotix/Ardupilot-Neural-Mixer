#!/usr/bin/env python3
# Author: Roberto Navoni, member of the ArduPilot Dev Team
# Contact: r.navoni74@gmail.com
# Developed by Roberto Navoni — DelphyAI LAB
# For information: r.navoni74@gmail.com
"""Widen a .nnm to the robot's current observation size with zero-weight columns.

When a robot gains extra observation channels after the twist (e.g. Microban's gesture clock),
policies trained on the old layout stop loading: the firmware checks obs_dim against robot.bin.
The new channels get mean 0, std 1 and zero weights in the first layer, so the network output is
bit-identical to the old one for any value of the new inputs.

Usage:
    python tools/robots/pad_nnm_obs.py --robot microban robots/microban/policies/walk_md.nnm
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import ROBOT_INDEX, load_profile  # noqa: E402
from export_nnm import pack_nnm_quantized, unpack_nnm  # noqa: E402


def pad_nnm(blob: bytes, obs_dim: int, n_joints: int) -> bytes:
    pk = unpack_nnm(blob)
    old = int(pk["mean"].size)
    if old == obs_dim:
        return blob
    if old > obs_dim:
        raise SystemExit(f"{pk['robot_id']}: file obs {old} is wider than the profile {obs_dim}")
    add = obs_dim - old
    twist_end = 6 + 3 * n_joints + 3
    if old < twist_end:
        raise SystemExit(f"file obs {old} is shorter than the base layout {twist_end}")
    # new channels sit after the twist and any extra channels the file already had
    mean = np.concatenate([pk["mean"], np.zeros(add, np.float32)])
    std = np.concatenate([pk["std"], np.ones(add, np.float32)])
    qlayers = []
    for k, (Wq, scale, b, act) in enumerate(pk["layers"]):
        if k == 0:
            Wq = np.concatenate([Wq, np.zeros((Wq.shape[0], add), np.int8)], axis=1)
        qlayers.append({"Wq": Wq, "w_scale": scale, "b": b, "act": act})
    return pack_nnm_quantized(pk["robot_id"], mean, std, qlayers, pk["default_pose"])


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("files", type=Path, nargs="+")
    ap.add_argument("--robot", required=True, choices=sorted(ROBOT_INDEX))
    args = ap.parse_args()
    profile = load_profile(args.robot)
    obs_dim, n = int(profile["obs_dim"]), int(profile["n_joints"])
    for f in args.files:
        blob = f.read_bytes()
        out = pad_nnm(blob, obs_dim, n)
        if out is blob:
            print(f"{f}: already {obs_dim} inputs")
            continue
        f.write_bytes(out)
        print(f"{f}: {unpack_nnm(blob)['mean'].size} -> {obs_dim} inputs")


if __name__ == "__main__":
    main()
