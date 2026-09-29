#!/usr/bin/env python3
# Author: Roberto Navoni, member of the ArduPilot Dev Team
# Contact: r.navoni74@gmail.com
# Developed by Roberto Navoni — DelphyAI LAB
# For information: r.navoni74@gmail.com
"""Convert Booster Gym's published T1 policy (deploy/models/T1.pt, TorchScript) to the NNMixer contract.

Booster's actor observation (47): gravity 3, gyro 3, command 3, gait cos, gait sin, q-q0 12, 0.1*qd 12,
previous action 12. Ours (47): gyro 3, gravity 3, q-q0 12, qd 12, previous action 12, twist 3, extra
[sin, cos]. Same MLP (47-256-128-128-12, ELU), so the conversion is a permutation of the first-layer
columns, a 0.1 factor folded into the qd columns, and int8 per-row quantization. Booster clips the
action at +-1 rad: ppo.yaml env.act_max is 1.0 for this robot.

Booster's deploy zeroes the command and the gait clock while the joystick is centred (stand still).
On the firmware that is NNM_CLOCK_HZ 1 while walking; see the catalog notes.

    .venv/bin/python tools/robots/import_booster_t1.py            # -> robots/booster_t1/policies/walk_booster.nnm
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import REPO_ROOT, load_profile, robot_dir  # noqa: E402
from export_nnm import pack_nnm, unpack_nnm  # noqa: E402
import deploy_contract as dc  # noqa: E402

ROBOT = "booster_t1"
DEFAULT_PT = REPO_ROOT / "third_party" / ROBOT / "deploy" / "models" / "T1.pt"


def booster_to_nnm_columns(nj: int) -> tuple[np.ndarray, np.ndarray]:
    """For each column of our observation, the Booster column it comes from and its scale factor."""
    ours = np.zeros(6 + 3 * nj + 3 + 2, dtype=int)
    scale = np.ones_like(ours, dtype=np.float32)
    ours[0:3] = np.arange(3, 6)                               # gyro
    ours[3:6] = np.arange(0, 3)                               # gravity
    ours[6:6 + nj] = np.arange(11, 11 + nj)                   # q - q0
    ours[6 + nj:6 + 2 * nj] = np.arange(23, 23 + nj)          # qd (Booster feeds 0.1 * qd)
    scale[6 + nj:6 + 2 * nj] = 0.1
    ours[6 + 2 * nj:6 + 3 * nj] = np.arange(35, 35 + nj)      # previous action
    tw = 6 + 3 * nj
    ours[tw:tw + 3] = np.arange(6, 9)                         # command
    ours[tw + 3] = 10                                         # extra[0] = sin
    ours[tw + 4] = 9                                          # extra[1] = cos
    return ours, scale


def main() -> None:
    import torch

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pt", type=Path, default=DEFAULT_PT)
    ap.add_argument("--out", type=Path, default=robot_dir(ROBOT) / "policies" / "walk_booster.nnm")
    args = ap.parse_args()
    profile = load_profile(ROBOT)
    nj, obs_dim = int(profile["n_joints"]), int(profile["obs_dim"])
    pol = torch.jit.load(str(args.pt), map_location="cpu").eval()
    sd = pol.state_dict()
    keys = sorted({k.split(".")[0] for k in sd}, key=int)
    Ws = [sd[f"{k}.weight"].numpy().astype(np.float32) for k in keys]
    bs = [sd[f"{k}.bias"].numpy().astype(np.float32) for k in keys]
    if Ws[0].shape[1] != obs_dim or Ws[-1].shape[0] != nj:
        raise SystemExit(f"{args.pt}: {Ws[0].shape[1]} inputs / {Ws[-1].shape[0]} outputs, "
                         f"profile has {obs_dim} / {nj}")
    src, scale = booster_to_nnm_columns(nj)
    W0 = Ws[0][:, src] * scale[None, :]
    layers = [{"W": W0 if i == 0 else W, "b": b, "act": i < len(Ws) - 1} for i, (W, b) in enumerate(zip(Ws, bs))]
    mean, std = np.zeros(obs_dim, np.float32), np.ones(obs_dim, np.float32)
    blob = pack_nnm(ROBOT, mean, std, layers, np.asarray(profile["q0"], np.float32))
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_bytes(blob)
    # parity: our int8 forward on our layout against the TorchScript on Booster's layout
    pk = unpack_nnm(blob)
    rng = np.random.default_rng(0)
    worst = 0.0
    for _ in range(200):
        ob = rng.normal(0, 0.5, 47).astype(np.float32)
        # our layout holds raw qd; Booster's column is 0.1*qd, so divide it back out
        ours = (ob[src] / scale).astype(np.float32)
        with torch.no_grad():
            ref = pol(torch.from_numpy(ob)[None])[0].numpy()
        got = dc.int8_forward(pk["layers"], pk["mean"], pk["std"], ours)
        worst = max(worst, float(np.max(np.abs(got - ref))))
    print(f"wrote {args.out} ({len(blob)} bytes): layers {[W.shape for W in Ws]}, "
          f"max |int8 - torch| over 200 random observations = {worst:.4f} rad")


if __name__ == "__main__":
    main()
