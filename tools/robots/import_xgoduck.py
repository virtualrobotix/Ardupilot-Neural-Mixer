#!/usr/bin/env python3
# Author: Roberto Navoni, member of the ArduPilot Dev Team
# Contact: r.navoni74@gmail.com
# Developed by Roberto Navoni — DelphyAI LAB
# For information: r.navoni74@gmail.com
"""Convert the eight XDuck (XGO Duck on Arduino Uno Q) ONNX policies to the NNMixer contract.

Same network as MicroDuck (61-512-256-128-14, ELU, baked normalizer) and same joint order. Three
things differ from the firmware contract and are folded into the weights, so nothing is retrained:

  observation   XDuck feeds the head command in columns 51..54 and zeros in 55..60. The firmware
                keeps 51..52 for the gesture clock and 53..60 for the NNM_POSE channels. The first
                layer is rewritten: clock columns get zero weight, the head columns move to 53..56,
                the constant contribution of the always-zero columns goes into the bias.
  action scale  roller and kicks run at 0.8 in the XDuck deployment. The last layer is scaled, and
                the previous-action columns are rescaled so the network still sees the raw action.
  home pose     the deployment home (rl_core.py), not each file's default_joint_pos metadata:
                22 deg hips and ankles for every policy, a fixed leg home for the roller.
                --home metadata keeps the metadata instead.

The files are numbered so NNM_POLICY, NNM_GETUP_IDX and the skill table have stable indices.

    .venv/bin/python tools/robots/import_xgoduck.py --src "XDucK-Ardupilot AP_NNMixer.zip"
    .venv/bin/python tools/robots/import_xgoduck.py --src third_party/xgoduck/python --check-c

Writes robots/xgoduck/policies/<n>_<name>.nnm and prints, per policy, the error of the rewrite in
float32 (must be rounding noise) and of the int8 pack against the original ONNX run by onnxruntime
on observations laid out as the XDuck deployment builds them.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import tempfile
import zipfile
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import REPO_ROOT, load_profile, robot_dir  # noqa: E402
from export_nnm import pack_nnm, parse_onnx_model, unpack_nnm  # noqa: E402
import deploy_contract as dc  # noqa: E402

ROBOT = "xgoduck"
DEG = np.pi / 180.0

# XDuck deployment home, policy joint order (rl_core.DEFAULT_DEG through POLICY_INDEX)
HOME_DEG = np.array([0, -5, -22, 0, 22, 20, 20, 0, 0, 0, 5, 22, 0, -22], dtype=np.float64)
# rl_core.ROLLER_LEG_DEFAULT_DEG, policy joint order; neck and head keep the model metadata
ROLLER_LEG_DEG = {0: 0.0, 1: 2.0, 2: -8.5, 3: 8.5, 4: 17.0, 9: 0.0, 10: -2.0, 11: 8.5, 12: -8.5, 13: -17.0}

# (output name, XDuck file, action scale, home: "deploy" | "roller", feeds the head command)
POLICIES = [
    ("0_walk", "xgoduck_walk.onnx", 1.0, "deploy", True),
    ("1_getup", "xgoduck_getup.onnx", 1.0, "deploy", False),
    ("2_roller", "xgoduck_roller.onnx", 0.8, "roller", False),
    ("3_pick", "xgoduck_pick.onnx", 1.0, "deploy", False),
    ("4_roulade", "xgoduck_roulade.onnx", 1.0, "deploy", False),
    ("5_sitstand", "xgoduck_sitstand.onnx", 1.0, "deploy", True),
    ("6_kick_left", "xgoduck_ballkick_left.onnx", 0.8, "deploy", False),
    ("7_kick_right", "xgoduck_ballkick_right.onnx", 0.8, "deploy", False),
]

NJ = 14
OBS = 61
A_PREV = slice(6 + 2 * NJ, 6 + 3 * NJ)          # 34..47
TWIST = 6 + 3 * NJ                              # 48
X_HEAD = slice(TWIST + 3, TWIST + 7)            # XDuck 51..54
X_ZERO = slice(TWIST + 7, OBS)                  # XDuck 55..60, always zero in the deployment
F_CLOCK = slice(TWIST + 3, TWIST + 5)           # firmware 51..52
F_HEAD = slice(TWIST + 5, TWIST + 9)            # firmware 53..56 = NNM_POSE channels 0..3
F_REST = slice(TWIST + 9, OBS)                  # firmware 57..60 = NNM_POSE channels 4..7


def read_sources(src: Path) -> dict[str, bytes]:
    """ONNX bytes by file name, from a directory or from the XDuck zip."""
    names = {p[1] for p in POLICIES}
    out: dict[str, bytes] = {}
    if src.is_dir():
        for p in src.rglob("xgoduck_*.onnx"):
            if p.name in names and not p.name.startswith("._"):
                out[p.name] = p.read_bytes()
    elif zipfile.is_zipfile(src):
        with zipfile.ZipFile(src) as zf:
            for n in zf.namelist():
                base = n.rsplit("/", 1)[-1]
                if base in names and "/._" not in n and "__MACOSX" not in n:
                    out[base] = zf.read(n)
    else:
        raise SystemExit(f"{src}: not a directory and not a zip")
    missing = sorted(names - set(out))
    if missing:
        raise SystemExit(f"{src}: missing {', '.join(missing)}")
    return out


def home_for(kind: str, meta: dict, use_metadata: bool) -> np.ndarray:
    meta_home = np.array([float(v) for v in meta["default_joint_pos"].split(",")], dtype=np.float64)
    if meta_home.size != NJ:
        raise SystemExit(f"default_joint_pos has {meta_home.size} values, expected {NJ}")
    if use_metadata:
        return meta_home.astype(np.float32)
    if kind == "roller":
        home = meta_home.copy()
        for i, deg in ROLLER_LEG_DEG.items():
            home[i] = deg * DEG
        return home.astype(np.float32)
    return (HOME_DEG * DEG).astype(np.float32)


def to_firmware(mean, std, layers, scale: float):
    """Rewrite the float layers for the firmware observation layout and fold the action scale."""
    mean = mean.astype(np.float64)
    std = std.astype(np.float64)
    W0 = layers[0]["W"].astype(np.float64)
    b0 = layers[0]["b"].astype(np.float64)
    # XDuck columns 55..60 are always zero: their normalized value is a constant -> bias
    b0 = b0 + W0[:, X_ZERO] @ ((0.0 - mean[X_ZERO]) / std[X_ZERO])
    W0n = np.zeros_like(W0)
    mean_n = np.zeros(OBS)
    std_n = np.ones(OBS)
    W0n[:, :TWIST + 3] = W0[:, :TWIST + 3]
    mean_n[:TWIST + 3] = mean[:TWIST + 3]
    std_n[:TWIST + 3] = std[:TWIST + 3]
    W0n[:, F_HEAD] = W0[:, X_HEAD]
    mean_n[F_HEAD] = mean[X_HEAD]
    std_n[F_HEAD] = std[X_HEAD]
    # firmware feeds scale * raw action as previous action; the network was trained on the raw one
    mean_n[A_PREV] *= scale
    std_n[A_PREV] *= scale
    out = [dict(l) for l in layers]
    out[0] = {"W": W0n.astype(np.float32), "b": b0.astype(np.float32), "act": layers[0]["act"]}
    out[-1] = {"W": (out[-1]["W"].astype(np.float64) * scale).astype(np.float32),
               "b": (out[-1]["b"].astype(np.float64) * scale).astype(np.float32), "act": out[-1]["act"]}
    return mean_n.astype(np.float32), std_n.astype(np.float32), out


def xduck_observations(mean, std, n: int, rng, head: bool) -> np.ndarray:
    """Observations as the XDuck deployment builds them: live channels around the training
    statistics, head command within its +-1 range when the policy uses it, zeros in 55..60."""
    x = (mean + std * rng.standard_normal((n, OBS))).astype(np.float32)
    x[:, X_HEAD] = rng.uniform(-1.0, 1.0, (n, 4)).astype(np.float32) if head else 0.0
    x[:, X_ZERO] = 0.0
    return x


def firmware_observations(x: np.ndarray, scale: float, rng) -> np.ndarray:
    """The same instants in the firmware layout. The clock columns get arbitrary values: the
    converted policy must ignore them (the get-up one-shot clock writes there)."""
    f = np.zeros_like(x)
    f[:, :TWIST + 3] = x[:, :TWIST + 3]
    f[:, A_PREV] = scale * x[:, A_PREV]
    f[:, F_CLOCK] = rng.uniform(-1.0, 1.0, (x.shape[0], 2)).astype(np.float32)
    f[:, F_HEAD] = x[:, X_HEAD]
    return f


def float_forward(mean, std, layers, obs):
    x = (obs.astype(np.float64) - mean) / std
    for l in layers:
        x = x @ l["W"].astype(np.float64).T + l["b"]
        if l["act"]:
            x = np.where(x > 0, x, np.expm1(np.minimum(x, 0)))
    return x


def c_forward(exe: Path, nnm: Path, obs: np.ndarray, tmp: Path) -> np.ndarray:
    (tmp / "obs.f32").write_bytes(np.ascontiguousarray(obs, np.float32).tobytes())
    subprocess.check_call([str(exe), str(nnm), str(tmp / "obs.f32"), str(tmp / "act.f32")])
    return np.frombuffer((tmp / "act.f32").read_bytes(), np.float32).reshape(obs.shape[0], NJ)


def main() -> None:
    import onnx
    import onnxruntime as ort

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--src", type=Path, default=REPO_ROOT / "third_party" / ROBOT / "python",
                    help="XDuck zip, or a directory holding python/xgoduck_*.onnx")
    ap.add_argument("--out-dir", type=Path, default=robot_dir(ROBOT) / "policies")
    ap.add_argument("--home", choices=("deploy", "metadata"), default="deploy",
                    help="deploy: the XDuck runtime home (default); metadata: each ONNX default_joint_pos")
    ap.add_argument("--samples", type=int, default=2000)
    ap.add_argument("--check-c", action="store_true",
                    help="also run the firmware C forward (tools/nnmixer_infer.c) on the written files")
    ap.add_argument("--report", type=Path, default=None, help="write the parity table as JSON")
    args = ap.parse_args()

    profile = load_profile(ROBOT)
    if int(profile["n_joints"]) != NJ or int(profile["obs_dim"]) != OBS:
        raise SystemExit(f"profile {ROBOT}: expected {NJ} joints and observation {OBS}")
    sources = read_sources(args.src)
    args.out_dir.mkdir(parents=True, exist_ok=True)

    exe = None
    tmp_ctx = tempfile.TemporaryDirectory()
    tmp = Path(tmp_ctx.name)
    if args.check_c:
        exe = tmp / "parity_nnm"
        subprocess.check_call(["cc", "-O2", "-I", str(REPO_ROOT / "tools"),
                               str(REPO_ROOT / "tools/robots/parity_nnm.c"),
                               str(REPO_ROOT / "tools/nnmixer_infer.c"), "-lm", "-o", str(exe)])

    rows = []
    for name, fname, scale, home_kind, head in POLICIES:
        data = sources[fname]
        model = onnx.load_model_from_string(data)
        mean, std, layers, _, meta = parse_onnx_model(model)
        dims = [layers[0]["W"].shape[1]] + [l["W"].shape[0] for l in layers]
        if dims[0] != OBS or dims[-1] != NJ:
            raise SystemExit(f"{fname}: network {dims}, expected {OBS} inputs and {NJ} outputs")
        want = ",".join(profile["joint_names"])
        if meta.get("joint_names") != want:
            raise SystemExit(f"{fname}: joint order differs from the {ROBOT} profile")
        home = home_for(home_kind, meta, args.home == "metadata")
        mean_f, std_f, layers_f = to_firmware(mean, std, layers, scale)
        blob = pack_nnm(ROBOT, mean_f, std_f, layers_f, home)
        out = args.out_dir / f"{name}.nnm"
        out.write_bytes(blob)

        rng = np.random.default_rng(0)
        x = xduck_observations(mean, std, args.samples, rng, head)
        f = firmware_observations(x, scale, rng)
        sess = ort.InferenceSession(data, providers=["CPUExecutionProvider"])
        inp = sess.get_inputs()[0].name
        ref = scale * np.vstack([sess.run(None, {inp: x[i:i + 1]})[0] for i in range(x.shape[0])])
        y_float = float_forward(mean_f, std_f, layers_f, f)
        pk = unpack_nnm(blob)
        y_int8 = np.stack([dc.int8_forward(pk["layers"], pk["mean"], pk["std"], o) for o in f])
        amp = float(np.sqrt(np.mean(ref ** 2)))
        row = {
            "file": out.name, "source": fname, "bytes": len(blob), "action_scale": scale,
            "home_deg": [round(float(v), 2) for v in np.rad2deg(home)],
            "remap_max_rad": float(np.max(np.abs(y_float - ref))),
            "int8_max_rad": float(np.max(np.abs(y_int8 - ref))),
            "int8_rms_rad": float(np.sqrt(np.mean((y_int8 - ref) ** 2))),
            "action_rms_rad": amp,
            "int8_rel": float(np.sqrt(np.mean((y_int8 - ref) ** 2)) / amp),
        }
        if exe is not None:
            y_c = c_forward(exe, out, f, tmp)
            row["c_vs_python_max_rad"] = float(np.max(np.abs(y_c - y_int8)))
            row["c_max_rad"] = float(np.max(np.abs(y_c - ref)))
        rows.append(row)

    tmp_ctx.cleanup()
    print(f"{len(rows)} policies -> {args.out_dir}  (home: {args.home}, {args.samples} observations each)")
    hdr = f"{'file':18s} {'KB':>5s} {'scale':>5s} {'remap max':>10s} {'int8 max':>9s} {'int8 rms':>9s} {'act rms':>8s} {'rel':>6s}"
    if exe is not None:
        hdr += f" {'C-py max':>9s}"
    print(hdr)
    for r in rows:
        line = (f"{r['file']:18s} {r['bytes'] / 1024:5.0f} {r['action_scale']:5.2f} {r['remap_max_rad']:10.2e} "
                f"{r['int8_max_rad']:9.4f} {r['int8_rms_rad']:9.4f} {r['action_rms_rad']:8.3f} "
                f"{100 * r['int8_rel']:5.2f}%")
        if exe is not None:
            line += f" {r['c_vs_python_max_rad']:9.2e}"
        print(line)
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps({"home": args.home, "samples": args.samples, "policies": rows},
                                          indent=2) + "\n")
        print(f"report: {args.report}")


if __name__ == "__main__":
    main()
