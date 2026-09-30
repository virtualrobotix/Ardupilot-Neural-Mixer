#!/usr/bin/env python3
# Author: Roberto Navoni, member of the ArduPilot Dev Team
# Contact: r.navoni74@gmail.com
# Developed by Roberto Navoni — DelphyAI LAB
"""Record Stage-I discovery rollouts, slow them by K, save motions for Stage II.

    .venv/bin/python tools/robots/getup_record.py \
        --nnm robots/microban/policies/getup_discover_run/getup_discover_itXXXX.nnm \
        --slow 2 --trials 12

Writes robots/microban/motions/getup_prone.npz and getup_supine_roll.npz.
Optionally verifies the slowed clip open-loop on BAM servos (--check-bam).
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import deploy_contract as dc  # noqa: E402
import getup_clips as getup  # noqa: E402
from export_nnm import unpack_nnm  # noqa: E402
from nnm_env import NNMixerEnv, load_ppo_config  # noqa: E402


def _success(env: NNMixerEnv) -> bool:
    """Standing and quiet. No q0 tolerance: the discovered stand keeps the arms up as
    counterweights; the clip is blended to q0 afterwards (see _blend_to_q0)."""
    tilt = env.tilt_deg()
    z = float(env.data.qpos[env.free_qpos + 2])
    still = float(np.linalg.norm(env.data.qvel[env.free_qvel:env.free_qvel + 6]))
    return tilt < 12.0 and z > 0.85 * env.home_z and still < 0.8


def _blend_to_q0(q: np.ndarray, phase: np.ndarray, root_q: np.ndarray, root_z: np.ndarray,
                 dur: float, blend_s: float, home_z: float, hold_s: float = 0.5
                 ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, float]:
    """Append a min-jerk transition from the last recorded pose to q0, then a short hold."""
    dt = 1.0 / 50.0
    n_b = max(2, int(round(blend_s / dt)))
    n_h = int(round(hold_s / dt))
    q0 = np.array([getup.Q0[n] for n in getup.JOINT_NAMES], dtype=np.float64)
    s = np.linspace(0.0, 1.0, n_b + 1)[1:]
    mj = 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5
    q_b = q[-1][None, :] + mj[:, None] * (q0 - q[-1])[None, :]
    z_b = root_z[-1] + mj * (home_z - root_z[-1])
    q_all = np.concatenate([q, q_b, np.repeat(q0[None, :], n_h, axis=0)], axis=0)
    z_all = np.concatenate([root_z, z_b, np.full(n_h, home_z)])
    rq_all = np.concatenate([root_q, np.repeat(root_q[-1][None, :], n_b + n_h, axis=0)], axis=0)
    ph_all = np.linspace(0.0, 1.0, len(q_all))
    return q_all, ph_all, rq_all, z_all, dur + (n_b + n_h) * dt


def _jerk(q: np.ndarray, dt: float) -> float:
    if len(q) < 3:
        return 1e9
    a = np.diff(q, n=2, axis=0) / (dt * dt)
    return float(np.mean(np.sum(a ** 2, axis=1)))


def _smooth(q: np.ndarray, win: int = 5) -> np.ndarray:
    if win < 3 or len(q) < win:
        return q
    k = np.ones(win) / win
    out = np.empty_like(q)
    for j in range(q.shape[1]):
        out[:, j] = np.convolve(q[:, j], k, mode="same")
    # keep endpoints (start lying / end q0)
    out[0], out[-1] = q[0], q[-1]
    return out


def _slow(q: np.ndarray, phase: np.ndarray, root_q: np.ndarray, root_z: np.ndarray,
          k: float) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, float]:
    """Resample trajectory slower by factor k (HumanUP uses 8; Microban default 3)."""
    T = len(q)
    dt = 1.0 / 50.0
    dur = float(phase[-1] and (T - 1) * dt or (T - 1) * dt)
    new_T = max(T, int(round(T * k)))
    t_old = np.linspace(0.0, 1.0, T)
    t_new = np.linspace(0.0, 1.0, new_T)
    q_s = np.stack([np.interp(t_new, t_old, q[:, j]) for j in range(q.shape[1])], axis=1)
    z_s = np.interp(t_new, t_old, root_z)
    # nlerp each quaternion component then renormalise
    rq = np.stack([np.interp(t_new, t_old, root_q[:, j]) for j in range(4)], axis=1)
    n = np.linalg.norm(rq, axis=1, keepdims=True)
    rq = rq / np.maximum(n, 1e-9)
    return q_s, t_new, rq, z_s, dur * k


def rollout(env: NNMixerEnv, pk: dict, side: str, horizon: int) -> dict | None:
    env.getup_assist_n = 0.0
    env.getup_p_rsi = env.getup_p_stand = 0.0
    env.getup_p_prone = 1.0 if side == "prone" else 0.0
    env.getup_p_supine = 1.0 if side == "supine" else 0.0
    env.reset()
    env.getup_side = side
    env.clip_phase = 0.0
    env._place_getup(side, 0.0, settle=True)
    env.gravity = dc.GravityFilter(env.contract.att_tau)
    for _ in range(dc.AUTOPILOT_LOOP_HZ // 10):
        env._apply_target(env.q_wire)
        import mujoco
        mujoco.mj_step(env.model, env.data)
        g, a = env._imu_frd()
        env.gravity.update(g, a, 1.0 / dc.AUTOPILOT_LOOP_HZ)
    obs = env._observe()
    env.steps = 0
    qs, phases, rqs, rzs = [], [], [], []
    stood = False
    up_steps = 0
    for _ in range(horizon):
        act = dc.int8_forward(pk["layers"], pk["mean"], pk["std"], obs)
        obs, _, _, timeout, _ = env.step(act)
        qs.append(env.data.qpos[env.qpos_idx].copy())
        phases.append(float(env.clip_phase))
        rqs.append(env.data.qpos[env.free_qpos + 3:env.free_qpos + 7].copy())
        rzs.append(float(env.data.qpos[env.free_qpos + 2]))
        up_steps = up_steps + 1 if _success(env) else 0
        if up_steps >= int(1.0 * env.contract.rate_hz):
            stood = True
            break
        if timeout:
            break
    if not stood:
        return None
    q = np.asarray(qs, dtype=np.float64)
    ph = np.asarray(phases, dtype=np.float64)
    # normalise phase to [0, 1] over the recorded clip
    if ph[-1] > ph[0]:
        ph = (ph - ph[0]) / (ph[-1] - ph[0] + 1e-9)
    else:
        ph = np.linspace(0.0, 1.0, len(ph))
    return {
        "q": q,
        "phase": ph,
        "root_quat": np.asarray(rqs, dtype=np.float64),
        "root_z": np.asarray(rzs, dtype=np.float64),
        "jerk": _jerk(q, 1.0 / env.contract.rate_hz),
        "home_z": float(env.home_z),
    }


def check_bam_openloop(cfg: dict, robot: str, q: np.ndarray, duration_s: float, side: str = "prone") -> dict:
    """Replay joint targets open-loop on BAM XL330; report final trunk tilt / head height."""
    import mujoco
    cfg = dict(cfg)
    cfg.setdefault("env", {})["gesture"] = "getup_discover"
    env = NNMixerEnv(robot, cfg, seed=0)
    env.getup_assist_n = 0.0
    env.reset()
    env._place_getup(side, 0.0, settle=True)
    T = len(q)
    for i in range(T):
        env._apply_target(q[i])
        for _ in range(env.contract.loop_steps_per_policy):
            mujoco.mj_step(env.model, env.data)
    if env.head_id < 0:
        env._getup_head()
    return {
        "tilt_deg": env.tilt_deg(),
        "head_z": float(env.data.xpos[env.head_id, 2]),
        "z": float(env.data.qpos[env.free_qpos + 2]),
        "duration_s": duration_s,
    }


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--robot", default="microban")
    ap.add_argument("--nnm", type=Path, required=True)
    ap.add_argument("--out-dir", type=Path, default=None)
    ap.add_argument("--slow", type=float, default=3.0, help="slowdown factor K (G1 HumanUP uses 8)")
    ap.add_argument("--blend-s", type=float, default=1.5,
                    help="min-jerk transition from the discovered stand to q0 appended to the clip")
    ap.add_argument("--hold-s", type=float, default=0.5, help="hold at q0 after the blend (s)")
    ap.add_argument("--trials", type=int, default=12)
    ap.add_argument("--horizon", type=int, default=500)
    ap.add_argument("--check-bam", action="store_true")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    out_dir = args.out_dir or (Path(__file__).resolve().parents[2] / "robots" / "microban" / "motions")
    out_dir.mkdir(parents=True, exist_ok=True)
    pk = unpack_nnm(args.nnm.read_bytes())
    cfg = load_ppo_config(args.robot)
    cfg.setdefault("env", {})["gesture"] = "getup_discover"

    for side, fname in (("prone", "getup_prone.npz"), ("supine", "getup_supine_roll.npz")):
        best = None
        for t in range(args.trials):
            env = NNMixerEnv(args.robot, cfg, seed=args.seed + t * 17 + (0 if side == "prone" else 1000))
            env.max_steps = 10 ** 9
            rec = rollout(env, pk, side, args.horizon)
            if rec is None:
                print(f"  {side} trial {t}: fail")
                continue
            print(f"  {side} trial {t}: ok  jerk={rec['jerk']:.1f}  T={len(rec['q'])}")
            if best is None or rec["jerk"] < best["jerk"]:
                best = rec
        if best is None:
            print(f"FAIL: no successful {side} rollout — keep training Stage I")
            raise SystemExit(2)
        q, ph, rq, rz, dur = _slow(best["q"], best["phase"], best["root_quat"], best["root_z"], args.slow)
        q = _smooth(q, win=5)
        q, ph, rq, rz, dur = _blend_to_q0(q, ph, rq, rz, dur, args.blend_s, best["home_z"], hold_s=args.hold_s)
        path = out_dir / fname
        np.savez_compressed(path, q=q, phase=ph, root_quat=rq, root_z=rz,
                            duration_s=np.array(dur), slow=np.array(args.slow),
                            joint_names=np.array(getup.JOINT_NAMES))
        print(f"wrote {path}  T={len(q)}  duration={dur:.2f}s  jerk={best['jerk']:.1f}")
        if args.check_bam:
            stats = check_bam_openloop(cfg, args.robot, q, dur, side)
            print(f"  BAM open-loop: tilt={stats['tilt_deg']:.1f} deg  "
                  f"head_z={stats['head_z']:.3f}  z={stats['z']:.3f}")
            if stats["tilt_deg"] > 45.0 or stats["head_z"] < 0.15:
                print("  WARN: slowed clip does not stand open-loop; try a smaller --slow or re-record")

    print("done. Stage II: train with --gesture getup (loads these motions automatically).")


if __name__ == "__main__":
    main()
