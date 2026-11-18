# Author: Roberto Navoni, member of the ArduPilot Dev Team
# Contact: r.navoni74@gmail.com
# Developed by Roberto Navoni — DelphyAI LAB
# For information: r.navoni74@gmail.com
"""Dense motion clips in the training env, and the frax.ik retargeter (skipped without frax).

    .venv/bin/python -m pytest tests/test_retarget_frax.py -q
    .venv-frax/bin/python -m pytest tests/test_retarget_frax.py -q
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools" / "robots"))
sys.path.insert(0, str(ROOT / "tools" / "mocap"))


def _motion_env(tmp_path, oneshot=False, amplitude=0.5):
    pytest.importorskip("mujoco")
    pytest.importorskip("bam")
    from nnm_env import NNMixerEnv, load_ppo_config

    cfg = load_ppo_config("microban")
    probe = NNMixerEnv("microban", cfg)
    names = probe.profile["joint_names"]
    q0 = np.asarray(probe.contract.q0)
    t = np.arange(50) / 50
    q = np.tile(q0, (50, 1))
    j = names.index("right_shoulder_pitch")
    q[:, j] += amplitude * np.sin(2 * np.pi * t)
    clip = tmp_path / "clip.npz"
    np.savez(clip, q=q, joint_names=np.array(names), fps=25.0, oneshot=oneshot)
    cfg.setdefault("env", {})["gesture"] = "motion"
    cfg["env"]["motion_file"] = str(clip)
    return NNMixerEnv("microban", cfg), q, j


def test_motion_clip_reference(tmp_path):
    env, q, j = _motion_env(tmp_path)
    assert env.gesture_spec["hz"] == pytest.approx(0.5)
    assert [env.profile["joint_names"][i] for i in env.gesture_ids] == ["right_shoulder_pitch"]
    for k in (0, 7, 31, 49):
        env.clip_phase = k / 50
        assert np.allclose(env._gesture_reference(None), q[k])
    env.clip_phase = 0.99
    assert np.allclose(env._gesture_reference(None), 0.5 * (q[49] + q[0]))


def test_motion_clip_clipped_to_act_max(tmp_path):
    env, _, j = _motion_env(tmp_path, amplitude=5.0)
    q0 = np.asarray(env.contract.q0)
    assert np.max(np.abs(env.motion_traj["q"] - q0)) <= env.contract.act_max + 1e-9


def test_frax_urdf_matches_mujoco(tmp_path):
    pytest.importorskip("frax")
    import frax
    import mujoco
    import retarget_frax as rf
    from common import load_profile, resolve_mjcf

    profile = load_profile("microban")
    names = list(profile["joint_names"])
    mjcf = resolve_mjcf("microban", profile)
    if mjcf is None or not mjcf.is_file():
        pytest.skip("microban MJCF not fetched")
    urdf, _ = rf.mjcf_to_urdf(mjcf, names, vel_max=6.0)
    urdf_path = tmp_path / "robot.urdf"
    urdf_path.write_text(urdf)
    robot = frax.Robot(str(urdf_path), floating_base="quaternion")
    model = mujoco.MjModel.from_xml_path(str(mjcf))
    data = mujoco.MjData(model)
    jidx = robot.joint_name_to_index
    col = {n: 7 + robot.joint_names[robot.nv_floating:].index(n) for n in names}
    rng = np.random.default_rng(0)
    for _ in range(5):
        qj = rng.uniform(-0.5, 0.5, len(names))
        q = np.array(robot.default_configuration, dtype=np.float64)
        q[:7] = [0, 0, 0, 1, 0, 0, 0]
        data.qpos[:] = model.qpos0
        data.qpos[:7] = [0, 0, 0, 1, 0, 0, 0]
        for n, v in zip(names, qj):
            q[col[n]] = v
            data.qpos[model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)]] = v
        mujoco.mj_kinematics(model, data)
        tfs = np.asarray(robot.joint_to_world_transforms(q))
        for n in names:
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)
            assert np.allclose(tfs[jidx[n]][:3, 3], data.xanchor[jid], atol=1e-6), n
