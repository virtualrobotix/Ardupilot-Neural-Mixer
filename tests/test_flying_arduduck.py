# Author: Roberto Navoni, member of the ArduPilot Dev Team
# Contact: r.navoni74@gmail.com
# Developed by Roberto Navoni — DelphyAI LAB
"""Flying ArduDuck: versioned MuJoCo scene, flight pack numbers and the hover controller."""
import sys
from pathlib import Path

import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco")

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools" / "robots"))
sys.path.insert(0, str(ROOT / "tools" / "unoq"))
sys.path.insert(0, str(ROOT / "unoq" / "app" / "python"))

import flying_arduduck_mjcf as fad  # noqa: E402

pytestmark = pytest.mark.skipif(not fad.XML_PATH.is_file(), reason="scene not generated")


def test_catalog_slot_and_contract():
    from common import ROBOT_INDEX, load_profile
    p = load_profile("flying_arduduck")
    assert ROBOT_INDEX["flying_arduduck"] == p["nnm_robot"] == 23
    x = load_profile("xgoduck")
    assert p["obs_dim"] == x["obs_dim"] == 61 and p["joint_names"] == x["joint_names"] and p["q0"] == x["q0"]
    assert p["flight"]["edf_count"] == 4 and abs(p["flight"]["mass_kg"] - 0.936) < 1e-9


def test_mass_budget_and_thrust():
    m, d = fad.load()
    pack = sum(m.body_mass[b] for b in range(m.nbody)
               if m.body(b).name.split("_")[0] in ("wing", "battery", "flight", "edf"))
    assert pack == pytest.approx(fad.PACK_KG, abs=1e-6)          # the fork's 936 g
    assert m.nu == 14 + 4
    total = float(m.body_subtreemass[1])
    assert 4 * fad.EDF_THRUST_N / (total * fad.G) > 1.3           # can lift off with margin


def test_thrust_centroid_under_the_centre_of_mass():
    m, d = fad.load()
    fad.set_stance(m, d)
    mujoco.mj_forward(m, d)
    com = d.subtree_com[1]
    sites = [d.site_xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, f"thrust_{n}")] for n, *_ in fad.EDFS]
    assert abs(float(np.mean([s[0] for s in sites])) - com[0]) < 1e-3
    assert abs(float(np.mean([s[1] for s in sites])) - com[1]) < 1e-3


def test_hover_reaches_and_holds_altitude():
    m, d = fad.load()
    log = fad.run_flight(m, d, seconds=7.0)
    hold = (log["t"] > 5.0)
    assert np.abs(log["z"][hold] - 0.60).max() < 0.01
    assert np.hypot(log["roll"], log["pitch"]).max() < 10.0
    assert np.abs(log["x"][hold]).max() < 0.15 and np.abs(log["y"][hold]).max() < 0.05
