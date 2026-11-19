# Author: Roberto Navoni, member of the ArduPilot Dev Team
# Contact: r.navoni74@gmail.com
# Developed by Roberto Navoni — DelphyAI LAB
# For information: r.navoni74@gmail.com
"""The XGO Duck training body behind the simulator window. Needs third_party/xgoduck_rl and bam."""
import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools' / 'unoq'))

pytest.importorskip('mujoco')
pytest.importorskip('bam.model')
scene = pytest.importorskip('xgoduck_scene')

if not (scene.RL_ROOT / scene.ROBOT_DIR / 'robot_allcollisions_backlash.xml').is_file():
    pytest.skip('third_party/xgoduck_rl is not cloned', allow_module_level=True)

WALK_HOME_DEG = (0, -5, -22, -0.28, 22, 20, 20, 0, 0, 0, 5, 22, 0.28, -22)


def test_body_matches_training():
    duck = scene.XgoDuck()
    m = duck.model
    assert m.nu == 14 and len(duck.play_qpos) == 14
    # the backpack moves 55 g out of the trunk: the total does not change
    assert sum(m.body_mass) == pytest.approx(0.7906, abs=2e-3)
    stack = m.body('electronics_stack')
    assert float(stack.mass[0]) == pytest.approx(scene.STACK_MASS)
    assert float(m.body('trunk_base').mass[0]) == pytest.approx(0.25 - scene.STACK_MASS)


def test_stands_on_walk_home_with_the_servo_model():
    duck = scene.XgoDuck()
    duck.reset(pose_deg=WALK_HOME_DEG, z=0.139158)
    target = [math.radians(v) for v in WALK_HOME_DEG]
    for _ in range(300):
        duck.step(target, 0.0)
    assert duck.tilt_deg() < 5.0
    # the encoder reads through the play, within the ±1.5 degrees of the gears and the servo droop
    for read, want in zip(duck.encoder(), target):
        assert abs(math.degrees(read - want)) < 8.0


def test_torque_off_keeps_the_pose():
    duck = scene.XgoDuck()
    before = duck.data.qpos.copy()
    for _ in range(100):
        duck.step(None, 0.0)
    assert (duck.data.qpos == before).all()
