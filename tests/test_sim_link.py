"""MuJoCo connector: upright gravity, a state packet the autopilot can parse, and a joint that follows."""
import struct
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'unoq' / 'app' / 'python'))

import pytest

mujoco = pytest.importorskip('mujoco')
from sim_link import COMMAND, STATE, SimLink  # noqa: E402


def test_scene_stands_and_reports_gravity():
    link = SimLink()
    if not link.available:
        pytest.skip(link.error)
    try:
        link.set_mode('simulate')
        time.sleep(1.5)
        st = link.status()
        assert st['mode'] == 'simulate'
        assert st['gravity_g'][2] < -0.8
        assert st['tilt_deg'] < 20
        assert len(st['joints_deg']) == 15
        packet = link._duck.packets(False)[0]
        assert len(packet) == 240 and packet[:4] == b'DQS1'
        version, flags, mask = struct.unpack_from('<BBH', packet, 4)
        assert version == 1 and flags & 1 and mask == 0x7FFF
        # a command in policy order moves the left hip roll (slot 1) and is not forwarded
        targets = [0.0] * 15
        targets[1] = 20.0
        packet = COMMAND.pack(b'DQC1', 1, 0, 0, 2, 0, 0, *targets)
        assert link.take_command(packet) is True
        time.sleep(0.5)
        rolled = link.status()['joints_deg'][1]
        assert rolled > 5
    finally:
        link.set_mode('off')


def test_state_struct_matches_the_packet():
    assert STATE.size == 240
