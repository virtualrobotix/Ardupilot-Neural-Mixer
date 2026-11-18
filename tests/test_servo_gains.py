# Author: Roberto Navoni, member of the ArduPilot Dev Team
# Contact: r.navoni74@gmail.com
# Developed by Roberto Navoni — DelphyAI LAB
# For information: r.navoni74@gmail.com
"""skills.bin version 2 (per-servo KP/KD table) written by tools/robots/pack_robot_bin.py.

The C++ parser (ardupilot/libraries/AP_NNMixer/AP_NNMixer_Policy.cpp) reads the same bytes; the
layout checked here is the one it expects: header, policies, skills, then n u8, mask u16, kp[n], kd[n].

    .venv/bin/python -m pytest tests/test_servo_gains.py -q
"""

from __future__ import annotations

import copy
import struct
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools" / "robots"))

import pack_robot_bin as P  # noqa: E402

HEADER = 8 + 4 + 8 + 32 + 4
MOUTH_ONLY = [{"id": i, "kp": None, "kd": None} for i in P.GAIN_SLOT_IDS[:14]] + [{"id": 34, "kp": 10, "kd": 20}]


def _profile(**extra):
    """Minimal profile with one policy, one skill and the deployment extras."""
    prof = {
        "n_joints": 14,
        "pose_limits": [0.5] * 8,
        "aux_max": 30.0,
        "deploy": {
            "head_joints": (10, 4),
            "alpha_leg": 0.35,
            "alpha_head": 0.45,
            "policies": [{"policy": 0, "wz_max": 0.0}],
        },
        "skills": [{"id": 1, "policy": 0, "duration_s": 2.0, "aux_deg": 0.0, "aux_until": 0.0}],
    }
    prof.update(extra)
    return prof


def _body_len(data: bytes) -> int:
    n_policy, n_skill = data[5], data[6]
    return HEADER + 20 * n_policy + 28 * n_skill


def test_v1_when_no_gains_is_unchanged_layout():
    data = P.pack_skills(_profile())
    assert data is not None
    assert data[:4] == b"NNMS" and data[4] == 1
    assert len(data) == _body_len(data)


def test_v2_appends_the_gain_tail_and_round_trips():
    prof = _profile(servo_gains=MOUTH_ONLY)
    data = P.pack_skills(prof)
    assert data[4] == 2
    body = _body_len(data)
    assert len(data) == body + 3 + 2 * 15
    n = data[body]
    mask = struct.unpack_from("<H", data, body + 1)[0]
    kp = list(data[body + 3:body + 3 + n])
    kd = list(data[body + 3 + n:body + 3 + 2 * n])
    assert n == 15
    assert mask == 1 << 14                      # only the mouth (slot 14) has a value
    assert kp[14] == 10 and kd[14] == 20
    assert all(v == 0 for v in kp[:14] + kd[:14])


def test_unset_slots_keep_their_bit_clear():
    entries = [dict(e) for e in MOUTH_ONLY]
    entries[0] = {"id": 10, "kp": 7, "kd": 30}
    tail = P.pack_servo_gains(entries)
    n, mask = struct.unpack_from("<BH", tail, 0)
    assert n == 15 and mask == (1 << 0) | (1 << 14)


def test_half_set_gain_is_refused():
    entries = [dict(e) for e in MOUTH_ONLY]
    entries[3] = {"id": 13, "kp": 5, "kd": None}
    with pytest.raises(SystemExit):
        P.pack_servo_gains(entries)


def test_gain_out_of_byte_range_is_refused():
    entries = [dict(e) for e in MOUTH_ONLY]
    entries[14] = {"id": 34, "kp": 256, "kd": 20}
    with pytest.raises(SystemExit):
        P.pack_servo_gains(entries)


def test_ids_must_follow_mcu_slot_order():
    entries = copy.deepcopy(MOUTH_ONLY)
    entries[0], entries[1] = entries[1], entries[0]
    with pytest.raises(SystemExit):
        P.pack_servo_gains(entries)


def test_xgoduck_profile_carries_the_mouth_gains():
    import json

    prof = json.loads((ROOT / "robots" / "xgoduck" / "robot" / "profile.json").read_text())
    gains = prof["servo_gains"]
    assert [g["id"] for g in gains] == list(P.GAIN_SLOT_IDS)
    assert gains[-1] == {"id": 34, "kp": 10, "kd": 20}
    assert all(g["kp"] is None and g["kd"] is None for g in gains[:-1])
