#!/usr/bin/env python3
# Author: Roberto Navoni, member of the ArduPilot Dev Team
# Contact: r.navoni74@gmail.com
# Developed by Roberto Navoni — DelphyAI LAB
"""MAVLink link that streams an 8-channel pose command as DEBUG_FLOAT_ARRAY NNM_POSE.

SITL default: udp:127.0.0.1:14550. Also accepts a serial device path.
Sends a heartbeat so the autopilot keeps the GCS link alive.
"""

from __future__ import annotations

import os
import time
from typing import Sequence

# DEBUG_FLOAT_ARRAY is MAVLink 2 only
os.environ.setdefault("MAVLINK20", "1")

import numpy as np
from pymavlink import mavutil


POSE_NAME = b"NNM_POSE"
POSE_DIM = 8


class PoseLink:
    def __init__(self, connection: str = "udp:127.0.0.1:14550", source_system: int = 255):
        self.master = mavutil.mavlink_connection(connection, source_system=source_system)
        self._seq = 0
        self._last_hb = 0.0

    def wait_heartbeat(self, timeout: float = 10.0) -> None:
        self.master.wait_heartbeat(timeout=timeout)

    def send_heartbeat(self) -> None:
        self.master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, 0,
        )
        self._last_hb = time.monotonic()

    def send_pose(self, values: Sequence[float]) -> None:
        """Send up to 8 floats; shorter vectors are zero-padded."""
        data = np.zeros(58, np.float32)  # DEBUG_FLOAT_ARRAY payload length
        v = np.asarray(values, dtype=np.float32).reshape(-1)[:POSE_DIM]
        data[:v.size] = v
        now = time.monotonic()
        if now - self._last_hb > 0.5:
            self.send_heartbeat()
        self.master.mav.debug_float_array_send(
            int(time.time_ns() // 1000), POSE_NAME, self._seq, data.tolist(),
        )
        self._seq = (self._seq + 1) & 0xFFFF


def main() -> None:
    import argparse

    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--conn", default="udp:127.0.0.1:14550")
    ap.add_argument("--hz", type=float, default=40.0)
    ap.add_argument("--values", type=float, nargs="+", default=None,
                    help="fixed 8 values to stream (default: zeros)")
    args = ap.parse_args()
    link = PoseLink(args.conn)
    print(f"pose_link -> {args.conn} at {args.hz} Hz")
    vals = args.values or [0.0] * POSE_DIM
    period = 1.0 / args.hz
    next_t = time.monotonic()
    while True:
        link.send_pose(vals)
        next_t += period
        dt = next_t - time.monotonic()
        if dt > 0:
            time.sleep(dt)


if __name__ == "__main__":
    main()
