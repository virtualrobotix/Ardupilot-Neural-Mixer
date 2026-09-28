#!/usr/bin/env python3
# Author: Roberto Navoni, member of the ArduPilot Dev Team
# Contact: r.navoni74@gmail.com
# Developed by Roberto Navoni — DelphyAI LAB
"""Ground station: webcam / keyboard / clip -> Microban 8-channel pose -> MAVLink NNM_POSE.

Sources:
  camera   — MediaPipe Pose world landmarks, retargeted by retarget_microban
  keyboard — arrow keys / wasd tweak the 8 channels (no camera)
  clip     — replay wave_right or dance as a pose stream (transport test without a camera)

Keys (camera/keyboard): c = calibrate stand, q = quit, 0 = rest pose.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from pose_link import PoseLink  # noqa: E402
from retarget_microban import (  # noqa: E402
    CLIP_KEYS, MicrobanRetarget, POSE_RANGES, pose_from_clip,
)

LABELS = ("r_pitch", "r_roll", "r_elbow", "l_pitch", "l_roll", "l_elbow", "knee", "sway")


def _draw_hud(frame, pose: np.ndarray, title: str) -> None:
    import cv2

    cv2.putText(frame, title, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (40, 220, 120), 2)
    for i, (lab, v) in enumerate(zip(LABELS, pose)):
        y = 52 + i * 22
        cv2.putText(frame, f"{lab:8s} {v:+.3f}", (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (230, 230, 230), 1)
        x0, w = 200, 180
        lo, hi = float(POSE_RANGES[i, 0]), float(POSE_RANGES[i, 1])
        t = 0.0 if hi <= lo else (float(v) - lo) / (hi - lo)
        cv2.rectangle(frame, (x0, y - 12), (x0 + w, y + 4), (60, 60, 60), 1)
        cv2.rectangle(frame, (x0, y - 12), (x0 + int(w * t), y + 4), (80, 180, 255), -1)


def run_camera(args) -> None:
    import cv2
    import mediapipe as mp

    link = PoseLink(args.conn)
    rt = MicrobanRetarget(mirror=args.mirror, rate_hz=args.hz)
    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        raise SystemExit(f"cannot open camera {args.camera}")
    pose = mp.solutions.pose.Pose(
        static_image_mode=False, model_complexity=1,
        enable_segmentation=False, min_detection_confidence=0.5,
        min_tracking_confidence=0.5,
    )
    period = 1.0 / args.hz
    next_t = time.monotonic()
    out = np.zeros(8, np.float32)
    print("camera: press 'c' to calibrate stand, 'q' quit, '0' rest")
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            res = pose.process(rgb)
            if res.pose_world_landmarks is not None:
                lms = res.pose_world_landmarks.landmark
                out = rt.retarget(lms)   # knee_bend stays 0 until 'c' calibrates the stand height
                # skeleton preview from image landmarks
                if res.pose_landmarks is not None:
                    mp.solutions.drawing_utils.draw_landmarks(
                        frame, res.pose_landmarks, mp.solutions.pose.POSE_CONNECTIONS)
            link.send_pose(out)
            _draw_hud(frame, out, f"NNM_POSE  {args.conn}  calib={'ok' if rt._calib_ready else 'press c'}")
            cv2.imshow("mocap_gcs", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("c") and res.pose_world_landmarks is not None:
                rt.calibrate_stand(res.pose_world_landmarks.landmark)
                print("stand calibrated")
            if key == ord("0"):
                out[:] = 0
                rt._filt[:] = 0
                rt._out[:] = 0
            next_t += period
            dt = next_t - time.monotonic()
            if dt > 0:
                time.sleep(dt)
    finally:
        cap.release()
        cv2.destroyAllWindows()
        pose.close()


def run_keyboard(args) -> None:
    import cv2

    link = PoseLink(args.conn)
    out = np.zeros(8, np.float32)
    idx = 0
    canvas = np.zeros((320, 480, 3), np.uint8)
    print("keyboard: a/d select channel, w/s adjust, 0 rest, q quit")
    period = 1.0 / args.hz
    next_t = time.monotonic()
    while True:
        canvas[:] = (20, 24, 32)
        _draw_hud(canvas, out, f"keyboard ch={LABELS[idx]}  {args.conn}")
        cv2.imshow("mocap_gcs", canvas)
        key = cv2.waitKey(1) & 0xFF
        step = 0.03
        if key == ord("q"):
            break
        elif key == ord("a"):
            idx = (idx - 1) % 8
        elif key == ord("d"):
            idx = (idx + 1) % 8
        elif key == ord("w"):
            out[idx] = float(np.clip(out[idx] + step, POSE_RANGES[idx, 0], POSE_RANGES[idx, 1]))
        elif key == ord("s"):
            out[idx] = float(np.clip(out[idx] - step, POSE_RANGES[idx, 0], POSE_RANGES[idx, 1]))
        elif key == ord("0"):
            out[:] = 0
        link.send_pose(out)
        next_t += period
        dt = next_t - time.monotonic()
        if dt > 0:
            time.sleep(dt)
    cv2.destroyAllWindows()


def run_clip(args) -> None:
    link = PoseLink(args.conn)
    name = args.clip
    if name not in CLIP_KEYS:
        raise SystemExit(f"unknown clip {name!r}; known: {', '.join(CLIP_KEYS)}")
    hz_clip = CLIP_KEYS[name]["hz"]
    phase = 0.0
    period = 1.0 / args.hz
    next_t = time.monotonic()
    print(f"clip {name} @ {hz_clip} Hz phase -> {args.conn}")
    try:
        import cv2
        use_cv = True
        canvas = np.zeros((320, 480, 3), np.uint8)
    except Exception:
        use_cv = False
        canvas = None
    while True:
        phase = (phase + hz_clip * period) % 1.0
        out = pose_from_clip(name, phase)
        link.send_pose(out)
        if use_cv:
            canvas[:] = (20, 24, 32)
            _draw_hud(canvas, out, f"clip {name}  phase={phase:.2f}")
            cv2.imshow("mocap_gcs", canvas)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        next_t += period
        dt = next_t - time.monotonic()
        if dt > 0:
            time.sleep(dt)
    if use_cv:
        cv2.destroyAllWindows()


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", choices=("camera", "keyboard", "clip"), default="camera")
    ap.add_argument("--conn", default="udp:127.0.0.1:14550")
    ap.add_argument("--hz", type=float, default=40.0)
    ap.add_argument("--camera", type=int, default=0)
    ap.add_argument("--mirror", action="store_true", help="user right arm -> robot left")
    ap.add_argument("--clip", default="wave_right", choices=sorted(CLIP_KEYS))
    args = ap.parse_args()
    if args.source == "camera":
        run_camera(args)
    elif args.source == "keyboard":
        run_keyboard(args)
    else:
        run_clip(args)


if __name__ == "__main__":
    main()
