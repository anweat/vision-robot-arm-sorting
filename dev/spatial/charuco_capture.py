#!/usr/bin/env python3
"""Capture ChArUco views from a list of arm poses for camera and hand-eye calibration.

Runs on the board with /root/arm-app/venv-vision/bin/python (needs cv2.aruco and Arm_Lib).
The board sheet stays still on the table; the arm visits each pose through safe_arm
(batch writes, small verified steps). At each pose it waits for a frame taken after the
motion finished, detects ChArUco corners, and saves the image plus the joint readback.

Without --execute it only prints the planned poses and their forward-kinematics checks.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
import urllib.request
from pathlib import Path

import cv2
import numpy as np

APP = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(APP / "dev" / "adapters"))
sys.path.insert(0, str(APP / "app" / "src" / "adapters"))
sys.path.insert(0, str(APP / "dev" / "spatial"))
sys.path.insert(0, str(APP / "app" / "src" / "spatial"))
import dofbot_kinematics as kin  # noqa: E402
import safe_arm  # noqa: E402

SNAPSHOT = "http://127.0.0.1:8080/snapshot.jpg"
MOUNT_ABOVE_MAT_M = 0.09   # user tape measurement 2026-09-24, about +-1 cm
MIN_TIP_ABOVE_MAT_M = 0.05

# around observe_v1 = [90,105,5,5]; servo 6 is left at its reading
POSES = [
    [90, 105, 5, 5, 95], [82, 105, 5, 5, 95], [98, 105, 5, 5, 95],
    [90, 100, 5, 10, 95], [90, 110, 5, 5, 95], [90, 105, 10, 5, 95],
    [90, 105, 5, 5, 110], [90, 105, 5, 5, 125], [84, 108, 5, 8, 105],
    [96, 102, 8, 5, 115], [90, 105, 5, 5, 95],
]


def board() -> cv2.aruco.CharucoBoard:
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    return cv2.aruco.CharucoBoard((7, 5), 0.015, 0.011, dictionary)


def frame_after(t_ready: float, timeout: float = 5.0) -> tuple[np.ndarray, float]:
    """First camera frame whose capture time is later than t_ready."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        with urllib.request.urlopen(SNAPSHOT, timeout=3) as resp:
            stamp = float(resp.headers["X-Frame-Time"])
            data = resp.read()
        if stamp > t_ready:
            return cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR), stamp
        time.sleep(0.1)
    raise RuntimeError("no fresh camera frame")


def tip_clearance(pose: list[int]) -> float:
    return kin.forward(pose[:4])["tip"][2] + MOUNT_ABOVE_MAT_M


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", type=Path, default=APP / "captures" / time.strftime("charuco_%Y%m%d_%H%M"))
    parser.add_argument("--settle", type=float, default=0.8, help="seconds to wait after motion before accepting frames")
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()

    for pose in POSES:
        clearance = tip_clearance(pose)
        print(json.dumps({"pose": pose, "tip_above_mat_cm": round(clearance * 100, 1)}))
        if clearance < MIN_TIP_ABOVE_MAT_M:
            print(f"refusing: tip would be {clearance * 100:.1f} cm above the mat")
            return 2
    if not args.execute:
        return 0

    args.out.mkdir(parents=True, exist_ok=True)
    detector = cv2.aruco.CharucoDetector(board())
    records = []
    for index, pose in enumerate(POSES):
        sets = [f"{j}={a}" for j, a in zip((1, 2, 3, 4, 5), pose)]
        argv = ["move", "--execute"] + [x for s in sets for x in ("--set", s)]
        if safe_arm.main(argv) != 0:
            print("motion stopped; keeping captures so far")
            break
        t_ready = time.time() + args.settle
        time.sleep(args.settle)
        image, stamp = frame_after(t_ready)
        corners, ids, _, _ = detector.detectBoard(image)
        name = f"view_{index:02d}.jpg"
        cv2.imwrite(str(args.out / name), image)
        arm = safe_arm.Arm(safe_arm.load_config(safe_arm.DEFAULT_CONFIG))
        record = {"index": index, "image": name, "frame_time": stamp, "command": pose,
                  "readback": arm.read(), "charuco_corners": 0 if ids is None else int(len(ids))}
        records.append(record)
        print(json.dumps(record))
    (args.out / "views.json").write_text(json.dumps(records, indent=2) + "\n")
    print("saved", args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
