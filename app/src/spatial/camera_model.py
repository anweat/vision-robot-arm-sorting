"""Wrist camera pose predicted from servo angles (URDF kinematics + hand-eye result).

Accuracy is about 1 cm at the table because of servo backlash (2026-09-24 calibration);
use it to predict where things should appear and to pick between PnP solutions, and
use the map markers or visual feedback for anything that needs millimetres.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Sequence

import numpy as np

import dofbot_kinematics as kin
from map_registration import ROOT, CameraPose, TableMap

DEFAULT_HAND_EYE = ROOT / "config" / "calibration" / "hand_eye.json"


class ArmCamera:
    def __init__(self, hand_eye: Path = DEFAULT_HAND_EYE):
        self.cam2grip = np.array(json.loads(Path(hand_eye).read_text())["cam2gripper"], float)

    def base_T_cam(self, servos: Sequence[float]) -> np.ndarray:
        return np.array(kin.camera_mount_pose(servos)) @ self.cam2grip  # the camera does not turn with J5

    def pose_in_map(self, servos: Sequence[float], table: TableMap) -> CameraPose:
        """Predicted map -> camera transform for the given servo angles."""
        map_T_cam = self.base_T_cam(servos)
        map_T_cam[:3, 3] = table.base_to_map(map_T_cam[:3, 3])
        cam_T_map = np.linalg.inv(map_T_cam)
        return CameraPose(cam_T_map[:3, :3], cam_T_map[:3, 3], [], float("nan"))

    def gripper_axis_base(self, servos: Sequence[float]) -> tuple[np.ndarray, np.ndarray]:
        """Servo-5 centre and unit direction of the last link (towards the tip), base frame."""
        pose = np.array(kin.servo5_pose(list(servos)[:5]))
        return pose[:3, 3], pose[:3, 2]
