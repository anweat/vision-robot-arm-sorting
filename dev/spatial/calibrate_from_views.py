# /// script
# requires-python = ">=3.9"
# dependencies = ["opencv-python>=4.8,<5", "numpy", "scipy"]
# ///
"""Camera intrinsics and camera-to-gripper (hand-eye) calibration from charuco_capture views.

  uv run dev/spatial/calibrate_from_views.py data/captures/2026-09-24/charuco_1915 \
      --out-dir app/config/calibration

Board: ChArUco 7x5, 15 mm squares, 11 mm markers, DICT_4X4_50 (docs/calibration/
charuco_15mm_fiducials_A4.pdf). Gripper poses come from the joint readback of each
view through the URDF model in dofbot_kinematics, so hand-eye accuracy is limited by
servo backlash (about 1-2 deg) and the unverified link lengths.

The arm's rotations between views are small, and servo-5 roll is nearly about the
optical axis, so the closed-form OpenCV hand-eye solvers are ill-conditioned here
(on 2026-09-24 TSAI refused and the others disagreed by several cm). Instead the
camera-to-gripper transform and the fixed board pose in the base frame are solved
together by minimising corner reprojection error over all views, with weak priors
from the earlier single-plane estimate (camera ~8.6 cm along the last link, optical
axis parallel to it). The cross-check maps the board into the base frame from every
view; its spread is the practical error of "camera sees point -> base coordinates".
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "app" / "src" / "spatial"))
import dofbot_kinematics as kin  # noqa: E402

MIN_CORNERS = 8


def make_board() -> cv2.aruco.CharucoBoard:
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    return cv2.aruco.CharucoBoard((7, 5), 0.015, 0.011, dictionary)


def to_matrix(rot: np.ndarray, t: np.ndarray) -> np.ndarray:
    m = np.eye(4)
    m[:3, :3], m[:3, 3] = rot, np.ravel(t)
    return m


def solve_hand_eye(readbacks, obj_pts, img_pts, rvecs, tvecs, k, dist, prior_offset, sigma_t, sigma_r,
                   sigma_joint_deg=0.0, board_z=None):
    """Least squares over camera-in-gripper (6), board-in-base (6) and optionally servo 1-5 zero offsets (5).
    With board_z (metres) the board height is fixed instead of fitted: near-parallel views cannot tell
    board height from camera offset along the link (2026-09-25: RMS 19-22 px for any height from -3 to -10.6 cm)."""
    from scipy.optimize import least_squares

    prior_t = np.array([0.0, 0.0, prior_offset])
    fit_joints = sigma_joint_deg > 0

    def poses(x):
        offsets = x[12:17] if fit_joints else np.zeros(5)
        # the camera rides on the wrist, so J5 does not enter (kin.camera_mount_pose)
        return [np.array(kin.camera_mount_pose([a + o for a, o in zip(rb[:4], offsets)])) for rb in readbacks]

    def full(x):
        return x if board_z is None else np.concatenate([x[:11], [board_z], x[11:]])

    def poses_x(x):
        return poses(full(x))

    def unpack(x):
        x = full(x)
        return to_matrix(cv2.Rodrigues(x[:3])[0], x[3:6]), to_matrix(cv2.Rodrigues(x[6:9])[0], x[9:12])

    def residuals(x):
        cam2grip, base2board = unpack(x)
        res = []
        for gp, obj, img in zip(poses_x(x), obj_pts, img_pts):
            cam_board = np.linalg.inv(gp @ cam2grip) @ base2board
            rv, _ = cv2.Rodrigues(cam_board[:3, :3])
            proj, _ = cv2.projectPoints(obj, rv, cam_board[:3, 3], k, dist)
            res.append((proj.reshape(-1, 2) - img.reshape(-1, 2)).ravel())
        xf = full(x)
        res.append((xf[3:6] - prior_t) / sigma_t)         # offset prior, in "pixels" of 1 sigma
        res.append(xf[:2] / sigma_r)                      # optical axis parallel to the link; roll is free
        if fit_joints:
            res.append(xf[12:17] / sigma_joint_deg)
        return np.concatenate(res)

    grip_poses = poses(np.zeros(17))
    x0 = np.zeros(17 if fit_joints else 12)
    x0[3:6] = prior_t
    first = grip_poses[0] @ to_matrix(np.eye(3), prior_t) @ to_matrix(cv2.Rodrigues(rvecs[0])[0], tvecs[0])
    x0[6:9] = cv2.Rodrigues(first[:3, :3])[0].ravel()
    x0[9:12] = first[:3, 3]
    # the roll of the camera about the link is unknown; try four starts and keep the best
    best = None
    for roll in (0, np.pi / 2, np.pi, 3 * np.pi / 2):
        start = x0.copy()
        start[:3] = [0, 0, roll]
        cam2grip0 = to_matrix(cv2.Rodrigues(start[:3])[0], start[3:6])
        first = grip_poses[0] @ cam2grip0 @ to_matrix(cv2.Rodrigues(rvecs[0])[0], tvecs[0])
        start[6:9] = cv2.Rodrigues(first[:3, :3])[0].ravel()
        start[9:12] = first[:3, 3]
        if board_z is not None:
            start = np.delete(start, 11)
        fit = least_squares(residuals, start, loss="soft_l1", f_scale=2.0)
        if best is None or fit.cost < best.cost:
            best = fit
    cam2grip, base2board = unpack(best.x)
    x = full(best.x)
    n_img = sum(len(o) for o in obj_pts)
    rms = float(np.sqrt(np.mean(best.fun[: 2 * n_img] ** 2)))
    offsets = x[12:17].round(2).tolist() if fit_joints else [0.0] * 5
    return cam2grip, base2board, {"rms_px": rms, "joint_offsets_deg": offsets, "poses": poses(x)}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("views_dir", type=Path)
    parser.add_argument("--out-dir", type=Path)
    parser.add_argument("--prior-offset-cm", type=float, default=8.6, help="camera distance along the last link from servo 5")
    parser.add_argument("--prior-sigma-cm", type=float, default=3.0)
    parser.add_argument("--prior-sigma-deg", type=float, default=10.0)
    parser.add_argument("--joint-sigma-deg", type=float, default=5.0,
                        help="prior on servo 1-5 zero offsets; 0 disables fitting them")
    parser.add_argument("--board-z-cm", type=float,
                        help="fix the board height in base coordinates (e.g. -3 for the table, see app/config/arm_limits.json)")
    parser.add_argument("--focal-guess", type=float, default=890.0,
                        help="initial focal length in px (single-plane estimate on 2026-09-24 was ~880-890)")
    args = parser.parse_args()

    board = make_board()
    detector = cv2.aruco.CharucoDetector(board)
    records = json.loads((args.views_dir / "views.json").read_text())
    used, obj_pts, img_pts, size = [], [], [], None
    for rec in records:
        image = cv2.imread(str(args.views_dir / rec["image"]))
        size = image.shape[1], image.shape[0]
        corners, ids, _, _ = detector.detectBoard(image)
        if ids is None or len(ids) < MIN_CORNERS or None in rec["readback"][:5]:
            continue
        obj, img = board.matchImagePoints(corners, ids)
        used.append(rec)
        obj_pts.append(obj.astype(np.float32))
        img_pts.append(img.astype(np.float32))

    # The views differ by only a few degrees, which leaves focal length and distance
    # nearly interchangeable; constrain the model instead of letting it diverge.
    flags = (cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_FIX_PRINCIPAL_POINT | cv2.CALIB_FIX_ASPECT_RATIO
             | cv2.CALIB_ZERO_TANGENT_DIST | cv2.CALIB_FIX_K3)
    k0 = np.array([[args.focal_guess, 0, size[0] / 2], [0, args.focal_guess, size[1] / 2], [0, 0, 1]])
    rms, k, dist, rvecs, tvecs = cv2.calibrateCamera(obj_pts, img_pts, size, k0, np.zeros(5), flags=flags)
    per_view = []
    for obj, img, rv, tv in zip(obj_pts, img_pts, rvecs, tvecs):
        proj, _ = cv2.projectPoints(obj, rv, tv, k, dist)
        per_view.append(float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - img.reshape(-1, 2)) ** 2, axis=1)))))
    hfov = 2 * np.degrees(np.arctan(size[0] / 2 / k[0, 0]))
    intrinsics = {
        "image_size": list(size), "views_used": len(used), "rms_px": round(rms, 3),
        "per_view_rms_px": [round(e, 3) for e in per_view],
        "camera_matrix": k.round(3).tolist(), "dist_coeffs": dist.ravel().round(5).tolist(),
        "hfov_deg": round(float(hfov), 1),
        "flags": "intrinsic guess, fixed principal point and aspect ratio, zero tangential, fixed k3",
    }
    print(json.dumps(intrinsics, indent=1))

    cam2grip, base2board, info = solve_hand_eye([rec["readback"] for rec in used], obj_pts, img_pts, rvecs, tvecs,
                                                k, dist, args.prior_offset_cm / 100, args.prior_sigma_cm / 100,
                                                np.radians(args.prior_sigma_deg), args.joint_sigma_deg,
                                                None if args.board_z_cm is None else args.board_z_cm / 100)
    grip_poses = info["poses"]
    origins = np.array([(gp @ cam2grip @ to_matrix(cv2.Rodrigues(rv)[0], tv))[:3, 3]
                        for gp, rv, tv in zip(grip_poses, rvecs, tvecs)])
    spread = np.linalg.norm(origins - origins.mean(axis=0), axis=1)
    hand_eye = {
        "method": "joint reprojection least squares with weak priors",
        "reprojection_rms_px": round(info["rms_px"], 3),
        "servo_zero_offsets_deg_1_to_5": info["joint_offsets_deg"],
        "t_cam_in_gripper_cm": (cam2grip[:3, 3] * 100).round(2).tolist(),
        "optical_axis_vs_link_deg": round(float(np.degrees(np.arccos(np.clip(cam2grip[2, 2], -1, 1)))), 2),
        "board_origin_base_cm": (base2board[:3, 3] * 100).round(2).tolist(),
        "board_normal_base": base2board[:3, 2].round(3).tolist(),
        "per_view_board_origin_spread_cm_rms": round(float(np.sqrt(np.mean(spread ** 2)) * 100), 2),
        "per_view_board_origin_spread_cm_max": round(float(spread.max() * 100), 2),
        "priors": {"offset_along_link_cm": args.prior_offset_cm, "sigma_cm": args.prior_sigma_cm,
                   "sigma_deg": args.prior_sigma_deg, "joint_sigma_deg": args.joint_sigma_deg,
                   "board_z_fixed_cm": args.board_z_cm},
        "cam2gripper": cam2grip.round(5).tolist(),
        "base2board": base2board.round(5).tolist(),
    }
    print(json.dumps({k2: v for k2, v in hand_eye.items() if k2 not in ("cam2gripper", "base2board")}, indent=1))

    if args.out_dir:
        args.out_dir.mkdir(parents=True, exist_ok=True)
        source = str(args.views_dir)
        (args.out_dir / "camera_intrinsics.json").write_text(
            json.dumps({"source": source, **intrinsics}, indent=2) + "\n")
        (args.out_dir / "hand_eye.json").write_text(
            json.dumps({"source": source, "gripper_frame": "dofbot_kinematics.camera_mount_pose (servo-5 axis centre, J5 held at 90)",
                        **hand_eye}, indent=2) + "\n")


if __name__ == "__main__":
    main()
