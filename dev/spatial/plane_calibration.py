# /// script
# requires-python = ">=3.9"
# dependencies = ["opencv-python>=4.8,<5", "numpy"]
# ///
"""Table-plane calibration of the board camera from a white sheet of known size.

The arm must sit in a fixed observation pose (e.g. observe_v0); the result is only
valid for that pose. The sheet lies flat on the table with one long edge against the
tape measure's inner edge.

Table frame ("tape frame", cm): Y = tape reading along the tape, X = lateral distance
from the tape's inner edge, positive towards the tape side (the arm's right when the
tape is on its right). Converting to the arm base frame needs the tape zero and edge
offsets relative to the servo-1 axis, measured separately.

  uv run dev/spatial/plane_calibration.py detect frame.jpg --sheet 10.5 14.8 --near 25
  -> prints corners, homography, and writes frame_rectified.jpg for checking tape ticks
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def find_sheet(image: np.ndarray) -> np.ndarray:
    """Four sheet corners in image pixels, ordered near-left, near-right, far-right, far-left.

    'Near' is the bottom of the image (closest to the arm)."""
    gray = cv2.GaussianBlur(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), (5, 5), 0)
    _, mask = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8), iterations=2)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    best = None
    for contour in sorted(contours, key=cv2.contourArea, reverse=True)[:5]:
        approx = cv2.approxPolyDP(contour, 0.02 * cv2.arcLength(contour, True), True)
        if len(approx) == 4 and cv2.isContourConvex(approx):
            best = approx.reshape(4, 2).astype(np.float32)
            break
    if best is None:
        raise RuntimeError("no four-cornered bright region found")
    refined = cv2.cornerSubPix(gray, best.copy(), (7, 7), (-1, -1),
                               (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 0.01))
    by_y = refined[np.argsort(refined[:, 1])]
    far = by_y[:2][np.argsort(by_y[:2, 0])]
    near = by_y[2:][np.argsort(by_y[2:, 0])]
    return np.array([near[0], near[1], far[1], far[0]], np.float32)


def sheet_table_points(width: float, length: float, near: float) -> np.ndarray:
    """Sheet corners in the tape frame, same order as find_sheet (right edge on the tape, X = 0)."""
    return np.array([[-width, near], [0, near], [0, near + length], [-width, near + length]], np.float32)


def focal_from_homography(table_to_img: np.ndarray, cx: float, cy: float) -> float:
    """Zhang's single-plane constraint with square pixels and a known principal point."""
    shift = np.array([[1, 0, -cx], [0, 1, -cy], [0, 0, 1]], float)
    h = shift @ table_to_img
    h1, h2 = h[:, 0], h[:, 1]
    # With w = diag(1/f^2, 1/f^2, 1): h1' w h2 = 0 and h1' w h1 = h2' w h2
    a1 = h1[0] * h2[0] + h1[1] * h2[1]
    b1 = h1[2] * h2[2]
    a2 = h1[0] ** 2 + h1[1] ** 2 - h2[0] ** 2 - h2[1] ** 2
    b2 = h1[2] ** 2 - h2[2] ** 2
    # least squares for u = 1/f^2 in a*u + b = 0
    a, b = np.array([a1, a2]), np.array([b1, b2])
    u = -(a @ b) / (a @ a)
    return float(1 / np.sqrt(u)) if u > 0 else float("nan")


def camera_centre(table_to_img: np.ndarray, focal: float, cx: float, cy: float) -> np.ndarray:
    k_inv = np.linalg.inv(np.array([[focal, 0, cx], [0, focal, cy], [0, 0, 1]]))
    m = k_inv @ table_to_img
    scale = 1 / np.linalg.norm(m[:, 0])
    r1, r2, t = m[:, 0] * scale, m[:, 1] * scale, m[:, 2] * scale
    if t[2] < 0:  # table must lie in front of the camera
        r1, r2, t = -r1, -r2, -t
    rot = np.column_stack([r1, r2, np.cross(r1, r2)])
    u, _, vt = np.linalg.svd(rot)
    rot = u @ vt
    return -rot.T @ t


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("image", type=Path)
    parser.add_argument("--sheet", type=float, nargs=2, default=(10.5, 14.8), metavar=("WIDTH", "LENGTH"),
                        help="sheet size in cm, lateral width then length along the tape")
    parser.add_argument("--near", type=float, required=True, help="tape reading at the sheet's near edge (cm)")
    parser.add_argument("--px-per-cm", type=float, default=20, help="scale of the rectified check image")
    parser.add_argument("--out", type=Path, help="write calibration JSON here")
    args = parser.parse_args()

    image = cv2.imread(str(args.image))
    corners = find_sheet(image)
    table = sheet_table_points(*args.sheet, args.near)
    img_to_table = cv2.getPerspectiveTransform(corners, table)
    table_to_img = np.linalg.inv(img_to_table)
    h, w = image.shape[:2]
    focal = focal_from_homography(table_to_img / table_to_img[2, 2], w / 2, h / 2)
    centre = camera_centre(table_to_img, focal, w / 2, h / 2) if np.isfinite(focal) else None

    # rectified view covering X -25..8 cm, Y 12..45 cm so the tape ticks can be read
    x0, x1, y0, y1, s = -25, 8, 12, 45, args.px_per_cm
    to_view = np.array([[s, 0, -x0 * s], [0, -s, y1 * s], [0, 0, 1]], float)
    view = cv2.warpPerspective(image, to_view @ img_to_table, (int((x1 - x0) * s), int((y1 - y0) * s)))
    for y in range(int(y0), int(y1) + 1):
        py = int((y1 - y) * s)
        cv2.line(view, (int((0 - x0) * s), py), (int((0 - x0) * s) + (8 if y % 5 else 20), py), (0, 0, 255), 1)
        if y % 5 == 0:
            cv2.putText(view, str(y), (int((0 - x0) * s) + 24, py + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)
    rect_path = args.image.with_name(args.image.stem + "_rectified.jpg")
    cv2.imwrite(str(rect_path), view)

    result = {
        "image": str(args.image),
        "image_size": [w, h],
        "sheet_cm": list(args.sheet),
        "sheet_near_tape_cm": args.near,
        "corners_px": corners.round(2).tolist(),
        "img_to_tape_frame": (img_to_table / img_to_table[2, 2]).tolist(),
        "focal_px_estimate": round(focal, 1),
        "camera_centre_tape_frame_cm": None if centre is None else np.round(centre, 2).tolist(),
        "rectified_check_image": str(rect_path),
    }
    print(json.dumps(result, indent=2))
    if args.out:
        args.out.write_text(json.dumps(result, indent=2) + "\n")


if __name__ == "__main__":
    main()
