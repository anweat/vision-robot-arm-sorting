"""Offline multiview block localization experiment; no hardware imports or motion.

The camera poses, horizontal field of view, and block geometry below are
simulation assumptions. Replace them with measured calibration before using
the reconstructed coordinates for a robot command.
"""

from __future__ import annotations

import argparse
import json
import math
import random
from dataclasses import dataclass
from pathlib import Path


Vec3 = tuple[float, float, float]
Pixel = tuple[float, float]


def add(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def scale(a: Vec3, factor: float) -> Vec3:
    return (a[0] * factor, a[1] * factor, a[2] * factor)


def dot(a: Vec3, b: Vec3) -> float:
    return sum(x * y for x, y in zip(a, b))


def cross(a: Vec3, b: Vec3) -> Vec3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def unit(a: Vec3) -> Vec3:
    length = math.sqrt(dot(a, a))
    if length < 1e-12:
        raise ValueError("zero-length direction")
    return scale(a, 1 / length)


@dataclass(frozen=True)
class Intrinsics:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float

    @classmethod
    def from_horizontal_fov(cls, width: int, height: int, degrees: float) -> Intrinsics:
        if not 0 < degrees < 180:
            raise ValueError("horizontal field of view must be between 0 and 180 degrees")
        focal = width / (2 * math.tan(math.radians(degrees) / 2))
        return cls(width, height, focal, focal, width / 2, height / 2)


@dataclass(frozen=True)
class Camera:
    center: Vec3
    right: Vec3
    down: Vec3
    forward: Vec3

    @classmethod
    def look_at(cls, center: Vec3, target: Vec3) -> Camera:
        forward = unit(sub(target, center))
        right = unit(cross(forward, (0.0, 0.0, 1.0)))
        down = cross(forward, right)
        return cls(center, right, down, forward)

    def project(self, point: Vec3, intrinsics: Intrinsics) -> Pixel:
        relative = sub(point, self.center)
        depth = dot(relative, self.forward)
        if depth <= 0:
            raise ValueError("point is behind the camera")
        return (
            intrinsics.cx + intrinsics.fx * dot(relative, self.right) / depth,
            intrinsics.cy + intrinsics.fy * dot(relative, self.down) / depth,
        )

    def ray(self, pixel: Pixel, intrinsics: Intrinsics) -> Vec3:
        x = (pixel[0] - intrinsics.cx) / intrinsics.fx
        y = (pixel[1] - intrinsics.cy) / intrinsics.fy
        return unit(add(self.forward, add(scale(self.right, x), scale(self.down, y))))


def solve_3x3(matrix: list[list[float]], vector: list[float]) -> Vec3:
    rows = [matrix[i][:] + [vector[i]] for i in range(3)]
    for col in range(3):
        pivot = max(range(col, 3), key=lambda row: abs(rows[row][col]))
        if abs(rows[pivot][col]) < 1e-10:
            raise ValueError("camera rays have insufficient parallax")
        rows[col], rows[pivot] = rows[pivot], rows[col]
        divisor = rows[col][col]
        rows[col] = [value / divisor for value in rows[col]]
        for row in range(3):
            if row != col:
                factor = rows[row][col]
                rows[row] = [a - factor * b for a, b in zip(rows[row], rows[col])]
    return (rows[0][3], rows[1][3], rows[2][3])


def triangulate(observations: list[tuple[Camera, Pixel]], intrinsics: Intrinsics) -> Vec3:
    """Find the point minimizing squared distance to all observed camera rays."""
    if len(observations) < 2:
        raise ValueError("at least two views are required")
    widest_baseline = max(
        math.dist(observations[i][0].center, observations[j][0].center)
        for i in range(len(observations)) for j in range(i + 1, len(observations))
    )
    if widest_baseline < 1e-6:
        raise ValueError("camera poses have no translation baseline")
    matrix = [[0.0] * 3 for _ in range(3)]
    vector = [0.0] * 3
    rays = [camera.ray(pixel, intrinsics) for camera, pixel in observations]
    widest_angle = max(
        math.acos(max(-1.0, min(1.0, dot(rays[i], rays[j]))))
        for i in range(len(rays)) for j in range(i + 1, len(rays))
    )
    if widest_angle < math.radians(1):
        raise ValueError("camera rays have insufficient parallax")
    for (camera, _), direction in zip(observations, rays):
        projector = [
            [float(i == j) - direction[i] * direction[j] for j in range(3)]
            for i in range(3)
        ]
        for i in range(3):
            vector[i] += dot(tuple(projector[i]), camera.center)
            for j in range(3):
                matrix[i][j] += projector[i][j]
    point = solve_3x3(matrix, vector)
    if any(dot(sub(point, camera.center), camera.forward) <= 0
           for camera, _ in observations):
        raise ValueError("reconstructed point is behind a camera")
    return point


def intersect_height(camera: Camera, pixel: Pixel,
                     intrinsics: Intrinsics, height_m: float) -> Vec3:
    """Intersect one camera ray with a known horizontal object-top plane."""
    direction = camera.ray(pixel, intrinsics)
    if abs(direction[2]) < 1e-10:
        raise ValueError("camera ray is parallel to the assumed plane")
    distance = (height_m - camera.center[2]) / direction[2]
    if distance <= 0:
        raise ValueError("assumed plane is behind the camera")
    return add(camera.center, scale(direction, distance))


def block_top_corners(center: Vec3, side_m: float, yaw_degrees: float) -> list[Vec3]:
    yaw = math.radians(yaw_degrees)
    cosine, sine = math.cos(yaw), math.sin(yaw)
    half = side_m / 2
    corners = [(-half, -half), (half, -half), (half, half), (-half, half)]
    return [
        (center[0] + x * cosine - y * sine, center[1] + x * sine + y * cosine, center[2])
        for x, y in corners
    ]


def estimate_block(corners: list[Vec3]) -> tuple[Vec3, float]:
    center = (
        sum(point[0] for point in corners) / len(corners),
        sum(point[1] for point in corners) / len(corners),
        sum(point[2] for point in corners) / len(corners),
    )
    edge = sub(corners[1], corners[0])
    yaw = math.degrees(math.atan2(edge[1], edge[0]))
    return center, yaw


def angle_error_degrees(actual: float, expected: float) -> float:
    return abs((actual - expected + 180) % 360 - 180)


def run_trial(
    cameras: list[Camera],
    true_intrinsics: Intrinsics,
    assumed_intrinsics: Intrinsics,
    corners: list[Vec3],
    noise_px: float,
    rng: random.Random,
    fixed_height_m: float | None = None,
) -> tuple[float, float, Vec3]:
    reconstructed = []
    for corner in corners:
        observations = []
        for camera in cameras:
            u, v = camera.project(corner, true_intrinsics)
            observations.append(
                (camera, (u + rng.gauss(0, noise_px), v + rng.gauss(0, noise_px)))
            )
        if fixed_height_m is None:
            reconstructed.append(triangulate(observations, assumed_intrinsics))
        else:
            reconstructed.append(intersect_height(
                observations[0][0], observations[0][1], assumed_intrinsics,
                fixed_height_m,
            ))
    center, yaw = estimate_block(reconstructed)
    true_center, true_yaw = estimate_block(corners)
    position_error_m = math.sqrt(dot(sub(center, true_center), sub(center, true_center)))
    return position_error_m * 1000, angle_error_degrees(yaw, true_yaw), center


def median(values: list[float]) -> float:
    ordered = sorted(values)
    return (ordered[(len(ordered) - 1) // 2] + ordered[len(ordered) // 2]) / 2


def summarize(
    cameras: list[Camera],
    true_intrinsics: Intrinsics,
    assumed_intrinsics: Intrinsics,
    corners: list[Vec3],
    noise_px: float,
    trials: int,
    seed: int,
    fixed_height_m: float | None = None,
) -> dict[str, float]:
    rng = random.Random(seed)
    position_errors = []
    yaw_errors = []
    for _ in range(trials):
        position_error, yaw_error, _ = run_trial(
            cameras, true_intrinsics, assumed_intrinsics, corners, noise_px, rng,
            fixed_height_m,
        )
        position_errors.append(position_error)
        yaw_errors.append(yaw_error)
    return {
        "median_position_error_mm": round(median(position_errors), 3),
        "p95_position_error_mm": round(sorted(position_errors)[math.ceil(trials * 0.95) - 1], 3),
        "median_yaw_error_deg": round(median(yaw_errors), 3),
    }


def render_svg(cameras: list[Camera], intrinsics: Intrinsics,
               corners: list[Vec3], path: Path) -> None:
    """Show the three ideal pinhole frames; pixels and camera poses are synthetic."""
    panel_width, panel_height = 360, 300
    pieces = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{panel_width * len(cameras)}" '
        f'height="{panel_height}" viewBox="0 0 {panel_width * len(cameras)} {panel_height}">',
        '<rect width="100%" height="100%" fill="#f4f1e9"/>',
    ]
    for index, camera in enumerate(cameras):
        left = index * panel_width + 20
        top = 38
        pixels = [camera.project(corner, intrinsics) for corner in corners]
        coords = " ".join(f"{left + u / 2:.2f},{top + v / 2:.2f}" for u, v in pixels)
        center, _ = estimate_block(corners)
        u, v = camera.project(center, intrinsics)
        zoom_x, zoom_y = left + 272, top + 52
        zoom_coords = " ".join(
            f"{zoom_x + (pu - u) * 3:.2f},{zoom_y + (pv - v) * 3:.2f}"
            for pu, pv in pixels
        )
        pieces.extend([
            f'<text x="{left}" y="25" font-family="sans-serif" font-size="16" '
            f'fill="#19324a">Synthetic frame {index + 1}</text>',
            f'<rect x="{left}" y="{top}" width="320" height="240" '
            'fill="#dde6e5" stroke="#8fa3a2"/>',
            f'<polygon points="{coords}" fill="#e9a73e" stroke="#263b49" '
            'stroke-width="2"/>',
            f'<line x1="{left + pixels[0][0] / 2:.2f}" '
            f'y1="{top + pixels[0][1] / 2:.2f}" '
            f'x2="{left + pixels[1][0] / 2:.2f}" '
            f'y2="{top + pixels[1][1] / 2:.2f}" stroke="#c84331" '
            'stroke-width="5"/>',
            f'<circle cx="{left + u / 2:.2f}" cy="{top + v / 2:.2f}" '
            'r="4" fill="#173f70"/>',
            f'<rect x="{left + 228}" y="{top + 8}" width="88" height="88" '
            'fill="#fff8e8" stroke="#718a8f"/>',
            f'<polygon points="{zoom_coords}" fill="#e9a73e" '
            'stroke="#263b49" stroke-width="2"/>',
            f'<line x1="{zoom_x + (pixels[0][0] - u) * 3:.2f}" '
            f'y1="{zoom_y + (pixels[0][1] - v) * 3:.2f}" '
            f'x2="{zoom_x + (pixels[1][0] - u) * 3:.2f}" '
            f'y2="{zoom_y + (pixels[1][1] - v) * 3:.2f}" '
            'stroke="#c84331" stroke-width="5"/>',
            f'<circle cx="{zoom_x}" cy="{zoom_y}" r="3" fill="#173f70"/>',
            f'<text x="{left + 230}" y="{top + 108}" font-family="sans-serif" '
            'font-size="11" fill="#19324a">detail enlarged</text>',
            f'<text x="{left}" y="294" font-family="sans-serif" font-size="12" '
            f'fill="#19324a">camera x = {camera.center[0]:+.2f} m</text>',
        ])
    pieces.append('</svg>')
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(pieces) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config", type=Path,
        default=Path(__file__).resolve().parents[2] / "app/config/multiview_synthetic.json",
        help="JSON camera path and block geometry; included default is hypothetical",
    )
    parser.add_argument("--trials", type=int, default=1000)
    parser.add_argument("--noise-px", type=float, default=1.5)
    parser.add_argument("--svg-output", type=Path,
                        help="optional path for three synthetic view frames")
    args = parser.parse_args()
    if args.trials < 1 or args.noise_px < 0:
        parser.error("trials must be positive and noise-px cannot be negative")

    config = json.loads(args.config.read_text(encoding="utf-8"))
    if config.get("status") != "synthetic_assumptions":
        parser.error("this demo only accepts a config labelled synthetic_assumptions")
    width, height = config["resolution_px"]
    true_fov = config["horizontal_fov_deg_assumed"]
    true_intrinsics = Intrinsics.from_horizontal_fov(width, height, true_fov)
    center = tuple(config["block_top_center_m_assumed"])
    corners = block_top_corners(center, side_m=config["block_side_m_assumed"],
                                yaw_degrees=config["block_yaw_deg_assumed"])
    camera_centers = [tuple(values) for values in config["camera_centers_m_assumed"]]
    if len(camera_centers) < 3:
        parser.error("at least three camera poses are needed for the two/three-view comparison")
    look_at = tuple(config["camera_look_at_m_assumed"])
    cameras = [Camera.look_at(position, look_at) for position in camera_centers]
    if args.svg_output:
        render_svg(cameras, true_intrinsics, corners, args.svg_output)
    # Compare two and three distinct observation poses. All metric poses are assumed.
    cases = {
        "one_view_known_top_height": (cameras[:1], true_intrinsics, center[2]),
        "two_views": (cameras[:2], true_intrinsics, None),
        "three_views": (cameras, true_intrinsics, None),
        "three_views_wrong_fov": (
            cameras,
            Intrinsics.from_horizontal_fov(width, height,
                                           config["sensitivity_fov_deg_assumed"]),
            None,
        ),
    }
    result = {
        "status": "synthetic_only",
        "assumptions": {
            **config,
            "no_lens_distortion": True,
            "corner_correspondences": "known exactly before pixel noise",
            "pixel_noise_sigma_px": args.noise_px,
            "trials": args.trials,
        },
        "projected_top_center_px": [
            [round(value, 2) for value in camera.project(center, true_intrinsics)]
            for camera in cameras
        ],
        "results": {
            name: summarize(poses, true_intrinsics, estimated_intrinsics, corners,
                            args.noise_px, args.trials, seed=42, fixed_height_m=height)
            for name, (poses, estimated_intrinsics, height) in cases.items()
        },
    }
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
