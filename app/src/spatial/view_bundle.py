"""The map and every scan view registered together from the map codes, with the servo pose as a prior.

Each view's camera pose from forward kinematics + hand-eye is off by the servos' per-pose error (about +-1.5
deg of roll and ~10 px per view, doc 30 §6) -- random from pose to pose, so no fixed model removes it. Solving
each view from its own codes alone (scene.py before 2026-11-20) needs >= 2 codes and is shaky with two small
ones; a view with fewer fell back to forward kinematics. Here all views are solved at once:

    unknowns   the map placement in the base frame (x, y, yaw; its height stays the calibrated table), a camera
               pose bias shared by the whole scan (what all views have in common: hand-eye, servo zeros) and,
               per view, a small correction on top of it (3 rotations, 3 translations, camera frame);
    residuals  every detected code corner against its projection (pixels, robust: Huber), plus each view's
               correction against the servo error it can plausibly have (a Gaussian prior, sigma
               SIGMA_ROT_DEG / SIGMA_T_M), plus weak priors on the shared bias and the placement;
    solver     Levenberg-Marquardt, numeric Jacobian (block structure: a view's correction only moves its own
               corners), numpy + OpenCV only.

So the placement is the consensus of all views weighed by what each one sees, a view with a single code is
still corrected (its 4 corners plus the prior), and a view with none gets forward kinematics with the shared
bias on the solved placement (without the bias such views came out twice as far off as with the old median
placement, which hides the bias in the placement). The corrections are the servo error each view had.

    from view_bundle import solve
    out = solve([base_T_cam ...], [{id: 4x2 corners} ...], table, k, dist, base_T_map0)
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Sequence

import cv2
import numpy as np

from map_registration import CameraPose, TableMap

SIGMA_ROT_DEG = 1.5        # per-view servo error, rotation (doc 30 §6: +-1.5 deg roll per image)
SIGMA_T_M = 0.005          # per-view servo error, camera translation (~10 px at the table)
SIGMA_PX = 1.0             # code corner detection (sub-pixel refinement; in-image residual 1.1-1.6 px)
HUBER_PX = 2.0             # corners further off than this count linearly (a code partly covered, a bad view)
SIGMA_PLACEMENT_M = 0.03   # weak prior on the map placement around the start (a map moved by ~1 cm, doc 30 §6)
SIGMA_PLACEMENT_DEG = 5.0
SIGMA_BIAS_ROT_DEG = 3.0   # weak prior on the scan's shared bias (fit_arm_errors bounds the hand-eye to 3 deg / 1.5 cm)
SIGMA_BIAS_T_M = 0.015


def _rot(rx: float, ry: float, rz: float) -> np.ndarray:
    r, _ = cv2.Rodrigues(np.array([rx, ry, rz], float))
    return r


def _placement(x: float, y: float, yaw: float, z: float) -> np.ndarray:
    m = np.eye(4)
    c, s = math.cos(yaw), math.sin(yaw)
    m[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
    m[:3, 3] = [x, y, z]
    return m


def _correct(base_T_cam: np.ndarray, d: np.ndarray) -> np.ndarray:
    """base_T_cam with a camera-frame correction d = (rx, ry, rz [rad], tx, ty, tz [m])."""
    c = np.eye(4)
    c[:3, :3] = _rot(*d[:3])
    c[:3, 3] = d[3:]
    return base_T_cam @ c


@dataclass
class Result:
    base_T_map: np.ndarray
    poses: list                      # CameraPose (map -> camera) per view
    corrections: list                # per view: (rx, ry, rz) deg, (tx, ty, tz) mm
    used: list                       # per view: codes that took part
    reprojection_px: list            # per view: rms of its corners after the solve (nan without codes)
    iterations: int
    stats: dict = field(default_factory=dict)


def solve(base_T_cams: Sequence[np.ndarray], markers: Sequence[dict], table: TableMap, k: np.ndarray,
          dist: np.ndarray, base_T_map0: np.ndarray | None = None, sigma_rot_deg: float = SIGMA_ROT_DEG,
          sigma_t_m: float = SIGMA_T_M, sigma_px: float = SIGMA_PX, huber_px: float = HUBER_PX,
          iterations: int = 30) -> Result:
    n = len(base_T_cams)
    base_T_cams = [np.asarray(b, float) for b in base_T_cams]
    z = float(table.offset[2]) if base_T_map0 is None else float(base_T_map0[2, 3])
    if base_T_map0 is None:
        p0 = np.array([table.offset[0], table.offset[1], 0.0])
    else:
        p0 = np.array([base_T_map0[0, 3], base_T_map0[1, 3], math.atan2(base_T_map0[1, 0], base_T_map0[0, 0])])
    # per view: the map corners of its codes (z = 0) and their detections
    obs = []
    for m in markers:
        ids = sorted(i for i in m if i in table.layout)
        if ids:
            obs.append((ids, np.vstack([np.column_stack([table.layout[i], np.zeros(4)]) for i in ids]),
                        np.vstack([np.asarray(m[i], float) for i in ids])))
        else:
            obs.append((ids, np.zeros((0, 3)), np.zeros((0, 2))))
    sr, st = math.radians(sigma_rot_deg), sigma_t_m
    prior_w = np.array([1 / sr] * 3 + [1 / st] * 3)

    def view_residual(i: int, place: np.ndarray, d: np.ndarray, weights: np.ndarray | None) -> np.ndarray:
        ids, pts, det = obs[i]
        if not ids:
            return np.zeros(0)
        cam_T_map = np.linalg.inv(_correct(base_T_cams[i], d)) @ _placement(place[0], place[1], place[2], z)
        if cam_T_map[2, 3] <= 0:
            return np.full(2 * len(det), 1e3)
        rv, _ = cv2.Rodrigues(cam_T_map[:3, :3])
        proj, _ = cv2.projectPoints(pts.reshape(-1, 1, 3), rv, cam_T_map[:3, 3], k, dist)
        r = (proj.reshape(-1, 2) - det).ravel() / sigma_px
        return r if weights is None else r * weights

    def unpack(x: np.ndarray):
        return x[:3], x[3:9], x[9:].reshape(n, 6)

    def compose(bias: np.ndarray, d: np.ndarray) -> np.ndarray:
        return np.concatenate([bias[:3] + d[:3], bias[3:] + d[3:]])     # small angles: the corrections add

    bias_w = np.array([1 / math.radians(SIGMA_BIAS_ROT_DEG)] * 3 + [1 / SIGMA_BIAS_T_M] * 3)
    x = np.concatenate([p0, np.zeros(6), np.zeros(6 * n)])
    place_w = np.array([1 / SIGMA_PLACEMENT_M] * 2 + [1 / math.radians(SIGMA_PLACEMENT_DEG)])
    weights = [None] * n

    def residuals(xv: np.ndarray) -> np.ndarray:
        place, bias, ds = unpack(xv)
        parts = [view_residual(i, place, compose(bias, ds[i]), weights[i]) for i in range(n)]
        parts.append((ds * prior_w).ravel())
        parts.append(bias * bias_w)
        parts.append((place - p0) * place_w)
        return np.concatenate(parts)

    lam, it = 1e-2, 0
    for it in range(1, iterations + 1):
        # Huber weights from the current corner residuals (IRLS)
        place, bias, ds = unpack(x)
        for i in range(n):
            r = view_residual(i, place, compose(bias, ds[i]), None)
            if r.size:
                e = np.hypot(r[0::2], r[1::2]) * sigma_px
                w = np.where(e <= huber_px, 1.0, np.sqrt(huber_px / np.maximum(e, 1e-9)))
                weights[i] = np.repeat(w, 2)
        r0 = residuals(x)
        cost = float(r0 @ r0)
        # numeric Jacobian: placement columns touch everything, a view's columns only its own rows
        jac = np.zeros((r0.size, x.size))
        h = np.full(x.size, 1e-5)
        for j in range(x.size):
            xp = x.copy()
            xp[j] += h[j]
            jac[:, j] = (residuals(xp) - r0) / h[j]
        g = jac.T @ r0
        a = jac.T @ jac
        improved = False
        for _ in range(8):
            step = np.linalg.solve(a + lam * np.diag(np.diag(a) + 1e-9), -g)
            xn = x + step
            rn = residuals(xn)
            if float(rn @ rn) < cost:
                x, lam, improved = xn, max(lam / 3, 1e-7), True
                break
            lam *= 4
        if not improved or np.max(np.abs(step)) < 1e-7:
            break

    place, bias, ds = unpack(x)
    base_T_map = _placement(place[0], place[1], place[2], z)
    poses, corr, used, rms = [], [], [], []
    for i in range(n):
        cam_T_map = np.linalg.inv(_correct(base_T_cams[i], compose(bias, ds[i]))) @ base_T_map
        r = view_residual(i, place, compose(bias, ds[i]), None) * sigma_px
        rms.append(float(np.sqrt(np.mean(r[0::2] ** 2 + r[1::2] ** 2))) if r.size else float("nan"))
        poses.append(CameraPose(cam_T_map[:3, :3], cam_T_map[:3, 3], obs[i][0], rms[-1]))
        corr.append(dict(rot_deg=[round(math.degrees(v), 2) for v in ds[i][:3]],
                         t_mm=[round(v * 1000, 1) for v in ds[i][3:]]))
        used.append(obs[i][0])
    seen = [i for i in range(n) if used[i]]
    rot = [math.degrees(float(np.linalg.norm(ds[i][:3]))) for i in seen]
    tr = [float(np.linalg.norm(ds[i][3:])) * 1000 for i in seen]
    stats = dict(views=n, with_codes=len(seen), codes=sum(len(u) for u in used),
                 reprojection_px_median=round(float(np.median([rms[i] for i in seen])), 2) if seen else None,
                 correction_rot_deg_median=round(float(np.median(rot)), 2) if rot else None,
                 correction_t_mm_median=round(float(np.median(tr)), 1) if tr else None,
                 bias_rot_deg=[round(math.degrees(v), 2) for v in bias[:3]], bias_t_mm=[round(v * 1000, 1) for v in bias[3:]])
    return Result(base_T_map, poses, corr, used, rms, it, stats)
