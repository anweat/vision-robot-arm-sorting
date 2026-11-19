"""Collision samples of the whole arm in the base frame, vectorized: links 1-4 from the course meshes
(config/arm_links_geometry.json, scripts/blender_extract_links.py) and the four-bar gripper (+ held block).

The link frames are the URDF chain of dofbot_kinematics (servo - 90 about z); link5 uses the real mesh frame
(gripper_geometry.link5_pose: mirrored J5 and the 2.15 mm offset).
"""

from __future__ import annotations

import json
import math
from functools import lru_cache

import numpy as np

from . import config as C
from . import gripper as GR

CHAIN = [((0, 0, 0.064), (0, 0, 0)), ((0, 0, 0.0435), (0, math.pi / 2, 0)),
         ((-0.08285, 0, 0), (0, 0, 0)), ((-0.08285, 0, 0), (0, 0, 0))]
LINKS = ("link1", "link2", "link3", "link4")


def _rot(axis: str, a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    m = np.eye(4)
    i, j = {"x": (1, 2), "y": (2, 0), "z": (0, 1)}[axis]
    m[i, i], m[i, j], m[j, i], m[j, j] = c, -s, s, c
    return m


@lru_cache(maxsize=1)
def _link_clouds() -> dict:
    d = json.loads((C.CONFIG_DIR / "arm_links_geometry.json").read_text(encoding="utf-8"))
    return {n: np.array(d["points_m"][n], dtype=float) for n in LINKS}


def link_frames(q4) -> list[np.ndarray]:
    m, out = np.eye(4), []
    for (xyz, (r, p, y)), s in zip(CHAIN, list(q4)[:4]):
        t = np.eye(4)
        t[:3, 3] = xyz
        m = m @ t @ _rot("z", y) @ _rot("y", p) @ _rot("x", r) @ _rot("z", math.radians(s - 90.0))
        out.append(m)
    return out


def links(q4, stride: int = 1, which=LINKS) -> np.ndarray:
    clouds = _link_clouds()
    frames = link_frames(q4)
    parts = [clouds[n][::stride] @ frames[LINKS.index(n)][:3, :3].T + frames[LINKS.index(n)][:3, 3] for n in which]
    return np.concatenate(parts) if parts else np.zeros((0, 3))


_GRIP: dict = {}


def _gripper_link(delta: float, parts: tuple, stride: int) -> np.ndarray:
    key = (round(delta, 2), parts, stride)
    if key not in _GRIP:
        _GRIP[key] = np.array(GR._link_points(delta, parts), dtype=float)[::stride]
    return _GRIP[key]


def link5(q5) -> np.ndarray:
    return np.array(GR.link5(q5), dtype=float)


def gripper(q5, delta: float, held=None, stride: int = 1, pose: np.ndarray | None = None) -> tuple[np.ndarray, np.ndarray]:
    """(all gripper points, the 'low' ones: fingers + held block), base frame."""
    T = link5(q5) if pose is None else pose
    R, t = T[:3, :3], T[:3, 3]
    fingers = _gripper_link(delta, ("fingers",), stride) @ R.T + t
    rest = _gripper_link(delta, ("links", "static"), stride) @ R.T + t
    if held is not None:
        hp = np.array(held.points_link(), dtype=float) @ R.T + t
        low = np.concatenate([fingers, hp])
    else:
        low = fingers
    return np.concatenate([low, rest]), low
