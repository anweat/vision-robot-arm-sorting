"""Margins of a pose, a joint-linear move or a gripper sweep against the World: joint limits, the platform
(fitted keep-out and the lid mesh), the table, the collision map (gripper, held block and links 1-4) and
precise boxes (the target block, a block just set down).

All clearances come from the World's Tuning (scene errors included); vectorized with numpy.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Sequence

import numpy as np

from . import body as B
from . import config as C
from .collision_map import CollisionMap
from .motion import Check
from .scene import Box
from .tuning import Tuning
import platform_model    # noqa: E402
import workspace         # noqa: E402
import dofbot_kinematics as kin   # noqa: E402

KEEP_OUT = workspace.load_keep_out(C.ARM)
PLATFORM = platform_model.load()


@dataclass
class World:
    """What a check runs against. `target`: the block being gripped, checked as a precise box (its own map
    cells are left out via `exclude`). `extra`: further precise (Box, margin) pairs."""
    cmap: CollisionMap
    tuning: Tuning
    target: Box | None = None
    exclude: tuple = ()
    extra: list = field(default_factory=list)

    def without(self, *labels) -> "World":
        return World(self.cmap, self.tuning, self.target, tuple(self.exclude) + tuple(labels), list(self.extra))

    def with_target(self, target: Box | None, label: int | None = None) -> "World":
        ex = tuple(self.exclude) + ((label,) if label else ())
        return World(self.cmap, self.tuning, target, ex, list(self.extra))

    def with_extra(self, box: Box, margin: float) -> "World":
        return World(self.cmap, self.tuning, self.target, self.exclude, list(self.extra) + [(box, margin)])

    def with_tuning(self, tuning: Tuning) -> "World":
        return World(self.cmap, tuning, self.target, self.exclude, list(self.extra))

    def block_labels(self) -> tuple:
        """Labels of every block in the map (the neighbours a grasp's fingers might brush)."""
        return tuple(o["id"] for o in self.cmap.objects if o.get("kind", "block") == "block")

    def tallest(self) -> float:
        tops = [self.cmap.tallest(self.tuning.table_z, self.exclude)] + [b.top for b, _ in self.extra]
        if self.target is not None:
            tops.append(self.target.top)
        return max(tops)


def platform_keep_out(q4: Sequence[float], cfg: dict = KEEP_OUT) -> float:
    """workspace.platform_clearance, vectorized (same hull, same shapes; tests/test_motion_planning
    compares the two)."""
    hull = np.array(workspace.platform_hull(list(q4)[:4], cfg))
    r, z = hull[:, 0], hull[:, 1]
    dr, dz = r - cfg["platform_radius_m"], z - cfg["platform_min_z_m"]
    worst = float(np.where((dr > 0) & (dz > 0), np.hypot(dr, dz), np.maximum(dr, dz)).min())
    square = cfg.get("platform_square") if cfg["platform_shape"] == "rounded_square" else None
    if square is not None:
        square = dict(square, top_z_m=cfg["platform_min_z_m"])
        yaw = math.radians(list(q4)[0] - 90.0)
        worst = min(worst, float((-platform_model.depth(r * -math.sin(yaw), r * math.cos(yaw), z, square)).min()))
    return worst - cfg["platform_margin_m"]


def limits_ok(q5: Sequence[float]) -> bool:
    return all(C.LIMITS[j][0] - 1e-6 <= q5[j - 1] <= C.LIMITS[j][1] + 1e-6 for j in range(1, 6))


def box_clearance(box: Box, pts: np.ndarray, margin: float) -> float:
    """Min over points of how far outside the box grown by `margin` they are (< 0 = inside)."""
    if pts.size == 0:
        return math.inf
    d = pts - np.array(box.centre)
    c, s = math.cos(math.radians(box.yaw_deg)), math.sin(math.radians(box.yaw_deg))
    lx, ly = c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1]
    out = np.maximum.reduce([np.abs(lx) - box.half[0] - margin, np.abs(ly) - box.half[1] - margin,
                             np.abs(d[:, 2]) - box.half[2] - margin])
    return float(out.min())


def pose_check(w: World, q5: Sequence[float], delta: float, held=None, *, target_margin: float | None = None,
               table_margin: float = 0.0, coarse: bool = False) -> Check:
    """`target_margin` None: the target is not checked (it is meant to be touched); `table_margin`: on top
    of the lowest finger / held-block point clearing the table."""
    t = w.tuning
    q5 = [float(v) for v in list(q5)[:5]]
    stride = int(t.search["coarse_stride"]) if coarse else 1
    pad = float(t.search.get("coarse_pad_m", 0.0)) if coarse else 0.0      # thinned points: grow the margins
    T = B.link5(q5)
    pts, low = B.gripper(q5, delta, held, stride, T)
    ck = t.raw.get("checks", {})
    links = B.links(q5[:4], stride)
    near = B.links(q5[:4], stride, tuple(ck.get("platform_links", ()))) if ck.get("platform_links") else pts[:0]
    body = np.concatenate([pts, near])
    mesh = float((-platform_model.depth(body[:, 0], body[:, 1], body[:, 2], PLATFORM)).min()) - t.clear["platform_mesh_m"]
    table = float(low[:, 2].min()) - t.table_z - table_margin
    m = t.map
    obstacles, name = w.cmap.clearance(pts, t.table_z, t.neighbour() + pad, t.obstacle_dz(), w.exclude,
                                       m.get("unknown_top_m"), m.get("out_of_map_free", True))
    if target_margin is not None and w.target is not None:
        d = box_clearance(w.target, pts, target_margin + pad)
        if d < obstacles:
            obstacles, name = d, w.target.name
    for box, margin in w.extra:
        d = box_clearance(box, pts, margin)
        if d < obstacles:
            obstacles, name = d, box.name
    arm, arm_name = w.cmap.clearance(links, t.table_z, t.arm_body() + pad, t.obstacle_dz(), w.exclude,
                                     m.get("unknown_top_m"), m.get("out_of_map_free", True))
    link_table = float(links[:, 2].min()) - t.table_z - t.clear["arm_body_m"]
    if link_table < arm:
        arm, arm_name = link_table, "table (arm links)"
    hull = platform_keep_out(q5[:4]) if ck.get("platform_hull", True) else float("inf")
    c = Check(limits_ok(q5), hull, mesh, table, obstacles, name)
    c.arm_body, c.arm_body_name = arm, arm_name
    return c


def interpolate(a: Sequence[float], b: Sequence[float], n: int) -> list[list[float]]:
    return [[x + (y - x) * i / n for x, y in zip(a, b)] for i in range(n + 1)]


def samples_for(a5: Sequence[float], b5: Sequence[float], t: Tuning) -> int:
    s = t.search
    ta, tb = kin.forward(list(a5)[:4])["tip"], kin.forward(list(b5)[:4])["tip"]
    joint = max(abs(x - y) for x, y in zip(list(a5)[:5], list(b5)[:5]))
    return max(int(s["min_samples"]), math.ceil(math.dist(ta, tb) * s["samples_per_m"]),
               math.ceil(joint / s["edge_max_joint_deg"]))


def move_check(w: World, a5: Sequence[float], b5: Sequence[float], delta: float, held=None, *,
               target_margin: float | None = None, table_margin: float = 0.0, coarse: bool = False,
               samples: int | None = None, skip_ends: bool = False) -> Check:
    out = Check()
    qs = interpolate(list(a5)[:5], list(b5)[:5], samples or samples_for(a5, b5, w.tuning))
    if skip_ends:
        qs = qs[1:-1]
    for q in qs:
        out = out.merge(pose_check(w, q, delta, held, target_margin=target_margin, table_margin=table_margin,
                                   coarse=coarse))
    return out


def sweep_check(w: World, q5: Sequence[float], delta_from: float, delta_to: float, *,
                target_margin: float | None = None, table_margin: float = 0.0, steps: int = 4) -> Check:
    """The fingers opening or closing at a fixed arm pose (a 'gripper' step)."""
    out = Check()
    for i in range(steps + 1):
        out = out.merge(pose_check(w, q5, delta_from + (delta_to - delta_from) * i / steps, None,
                                   target_margin=target_margin, table_margin=table_margin))
    return out
