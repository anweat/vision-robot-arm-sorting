"""Margins of a pose or a joint-linear move: joint limits, platform (fitted keep-out and mesh), table, boxes."""

from __future__ import annotations

import math
from typing import Sequence

from . import config as C
from . import gripper as GR
from .motion import Check
from .scene import Box
import platform_model    # noqa: E402
import workspace         # noqa: E402

KEEP_OUT = workspace.load_keep_out(C.ARM)
PLATFORM = platform_model.load()


def limits_ok(q5: Sequence[float]) -> bool:
    return all(C.LIMITS[j][0] - 1e-6 <= q5[j - 1] <= C.LIMITS[j][1] + 1e-6 for j in range(1, 6))


def pose_check(q5: Sequence[float], delta: float, held=None, boxes: Sequence[tuple] = (),
               table_margin: float = 0.0) -> Check:
    """`boxes`: (Box, margin) pairs. `table` is the lowest finger (or held block) point above the model
    table, less `table_margin`."""
    pts = GR.points(q5, delta, held)
    low = min(p[2] for p in GR.points(q5, delta, held, ("fingers",)))
    mesh = min(-platform_model.depth(p[0], p[1], p[2], PLATFORM) for p in pts) - C.M["platform_mesh_m"]
    worst, name = math.inf, None
    for box, margin in boxes:
        for p in pts:
            if not box.near(p, margin + 0.002):
                continue
            d = -box.depth(p, margin)
            if d < worst:
                worst, name = d, box.name
    return Check(limits_ok(q5), workspace.platform_clearance(list(q5)[:4], KEEP_OUT), mesh,
                 low - C.TABLE_Z - table_margin, worst, name)


def interpolate(a: Sequence[float], b: Sequence[float], n: int) -> list[list[float]]:
    return [[x + (y - x) * i / n for x, y in zip(a, b)] for i in range(n + 1)]


def move_check(a5: Sequence[float], b5: Sequence[float], delta: float, held=None,
               boxes: Sequence[tuple] = (), table_margin: float = 0.0, samples: int | None = None) -> Check:
    out = Check()
    for q in interpolate(list(a5)[:5], list(b5)[:5], samples or C.PATH["samples"]):
        out = out.merge(pose_check(q, delta, held, boxes, table_margin))
    return out


def sweep_check(q5: Sequence[float], delta_from: float, delta_to: float, boxes: Sequence[tuple] = (),
                table_margin: float = 0.0, steps: int = 4) -> Check:
    """The fingers opening or closing at a fixed arm pose (a 'gripper' step)."""
    out = Check()
    for i in range(steps + 1):
        out = out.merge(pose_check(q5, delta_from + (delta_to - delta_from) * i / steps, None, boxes, table_margin))
    return out


def with_margin(boxes: Sequence[Box], margin: float) -> list[tuple]:
    return [(b, margin) for b in boxes]
