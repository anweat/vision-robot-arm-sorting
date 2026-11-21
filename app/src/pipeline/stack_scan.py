"""Stack-aware block scan: how high every block stands (on the table, on one block, on two) and what holds it up.

A separate scan and a separate solver next to the verified pipeline (run.build_scene, flow.stages.scan/locate),
which assume every block on the table. Its output is a located.json in the flow's format with `bottom_m` (and
`level`, `support`) on each block, which flow.stages.build_map and the planner already read. Doc 41.

Why new views. The field-scan views (field_scan.SCAN_POSES) all have their camera 10-13 cm in front of the base
and 18-21 cm up, looking out: every ray to a block runs the same way, so a block 3 cm higher reads as a table
block 2-4 cm further out in all of them alike (the parallax between them was 0.25 cm for a 2-stack's top,
2026-11-21). The camera sees 36 x 28 deg (f = 977 px) and looks in the arm's own vertical plane, so other
ray directions come from moving the camera, not turning it: over and beyond the blocks, or beside them with
the block off the image centre. `plan_views` searches J1-J4 for the poses under which the level hypotheses
of each block (the places it would be at if it stood 0, 3 or 6 cm up) fall furthest apart in the image, each
hypothesis's cube whole in frame, the gripper >= 4 cm over the table and the move clear of the console's checks
(console_grasp.check_path: platform keep-out, table, bins, and here every place a block could be, as a tower up
to its top there). The best reachable is ~0.3 block sizes apart (a camera pulled back over the base looking out at
38-43 deg, or reaching forward and looking down at 70+ deg); the whole 9 cm column in frame with the gripper
above a 3-stack is out of reach. Each block collects two views' worth (doc 41 §2).

How a height is told. A block hypothesis is (x, y, yaw, level). Its predicted image box in a view is the box
of its projected cube (for a raised one also of its whole column: the detector sometimes boxes a stack as one),
clipped to the frame; a detector box matches it by centre (relative to its size) and size. Each detector box
seeds a hypothesis at each level; the other views lend the box that matches best; x, y, yaw are fitted to all
of them at once; views that should show it whole but have no box count against it; a raised one also needs
boxes where the column under it shows (never its own box) and must have a whole box of its own. A raised
hypothesis wins only by RAISE_MARGIN over the best lower one grown from the same boxes, so weak evidence
leaves a block on the table. A block partly hidden by a nearer one is refitted to its visible parts
(`refit_occluded`). Then roi_pose fits the top-face silhouette at the chosen level (camera moved down by the
level, `raised`) for the final x, y, yaw.

What holds it up. A block at level L > 0 needs one at L - 1 under it (<= SUPPORT_CM off): a detected one, or
one found from the boxes left over, or else an inferred support of unknown class. The boxes of a stack's
covered blocks are then dealt out by level, top down, to the part each level leaves visible (`sort_columns`);
a covered block's class needs two views agreeing, its yaw is not known (sigma 45 deg in located.json). Two
blocks of one level nearer than SAME_LEVEL_CM are one; a block seen only cut by the frame next to a block seen
whole is part of it.

    python3 src/pipeline/stack_scan.py scan [--console URL] [--bins DESIGN] [--out DIR]   # moves the arm
    python3 src/pipeline/stack_scan.py measure <run_dir> [--bins DESIGN] [--located out.json]
"""
from __future__ import annotations

import argparse
import contextlib
import io
import json
import math
import sys
import time
from dataclasses import dataclass, field, replace
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
for sub in ("", "pipeline", "tasks", "tools", "spatial", "vision"):
    p = str(ROOT / "src" / sub) if sub else str(ROOT / "src")
    if p not in sys.path:
        sys.path.insert(0, p)

BLOCK = 0.03
LEVELS = (0, 1, 2)
DET_CONF = 0.3              # lower than the scene's 0.4: a stack's lower blocks score 0.36-0.6 (2026-11-21 sim)
CENTRE_SIGMA = 0.08         # box centre off the predicted one, in box sizes
SIZE_SIGMA = 0.10           # log size ratio
CUT_WEIGHT = 0.5            # a box cut by the frame tells less (its own centre and size are cut too)
MIN_MATCH = 0.25            # a view lends its best box to a hypothesis when it matches this well
COLUMN_WEIGHT = 0.8         # a box of a raised block's whole column counts this much of a box of the block
MISS_WEIGHT = 0.3           # a view that should show the block whole and has no box for it
MIN_SCORE = 0.9             # a block needs this much evidence (two good views, or one strong)
MIN_SCORE_CUT = 0.6         # ... a table block seen in two views, cut by the frame in both (the map's edges and
                            # near strip: 7 of 87 real blocks, 2026-11-21); marked approx
CUT_NEAR_CM = 3.5           # ... unless it is this near a block seen whole (as run.DET_CUT_MERGE_CM)
RAISE_MARGIN = 0.25         # a raised block must beat the best lower hypothesis of its boxes by this (doc 41 §5:
                            # real table blocks at most +0.07, the closest true raise in the sim +0.34)
SUPPORT_CM = 2.2            # a block's support lies this near under it
SAME_LEVEL_CM = 2.6         # two blocks of one level this near are one (3 cm cubes)
YAWS = np.radians(np.arange(0.0, 90.0, 7.5))
BELOW_WEIGHT = 0.5          # a raised block: a box over the visible part of the column under it, per view
BELOW_IOU = 0.4             # ... how well such a box must cover that part (never the block's own box)
BELOW_MISS_WEIGHT = 0.0     # ... that part in frame and nothing detected there: no penalty (0.3 sank a true level-1
                            # top whose lower block the detector missed in 5 views, 2026-11-21 sim)
SUPPORT_MATCH = 0.35        # a leftover box this good for a support's visible part identifies it
APPROX_SIGMA_M = 0.015      # a block seen only cut by the frame: 1.8 cm off the scene's square on 3 real near-strip blocks
COVERED_CLASS_VIEWS = 2     # a covered block gets a class only from this many views agreeing
COVERED_SIGMA_M = 0.006     # a block under another: its top face is hidden, x, y from its sides only
COVERED_SIGMA_YAW_DEG = 45.0  # ... and its yaw is not known (2026-11-21 sim: 15-40 deg off)
BIN_FLOOR_M = 0.002         # acrylic bin plate (bins_acrylic_v2: 2 mm)


# ------------------------------------------------------------------------------------------- geometry

def raised(pose, z0: float):
    """The pose with the world moved down by z0: a point at height z0 + h is where a point at h was."""
    return replace(pose, t=np.asarray(pose.t, float) + np.asarray(pose.R, float) @ np.array([0.0, 0.0, z0]))


def z_centre(level: int, floor: float = 0.0) -> float:
    return floor + BLOCK * (level + 0.5)


def cube_corners(xy, z_c: float, yaws) -> np.ndarray:
    """(..., Y, 8, 3) corners of cubes centred at (xy, z_c) for each yaw. xy: (..., 2)."""
    xy = np.asarray(xy, float)
    h = BLOCK / 2
    u = np.array([-h, h, h, -h] * 2)
    v = np.array([-h, -h, h, h] * 2)
    w = np.array([-h] * 4 + [h] * 4)
    c, s = np.cos(yaws), np.sin(yaws)                          # (Y,)
    x = xy[..., None, None, 0] + c[:, None] * u - s[:, None] * v
    y = xy[..., None, None, 1] + s[:, None] * u + c[:, None] * v
    z = np.broadcast_to(z_c + w, x.shape)
    return np.stack([x, y, z], axis=-1)


class View:
    """One frame: its pose in the map frame (map metres, z up from the table), intrinsics and boxes."""

    def __init__(self, index: int, image: np.ndarray, pose, k: np.ndarray, dist: np.ndarray, meta: dict):
        self.index, self.image, self.pose, self.k = index, image, pose, k
        self.dist = np.zeros(5) if dist is None else np.asarray(dist, float).ravel()
        self.h, self.w = image.shape[:2]
        self.meta = meta
        self.boxes: list[Box] = []

    def project(self, pts: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """(uv, in_front) of map points (..., 3), distorted pixels as the detector sees them."""
        cam = pts @ np.asarray(self.pose.R).T + np.asarray(self.pose.t).ravel()
        front = cam[..., 2] > 0.02
        z = np.where(front, cam[..., 2], 1.0)
        x, y = cam[..., 0] / z, cam[..., 1] / z
        d = self.dist
        k1, k2, p1, p2 = d[:4]
        k3 = d[4] if len(d) > 4 else 0.0
        r2 = x * x + y * y
        rad = 1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 ** 3
        xd = x * rad + 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
        yd = y * rad + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y
        return np.stack([self.k[0, 0] * xd + self.k[0, 2], self.k[1, 1] * yd + self.k[1, 2]], axis=-1), front

    def boxes_of(self, corners: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Predicted boxes of cubes (..., 8, 3): (clipped box (..., 4), whole-in-frame (...), visible (...))."""
        uv, front = self.project(corners)
        x0, y0 = uv[..., 0].min(-1), uv[..., 1].min(-1)
        x1, y1 = uv[..., 0].max(-1), uv[..., 1].max(-1)
        ok = front.all(-1)
        whole = ok & (x0 >= 3) & (y0 >= 3) & (x1 <= self.w - 4) & (y1 <= self.h - 4)
        cb = np.stack([np.clip(x0, 0, self.w - 1), np.clip(y0, 0, self.h - 1),
                       np.clip(x1, 0, self.w - 1), np.clip(y1, 0, self.h - 1)], axis=-1)
        vis = ok & (cb[..., 2] - cb[..., 0] >= 12) & (cb[..., 3] - cb[..., 1] >= 12)
        return cb, whole, vis


@dataclass
class Box:
    view: int
    bbox: np.ndarray            # x0, y0, x1, y1 (distorted pixels)
    category: str
    conf: float
    cut: bool
    id: int = -1

    @property
    def centre(self) -> np.ndarray:
        return np.array([(self.bbox[0] + self.bbox[2]) / 2, (self.bbox[1] + self.bbox[3]) / 2])

    @property
    def size(self) -> float:
        return math.sqrt(max(1.0, (self.bbox[2] - self.bbox[0]) * (self.bbox[3] - self.bbox[1])))


def box_match(pred: np.ndarray, box: Box) -> np.ndarray:
    """0..1: how well predicted boxes (..., 4) fit a detector box, by centre (in box sizes) and size."""
    pc = np.stack([(pred[..., 0] + pred[..., 2]) / 2, (pred[..., 1] + pred[..., 3]) / 2], axis=-1)
    ps = np.sqrt(np.maximum(1.0, (pred[..., 2] - pred[..., 0]) * (pred[..., 3] - pred[..., 1])))
    dc = np.linalg.norm(pc - box.centre, axis=-1) / box.size
    ds = np.log(ps / box.size)
    return np.exp(-0.5 * ((dc / CENTRE_SIGMA) ** 2 + (ds / SIZE_SIGMA) ** 2))


def iou(a, b) -> float:
    x0, y0, x1, y1 = max(a[0], b[0]), max(a[1], b[1]), min(a[2], b[2]), min(a[3], b[3])
    inter = max(0.0, x1 - x0) * max(0.0, y1 - y0)
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union > 0 else 0.0


def cast_xy(view: View, uv, z: float) -> np.ndarray | None:
    """Map xy where the ray of pixel uv meets the plane Z = z."""
    try:
        return np.asarray(view.pose.pixel_to_plane(tuple(uv), view.k, view.dist, z=z))[:2]
    except ValueError:
        return None


# ------------------------------------------------------------------------------------------ hypotheses

@dataclass(eq=False)                     # identity: two hypotheses are never "equal"
class Hyp:
    level: int
    xy: np.ndarray
    yaw: float
    floor: float = 0.0
    members: dict = field(default_factory=dict)      # view -> (Box, match)
    misses: list = field(default_factory=list)
    score: float = 0.0
    seed: int = -1

    @property
    def z(self) -> float:
        return z_centre(self.level, self.floor)

    def key(self):
        return (self.level, frozenset(b.id for b, _ in self.members.values()))


class Solver:
    def __init__(self, views: list[View], bins_map: list[dict] | None = None):
        self.views = views
        self.boxes = [b for v in views for b in v.boxes]
        for i, b in enumerate(self.boxes):
            b.id = i
        self.bins = bins_map or []
        self._below: dict = {}

    # --- bins: a block whose centre is inside a bin's opening stands on its floor plate
    def floor_at(self, xy) -> tuple[float, str | None]:
        for b in self.bins:
            a = -b["yaw"]
            dx, dy = xy[0] - b["xy"][0], xy[1] - b["xy"][1]
            lx, ly = dx * math.cos(a) - dy * math.sin(a), dx * math.sin(a) + dy * math.cos(a)
            inside = b["inner"] / 2 - BLOCK / 2 + 0.004     # a block's centre this far in stands on the floor
            if abs(lx) <= inside and abs(ly) <= inside:
                return BIN_FLOOR_M, b["name"]
        return 0.0, None

    def predict(self, view: View, xy, z_c: float, yaws=YAWS):
        return view.boxes_of(cube_corners(xy, z_c, yaws))

    def predict_column(self, view: View, xy, level: int, floor: float, yaws=YAWS):
        """The box of a whole column (levels 0..level at one place and yaw): the detector boxes a stack as one
        block now and then (2026-11-21 sim, 2-stack: 4 of 11 views)."""
        return view.boxes_of(np.concatenate([cube_corners(xy, z_centre(lv, floor), yaws) for lv in range(level + 1)],
                                            axis=-2))

    def match_any(self, view: View, h_level: int, floor: float, xy, z_c: float, yaws, box: Box):
        """A box against a hypothesis: its cube, or for a raised one also its whole column (COLUMN_WEIGHT)."""
        pred, whole, vis = self.predict(view, xy, z_c, yaws)
        m = box_match(pred, box)
        if h_level > 0:
            pc, _, vc = self.predict_column(view, xy, h_level, floor, yaws)
            m = np.maximum(m, COLUMN_WEIGHT * np.where(vc, box_match(pc, box), 0.0))
        return m, whole, vis

    def evaluate(self, h: Hyp, exclude: set[int] = frozenset(), assign: bool = True) -> Hyp:
        """Members (one box per view, the best match), misses and score of h at its xy / yaw."""
        members, misses, score = {}, [], 0.0
        for v in self.views:
            pred, whole, vis = self.predict(v, h.xy, h.z, np.array([h.yaw]))
            pred, whole, vis = pred[0], bool(whole[0]), bool(vis[0])
            if not vis:
                continue
            if assign:
                best = None
                for b in v.boxes:
                    if b.id in exclude:
                        continue
                    m = float(self.match_any(v, h.level, h.floor, h.xy, h.z, np.array([h.yaw]), b)[0][0])
                    if best is None or m > best[1]:
                        best = (b, m)
                if best is not None and best[1] >= MIN_MATCH:
                    members[v.index] = best
                elif whole:
                    misses.append(v.index)
            else:
                if v.index in h.members:
                    b = h.members[v.index][0]
                    members[v.index] = (b, float(self.match_any(v, h.level, h.floor, h.xy, h.z, np.array([h.yaw]), b)[0][0]))
                elif whole:
                    misses.append(v.index)
        for b, m in members.values():
            score += b.conf * m * (CUT_WEIGHT if b.cut else 1.0)
        score -= MISS_WEIGHT * len(misses)
        h.members, h.misses = members, misses
        h.own, h.below = score, []
        if h.level > 0:                       # what holds it up must show somewhere below it
            if not any(not b.cut and m >= 0.5 for b, m in members.values()):
                score = -1.0                  # a raised block seen only cut by the frame is not measured here
            else:
                below = 0.0
                for v in self.views:
                    e = self.below_evidence(v, h)
                    if e is not None:
                        h.below.append((v.index, round(e, 2)))
                        below += BELOW_WEIGHT * e if e > 0 else -BELOW_MISS_WEIGHT
                # it backs the block's own boxes, never stands in for them (a cut box over a neighbour's column
                # scored 2.4 from the neighbour alone, 2026-11-06 real scan)
                score += min(below, max(score, 0.0))
        h.score = score
        return h

    def below_evidence(self, view: View, h: Hyp) -> float | None:
        """For a raised hypothesis: the block right under it (at its xy and yaw), as much of it as h does not hide. Some box over that part (conf x IoU quality, any box: a lower block's box may also be
        another hypothesis's) -> > 0; that part whole in frame, big enough and no box there -> 0 (a miss);
        not seen -> None."""
        own = h.members.get(view.index, (None,))[0]
        key = (view.index, h.level, round(float(h.xy[0]), 4), round(float(h.xy[1]), 4), round(h.yaw, 3), h.floor,
               None if own is None else own.id)
        if key in self._below:
            return self._below[key]
        out = None
        col = [Hyp(h.level - 1, h.xy, h.yaw, h.floor)]     # the block right under it (the whole column under a
                                                          # level-2 one met a neighbour's box and raised a level-1
                                                          # top to 2, 2026-11-21 sim)
        corners = np.concatenate([cube_corners(c.xy, c.z, np.array([c.yaw]))[0] for c in col])
        uv, front = view.project(corners)
        top_uv, top_front = view.project(cube_corners(h.xy, h.z, np.array([h.yaw]))[0])
        if front.all() and top_front.all():
            x0, y0 = np.floor(np.minimum(uv.min(0), top_uv.min(0))).astype(int) - 2
            x1, y1 = np.ceil(np.maximum(uv.max(0), top_uv.max(0))).astype(int) + 2
            if 0 < x1 - x0 < 3000 and 0 < y1 - y0 < 3000:
                m = np.zeros((y1 - y0, x1 - x0), np.uint8)
                for c in col:
                    cu, _ = view.project(cube_corners(c.xy, c.z, np.array([c.yaw]))[0])
                    cv2.fillConvexPoly(m, cv2.convexHull((cu - [x0, y0]).astype(np.int32)), 1)
                full = int(m.sum())
                cv2.fillConvexPoly(m, cv2.convexHull((top_uv - [x0, y0]).astype(np.int32)), 0)
                ys, xs = np.nonzero(m)
                if len(xs) >= 0.25 * full and len(xs) >= 300:
                    bx = np.array([xs.min() + x0, ys.min() + y0, xs.max() + x0, ys.max() + y0], float)
                    whole = bx[0] >= 3 and bx[1] >= 3 and bx[2] <= view.w - 4 and bx[3] <= view.h - 4
                    bx[[0, 2]] = np.clip(bx[[0, 2]], 0, view.w - 1)
                    bx[[1, 3]] = np.clip(bx[[1, 3]], 0, view.h - 1)
                    if bx[2] - bx[0] >= 12 and bx[3] - bx[1] >= 12:
                        own = h.members.get(view.index, (None,))[0]
                        best = max((b.conf * min(1.0, iou(bx, b.bbox) / 0.6) for b in view.boxes
                                    if b is not own and iou(bx, b.bbox) >= BELOW_IOU), default=0.0)
                        if best > 0 or whole:
                            out = float(best)
        self._below[key] = out
        return out

    def fit(self, h: Hyp, span: float = 0.012, step: float = 0.003) -> Hyp:
        """x, y, yaw maximising the summed match of h's member boxes (one cube seen by all of them)."""
        mem = [(self.views[v], b) for v, (b, _) in h.members.items()]
        if not mem:
            return h
        best = (-1.0, h.xy, h.yaw)
        centre = np.asarray(h.xy, float)
        for _ in range(3):
            offs = np.arange(-span, span + 1e-9, step)
            gx, gy = np.meshgrid(offs, offs)
            xy = centre + np.stack([gx.ravel(), gy.ravel()], axis=-1)            # (N, 2)
            total = np.zeros((len(xy), len(YAWS)))
            for v, b in mem:
                m, _, vis = self.match_any(v, h.level, h.floor, xy, h.z, YAWS, b)
                total += np.where(vis, m, 0.0) * b.conf * (CUT_WEIGHT if b.cut else 1.0)
            i, j = np.unravel_index(np.argmax(total), total.shape)
            if total[i, j] > best[0]:
                best = (float(total[i, j]), xy[i], float(YAWS[j]))
            centre, span, step = best[1], span / 3, step / 3
        h.xy, h.yaw = np.asarray(best[1], float), best[2]
        return h

    def grow(self, seed: Box, level: int, exclude: set[int] = frozenset()) -> Hyp | None:
        v = self.views[seed.view]
        floor = 0.0
        xy = cast_xy(v, seed.centre, z_centre(level))
        if xy is None:
            return None
        floor, _ = self.floor_at(xy)
        h = Hyp(level, xy, 0.0, floor, {seed.view: (seed, 1.0)}, seed=seed.id)
        h = self.fit(h, span=0.015, step=0.003)
        for _ in range(3):
            h = self.evaluate(h, exclude=exclude)
            if seed.view not in h.members or h.members[seed.view][0].id != seed.id:
                h.members[seed.view] = (seed, 0.0)            # the seed stays in: it is what this hypothesis explains
            before = h.key()
            h = self.fit(h)
            h.floor = self.floor_at(h.xy)[0]
            if h.key() == before:
                break
        return self.evaluate(h, assign=False)

    def hypotheses(self) -> list[Hyp]:
        out, seen = [], set()
        for b in self.boxes:
            for lv in LEVELS:
                h = self.grow(b, lv)
                if h is None or not (-0.17 < h.xy[0] < 0.17 and -0.03 < h.xy[1] < 0.24):
                    continue
                if h.key() in seen:
                    continue
                seen.add(h.key())
                out.append(h)
        return out

    # --- choose blocks
    def select(self, hyps: list[Hyp], min_score: float = MIN_SCORE) -> list[Hyp]:
        """Greedy, best explanation first; one box to one block; a raised block only by RAISE_MARGIN over the
        best lower hypothesis among its own boxes."""
        chosen, used = [], set()
        pool = list(hyps)
        while True:
            live = []
            for h in pool:
                ids = {b.id for b, _ in h.members.values()}
                if ids & used:
                    keep = {v: bm for v, bm in h.members.items() if bm[0].id not in used}
                    if not keep:
                        continue
                    h = replace(h, members=keep)
                    h = self.evaluate(h, assign=False)
                live.append(h)
            pool = [h for h in live if (h.score >= min_score and getattr(h, "own", h.score) >= min_score)
                    or (h.level == 0 and h.score >= min(min_score, MIN_SCORE_CUT) and len(h.members) >= 2
                        and all(b.cut for b, _ in h.members.values()))]
            if not pool:
                break
            pool.sort(key=lambda h: -h.score)
            best = pool[0]
            ids = {b.id for b, _ in best.members.values()}
            if best.level > 0:
                lower = [h for h in pool if h.level < best.level and ids & {b.id for b, _ in h.members.values()}]
                for b, _ in sorted(best.members.values(), key=lambda bm: -bm[0].conf * bm[1])[:2]:
                    for lv in range(best.level):       # the same boxes one or two levels down, grown afresh
                        g = self.grow(b, lv, exclude=used)
                        if g is not None:
                            lower.append(g)
                if lower:
                    alt = max(lower, key=lambda h: h.score)
                    best.alt = dict(level=alt.level, score=round(alt.score, 3))
                    if best.score < alt.score + RAISE_MARGIN:
                        if alt.score < min_score:
                            pool.remove(best)          # neither is good enough: drop this one, try the rest
                            continue
                        best = alt
                        ids = {b.id for b, _ in best.members.values()}
            if not hasattr(best, "alt"):
                higher = [h for h in pool if h.level > best.level and ids & {b.id for b, _ in h.members.values()}]
                if higher:
                    alt = max(higher, key=lambda h: h.score)
                    best.alt = dict(level=alt.level, score=round(alt.score, 3))
            chosen.append(best)
            used |= ids
            pool = [h for h in pool if h is not best]
        return chosen

    # --- stacks: which of the leftover boxes belongs to which covered block
    def sort_columns(self, blocks: list[Hyp]) -> None:
        """For each stack, the boxes of its covered blocks (and the boxes nobody took) go to the level whose part
        left visible by the blocks above matches them best: a stack's middle block shows only its sides, which
        also fit a table block a little nearer the camera (2026-11-21 sim: the middle box taken for the bottom
        block's, the bottom one unseen). A covered block with no box keeps no class; its x, y, yaw are refitted
        to its boxes' visible parts near the block above it."""
        above_of = {id(h): [] for h in blocks}
        for h in blocks:
            s = getattr(h, "support", None)
            chain = []
            while s is not None:
                chain.append(s)
                s = getattr(s, "support", None)
            for c in chain:
                above_of[id(c)].append(h)
        covered = [h for h in blocks if above_of[id(h)]]
        if not covered:
            return
        others = {b.id for h in blocks if h not in covered for b, _ in h.members.values()}
        taken: set[int] = set()
        for h in sorted(covered, key=lambda k: -k.level):          # top down: the middle block before the bottom
            above = above_of[id(h)]
            top = min(above, key=lambda a: a.level)
            pools = {v.index: [b for b in v.boxes if b.id not in others and b.id not in taken] for v in self.views}
            best = None
            for dx in (-0.006, 0.0, 0.006):
                for dy in (-0.006, 0.0, 0.006):
                    for yaw in YAWS[::2]:
                        c = Hyp(h.level, top.xy + [dx, dy], float(yaw), h.floor)
                        got, total = {}, 0.0
                        for v in self.views:
                            if not pools[v.index]:
                                continue
                            pb = self.visible_box(v, c, above)
                            if pb is None:
                                continue
                            bm = max(((bb, float(box_match(pb, bb))) for bb in pools[v.index]), key=lambda t: t[1])
                            if bm[1] >= SUPPORT_MATCH:
                                got[v.index] = bm
                                total += bm[0].conf * bm[1]
                        if best is None or total > best[0]:
                            best = (total, c, got)
            _, c, got = best
            h.covered = True
            h.members = got
            h.inferred = not got
            if got:
                h.xy, h.yaw = c.xy, c.yaw
            else:                               # nothing of it shows: under the block above it, yaw unknown
                h.xy, h.yaw = np.asarray(top.xy, float), top.yaw
            taken |= {bb.id for bb, _ in got.values()}

    def refit_occluded(self, h: Hyp, blocks: list[Hyp]) -> None:
        """A block partly hidden by a nearer block in some of its views has a box of its visible part only; fitted
        as a whole cube it is pulled away from the camera (2026-11-21 sim with joint jitter: a table block behind a
        2-stack came out 3 cm off). Refit x, y, yaw to the visible parts, the nearer blocks in front."""
        def nearer(v: View, x) -> list[Hyp]:
            d = np.linalg.norm(v.pose.centre - [x.xy[0], x.xy[1], x.z])
            return [o for o in blocks if o is not h and o.members is not None and
                    np.linalg.norm(v.pose.centre - [o.xy[0], o.xy[1], o.z]) < d - 0.01 and
                    np.linalg.norm(o.xy - x.xy) < 0.08]
        occ = {vi: nearer(self.views[vi], h) for vi in h.members}
        if not any(occ.values()):
            return

        def total(c: Hyp) -> float:
            t = 0.0
            for vi, (b, _) in h.members.items():
                v = self.views[vi]
                if occ[vi]:
                    pb = self.visible_box(v, c, occ[vi])
                    m = 0.0 if pb is None else float(box_match(pb, b))
                else:
                    pred, _, vis = self.predict(v, c.xy, c.z, np.array([c.yaw]))
                    m = float(box_match(pred[0], b)) if vis[0] else 0.0
                t += b.conf * m
            return t

        best = (total(h), h.xy, h.yaw)
        for span, step, yaws in ((0.03, 0.005, YAWS[::2]), (0.004, 0.002, None)):
            centre, yaw0 = best[1], best[2]
            ys = yaws if yaws is not None else [yaw0 + d for d in np.radians((-7.5, 0.0, 7.5))]
            for dx in np.arange(-span, span + 1e-9, step):
                for dy in np.arange(-span, span + 1e-9, step):
                    for yaw in ys:
                        c = Hyp(h.level, centre + [dx, dy], float(yaw), h.floor)
                        t = total(c)
                        if t > best[0] + 1e-6:
                            best = (t, c.xy, c.yaw)
        h.xy, h.yaw = np.asarray(best[1], float), best[2]
        h.occluded_in = sorted(vi for vi, o in occ.items() if o)

    # --- supports
    def visible_box(self, view: View, h: Hyp, above: list[Hyp]):
        """The predicted box of the part of h not hidden by the blocks above it, or None."""
        corners = cube_corners(h.xy, h.z, np.array([h.yaw]))[0]
        uv, front = view.project(corners)
        if not front.all():
            return None
        x0, y0 = np.floor(uv.min(0)).astype(int) - 2
        x1, y1 = np.ceil(uv.max(0)).astype(int) + 2
        w, hh = x1 - x0, y1 - y0
        if w <= 0 or hh <= 0 or w > 2000 or hh > 2000:
            return None
        m = np.zeros((hh, w), np.uint8)
        cv2.fillConvexPoly(m, cv2.convexHull((uv - [x0, y0]).astype(np.int32)), 1)
        for a in above:
            ua, fa = view.project(cube_corners(a.xy, a.z, np.array([a.yaw]))[0])
            if fa.all():
                cv2.fillConvexPoly(m, cv2.convexHull((ua - [x0, y0]).astype(np.int32)), 0)
        ys, xs = np.nonzero(m)
        if len(xs) < 150:
            return None
        bx = np.array([xs.min() + x0, ys.min() + y0, xs.max() + x0, ys.max() + y0], float)
        bx[[0, 2]] = np.clip(bx[[0, 2]], 0, view.w - 1)
        bx[[1, 3]] = np.clip(bx[[1, 3]], 0, view.h - 1)
        if bx[2] - bx[0] < 12 or bx[3] - bx[1] < 12:
            return None
        return bx

    def find_support(self, top: Hyp, above: list[Hyp], used: set[int]) -> Hyp:
        """The block under `top`: searched near it among the boxes nobody took, by its visible part."""
        lv = top.level - 1
        best = None
        for dx in (-0.006, 0.0, 0.006):
            for dy in (-0.006, 0.0, 0.006):
                for yaw in YAWS[::2]:
                    h = Hyp(lv, top.xy + [dx, dy], float(yaw), top.floor)
                    members, score = {}, 0.0
                    for v in self.views:
                        pb = self.visible_box(v, h, above)
                        if pb is None:
                            continue
                        cand = [(b, float(box_match(pb, b))) for b in v.boxes if b.id not in used]
                        if cand:
                            b, m = max(cand, key=lambda t: t[1])
                            if m >= SUPPORT_MATCH:
                                members[v.index] = (b, m)
                                score += b.conf * m
                    if best is None or score > best.score:
                        h.members, h.score = members, score
                        best = h
        if best is None or not best.members:
            return Hyp(lv, np.asarray(top.xy, float), top.yaw, top.floor, score=0.0)
        return best


# ------------------------------------------------------------------------------------------ the scan

def load(run: Path, conf: float = DET_CONF, detector=None):
    """(Scan, views) of a run: the scene's registration and camera poses (bundle), detector boxes per view."""
    import scene as S
    with contextlib.redirect_stdout(io.StringIO()):
        scan = S.Scan(run)
        scan.register()
        scan.camera_poses()
    if detector is None:
        from blocks_detector import BlocksDetector
        detector = BlocksDetector()
    views = []
    for i, (img, pose, meta) in enumerate(zip(scan.images, scan.poses, scan.views)):
        v = View(i, img, pose, scan.k, scan.dist, meta)
        for d in detector.detect(img, conf=conf):
            x0, y0, x1, y1 = d["bbox"]
            cut = x0 <= 2 or y0 <= 2 or x1 >= v.w - 3 or y1 >= v.h - 3
            v.boxes.append(Box(i, np.array(d["bbox"], float), d["category"], float(d["conf"]), cut))
        views.append(v)
    return scan, views


def bins_in_map(scan, design: str | None, run: Path) -> list[dict]:
    """The bins of `design` (found from bin views, else their planned places) in the scan's map frame."""
    if not design:
        return []
    import run as R
    with contextlib.redirect_stdout(io.StringIO()):
        found = R.locate_bins(run, design)
    inv = np.linalg.inv(scan.base_T_map)
    yaw_map = math.atan2(scan.base_T_map[1, 0], scan.base_T_map[0, 0])
    out = []
    for b in found.bins.values():
        p = inv @ np.array([b.xy[0], b.xy[1], 0.0, 1.0])
        out.append(dict(name=f"bin{b.number}", category=b.category, xy=p[:2], yaw=math.radians(b.yaw_deg) - yaw_map,
                        inner=found.inner_m, base_xy=[float(b.xy[0]), float(b.xy[1])], yaw_deg=float(b.yaw_deg),
                        design=design, outer=found.design["geometry_mm"]["outer"] / 1000,
                        height=found.design["geometry_mm"]["rim_top_above_table"] / 1000, source=b.source))
    return out


def roi_refine(views: list[View], h: Hyp, skip=()) -> dict | None:
    """roi_pose's silhouette fit of the top face at h's level in each view with a whole box (and not partly hidden
    by a nearer block, `skip`): pooled x, y, yaw."""
    import roi_pose as RP
    fits = []
    for vi, (b, m) in h.members.items():
        if b.cut or m < 0.3 or vi in skip:
            continue
        v = views[vi]
        z0 = h.z - BLOCK / 2
        with contextlib.redirect_stdout(io.StringIO()):
            f = RP.solve_top_face(v.image, b.bbox, raised(v.pose, z0), v.k, v.dist,
                                  initial_yaw=math.degrees(h.yaw), prior_xy=list(h.xy))
        if f.get("ok"):
            fits.append(f)
    if not fits:
        return None
    xy = np.median([f["map_cm"] for f in fits], axis=0) / 100
    a = np.radians(np.array([f["yaw_deg"] for f in fits]) * 4)
    yaw = math.degrees(math.atan2(np.sin(a).mean(), np.cos(a).mean())) / 4
    return dict(xy=xy, yaw_deg=yaw, n=len(fits),
                spread_cm=round(float(np.median([np.linalg.norm(np.array(f["map_cm"]) / 100 - xy) for f in fits])) * 100, 2))


def fold(d: float) -> float:
    return (d + 45) % 90 - 45


def measure(run: Path, bins_design: str | None = None, roi: bool = True, detector=None) -> dict:
    """Every block of a run with its level and support, in map coordinates; written to <run>/stack_scene.json."""
    run = Path(run)
    scan, views = load(run, detector=detector)
    bins = bins_in_map(scan, bins_design, run)
    sol = Solver(views, bins)
    hyps = sol.hypotheses()
    chosen = sol.select(hyps)
    # one block per place and level: the better-evidenced of two that nearly coincide
    chosen.sort(key=lambda h: -h.score)
    kept: list[Hyp] = []
    for h in chosen:
        if any(k.level == h.level and np.linalg.norm(k.xy - h.xy) < SAME_LEVEL_CM / 100 for k in kept):
            continue
        kept.append(h)
    # a block seen only through boxes cut by the frame, next to a block seen whole: those boxes are that block's
    # (or the stack's under it) -- a cut box's centre is pulled inwards (2026-11-21 sim with joint jitter: the
    # lower block of a 2-stack, cut in two height views, came out 2.7 cm off as a block of its own)
    whole = [h for h in kept if any(not b.cut for b, _ in h.members.values())]
    kept = [h for h in kept if h in whole or
            all(np.linalg.norm(k.xy - h.xy) >= CUT_NEAR_CM / 100 for k in whole)]
    used = {b.id for h in kept for b, _ in h.members.values()}
    # supports, top down: a raised block with nothing under it gets one, found from leftover boxes or inferred
    blocks = sorted(kept, key=lambda h: -h.level)
    i = 0
    while i < len(blocks):
        h = blocks[i]
        if h.level > 0:
            under = [k for k in blocks if k.level == h.level - 1
                     and np.linalg.norm(k.xy - h.xy) <= SUPPORT_CM / 100]
            if under:
                h.support = min(under, key=lambda k: np.linalg.norm(k.xy - h.xy))
            else:
                above = [a for a in blocks if a.level >= h.level and np.linalg.norm(a.xy - h.xy) < 0.04]
                s = sol.find_support(h, above, used)
                s.inferred = not s.members
                used |= {b.id for b, _ in s.members.values()}
                h.support = s
                blocks.append(s)
                blocks.sort(key=lambda k: -k.level)
                i = 0
                continue
        i += 1
    sol.sort_columns(blocks)
    for h in blocks:
        if h.members and not getattr(h, "covered", False):
            sol.refit_occluded(h, blocks)
    # refine with the silhouette at the chosen level (a covered block shows no top face: its fit stays)
    out = []
    for n, h in enumerate(sorted(blocks, key=lambda k: (k.level, -k.score))):
        r = roi_refine(views, h, skip=getattr(h, "occluded_in", ())) if roi and h.members \
            and not getattr(h, "inferred", False) and not getattr(h, "covered", False) else None
        xy, yaw = (r["xy"], r["yaw_deg"]) if r else (h.xy, math.degrees(h.yaw))
        h.final_xy, h.final_yaw = xy, fold(yaw)
        votes: dict = {}
        for b, m in h.members.values():
            votes[b.category] = votes.get(b.category, 0.0) + b.conf * max(m, 0.2)
        h.votes = {k: round(v, 2) for k, v in sorted(votes.items(), key=lambda t: -t[1])}
        h.roi = r
    for n, h in enumerate(sorted(blocks, key=lambda k: (k.final_xy[1], k.final_xy[0], k.level))):
        h.name = f"s{n + 1}"
    for h in blocks:
        floor, bin_name = sol.floor_at(h.final_xy)
        lead = max(h.votes, key=h.votes.get) if h.votes else None
        if lead and getattr(h, "covered", False) and \
                len({v for v, (b, _) in h.members.items() if b.category == lead}) < COVERED_CLASS_VIEWS:
            lead = None                     # one box of a covered block (often cut) named it wrong 2 times in 8
        parallax = view_spread(views, h)
        out.append(dict(
            name=h.name, level=h.level, bottom_cm=round((floor + BLOCK * h.level) * 100, 2),
            map_cm=[round(float(v) * 100, 2) for v in h.final_xy], yaw_deg=round(float(h.final_yaw), 1),
            category=lead, votes=h.votes, score=round(float(h.score), 3), alt=getattr(h, "alt", None),
            inferred=bool(getattr(h, "inferred", False)), covered=bool(getattr(h, "covered", False)),
            approx=bool(h.members) and (all(b.cut for b, _ in h.members.values())
                                        or set(getattr(h, "occluded_in", ())) >= set(h.members)),
            in_bin=bin_name,
            support=getattr(getattr(h, "support", None), "name", None),
            views=len(h.members), misses=h.misses, parallax=parallax,
            roi=None if h.roi is None else dict(n=h.roi["n"], spread_cm=h.roi["spread_cm"]),
            below=getattr(h, "below", []),
            boxes=[dict(view=v, bbox=[round(float(x), 1) for x in b.bbox], category=b.category, conf=round(b.conf, 2),
                        match=round(m, 2), cut=b.cut) for v, (b, m) in sorted(h.members.items())]))
    out.sort(key=lambda o: o["name"])
    leftover = [dict(view=b.view, bbox=[round(float(x), 1) for x in b.bbox], category=b.category, conf=round(b.conf, 2))
                for b in sol.boxes if b.id not in {bb.id for h in blocks for bb, _ in h.members.values()}]
    res = dict(run=str(run), registration={k: v for k, v in scan.registration.items() if k != "per_view"},
               views=[dict(image=v.meta["image"], purpose=v.meta.get("purpose", "field"), boxes=len(v.boxes))
                      for v in views],
               bins=[dict(name=b["name"], category=b["category"], map_cm=[round(float(x) * 100, 2) for x in b["xy"]])
                     for b in bins],
               blocks=out, unused_boxes=leftover, t=time.time())
    (run / "stack_scene.json").write_text(json.dumps(res, indent=1, ensure_ascii=False, default=_plain))
    return res


def view_spread(views: list[View], h: Hyp) -> float:
    """How far apart the level hypotheses of this block fall in its member views, in box sizes: the parallax its
    height was told with (the largest difference between two views of the shift a 3 cm rise makes)."""
    shifts = []
    for vi, (b, _) in h.members.items():
        v = views[vi]
        a, _ = v.project(np.array([h.xy[0], h.xy[1], h.z]))
        c = cast_xy(v, a, h.z + BLOCK)
        if c is None:
            continue
        shifts.append(c - h.xy)
    if len(shifts) < 2:
        return 0.0
    return round(max(float(np.linalg.norm(p - q)) for p in shifts for q in shifts) * 100, 2)


def _plain(o):
    if isinstance(o, np.generic):
        return o.item()
    if isinstance(o, np.ndarray):
        return o.tolist()
    raise TypeError(type(o))


def located(scene: dict, run: Path, tuning=None, bins_design: str | None = None) -> dict:
    """stack_scene -> located.json in flow.stages.locate's format, each block with bottom_m / level / support."""
    from planning import tuning as PT
    t = tuning or PT.default()
    reg = scene.get("registration") or {}
    ox, oy = reg["map_origin_base_cm"][0] / 100, reg["map_origin_base_cm"][1] / 100
    spread = max(reg.get("spread_cm", [0, 0])[:2]) / 100
    sigma = math.hypot(float(t.errors["target_xy_m"]), spread)
    objects, ids = [], {}
    for i, b in enumerate(scene["blocks"], start=1):
        ids[b["name"]] = f"b{i}"
    for i, b in enumerate(scene["blocks"], start=1):
        mx, my = b["map_cm"][0] / 100, b["map_cm"][1] / 100
        objects.append(dict(
            id=i, name=ids[b["name"]], kind="block", **{"class": b["category"]},
            centre_m=[round(ox + mx, 5), round(oy + my, 5)], yaw_deg=b["yaw_deg"], size_m=[BLOCK] * 3,
            sigma_xy_m=round(max(sigma, APPROX_SIGMA_M) if b["approx"] else
                             max(sigma, COVERED_SIGMA_M) if b["covered"] else sigma, 4),
            approx=b["approx"],
            sigma_yaw_deg=float(t.errors["target_yaw_deg"]) if not b["covered"] else COVERED_SIGMA_YAW_DEG,
            bottom_m=round(b["bottom_cm"] / 100, 4), level=b["level"], covered=b["covered"],
            support=ids.get(b["support"]) if b["support"] else None, inferred=b["inferred"], in_bin=b["in_bin"],
            score=b["score"], colour=None, sticker=None, views=b["views"], map_cm=b["map_cm"],
            source="stack_scan" + (" (inferred support)" if b["inferred"] else "")))
    if bins_design:
        import run as R
        with contextlib.redirect_stdout(io.StringIO()):
            found = R.locate_bins(Path(run), bins_design)
        g = found.design["geometry_mm"]
        for b in found.bins.values():
            objects.append(dict(
                id=len(objects) + 1, name=f"bin{b.number}", kind="bin", **{"class": b.category}, design=bins_design,
                centre_m=[round(float(b.xy[0]), 5), round(float(b.xy[1]), 5)], yaw_deg=round(float(b.yaw_deg), 2),
                size_m=[g["outer"] / 1000, g["outer"] / 1000, g["rim_top_above_table"] / 1000],
                sigma_xy_m=round(float(t.errors["obstacle_xy_m"]) if b.source == "markers" else 0.01, 4),
                source=b.source))
    return dict(scan_dir=str(run), registration=reg, placed_by="scan registration (stack_scan)",
                registration_yaw_applied=False, table_z_m=t.table_z, objects=objects, made_by="stack_scan", t=time.time())


# ------------------------------------------------------------------------------------- the height views

SAFE_ABOVE_TABLE_M = 0.04       # gripper points at a height view (the columns themselves are obstacles)
FRAME_MARGIN_PX = 12
GAIN_GOAL = 0.35                # level hypotheses this far apart (in block sizes) in some view: told apart
                                # (2.9 CENTRE_SIGMA; 0.6 is out of reach for most of the map, see doc 41)
CREDIT_CAP = 2 * GAIN_GOAL      # gain a column collects over its height views
PLAN_MIN_SCORE = 0.3            # a block worth a height view (blocks cut by the frame in every field view too)
ABOVE_RING_M = 0.035            # the cube one level up must show within this ring (see plan_views)
ONE_SEEN_GAIN = 0.3             # of two neighbouring level hypotheses only one is whole in frame
MAX_HEIGHT_VIEWS = 6
_GRID = None


def _pose_grid():
    """(q234 at J1 = 90, base_T_cam, lowest gripper point z) over J2-J4 in 5 deg steps, gripper kept high."""
    global _GRID
    if _GRID is None:
        import console_grasp as G
        import dofbot_kinematics as kin
        from camera_model import ArmCamera
        cam = ArmCamera()
        qs, ts = [], []
        for j2 in range(-10, 176, 5):
            for j3 in range(-10, 176, 5):
                for j4 in range(-10, 176, 5):
                    q = [90, j2, j3, j4]
                    p5 = np.array(kin.servo5_pose(q + [90]))
                    o, z = p5[:3, 3], p5[:3, 2]
                    low = min(o[2], o[2] + kin.GRIPPER * z[2])
                    if low < G.TABLE_Z + SAFE_ABOVE_TABLE_M:
                        continue
                    qs.append(q)
                    ts.append(cam.base_T_cam(q + [G.ROLL_ZERO]))
        _GRID = (np.array(qs), np.stack(ts))
    return _GRID


def _rot_z(deg: float) -> np.ndarray:
    a = math.radians(deg)
    m = np.eye(4)
    m[:2, :2] = [[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]]
    return m


def columns(sol: Solver, hyps: list[Hyp]) -> list[dict]:
    """Where each block would be at each level, from the field views: [{levels: {L: map xy}, ids}]. One column
    per block the field views alone pick (Solver.select); its levels are its best box cast to each level's
    middle plane and fitted there (chaining hypotheses by shared boxes merged unrelated blocks)."""
    cols = []
    for h in sol.select(hyps, min_score=PLAN_MIN_SCORE):   # where to look: weak and cut-only blocks too
        b, _ = max(h.members.values(), key=lambda bm: (not bm[0].cut, bm[0].conf * bm[1]))
        levels = {}
        for lv in LEVELS:
            g = sol.grow(b, lv)
            if g is not None:
                levels[lv] = g.xy.copy()
            else:
                xy = cast_xy(sol.views[b.view], b.centre, z_centre(lv))
                if xy is not None:
                    levels[lv] = xy
        if len(levels) == len(LEVELS):
            cols.append(dict(levels=levels, ids={bb.id for bb, _ in h.members.values()}))
    return cols


def plan_views(cols: list[dict], base_T_map: np.ndarray, current: list, bins: list[dict] = (),
               max_views: int = MAX_HEIGHT_VIEWS) -> list[dict]:
    """Height views for the block columns: poses under which each column's level hypotheses fall furthest apart
    in the image (`gain`, in block sizes), every hypothesis's cube whole in frame, the gripper out of the columns, the move
    clear of console_grasp.check_path with the block places (column_obstacles) and bins as obstacles. Greedy cover, most gain first."""
    import console_grasp as G
    from map_registration import load_intrinsics
    if not cols:
        return []
    k, _ = load_intrinsics()
    qs, ts = _pose_grid()
    # per column: for each level hypothesis its cube (any yaw: a 2.2 cm ring at its bottom and top), base frame.
    # The whole 9 cm column in frame as well left almost no pose (the camera sees 36 x 28 deg).
    # The cube one level up at the same place must show too: the box a column was seeded from may be a stack's
    # middle block, the top being cut by the frame in every field view (2026-11-21 sim: the height views then
    # showed the middle block well and the top cut again, and the top was lost).
    # A middle block's box shows only its sides, so the places cast from it are ~2 cm off: the cube above is asked
    # for over a wider ring (ABOVE_RING_M).
    pts = []
    for c in cols:
        for lv in LEVELS:
            xy = c["levels"][lv]
            top = BLOCK * min(lv + 2, len(LEVELS))
            for zz, r in ((BLOCK * lv, 0.022), (BLOCK * (lv + 1), 0.022), (top, ABOVE_RING_M if lv + 1 < len(LEVELS) else 0.022)):
                for a in np.linspace(0, 2 * math.pi, 8, endpoint=False):
                    pts.append(base_T_map @ [xy[0] + r * math.cos(a), xy[1] + r * math.sin(a), zz, 1.0])
    pts = np.array(pts).reshape(len(cols), len(LEVELS), 24, 4)
    centres = np.array([[base_T_map @ [*c["levels"][lv], z_centre(lv), 1.0] for lv in LEVELS] for c in cols])
    cands = []
    for j1 in range(20, 161, 3):
        T = np.einsum("ij,njk->nik", _rot_z(j1 - 90), ts)                    # base_T_cam
        Ti = np.linalg.inv(T)
        cam = np.einsum("nij,clpj->nclpi", Ti[:, :3, :], pts)                   # (N, C, L, 16, 3)
        z = cam[..., 2]
        u = k[0, 0] * cam[..., 0] / np.maximum(z, 1e-3) + k[0, 2]
        v = k[1, 1] * cam[..., 1] / np.maximum(z, 1e-3) + k[1, 2]
        m = FRAME_MARGIN_PX
        seen = ((z > 0.05) & (u > m) & (u < 640 - m) & (v > m) & (v < 480 - m)).all(axis=3)     # (N, C, L)
        cc = np.einsum("nij,clj->ncli", Ti[:, :3, :], centres)
        cu = k[0, 0] * cc[..., 0] / np.maximum(cc[..., 2], 1e-3)
        cv_ = k[1, 1] * cc[..., 1] / np.maximum(cc[..., 2], 1e-3)
        size = k[0, 0] * BLOCK / np.maximum(cc[..., 2], 1e-3)               # a block's size at each hypothesis
        sep = np.hypot(np.diff(cu, axis=2), np.diff(cv_, axis=2)) / np.minimum(size[..., 1:], size[..., :-1])
        # two neighbouring levels: both whole in frame -> how far apart; only one -> the other shows as a
        # missing box (ONE_SEEN, occlusion makes that weaker); neither -> nothing
        both, one = seen[..., 1:] & seen[..., :-1], seen[..., 1:] ^ seen[..., :-1]
        pair = np.where(both, np.minimum(sep, 1.0), np.where(one, ONE_SEEN_GAIN, 0.0))
        gain = pair.min(axis=2)                                               # (N, C)
        good = np.nonzero(gain.max(axis=1) >= 0.2)[0]
        for i in good:
            cands.append((j1, qs[i], gain[i]))
    if not cands:
        return []
    obstacles = column_obstacles(cols, base_T_map, bins)
    saved = list(G.OBSTACLES)
    G.set_obstacles(saved + obstacles)
    try:
        # each column wants two views' worth of gain (CREDIT_CAP): one view seldom shows a far stack whole and
        # well apart from its other levels, and a block seen whole once only scores 0.8 (doc 41 §3)
        chosen, have, here = [], np.zeros(len(cols)), list(current)

        def adds(g):
            return np.minimum(np.minimum(g, GAIN_GOAL), CREDIT_CAP - have).clip(0)

        for _ in range(max_views):
            if not (CREDIT_CAP - have > 1e-6).any():
                break
            ranked = sorted(cands, key=lambda c: -adds(c[2]).sum() + 0.002 * abs(c[0] - here[0]))
            pick = None
            for j1, q, g in ranked[:400]:
                if adds(g).sum() < 0.08:
                    break
                pose = [int(j1), int(q[1]), int(q[2]), int(q[3])]
                if any(pose == c["pose"] for c in chosen) or not path_ok(here, pose):
                    continue
                pick = (pose, g)
                break
            if pick is None:
                break
            chosen.append(dict(pose=pick[0], gain=[round(float(x), 2) for x in pick[1]]))
            have = have + adds(pick[1])
            here = pick[0]
        return chosen
    finally:
        G.set_obstacles(saved)


def path_ok(a, b) -> bool:
    """console_grasp.check_path, or the way round over HIGH_LEG as move_to takes it when a bin is in the way."""
    import console_grasp as G
    try:
        G.check_path(a, b)
        return True
    except G.PathBlocked:
        legs = [[a[0], *G.HIGH_LEG[1:]], [b[0], *G.HIGH_LEG[1:]], list(b)]
        here = a
        try:
            for leg in legs:
                G.check_path(here, leg)
                here = leg
            return True
        except G.ConsoleError:
            return False
    except G.ConsoleError:
        return False


def column_obstacles(cols: list[dict], base_T_map: np.ndarray, bins: list[dict] = ()) -> list:
    """console_grasp obstacle boxes (rot, centre, half): where each block could be, as a tower up to its top at that
    level (the place it would have on the table 3 cm high, the place it would have one block up 6 cm, ...), and
    the bins. Every place 9 cm high instead kept the near field views and the arm's own start out (2026-11-21 sim:
    the level-2 places of blocks in the bins lie 2-3 cm nearer the base, over the near strip)."""
    import console_grasp as G
    out = []
    in_bin = Solver([], [b for b in bins if "xy" in b]).floor_at
    for c in cols:
        for lv in LEVELS:
            p = base_T_map @ [*c["levels"][lv], 0.0, 1.0]
            h = BLOCK * (lv + 1) / 2
            if in_bin(c["levels"][0])[1] and lv:   # a block in a bin is taken as one on its floor (inside the bin's
                continue                            # own keep-out): its raised places, cast out of the bin over the
                                                    # near strip, kept every path off that side (2026-11-21 sim)
            out.append(([[1, 0], [0, 1]], [float(p[0]), float(p[1]), G.TABLE_Z + h], [0.021, 0.021, h]))
    for b in bins:
        c, s = math.cos(math.radians(b["yaw_deg"])), math.sin(math.radians(b["yaw_deg"]))
        out.append(([[c, -s], [s, c]], [b["base_xy"][0], b["base_xy"][1], G.TABLE_Z + b["height"] / 2],
                    [b["outer"] / 2, b["outer"] / 2, b["height"] / 2]))
    return out


def scan(run: Path | None = None, console: str | None = None, bins_design: str | None = None,
         field_poses=None, plan_only: bool = False) -> Path:
    """Field views, then height views planned from them; all in <run>/views.json (height views marked
    purpose "height"). Moves the arm."""
    import console_grasp as G
    import console_step as CS
    import field_scan as FS
    if console:                     # console_grasp and field_scan copy console_step.BASE at import
        CS.BASE = G.BASE = FS.BASE = console.rstrip("/")
    try:
        sim = CS.call("/api/bootstrap").get("status", {}).get("mode") == "simulation"
    except Exception:               # noqa: BLE001  -- no answer: treat as the real arm
        sim = False
    if sim:                         # the simulator's own logs, never the real arm's (as flow.stages.scan)
        G.RUNS, G.PLAN = ROOT / "data" / "sim", ROOT / "data" / "sim" / "plan.json"
    run = Path(run or ROOT / "data" / ("sim" if sim else "runs") / "stack_scan" / time.strftime("%Y%m%d_%H%M%S"))
    run.mkdir(parents=True, exist_ok=True)
    poses = [list(p) for p in (field_poses or FS.SCAN_POSES)]
    views = []
    rel = str(run.relative_to(ROOT)) if run.is_relative_to(ROOT) else str(run)
    with G.lease(None) as sess:
        j5 = G.ROLL_ZERO

        def take(pose, purpose, extra=None):
            try:
                G.move_to(sess, pose, j5)
            except G.ConsoleError as e:            # PathBlocked or a refused path: the other views still count
                G.log("stack_view_skipped", run=rel, pose=pose, purpose=purpose, reason=str(e))
                return
            jpeg, s = FS.frame_at_rest(board=False)
            name = f"view_{len(views):02d}.jpg"
            (run / name).write_bytes(jpeg)
            views.append(dict(image=name, readback=s["pose"], command=pose + [j5], purpose=purpose, t=time.time(),
                              **(extra or {})))
            (run / "views.json").write_text(json.dumps(views, indent=1, ensure_ascii=False))
            G.log("stack_view", run=rel, view=len(views) - 1, pose=s["pose"], purpose=purpose)

        def solve_columns():
            sc, vs = load(run)
            bs = bins_in_map(sc, bins_design, run)
            so = Solver(vs, bs)
            return sc, bs, columns(so, so.hypotheses())

        # the field views as the field scan takes them (the low ones put the gripper ~5 cm over the map's near strip:
        # a stack standing there would be in their way, as in the field scan itself -- doc 41 §6). Holding them
        # back until the high views' block places were obstacles skipped 2-3 of the 3 low views in every one of the
        # 7 sim cases, lost blocks near the base and once left no way to any height view (2026-11-21).
        saved = list(G.OBSTACLES)
        try:
            for p in poses:
                take(p, "field")
            scan_, bins, cols = solve_columns()
            here = G.status()["pose"]
            plan = plan_views(cols, scan_.base_T_map, here, bins)
            (run / "height_plan.json").write_text(json.dumps(dict(
                columns=[{str(k): [round(float(x) * 100, 2) for x in v] for k, v in c["levels"].items()} for c in cols],
                views=plan), indent=1))
            if not plan_only:
                G.set_obstacles(saved + column_obstacles(cols, scan_.base_T_map, bins))
                for p in plan:
                    take(p["pose"], "height", dict(gain=p["gain"]))
        finally:
            G.set_obstacles(saved)
    return run


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("scan", help="field + height views (moves the arm), then measure")
    s.add_argument("--console")
    s.add_argument("--bins")
    s.add_argument("--out", type=Path)
    s.add_argument("--located", type=Path)
    m = sub.add_parser("measure", help="offline: a run's views -> stack_scene.json (+ located.json)")
    m.add_argument("run", type=Path)
    m.add_argument("--bins")
    m.add_argument("--located", type=Path)
    m.add_argument("--no-roi", action="store_true")
    a = ap.parse_args(argv)
    run = scan(a.out, a.console, a.bins) if a.cmd == "scan" else a.run
    res = measure(run, a.bins, roi=not getattr(a, "no_roi", False))
    for b in res["blocks"]:
        print(f"{b['name']:4s} level {b['level']} bottom {b['bottom_cm']:4.1f} cm  map ({b['map_cm'][0]:6.2f}, "
              f"{b['map_cm'][1]:6.2f})  yaw {b['yaw_deg']:6.1f}  {b['category'] or '?':10s} views {b['views']} "
              f"score {b['score']:.2f} alt {b['alt']} support {b['support']}{' inferred' if b['inferred'] else ''}"
              f"{' in ' + b['in_bin'] if b['in_bin'] else ''}")
    if a.located:
        loc = located(res, run, bins_design=a.bins)
        a.located.write_text(json.dumps(loc, indent=1, ensure_ascii=False, default=_plain))
        print("wrote", a.located)
    return 0


if __name__ == "__main__":
    sys.exit(main())
