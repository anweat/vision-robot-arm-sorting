"""The gripper's states (link-angle changes of the four-bar) and its collision samples in the base frame."""

from __future__ import annotations

import math

from dataclasses import dataclass
from functools import lru_cache

from . import config as C
import gripper_geometry as gg   # noqa: E402  (src/spatial, put on the path by config)
import gripper_model            # noqa: E402

CLOSED = gg.delta_range()[0]
WIDEST = gg.delta_range()[1]


@dataclass(frozen=True)
class Opening:
    action: str       # console gripper action that commands it ("open"; "narrow"/"tight" do not exist yet)
    j6: float
    delta: float
    gap_m: float


OPENINGS = tuple(Opening(a, float(j6), gg.delta_for_j6(j6), gg.gap_for_delta(gg.delta_for_j6(j6)))
                 for a, j6 in C.G["openings"])


RELEASE_J6_STEP = 10            # release openings between the console's: every this many degrees of J6


@lru_cache(maxsize=16)
def release_openings(width: float = C.BLOCK) -> tuple:
    """Openings to let go of a `width` block, widest first: the console's (OPENINGS) and, in between, every
    RELEASE_J6_STEP of J6 (action "j6": reached by J6 jogs, flow.stages.Console.gripper), down to the
    narrowest that still leaves release.min_extra_gap_m between the pads and the block. Inside a bin or next
    to a stack the widest may meet a wall; a narrower one is the dynamic jaw width."""
    min_gap = width + float(C.R.get("min_extra_gap_m", 0.008))
    out = list(OPENINGS)
    for j6 in range(int(OPENINGS[0].j6) + RELEASE_J6_STEP, 180, RELEASE_J6_STEP):
        if any(abs(o.j6 - j6) < RELEASE_J6_STEP / 2 for o in out):
            continue
        d = gg.delta_for_j6(j6)
        out.append(Opening("j6", float(j6), d, gg.gap_for_delta(d)))
    return tuple(sorted((o for o in out if o.gap_m >= min_gap), key=lambda o: -o.gap_m))


@lru_cache(maxsize=64)
def contact(width: float = C.BLOCK) -> float:
    return gg.contact_delta(width)


@lru_cache(maxsize=512)
def delta_for_j6(j6: float) -> float:
    return gg.delta_for_j6(j6)


def link5(q5) -> list:
    return gg.link5_pose(q5, C.ROLL_ZERO, C.ROLL_SIGN)


_CACHE: dict = {}


def _link_points(delta: float, parts: tuple) -> list:
    key = (round(delta, 2), parts)
    if key not in _CACHE:
        _CACHE[key] = gg.points_link(delta, parts)
    return _CACHE[key]


@dataclass
class Held:
    """A block in the closed fingers: centre and axes (columns of rot) in the link5 frame. Its x axis is the
    jaw axis (the pads square it), z is the table's up direction at the moment it was gripped."""
    centre: list
    rot: list
    side: float = C.BLOCK
    tilt_deg: float = 0.0          # how far the pads turned it out of level when they squared it at the grasp

    def points_link(self) -> list:
        h = self.side * 0.48
        out = []
        for i in (-1, 0, 1):
            for j in (-1, 0, 1):
                for k in (-1, 0, 1):
                    out.append([self.centre[r] + h * (i * self.rot[r][0] + j * self.rot[r][1] + k * self.rot[r][2])
                                for r in range(3)])
        return out

    def to_dict(self) -> dict:
        return dict(centre_mm=[round(v * 1000, 2) for v in self.centre],
                    rot=[[round(v, 4) for v in row] for row in self.rot], side_mm=self.side * 1000,
                    tilt_deg=round(self.tilt_deg, 1))


def held_from_grasp(q5, width: float = C.BLOCK, block_centre=None) -> Held:
    """The block as the fingers hold it after closing at arm pose q5 (block flat on the table before).

    `block_centre` (base frame): where the block's centre was. The pads close on it where they stand -- 18 mm
    over its bottom (trims.pad_centre_height_m), so 3 mm above its centre, and wherever the grasp offset or
    the arm's error put them across the jaw -- and push it to their middle only along the jaw axis. So its
    centre in the fingers is the pad middle plus that offset with the jaw-axis part removed; without
    `block_centre` it is taken at the pad middle (before 2026-11-21: off by those 3 mm, turned with the
    pitch at the release)."""
    pose = link5(q5)
    up = [pose[2][0], pose[2][1], pose[2][2]]           # world z in link5 coordinates (row 2 of R)
    x = [1.0, 0.0, 0.0]
    z = [up[0] - up[0], up[1], up[2]]                   # up with its jaw-axis part removed
    n = (z[1] ** 2 + z[2] ** 2) ** 0.5
    z = [0.0, z[1] / n, z[2] / n]
    y = [z[1] * x[2] - z[2] * x[1], z[2] * x[0] - z[0] * x[2], z[0] * x[1] - z[1] * x[0]]
    rot = [[x[r], y[r], z[r]] for r in range(3)]
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, sum(up[k] * z[k] for k in range(3))))))
    centre = list(gg.pad_centre_link(contact(width)))
    if block_centre is not None:
        d = [block_centre[r] - gg.to_base(pose, centre)[r] for r in range(3)]       # base frame
        local = [sum(pose[r][k] * d[r] for r in range(3)) for k in range(3)]      # into link5 (R^T d)
        local[0] = 0.0                                                            # squared along the jaw
        centre = [centre[k] + local[k] for k in range(3)]
    return Held(centre, rot, width, tilt)


def held_default(width: float = C.BLOCK) -> Held:
    """Axes along link5: for a block of unknown orientation in the fingers."""
    return Held(gg.pad_centre_link(contact(width)), [[1, 0, 0], [0, 1, 0], [0, 0, 1]], width)


def held_base(q5, held: Held) -> tuple[list, list]:
    """(centre, rotation) of the held block in the base frame at arm pose q5."""
    pose = link5(q5)
    centre = gg.to_base(pose, held.centre)
    rot = [[sum(pose[r][k] * held.rot[k][c] for k in range(3)) for c in range(3)] for r in range(3)]
    return centre, rot


def points(q5, delta: float, held: "Held | None" = None, parts: tuple = ("fingers", "links", "static")) -> list:
    pose = link5(q5)
    pts = [gg.to_base(pose, p) for p in _link_points(delta, parts)]
    if held is not None:
        pts += [gg.to_base(pose, p) for p in held.points_link()]
    return pts


def lowest(q5, delta: float, held: "Held | None" = None) -> float:
    return min(p[2] for p in points(q5, delta, held, ("fingers",)))


def pad_centre(q5, delta: float) -> list:
    return gg.to_base(link5(q5), gg.pad_centre_link(delta))


def jaw_axis(q5) -> list:
    p = link5(q5)
    return [p[0][0], p[1][0], p[2][0]]


def j6_for_width(width: float) -> float:
    """J6 at which the free jaw would stand at `width` (official table): the grip readback to expect."""
    return gripper_model.servo6_for_gap(width)
