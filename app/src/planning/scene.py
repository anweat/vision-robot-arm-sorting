"""Things on the table, as oriented boxes in the arm base frame (metres)."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field

from . import config as C


@dataclass
class Box:
    """A box standing upright: centre, half sizes, yaw about z (deg)."""
    kind: str
    name: str
    centre: tuple
    half: tuple
    yaw_deg: float = 0.0
    radius: float = field(init=False, repr=False)

    def __post_init__(self):
        self.centre, self.half = tuple(float(v) for v in self.centre), tuple(float(v) for v in self.half)
        self.radius = math.sqrt(sum(h * h for h in self.half))
        self._c, self._s = math.cos(math.radians(self.yaw_deg)), math.sin(math.radians(self.yaw_deg))

    @property
    def top(self) -> float:
        return self.centre[2] + self.half[2]

    def depth(self, p, margin: float = 0.0) -> float:
        """How far p is inside the box grown by `margin` (> 0 inside)."""
        dx, dy, dz = p[0] - self.centre[0], p[1] - self.centre[1], p[2] - self.centre[2]
        lx, ly = self._c * dx + self._s * dy, -self._s * dx + self._c * dy
        return min(self.half[0] + margin - abs(lx), self.half[1] + margin - abs(ly), self.half[2] + margin - abs(dz))

    def near(self, p, reach: float) -> bool:
        return abs(p[0] - self.centre[0]) <= self.radius + reach and abs(p[1] - self.centre[1]) <= self.radius + reach

    def to_dict(self) -> dict:
        return dict(kind=self.kind, name=self.name, centre_cm=[round(v * 100, 2) for v in self.centre],
                    half_cm=[round(v * 100, 2) for v in self.half], yaw_deg=round(self.yaw_deg, 1))


def block(bx: float, by: float, yaw_deg: float = 0.0, name: str = "block", side: float = C.BLOCK) -> Box:
    return Box("block", name, (bx, by, C.TABLE_Z + side / 2), (side / 2,) * 3, yaw_deg or 0.0)


def box_on_table(bx: float, by: float, sx: float, sy: float, height: float, yaw_deg: float = 0.0,
                 name: str = "box", kind: str = "box") -> Box:
    return Box(kind, name, (bx, by, C.TABLE_Z + height / 2), (sx / 2, sy / 2, height / 2), yaw_deg)


def map_to_base(mx: float, my: float) -> tuple[float, float]:
    dx, dy = C.map_placement()
    return mx + dx, my + dy


def block_on_map(mx_cm: float, my_cm: float, yaw_deg: float = 0.0, name: str = "block") -> Box:
    bx, by = map_to_base(mx_cm / 100, my_cm / 100)
    return block(bx, by, yaw_deg, name)


def bin_boxes(centre_xy, yaw_deg: float = 0.0, name: str = "bin", design: str = C.BINS_NAME) -> list[Box]:
    """An open acrylic bin: four walls and its floor plate, so a block can be let go inside it."""
    g = json.loads((C.CONFIG_DIR / f"{design}.json").read_text(encoding="utf-8"))["geometry_mm"]
    t, outer, h = g["thickness"] / 1000, g["outer"] / 1000, g["rim_top_above_table"] / 1000
    c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    out = []
    for i, (lx, ly, hx, hy) in enumerate(((outer / 2 - t / 2, 0, t / 2, outer / 2), (-outer / 2 + t / 2, 0, t / 2, outer / 2),
                                          (0, outer / 2 - t / 2, outer / 2, t / 2), (0, -outer / 2 + t / 2, outer / 2, t / 2))):
        x, y = centre_xy[0] + c * lx - s * ly, centre_xy[1] + s * lx + c * ly
        out.append(Box("bin", f"{name}/wall{i}", (x, y, C.TABLE_Z + h / 2), (hx, hy, h / 2), yaw_deg))
    plate = g["plate"]
    py0, py1 = (v / 1000 for v in g["plate_y_range"])
    pcy = (py0 + py1) / 2
    out.append(Box("bin", f"{name}/plate", (centre_xy[0] - s * pcy, centre_xy[1] + c * pcy, C.TABLE_Z + t / 2),
                   (plate[0] / 2000, (py1 - py0) / 2, t / 2), yaw_deg))
    return out


def planned_bins(design: str = C.BINS_NAME) -> dict[str, list[Box]]:
    """The bins of a design at their planned places (config/<design>.json planned_centre_base_m)."""
    d = json.loads((C.CONFIG_DIR / f"{design}.json").read_text(encoding="utf-8"))
    out = {}
    for n, b in d["bins"].items():
        yaw = b.get("planned_flange_direction_deg", 90.0) - 90.0
        out[n] = bin_boxes(b["planned_centre_base_m"], yaw, f"bin{n}", design)
    return out


def bin_rim_z(design: str = C.BINS_NAME) -> float:
    g = json.loads((C.CONFIG_DIR / f"{design}.json").read_text(encoding="utf-8"))["geometry_mm"]
    return C.TABLE_Z + g["rim_top_above_table"] / 1000
