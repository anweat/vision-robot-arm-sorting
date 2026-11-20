"""Scene errors, clearances, trims and search settings (config/planning_tuning.json), separate from the model.

    t = tuning.load()                                   # the file (or $ARM_PLANNING_TUNING)
    t = tuning.load(overrides={"errors": {"target_xy_m": 0.006}, "trims": {"j5_trim_deg": -2}})
    t.neighbour()  t.target_open(0.03)  t.table_z ...   # the clearances the checks use

Errors widen clearances: `combine` "sum" adds every error that applies (worst case), "rss" adds their
root-sum-square. A collision map's own per-object sigma_xy_m replaces obstacle_xy_m when larger.
"""

from __future__ import annotations

import copy
import json
import math
import os
from dataclasses import dataclass, field
from pathlib import Path

from . import config as C

PATH = C.CONFIG_DIR / "planning_tuning.json"


def _merge(base: dict, over: dict) -> dict:
    out = copy.deepcopy(base)
    for k, v in (over or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = _merge(out[k], v)
        else:
            out[k] = copy.deepcopy(v)
    return out


def _clean(d: dict) -> dict:
    return {k: (_clean(v) if isinstance(v, dict) else v) for k, v in d.items() if not k.startswith("_")}


@dataclass
class Tuning:
    raw: dict
    source: str = ""
    obstacle_sigma_m: float = 0.0          # set from a collision map's objects
    notes: list = field(default_factory=list)

    @property
    def errors(self) -> dict:
        return self.raw["errors"]

    @property
    def clear(self) -> dict:
        return self.raw["clearances"]

    @property
    def trims(self) -> dict:
        return self.raw["trims"]

    @property
    def search(self) -> dict:
        return self.raw["search"]

    @property
    def map(self) -> dict:
        return self.raw["map"]

    # ------------------------------------------------------------------------------- derived values
    @property
    def table_z(self) -> float:
        return C.TABLE_Z + float(self.trims.get("table_z_m", 0.0))

    def combine(self, base: float, *errs: float) -> float:
        errs = [abs(float(e)) for e in errs if e]
        if self.raw.get("combine", "sum") == "rss":
            return base + math.sqrt(sum(e * e for e in errs))
        return base + sum(errs)

    def obstacle_xy(self) -> float:
        return max(float(self.errors["obstacle_xy_m"]), self.obstacle_sigma_m)

    def neighbour(self) -> float:
        """Gripper and held block to anything in the map (horizontal ball radius)."""
        return self.combine(self.clear["neighbour_m"], self.obstacle_xy(), self.errors["arm_xy_m"])

    def obstacle_dz(self) -> float:
        """Extra height on everything in the map (its height error and the arm's z error)."""
        return self.combine(0.0, self.errors["obstacle_height_m"], self.errors["arm_z_m"])

    def arm_body(self) -> float:
        return self.combine(self.clear["arm_body_m"], self.obstacle_xy(), self.errors["arm_xy_m"])

    def target_open(self, width: float) -> float:
        """Open fingers to the target while they arrive and open: its position and yaw errors (a yawed block
        reaches (w/2)(cos e + sin e - 1) further out at its corners) and the arm's."""
        e = math.radians(float(self.errors["target_yaw_deg"]))
        corner = width / 2 * (math.cos(e) + math.sin(e) - 1.0)
        return self.combine(self.clear["target_open_m"], self.errors["target_xy_m"], self.errors["arm_xy_m"], corner)

    def target_touch(self) -> float:
        return float(self.clear["target_touch_m"])

    def target_touch_inset(self) -> float:
        """How far the closing sweep's target box is shrunk on each side (its core): the pads may meet the
        block's sides, not come down on its top."""
        return float(self.clear.get("target_touch_inset_m", 0.0))

    def table(self, base: float) -> float:
        """A table clearance plus the table and arm height errors."""
        return self.combine(base, self.errors["table_z_m"], self.errors["arm_z_m"])

    def with_map(self, cmap) -> "Tuning":
        t = Tuning(self.raw, self.source, cmap.object_sigma(0.0) if cmap is not None else 0.0, list(self.notes))
        return t

    def to_dict(self) -> dict:
        return dict(source=self.source, values=_clean(self.raw),
                    derived=dict(table_z_cm=round(self.table_z * 100, 3), neighbour_mm=round(self.neighbour() * 1000, 2),
                                 obstacle_dz_mm=round(self.obstacle_dz() * 1000, 2),
                                 arm_body_mm=round(self.arm_body() * 1000, 2),
                                 target_open_mm=round(self.target_open(C.BLOCK) * 1000, 2)))


def load(path: str | Path | None = None, overrides: dict | None = None) -> Tuning:
    p = Path(path or os.environ.get("ARM_PLANNING_TUNING") or PATH)
    raw = json.loads(p.read_text(encoding="utf-8"))
    if overrides:
        raw = _merge(raw, overrides)
    return Tuning(raw, str(p) + (" + overrides" if overrides else ""))


DEFAULT = None


def default() -> Tuning:
    global DEFAULT
    if DEFAULT is None:
        DEFAULT = load()
    return DEFAULT
