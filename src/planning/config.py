"""Parameters for src/planning: config/motion_planning.json, plus read-only facts from other configs."""

from __future__ import annotations

import json
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src" / "spatial"))

CONFIG_DIR = ROOT / "config"
P = json.loads((CONFIG_DIR / "motion_planning.json").read_text(encoding="utf-8"))
ARM = json.loads((CONFIG_DIR / "arm_limits.json").read_text(encoding="utf-8"))

TABLE_Z = float(P["table_z_model_m"])
BLOCK = float(P["block_side_m"])
G = P["grasp"]
R = P["release"]
M = P["margins"]
ROLL_ZERO = float(P["roll"]["zero_deg"])
ROLL_SIGN = float(P["roll"]["sign"])
LIMITS = {int(k): tuple(v) for k, v in P["joint_limits_deg"].items() if not k.startswith("_")}
PATH = P["path"]
MAP_NAME = os.environ.get("ARM_MAP", P["map"])
BINS_NAME = P["bins"]


def map_placement(name: str = MAP_NAME) -> tuple[float, float]:
    """(dx, dy): base = map + (dx, dy), from the map's placement."""
    d = json.loads((CONFIG_DIR / f"{name}.json").read_text(encoding="utf-8"))["placement"]["base_from_map"]
    return float(d["dx"]), float(d["dy"])
