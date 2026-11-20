"""Replay the real arm's grasps against the simulator's grasp rule (doc 34 §3): does the four-bar rule say
"held" where the real gripper held, and "closed on nothing" where it closed on nothing?

    uv run --with numpy --with 'opencv-python<5' python dev/tools/grasp_replay.py [data/runs/console_grasp/log.jsonl]

For every `gripped` event of the real log: the block is put where the run believed it was (console_grasp: the
`planned` block_base_cm; pipeline: the `cube` map position of the pipeline_sorted/missed that follows, through
that run's map registration), turned to the `hovering` event's block_yaw_deg when the log has it (else square
to the jaw, which the console aligned with J5), and the simulated arm goes hover -> grip pose with the gripper
open, then closes. Real outcome from the readback: <= 131 held, >= 132 closed on nothing; a readback near the
open command means the gripper never closed (the sleeping-servo problem, doc 29 §6) and is left out.

The block position is the run's estimate, not the truth (map registration ~1 cm), and the commanded pose is
replayed (the real J2 sat up to ~2 deg off, doc 30 §10.3), so this checks the rule's margins, not its exactness.
The old rule (kinematic tip within 2 cm of the centre, +-2 cm in height) is scored on the same inputs.
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "dev" / "sim"))
import world as W  # noqa: E402

LOG = ROOT / "data" / "runs" / "console_grasp" / "log.jsonl"


def cases(events: list[dict]) -> list[dict]:
    out, origin, planned, hover = [], None, None, None
    for i, e in enumerate(events):
        k = e["event"]
        if k == "pipeline_start":
            origin, planned = e.get("registration", {}).get("map_origin_base_cm"), None
        elif k == "planned" and "block_base_cm" in e:
            planned = e
        elif k == "hovering":
            hover = e
        elif k == "gripped" and hover is not None:
            xy, source = None, None
            if planned is not None and planned["t"] > hover["t"] - 120:
                xy, source = [v / 100 for v in planned["block_base_cm"][:2]], "console_grasp"
            elif origin is not None:
                for f in events[i + 1:i + 8]:
                    if f["event"] in ("pipeline_sorted", "pipeline_missed") and "cube" in f:
                        xy = [(f["cube"][0] + origin[0]) / 100, (f["cube"][1] + origin[1]) / 100]
                        source = "pipeline"
                        break
            rb = (e.get("grip") or {}).get("readback")
            if xy is None or rb is None:
                continue
            outcome = "held" if 100 <= rb <= 131 else "empty" if rb >= 132 else "not_closed"
            out.append(dict(t=e["t"], source=source, xy=xy, yaw=hover.get("block_yaw_deg"), aligned=hover.get("aligned"),
                            hover=hover["pose"][:5], grip=e["pose"][:5], readback=rb, real=outcome))
            planned = None
    return out


def old_rule(world: W.World, q5, c) -> bool:
    tip = np.array(W.kin.forward(list(q5) + [133])["tip"])
    return float(np.linalg.norm(c[:2] - tip[:2])) <= 0.02 and c[2] - 0.02 <= tip[2] <= c[2] + 0.02


def replay(case: dict) -> dict:
    w = W.World(0, map_name=None)
    arm = W.WorldArm(w)
    if case["yaw"] is None:            # the console squared J5 to the block: put the block square to the jaw
        jaw = W.BODY.link5(case["grip"])[:3, 0]
        yaw = math.degrees(math.atan2(jaw[1], jaw[0]))
    else:
        yaw = case["yaw"]
    m = w.table.base_to_map([case["xy"][0], case["xy"][1], 0])
    bid = w.place("other", [m[0] * 100, m[1] * 100], yaw)
    c = w.blocks[bid]["pose"][:3, 3].copy()
    arm.write6(case["hover"] + [30], 0)
    arm.write6(case["grip"] + [30], 0)
    arm.write6(case["grip"] + [133], 0)
    return dict(new="held" if w.held == bid else "empty", old="held" if old_rule(w, case["grip"], c) else "empty")


def main() -> int:
    log = Path(sys.argv[1]) if len(sys.argv) > 1 else LOG
    events = [json.loads(line) for line in log.read_text().splitlines() if line.strip()]
    rows = [dict(c, **replay(c)) for c in cases(events)]
    used = [r for r in rows if r["real"] != "not_closed"]
    print(f"{len(rows)} grips with a block position; {len(rows) - len(used)} where the gripper never closed (left out)")
    for rule in ("new", "old"):
        agree = sum(r[rule] == r["real"] for r in used)
        held = [r for r in used if r["real"] == "held"]
        empty = [r for r in used if r["real"] == "empty"]
        print(f"{rule} rule: agrees {agree}/{len(used)}; real held -> {sum(r[rule] == 'held' for r in held)}/{len(held)} held; "
              f"real empty -> {sum(r[rule] == 'empty' for r in empty)}/{len(empty)} empty")
    for r in used:
        if r["new"] != r["real"]:
            print("  differs:", {k: r[k] for k in ("source", "xy", "yaw", "aligned", "grip", "readback", "real", "new", "old")})
    return 0


if __name__ == "__main__":
    sys.exit(main())
