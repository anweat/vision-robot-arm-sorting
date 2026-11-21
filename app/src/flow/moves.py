"""How a plan's steps are sent to the arm: one at a time, or joined into continuous runs (docs/current/42 §4).

`steps` (the way since 2026-11-16): every step is its own console move; the arm stops after each, the flow reads
back, corrects towards the command (stages._settle) and checks it before the next.

`continuous`: the steps between the points where something has to be judged go to the console as one trajectory
(/api/trajectory): the console checks the whole chain first, then sends it segment after segment without the
settle wait, stopping only at the run's stops. The path is the same joint-space line through the same commands;
what goes is the stop, the readback round trip and the correction at every intermediate pose. A run

  * holds moves and gripper openings (open / wide / any J6 opening: a release is a J6 target, not jogs);
  * never holds a `grip`: the console judges the grip (holding / empty) and the check step needs it, so a grip is
    sent alone, as in `steps`;
  * ends with a place's `lower` when the readback correction is on (planning/correct.py works at the set-down);
  * stops (settles, normal tolerance) at every gripper step -- the fingers open fully before the arm goes on --
    and at its last step; the flow then settles and checks the last step as in `steps`.

Scans keep their stops: a picture needs the arm still (pipeline/observe.py, adapters/capture.py).
"""
from __future__ import annotations

from typing import Sequence

MODES = ("steps", "continuous")
PASS_TOLERANCE_DEG = 8.0         # the flow's check of a passed-through step (the console's motion.pass_tolerance_deg)


def joinable(step: dict) -> bool:
    return step["kind"] == "move" or (step["kind"] == "gripper" and step.get("action") != "grip")


def ends_run(step: dict, correct: bool) -> bool:
    return correct and step["plan"] == "place" and step["name"] == "lower"


def runs(steps: Sequence[dict], chosen: Sequence[int], mode: str = "steps", correct: bool = True) -> list[list[int]]:
    """The chosen step indices grouped into what is sent together; in `steps` mode one step per group."""
    if mode not in MODES:
        raise ValueError(f"motion mode {mode!r}: one of {', '.join(MODES)}")
    if mode == "steps":
        return [[i] for i in chosen]
    out, cur = [], []
    for i in chosen:
        s = steps[i]
        if not joinable(s):
            if cur:
                out.append(cur)
                cur = []
            out.append([i])
            continue
        cur.append(i)
        if ends_run(s, correct):
            out.append(cur)
            cur = []
    if cur:
        out.append(cur)
    return out


def waypoint(step: dict) -> list:
    """The trajectory waypoint of a step: a move keeps J6 at its last command (null), a gripper step moves J6 only."""
    c = [float(v) for v in step["command"]]
    if step["kind"] == "move":
        return [round(v) for v in c[:5]] + [None]
    return [None] * 5 + [round(c[5])]


def stops(steps: Sequence[dict], run: Sequence[int]) -> list[int]:
    """Positions within the run (0-based) where the arm stops: every gripper step, and the last."""
    out = [n for n, i in enumerate(run) if steps[i]["kind"] == "gripper"]
    return sorted(set(out) | {len(run) - 1})
