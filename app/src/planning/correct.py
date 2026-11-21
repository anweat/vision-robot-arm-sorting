"""Readback correction of a set-down: where the held block really is, from the servo readback, before letting go.

A plan puts the held block's centre on the destination through the commanded joints (release._one solves for
the block's centre, not the pad middle: with the jaw pitched or rolled the two are apart). The servos stop a
little off the command (readback 1-3 deg after the settle, docs/current/32), and the arm is long: 1 deg of J2
is ~4 mm at a block 25 cm out. So at the set-down pose, after the settle, the readback and the grasp's model of
the block in the fingers (gripper.Held: centre and axes in the link5 frame) give where the block is; when it
is off the planned centre, a small move (J1-J4, least squares, J5 kept) takes it there -- sideways onto the
block below, and in height to the release gap (a descent that stopped short leaves it higher, and it falls).

Measured where it matters: the error before the descent is what the move *to* that pose left, not what the
descent will (2026-11-21 simulator, stop_short_deg 1.5: correcting the descent from it put a stacked block
9.7 mm off and pressed it onto the one below).

What it cannot see: anything the readback does not show -- link sag, backlash, the hand-eye (a close look sees
those, doc 39 §5) -- and a block that sits in the fingers other than the model says (a grip squared at an angle
leaves it off the pad middle; the J6 readback only tells its width). It corrects the servo error, nothing more.
"""

from __future__ import annotations

import math
from typing import Sequence

import numpy as np

from . import gripper as GR

MAX_ERROR_M = 0.012        # a larger offset is not a servo stopping short: leave the plan alone and say so
MAX_STEP_DEG = 4.0         # the corrected set-down may move each of J1-J4 at most this far
MIN_ERROR_M = 0.0015       # below this nothing is changed (a stack's top is 30 mm; the sim lands within 2-4 mm)
TRIES = 2                  # nudges at one set-down pose
SETTLE_DEG = 0.4           # each nudge is settled (flow.stages._settle) to this, past its goal by the remaining
SETTLE_TRIES = 3           #   error as every move is: a servo that stops short does not move for a fraction of a degree


def held_from_summary(d: dict) -> GR.Held:
    """gripper.Held from a pick plan's summary.held_block."""
    return GR.Held([v / 1000 for v in d["centre_mm"]], d["rot"], d.get("side_mm", 30.0) / 1000,
                   float(d.get("tilt_deg", 0.0)))


def centre(q5: Sequence[float], held: GR.Held) -> np.ndarray:
    return np.array(GR.held_base([float(v) for v in q5[:5]], held)[0])


def reach(target: Sequence[float], rb: Sequence[float], held: GR.Held) -> dict:
    """{command, error_mm, moved_deg, applied, reason}: the joints (from the readback `rb`, J1-J4, J5 kept)
    that put the held block's centre at `target` (base frame); `command` is rb + moved_deg."""
    target = np.asarray(target, float)
    err = centre(rb, held) - target
    out = dict(error_mm=[round(float(v) * 1000, 2) for v in err], command=list(rb[:5]), moved_deg=None,
               applied=False, reason=None)
    n = float(np.linalg.norm(err))
    if n < MIN_ERROR_M:
        out["reason"] = "within the servo tolerance"
        return out
    if n > MAX_ERROR_M:
        out["reason"] = f"{n * 1000:.1f} mm is more than a servo stopping short (> {MAX_ERROR_M * 1000:.0f}): not corrected"
        return out
    q = np.array(rb[:5], float)
    for _ in range(4):
        c = centre(q, held)
        r = target - c
        if np.linalg.norm(r) < 2e-4:
            break
        jac = np.zeros((3, 4))
        for j in range(4):
            dq = q.copy()
            dq[j] += 0.5
            jac[:, j] = (centre(dq, held) - c) / 0.5
        q[:4] += np.linalg.lstsq(jac, r, rcond=None)[0]
    moved = q[:4] - np.array(rb[:4], float)
    out["moved_deg"] = [round(float(v), 2) for v in moved]
    if np.abs(moved).max() > MAX_STEP_DEG:
        out["reason"] = f"needs {np.abs(moved).max():.1f} deg on one joint (> {MAX_STEP_DEG:.0f}): not corrected"
        return out
    out.update(command=[round(float(v), 2) for v in q], applied=True)
    return out


def nudge(cmd: Sequence[float], rb: Sequence[float], held: GR.Held) -> dict:
    """{command, error_mm, moved_deg, applied, reason}: a new command for the pose the arm is at (commanded
    `cmd`, read back `rb`) that puts the held block where `cmd` meant it to be. The offset between command and
    readback is taken to stay for a move this small."""
    return set_down(cmd, rb, cmd, held)


def set_down(above_cmd: Sequence[float], above_rb: Sequence[float], lower_cmd: Sequence[float],
             held: GR.Held) -> dict:
    """As nudge, for a pose `lower_cmd` other than the one read back (kept for comparisons; see the module
    note on why the descent is not corrected from the pose above)."""
    err = centre(above_rb, held) - centre(above_cmd, held)
    out = dict(error_mm=[round(float(v) * 1000, 2) for v in err], command=list(lower_cmd[:5]), moved_deg=None,
               applied=False, reason=None)
    n = float(np.linalg.norm(err))
    if n < MIN_ERROR_M:
        out["reason"] = "within the servo tolerance"
        return out
    if n > MAX_ERROR_M:
        out["reason"] = f"{n * 1000:.1f} mm is more than a servo stopping short (> {MAX_ERROR_M * 1000:.0f}): not corrected"
        return out
    goal = centre(lower_cmd, held) - err
    q = np.array(lower_cmd[:5], float)
    for _ in range(4):                     # Gauss-Newton on J1-J4, numerical Jacobian
        c = centre(q, held)
        r = goal - c
        if np.linalg.norm(r) < 2e-4:
            break
        jac = np.zeros((3, 4))
        for j in range(4):
            dq = q.copy()
            dq[j] += 0.5
            jac[:, j] = (centre(dq, held) - c) / 0.5
        step = np.linalg.lstsq(jac, r, rcond=None)[0]
        q[:4] += step
    moved = q[:4] - np.array(lower_cmd[:4], float)
    out["moved_deg"] = [round(float(v), 2) for v in moved]
    if np.abs(moved).max() > MAX_STEP_DEG:
        out["reason"] = f"needs {np.abs(moved).max():.1f} deg on one joint (> {MAX_STEP_DEG:.0f}): not corrected"
        return out
    residual = float(np.linalg.norm(goal - centre(q, held)))
    if residual > 5e-4:
        out["reason"] = f"no joint change takes it there ({residual * 1000:.1f} mm left): not corrected"
        return out
    out.update(command=[round(float(v), 2) for v in q], applied=True)
    return out


def lower_ok(world, above_rb: Sequence[float], lower: Sequence[float], held: GR.Held) -> "object":
    """A corrective move with the block held, checked as the planner checks a set-down (planner._plan_place)."""
    from . import checks as K
    from . import route as RT
    c_delta = GR.contact(held.side)
    # the arm is already at the start (perhaps pressing the block onto the one below, or a finger on a bin
    # wall: that is what is being corrected): it is not checked, every sample after it is
    n = K.samples_for(above_rb[:5], lower[:5], world.tuning)
    out = None
    for q in K.interpolate(list(above_rb[:5]), list(lower[:5]), max(n, 2))[1:]:
        c = K.pose_check(world, q, c_delta, held,
                         table_margin=RT.table_allowance(world, above_rb[:5], lower[:5], c_delta, held))
        out = c if out is None else out.merge(c)
    return out


def describe(r: dict) -> str:
    e = r["error_mm"]
    s = f"block {math.hypot(e[0], e[1]):.1f} mm off sideways, {e[2]:+.1f} mm in height at the readback"
    return s + (f"; nudged {r['moved_deg']} deg" if r["applied"] else f"; {r['reason']}")
