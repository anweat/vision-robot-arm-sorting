"""plan_transit / plan_pick / plan_place: MotionPlans on the four-bar gripper model.

Grasp. The TCP is the middle of the pads *closed on the block*: the four-bar pulls the pads 26 mm back
along the gripper axis when it opens, so the descent target comes from the closed state and the open
fingers arrive higher and wider. A candidate (pitch, J5, opening width, pad height) must
  * reach the pad target inside the joint limits, with the fingertips above the table at contact and if
    they close on nothing;
  * stay out of the platform (fitted keep-out and the lid mesh);
  * have the opening fingers clear the target block and everything else at the grasp pose, on the way down
    from the hover and while closing (a finger that lands on the block flicks it; 2026-09-27);
  * square the jaw to the block faces (both face pairs, any half turn) without tilting it out of the table
    plane by more than max_jaw_tilt_deg.
The best is aligned, flat, widest open, roomiest, nearest the preferred pitch, least rolled.

Release. The held block's pose in the fingers is kept from the grasp (`gripper.Held`); the release pose
puts its centre over the target with its faces at the requested yaw and its bottom level, and the
fingers must be able to open there (pads back and apart) without touching anything.

Transit. Straight (joint-linear) if clear; otherwise up over everything at the start's bearing, across,
and down at the goal's.
"""

from __future__ import annotations

import math
from typing import Sequence

from . import checks as K
from . import config as C
from . import gripper as GR
from . import scene as S
from . import solver as V
from .motion import Check, MotionPlan, Step

NEIGH, TARGET = C.M["neighbour_m"], C.M["target_m"]


def _pairs(boxes: Sequence[S.Box], margin: float = NEIGH, target: S.Box | None = None) -> list[tuple]:
    out = [(b, margin) for b in boxes]
    if target is not None:
        out.append((target, TARGET))
    return out


def _table_allowance(a5, b5, delta, held) -> float:
    """A move may start or end low (a grasp or release pose) but not dip below its ends on the way."""
    ends = min(GR.lowest(a5, delta, held), GR.lowest(b5, delta, held)) - C.TABLE_Z
    return min(C.M["path_table_m"], ends - 0.002)


# --------------------------------------------------------------------------------------------- transit

def route(a5: Sequence[float], b5: Sequence[float], delta: float, held=None,
          boxes: Sequence[tuple] = ()) -> tuple[list, list] | None:
    """(waypoints, checks) from a5 to b5, J1-J5, or None."""
    a5, b5 = list(a5)[:5], list(b5)[:5]
    margin = _table_allowance(a5, b5, delta, held)
    direct = K.move_check(a5, b5, delta, held, boxes, margin)
    if direct.ok:
        return [b5], [direct]
    top = max([C.TABLE_Z + C.BLOCK] + [b.top for b, _ in boxes])
    for above in C.PATH["via_above_obstacles_m"]:
        va, vb = _above(a5, top + above), _above(b5, top + above)
        if va is None or vb is None:
            continue
        legs, checks, here = [va, vb, b5], [], a5
        for leg in legs:
            c = K.move_check(here, leg, delta, held, boxes, _table_allowance(here, leg, delta, held))
            if not c.ok:
                break
            checks.append(c)
            here = leg
        else:
            return legs, checks
    return None


def _above(q5: list, z: float) -> list | None:
    """The FK tip straight above q5's at height z, at q5's own pitch if it reaches, else the flattest that
    does (the rise and the descent are then vertical; the pitch changes on the high leg)."""
    t = V.tip(q5)
    own = round(V.pitch_of(q5))
    for pitch in [own] + [p for p in C.PATH["via_pitches_deg"] if p != own]:
        q = V.kin.inverse(t[0], t[1], z, pitch)
        if q is None:
            continue
        v = [float(x) for x in q] + [q5[4]]
        if K.limits_ok(v):
            return v
    return None


def plan_transit(start5: Sequence[float], goal5: Sequence[float], j6: float, held: "GR.Held | None" = None,
                 boxes: Sequence[S.Box] = ()) -> MotionPlan:
    delta = GR.contact(held.side) if held is not None else GR.delta_for_j6(j6)
    end = K.pose_check(list(goal5)[:5], delta, held, _pairs(boxes))
    if not end.ok:
        return MotionPlan("transit", False, reasons=[f"the goal itself is not clear: {end.to_dict()}"])
    r = route(start5, goal5, delta, held, _pairs(boxes))
    if r is None:
        return MotionPlan("transit", False, reasons=["no clear way, straight or over the top"])
    legs, checks = r
    steps = [Step("move", "transit" if len(legs) == 1 else f"transit {i + 1}/{len(legs)}", leg, j6, held is not None, c)
             for i, (leg, c) in enumerate(zip(legs, checks))]
    return MotionPlan("transit", True, steps, dict(waypoints=len(legs)))


# ----------------------------------------------------------------------------------------------- grasp

def grasp_candidates(bx: float, by: float, yaw_deg: float | None, others: Sequence[S.Box] = (),
                     prefer: float | None = None, width: float = C.BLOCK, explain: list | None = None,
                     limit: int = 8) -> list[dict]:
    """Feasible grasps of the block centred at base (bx, by), best first."""
    prefer = C.G["prefer_pitch_deg"] if prefer is None else prefer
    why = explain if explain is not None else []
    reach = math.hypot(bx, by)
    if reach > C.G["max_reach_m"]:
        why.append(f"{reach * 100:.1f} cm from the base (> {C.G['max_reach_m'] * 100:.0f})")
        return []
    target = S.block(bx, by, yaw_deg or 0.0, "target", width)
    c_delta = GR.contact(width)
    out = []
    for pitch in sorted(C.G["pitches_deg"], key=lambda p: abs(p - prefer)):
        q0 = V.solve_tcp([bx, by, C.TABLE_Z + C.G["pad_centre_height_m"]], pitch, C.ROLL_ZERO, c_delta)
        if q0 is None:
            why.append(f"pitch {pitch}: out of reach")
            continue
        for roll in V.roll_candidates(q0, yaw_deg)[:4]:
            for opening in GR.OPENINGS:
                note: list = []
                cand = _grasp(bx, by, pitch, roll, opening, c_delta, target, others, yaw_deg, note)
                if cand is not None:
                    out.append(cand)
                    break
                why.append(f"pitch {pitch} J5 {roll['j5']} {opening.action}: {note[0] if note else '?'}")
    tol, tilt = C.G["aligned_tolerance_deg"], C.G["max_jaw_tilt_deg"]
    out.sort(key=lambda c: (not c["aligned"], c["tilt_deg"] > tilt, round(c["residual_deg"] or 0.0),
                            -c["open_gap_mm"], -min(c["room_cm"], 1.0), abs(c["pitch"] - prefer),
                            abs(c["j5"] - C.ROLL_ZERO)))
    return out[:limit]


def _grasp(bx, by, pitch, roll, opening, c_delta, target, others, yaw_deg, note) -> dict | None:
    j5 = roll["j5"]
    base_z = C.TABLE_Z + C.G["pad_centre_height_m"]
    others_p = _pairs(others)
    sweep_end = GR.gg.delta_for_gap(target.half[0] * 2 + 0.004)
    q = close = None
    # lowest pads first; a tilted jaw may need them a few mm higher: one finger hangs lower, and the
    # finger above the pads leans in over the block's top edge
    for lift in C.G["pad_lifts_m"]:
        pad = [bx, by, base_z + lift]
        if pad[2] > C.TABLE_Z + target.half[2] * 2 - C.G["pad_below_top_m"]:
            break
        q = V.solve_tcp(pad, pitch, j5, c_delta)
        if q is None:
            note.append("no IK")
            continue
        g5 = q + [j5]
        if not K.limits_ok(g5):
            note.append(f"grasp pose {[round(v) for v in g5]} outside the joint limits")
            q = None
            continue
        if (GR.lowest(g5, c_delta) < C.TABLE_Z + C.G["fingertip_table_margin_m"]
                or GR.lowest(g5, GR.CLOSED) < C.TABLE_Z + C.G["closed_fingertip_table_margin_m"]):
            note.append("fingertips too near the table")
            q = None
            continue
        # open at the grasp pose, then closing until 2 mm a side: target with the touch margin, rest with NEIGH
        close = K.sweep_check(g5, opening.delta, sweep_end, others_p + [(target, TARGET)], steps=4)
        if close.ok:
            break
        if close.obstacles < 0:
            note.append(f"opening/closing fingers hit {close.worst_obstacle}")
        else:
            note.append("grasp pose in the platform keep-out" if min(close.platform, close.platform_mesh) < 0
                        else "fingers below the table while closing")
        q = None
    if q is None:                     # note[0] is the reason at the lowest pad height
        if not note:
            note.append("no grasp pose")
        return None
    g5 = q + [j5]
    room = K.pose_check(g5, opening.delta, None, _pairs(others, 0.0)).obstacles
    for rise in C.G["hover_rises_m"]:
        h = V.solve_tcp([pad[0], pad[1], pad[2] + rise], pitch, j5, c_delta)
        if h is None:
            continue
        h5 = h + [j5]
        down = K.move_check(h5, g5, opening.delta, None, others_p + [(target, TARGET)],
                            C.G["fingertip_table_margin_m"])
        if not (K.limits_ok(h5) and down.ok):
            continue
        aligned = yaw_deg is None or (roll["residual_deg"] <= C.G["aligned_tolerance_deg"]
                                      and roll["tilt_deg"] <= C.G["max_jaw_tilt_deg"])
        return dict(pitch=pitch, j5=j5, grasp=g5, hover=h5, opening=opening, pad=pad, rise=rise,
                    aligned=aligned, residual_deg=roll["residual_deg"], tilt_deg=roll["tilt_deg"],
                    open_gap_mm=opening.gap_m * 1000, room_cm=100.0 if room == math.inf else room * 100,
                    descend_check=down, close_check=close)
    note.append("no hover with a clear way down")
    return None


def plan_pick(start5: Sequence[float], bx: float, by: float, yaw_deg: float | None,
              others: Sequence[S.Box] = (), start_j6: float = 30, prefer: float | None = None,
              width: float = C.BLOCK) -> MotionPlan:
    """Transit to the hover, open, descend, close on the block, lift back to the hover."""
    why: list = []
    cands = grasp_candidates(bx, by, yaw_deg, others, prefer, width, why)
    target = S.block(bx, by, yaw_deg or 0.0, "target", width)
    c_delta = GR.contact(width)
    start_delta = GR.delta_for_j6(start_j6)
    for cand in cands:
        o = cand["opening"]
        r = route(start5, cand["hover"], start_delta, None, _pairs(others) + [(target, TARGET)])
        if r is None:
            why.append(f"pitch {cand['pitch']} J5 {cand['j5']}: no clear way to the hover")
            continue
        legs, legs_checks = r
        h5, g5 = cand["hover"], cand["grasp"]
        held = GR.held_from_grasp(g5, width)
        open_check = K.sweep_check(h5, start_delta, o.delta, _pairs(others) + [(target, TARGET)])
        lift_check = K.move_check(g5, h5, c_delta, held, _pairs(others), _table_allowance(g5, h5, c_delta, held))
        if not open_check.ok or not lift_check.ok:
            why.append(f"pitch {cand['pitch']} J5 {cand['j5']}: cannot open at the hover / lift the block")
            continue
        grip = C.G["grip_command_deg"]
        steps = [Step("move", "to hover" if len(legs) == 1 else f"to hover {i + 1}/{len(legs)}", leg, start_j6, False, c)
                 for i, (leg, c) in enumerate(zip(legs, legs_checks))]
        steps += [
            Step("gripper", "open", h5, o.j6, False, open_check, action=o.action,
                 note=f"jaw {o.gap_m * 1000:.0f} mm; pads {abs(GR.gg.pad_centre_link(o.delta)[2] - GR.gg.pad_centre_link(c_delta)[2]) * 1000:.0f} mm back from closed"),
            Step("move", "descend", g5, o.j6, False, cand["descend_check"], note="joint-linear from the hover"),
            Step("gripper", "grip", g5, grip, False, cand["close_check"], action="grip",
                 note=f"closes on {width * 1000:.0f} mm at J6 ~{GR.j6_for_width(width):.0f} (table); pads sweep "
                      f"{(GR.gg.pad_centre_link(c_delta)[2] - GR.gg.pad_centre_link(o.delta)[2]) * 1000:.0f} mm forward"),
            Step("move", "lift", h5, grip, True, lift_check),
        ]
        tip = V.tip(g5)
        summary = dict(
            block_base_cm=[round(bx * 100, 2), round(by * 100, 2)], block_yaw_deg=yaw_deg,
            pitch=cand["pitch"], pitch_verified=C.G["verified_pitch_range_deg"][0] <= cand["pitch"] <= C.G["verified_pitch_range_deg"][1],
            j5=round(cand["j5"], 1), aligned=cand["aligned"],
            residual_deg=None if cand["residual_deg"] is None else round(cand["residual_deg"], 1),
            jaw_tilt_deg=round(cand["tilt_deg"], 1), opening=o.action, open_j6=o.j6, open_gap_mm=round(o.gap_m * 1000, 1),
            pad_centre_target_cm=[round(v * 100, 2) for v in cand["pad"]],
            pad_above_table_cm=round((cand["pad"][2] - C.TABLE_Z) * 100, 2),
            fingertips_above_table_cm=round((GR.lowest(g5, c_delta) - C.TABLE_Z) * 100, 2),
            fk_tip_at_grasp_cm=[round(v * 100, 2) for v in tip], hover_rise_cm=round(cand["rise"] * 100, 1),
            room_to_neighbours_cm=round(min(cand["room_cm"], 100.0), 2), held_block=held.to_dict(),
            table_z_model_cm=round(C.TABLE_Z * 100, 2), alternatives=len(cands) - 1)
        plan = MotionPlan("pick", True, steps, summary, why[-6:])
        plan.held = held
        return plan
    return MotionPlan("pick", False, reasons=why or ["no candidate"])


# --------------------------------------------------------------------------------------------- release

def plan_place(start5: Sequence[float], held: GR.Held, bx: float, by: float, yaw_deg: float | None = None,
               others: Sequence[S.Box] = (), rim_z: float | None = None, prefer_pitch: float | None = None) -> MotionPlan:
    """Carry the held block over (bx, by), set it down on the table (rim_z None) or let it go above a bin
    rim at rim_z, open, back off upwards."""
    side = held.side
    c_delta = GR.contact(side)
    prefer = V.pitch_of(start5) if prefer_pitch is None else prefer_pitch
    z0 = (C.TABLE_Z + side / 2 + C.R["table_gap_m"]) if rim_z is None else (rim_z + C.R["bin_clearance_m"] + side / 2)
    why: list = []
    for dp in C.R["pitch_spread_deg"]:
        pitch = round(prefer) + dp
        q0 = V.solve_tcp([bx, by, z0], pitch, C.ROLL_ZERO, c_delta)
        if q0 is None:
            why.append(f"pitch {pitch}: out of reach")
            continue
        rolls = (V.roll_candidates(q0, yaw_deg)[:4] if yaw_deg is not None else
                 [dict(j5=float(start5[4]), residual_deg=None, tilt_deg=V.jaw_bearing_tilt(q0 + [start5[4]])[1])])
        for roll in rolls:
            plan = _place_at(start5, held, bx, by, z0, yaw_deg, pitch, roll, others, rim_z, why)
            if plan is not None:
                return plan
    return MotionPlan("place", False, reasons=why or ["no release pose"])


def _block_bottom(centre, rot, side) -> float:
    return centre[2] - side / 2 * (abs(rot[2][0]) + abs(rot[2][1]) + abs(rot[2][2]))


def _place_at(start5, held, bx, by, z0, yaw_deg, pitch, roll, others, rim_z, why) -> MotionPlan | None:
    j5, c_delta, side = roll["j5"], GR.contact(held.side), held.side
    # solve so the *held block's* centre (not the pad middle) lands on the target and its lowest corner
    # sits the release gap above the table (or the clearance above the rim), then level-check it
    tx, ty, z = bx, by, z0
    q = None
    for _ in range(3):
        q = V.solve_tcp([bx, by, z], pitch, j5, c_delta)
        if q is None:
            break
        centre, rot = GR.held_base(q + [j5], held)
        dx, dy = tx - centre[0], ty - centre[1]
        floor = C.TABLE_Z + C.R["table_gap_m"] if rim_z is None else rim_z + C.R["bin_clearance_m"]
        need = floor - _block_bottom(centre, rot, side)
        if abs(dx) < 3e-4 and abs(dy) < 3e-4 and abs(need) < 3e-4:
            break
        bx, by, z = bx + dx, by + dy, z + need
    if q is None:
        why.append(f"pitch {pitch} J5 {j5}: no IK for the release")
        return None
    r5 = q + [j5]
    if yaw_deg is not None and roll["residual_deg"] is not None and roll["residual_deg"] > C.G["aligned_tolerance_deg"]:
        why.append(f"pitch {pitch} J5 {j5}: the block would land {roll['residual_deg']:.0f} deg off the requested yaw "
                   f"(J5 only turns one way from its zero, {C.LIMITS[5][0]}-{C.LIMITS[5][1]})")
        return None
    centre, rot = GR.held_base(r5, held)
    level = math.degrees(math.acos(max(-1.0, min(1.0, rot[2][2]))))
    if level > C.R["max_block_tilt_deg"]["table" if rim_z is None else "bin"]:
        why.append(f"pitch {pitch} J5 {j5}: the block would come down {level:.0f} deg off level")
        return None
    if not K.limits_ok(r5):
        why.append(f"pitch {pitch} J5 {j5}: release pose outside the joint limits")
        return None
    hold_check = K.pose_check(r5, c_delta, held, _pairs(others), -0.0005)
    if not hold_check.ok:
        why.append(f"pitch {pitch} J5 {j5}: release pose hits {hold_check.worst_obstacle or 'the platform/table'}")
        return None
    placed = S.Box("block", "placed", tuple(centre), (side / 2,) * 3,
                   math.degrees(math.atan2(rot[1][0], rot[0][0])))
    opening, open_check = None, None
    for o in GR.OPENINGS:
        chk = K.sweep_check(r5, c_delta, o.delta, _pairs(others))
        if chk.ok:
            opening, open_check = o, chk
            break
    if opening is None:
        why.append(f"pitch {pitch} J5 {j5}: the fingers cannot open there without touching something")
        return None
    if rim_z is None:
        # set down on the table: come down onto the spot from above, back off upwards
        a = V.solve_tcp([centre[0], centre[1], z + C.R["rise_m"]], pitch, j5, c_delta)
        if a is None:
            why.append(f"pitch {pitch}: no pose above the release")
            return None
        a5 = a + [j5]
        lower_check = K.move_check(a5, r5, c_delta, held, _pairs(others), _table_allowance(a5, r5, c_delta, held))
        after = _pairs(others) + [(placed, TARGET)]
        back_check = K.move_check(r5, a5, opening.delta, None, after, _table_allowance(r5, a5, opening.delta, None))
        if not (K.limits_ok(a5) and lower_check.ok and back_check.ok):
            why.append(f"pitch {pitch} J5 {j5}: no clear way down to / up from the release")
            return None
        approach, back = a5, a5
    else:
        # over a bin: arrive at drop height, let go, go back to where the carry started (the block falls)
        approach, lower_check = None, None
        rb = route(r5, start5, opening.delta, None, _pairs(others))
        if rb is None:
            why.append(f"pitch {pitch} J5 {j5}: no clear way back from the drop with the fingers open")
            return None
        back, back_check = rb[0][-1], rb[1][-1]
        back_legs = rb
    r = route(start5, approach or r5, c_delta, held, _pairs(others))
    if r is None:
        why.append(f"pitch {pitch} J5 {j5}: no clear way to {'above ' if approach else ''}the target")
        return None
    legs, legs_checks = r
    grip = C.G["grip_command_deg"]
    steps = [Step("move", "carry" if len(legs) == 1 else f"carry {i + 1}/{len(legs)}", leg, grip, True, c)
             for i, (leg, c) in enumerate(zip(legs, legs_checks))]
    if lower_check is not None:
        steps.append(Step("move", "lower", r5, grip, True, lower_check))
    steps.append(Step("gripper", "release", r5, opening.j6, False, open_check, action=opening.action,
                      note="pads move back and apart; " + ("the block is set down" if rim_z is None
                                                            else "the block drops into the bin")))
    if rim_z is None:
        steps.append(Step("move", "back off", back, opening.j6, False, back_check))
    else:
        for i, (leg, c) in enumerate(zip(*back_legs)):
            steps.append(Step("move", "back off" if len(back_legs[0]) == 1 else f"back off {i + 1}/{len(back_legs[0])}",
                              leg, opening.j6, False, c))
    summary = dict(target_base_cm=[round(placed.centre[0] * 100, 2), round(placed.centre[1] * 100, 2)],
                   block_centre_cm=[round(v * 100, 2) for v in centre], block_yaw_deg=round(placed.yaw_deg, 1),
                   requested_yaw_deg=yaw_deg, block_level_deg=round(level, 1),
                   block_bottom_above_table_cm=round((_block_bottom(centre, rot, side) - C.TABLE_Z) * 100, 2),
                   pitch=pitch, j5=round(j5, 1), jaw_tilt_deg=round(roll["tilt_deg"], 1),
                   into="table" if rim_z is None else "bin", opening=opening.action, open_j6=opening.j6)
    return MotionPlan("place", True, steps, summary, why[-6:])
