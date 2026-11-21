"""MotionPlans: compose the solvers (grasp, release) with the router (route) against a World.

    w = planning.world(cmap)                       # collision map + tuning
    pick = plan_pick(w, start5, Target.from_map(cmap, "b1"))
    place = plan_place(w.without(pick.summary["target_label"]), pick.end, pick.held, Destination(x, y))
    move = plan_transit(w, start5, goal5, j6)

Every step carries its own checks (full point set); a failed plan lists why each candidate was refused.
"""

from __future__ import annotations

from typing import Sequence

from . import checks as K
from . import config as C
from . import grasp as GP
from . import gripper as GR
from . import release as RL
from . import route as RT
from . import solver as V
from . import tuning as TU
from .grasp import Target
from .motion import MotionPlan, Step
from .release import Destination


def _trim(w: K.World) -> list:
    return [0.0, 0.0, 0.0, 0.0, float(w.tuning.trims.get("j5_trim_deg", 0.0)), 0.0]


def _site_flags(steps: list) -> dict:
    """What a plan asks of the arm beyond what has run on it: J5 outside the verified range, and the
    console's standard keep-out hull at the step ends (< 0: the console's standard profile refuses that move;
    the plan itself is checked on the meshes, tuning checks.platform_hull)."""
    lo, hi = C.J5_VERIFIED
    j5s = [s.joints[4] for s in steps if len(s.joints) > 4]
    hull = min((K.platform_keep_out(s.joints[:4]) for s in steps), default=float("inf"))
    return dict(j5_verified=all(lo - 1e-6 <= j <= hi + 1e-6 for j in j5s),
                j5_range=[round(min(j5s), 1), round(max(j5s), 1)] if j5s else None,
                console_hull_min_cm=None if hull == float("inf") else round(hull * 100, 2))


def _legs(name: str, r: RT.Route, j6: float, held: bool, trim: list) -> list[Step]:
    n = len(r.legs)
    return [Step("move", name if n == 1 else f"{name} {i + 1}/{n}", leg, j6, held, c,
                 note=r.method if i == 0 else "", trim=trim) for i, (leg, c) in enumerate(zip(r.legs, r.checks))]


def plan_transit(w: K.World, start5: Sequence[float], goal5: Sequence[float], j6: float,
                 held: "GR.Held | None" = None) -> MotionPlan:
    delta = GR.contact(held.side) if held is not None else GR.delta_for_j6(j6)
    r = RT.plan_route(w, start5, goal5, delta, held)
    if not r.ok:
        return MotionPlan("transit", False, reasons=[r.reason], summary=dict(route=r.stats))
    steps = _legs("transit", r, j6, held is not None, _trim(w))
    return MotionPlan("transit", True, steps, dict(method=r.method, waypoints=len(r.legs), route=r.stats, **_site_flags(steps)))


def plan_pick(w: K.World, start5: Sequence[float], target: Target, start_j6: float = 30,
              prefer: float | None = None) -> MotionPlan:
    """Route to the hover, open, descend, close on the block, lift back to the hover (best grasp that has a
    clear way there)."""
    for plan in pick_plans(w, start5, target, start_j6, prefer):
        return plan
    return MotionPlan("pick", False, reasons=getattr(pick_plans, "last_why", None) or ["no candidate"],
                      summary=dict(target=target.to_dict()))


AVOID_J5_DEG = 30.0       # a retry's jaw must turn at least this much from a jaw that closed on nothing

TIER_NOTES = {
    "full": "",
    "touching": "the neighbouring blocks checked as exact boxes with no clearance: the fingers may come up to them "
                "(bins and everything else keep every clearance)",
    "pipeline": "neighbouring blocks ignored at the grasp, as the verified console pipeline does: the fingers may "
                "brush or nudge them (the real arm held 4 such blocks, doc 34 §8)",
}


LIFT_OFF_M = 0.003        # a block lifted off another: how far its corners may start inside the one below
TOUCH_M = 0.0005          # "touching": the gripper may come this near a neighbouring block, not into it


def _local(w: K.World) -> K.World:
    """The relaxed tiers' tuning for what stays in the map (bins, boxes): the position errors only -- no base
    neighbour clearance, and not the map's largest object sigma applied to everything (one bin placed by
    design, sigma 1 cm, grew the clearance round every object to ~14 mm). With the configured errors that is
    ~5 mm round a bin wall: at 0 the simulated fingers brushed the walls, at ~9 mm most blocks within 3.5 cm of
    a bin were refused (2026-11-20 simulator rounds, doc 34 §8)."""
    t = w.tuning
    raw = TU._merge(t.raw, {"clearances": {"neighbour_m": 0.0}})
    return w.with_tuning(type(t)(raw, t.source + " (local)", 0.0, list(t.notes)))


def touching(w: K.World, skip: Sequence = ()) -> K.World:
    """The other blocks taken out of the map and checked as exact boxes the gripper may come up to (TOUCH_M, no
    position error); the bins and everything else stay in the map with the configured clearance and errors
    (_local: without the map-wide sigma) -- a bin wall is not to be brushed."""
    others = [lab for lab in w.block_labels() if lab not in skip and lab not in w.exclude]   # not the held block
    out = _local(w).without(*others)
    for lab in others:
        out = out.with_extra(w.cmap.object_box(lab, w.tuning.table_z), TOUCH_M)
    return out


def grasp_tiers(w: K.World, target: Target) -> list[tuple[str, K.World]]:
    """The worlds a grasp is solved in, strictest first (motion_planning.json grasp.tiers): "full" (every
    clearance), "touching" (the neighbouring blocks as exact boxes the gripper may come up to), "pipeline" (the
    other blocks left out of the grasp-local checks -- descend, open, close, lift). Bins, platform, table and
    the target keep their checks in every tier, and the way to the hover still keeps clear of everything."""
    out = []
    others = [lab for lab in w.block_labels() if lab != target.label]
    for tier in C.G.get("tiers", ["full"]):
        if tier == "full":
            out.append((tier, w))
        elif tier == "touching":
            out.append((tier, touching(w, skip=(target.label,))))
        elif tier == "pipeline":
            out.append((tier, _local(w).without(*others)))
    return out


def _jaw_turn(a: float, b: float) -> float:
    """Angle between two jaw directions given as J5 (a half turn is the same jaw)."""
    return abs((a - b + 90.0) % 180.0 - 90.0)


def pick_plans(w: K.World, start5: Sequence[float], target: Target, start_j6: float = 30,
               prefer: float | None = None, why: list | None = None, avoid_j5: Sequence[float] = ()):
    """Feasible pick MotionPlans, best grasp first, tier by tier (grasp_tiers; a generator: plan_pick_and_place
    walks it until a place works). Refusals are appended to `why`. `avoid_j5`: J5 values whose grasps closed on
    nothing; grasps within AVOID_J5_DEG of one (either half turn) are passed over, so a retry closes the jaw
    across the other pair of faces -- a position error across the jaw (10 mm tolerance) becomes one along it,
    where the open pads have ~16 mm to spare."""
    why = [] if why is None else why
    pick_plans.last_why = why
    passed_over, given = [], 0
    for tier, wl in grasp_tiers(w, target):
        n = len(why)
        for plan in _pick_plans(w, wl, tier, start5, target, start_j6, prefer, why):
            j5 = plan.summary["j5"]
            if any(_jaw_turn(j5, a) < AVOID_J5_DEG for a in avoid_j5):
                why.append(f"pitch {plan.summary['pitch']} J5 {j5}: passed over, a grasp at J5 {list(avoid_j5)} closed on nothing")
                passed_over.append(plan)
                continue
            given += 1
            yield plan
        if len(why) > n:
            why[n:] = [f"[{tier}] {r}" for r in why[n:]]
    if not given:                        # no other jaw: try the same one again rather than give the block up
        for plan in passed_over:
            plan.summary["retry_same_jaw"] = True
            yield plan


def _pick_plans(w: K.World, wl: K.World, tier: str, start5, target: Target, start_j6, prefer, why: list):
    """Pick plans for one tier: grasp-local checks in `wl`; the way to the hover in `w` for the full tier, else
    in the touching world (neighbours as exact boxes -- never ignored on the way, only at the grasp)."""
    t = w.tuning
    cands = GP.candidates(wl, target, prefer, why)
    wt = GP.world_for(w if tier == "full" else touching(w, skip=(target.label,)), target)
    w_held = wl.without(target.label) if target.label else wl
    if target.support is not None:        # lifted off a block: it starts on it (corners of a block held with a
        # tilted jaw a little into its top), and only goes up: the lift may begin LIFT_OFF_M inside it
        w_held = w_held.without(target.support_label).with_extra(target.support, -LIFT_OFF_M)
    c_delta = GR.contact(target.size)
    start_delta = GR.delta_for_j6(start_j6)
    t_open = t.target_open(target.size)
    trim = _trim(w)
    grip = float(t.trims["grip_command_deg"])
    for n, cand in enumerate(cands):
        o = cand.opening
        r = RT.plan_route(wt, start5, cand.hover, start_delta, None, target_margin=t_open)
        wlt = GP.world_for(wl, target)
        if not r.ok:
            why.append(f"pitch {cand.pitch} J5 {cand.j5}: no way to the hover ({r.reason})")
            continue
        h5, g5 = cand.hover, cand.grasp
        held = GR.held_from_grasp(g5, target.size, target.box(t.table_z).centre)
        open_check = K.sweep_check(wlt, h5, start_delta, o.delta, target_margin=t_open)
        lift_check = K.move_check(w_held, g5, h5, c_delta, held,
                                  table_margin=RT.table_allowance(w_held, g5, h5, c_delta, held))
        if not open_check.ok or not lift_check.ok:
            bad = open_check if not open_check.ok else lift_check
            why.append(f"pitch {cand.pitch} J5 {cand.j5}: {'open at the hover' if not open_check.ok else 'lift'}: {bad.failed()}")
            continue
        back = abs(GR.gg.pad_centre_link(o.delta)[2] - GR.gg.pad_centre_link(c_delta)[2]) * 1000
        steps = _legs("to hover", r, start_j6, False, trim)
        steps += [
            Step("gripper", "open", h5, o.j6, False, open_check, action=o.action,
                 note=f"jaw {o.gap_m * 1000:.0f} mm; pads {back:.0f} mm back from closed", trim=trim),
            Step("move", "descend", g5, o.j6, False, cand.checks["descend"], note="joint-linear from the hover", trim=trim),
            Step("gripper", "grip", g5, grip, False, cand.checks["close"], action="grip",
                 note=f"closes on {target.size * 1000:.0f} mm at J6 ~{GR.j6_for_width(target.size):.0f} (table); "
                      f"pads sweep {back:.0f} mm forward", trim=trim),
            Step("move", "lift", h5, grip, True, lift_check, trim=trim),
        ]
        summary = dict(
            target=target.to_dict(), target_label=target.label, grasp=cand.to_dict(), candidate=n,
            pitch=cand.pitch, pitch_verified=C.G["verified_pitch_range_deg"][0] <= cand.pitch <= C.G["verified_pitch_range_deg"][1],
            j5=round(cand.j5, 1), aligned=cand.aligned, opening=o.action, open_j6=o.j6,
            pad_above_table_cm=round((cand.pad[2] - t.table_z) * 100, 2),
            fingertips_above_table_cm=round((GR.lowest(g5, c_delta) - t.table_z) * 100, 2),
            fk_tip_at_grasp_cm=[round(v * 100, 2) for v in V.tip(g5)],
            room_to_neighbours_mm=None if cand.room_m == float("inf") else round(cand.room_m * 1000, 1),
            held_block=held.to_dict(), route=dict(method=r.method, **r.stats), table_z_cm=round(t.table_z * 100, 2),
            alternatives=len(cands) - 1, grasp_tier=tier, tier_note=TIER_NOTES.get(tier, ""), **_site_flags(steps))
        yield MotionPlan("pick", True, steps, summary, why[-6:], held=held)


def plan_pick_and_place(w: K.World, start5: Sequence[float], target: Target, dest: Destination,
                        start_j6: float = 30, prefer: float | None = None,
                        avoid_j5: Sequence[float] = ()) -> tuple[MotionPlan, MotionPlan]:
    """The best grasp whose carried block can also be released at `dest` (the grasp fixes the block's pose
    in the fingers, and with it which release poses keep it level). Returns (pick, place)."""
    why: list = []
    first = None
    for pick in pick_plans(w, start5, target, start_j6, prefer, why, avoid_j5):
        first = first or pick
        place = plan_place(w.without(target.label) if target.label else w, pick.end, pick.held, dest,
                           from_tier=pick.summary.get("grasp_tier", "full"))
        if place.ok:
            pick.summary["chosen_for_place"] = True
            return pick, place
        why.append(f"grasp {pick.summary['candidate']} (pitch {pick.summary['pitch']} J5 {pick.summary['j5']}): "
                   f"place: {place.reasons[0] if place.reasons else '?'}")
    if first is None:
        return MotionPlan("pick", False, reasons=why or ["no candidate"], summary=dict(target=target.to_dict())), \
            MotionPlan("place", False, reasons=["no pick"], summary=dict(destination=dest.to_dict()))
    return first, MotionPlan("place", False, reasons=[r for r in why if "place:" in r] or why,
                             summary=dict(destination=dest.to_dict()))


def plan_place(w: K.World, start5: Sequence[float], held: GR.Held, dest: Destination,
               prefer_pitch: float | None = None, from_tier: str = "full") -> MotionPlan:
    """Carry the held block over the destination, set it down (table) or let go above the rim (bin), open,
    back off. `w` must not contain the held block (World.without(its label)). Tried with every clearance,
    then "touching" (never with the neighbours left out, so a block is not set down on another). `from_tier`:
    the pick's tier -- the carry starts where the pick ended, so a place does not start stricter than it."""
    why: list = []
    order = ("full", "touching")
    first = order.index(from_tier) if from_tier in order else len(order) - 1
    for tier, wl in (("full", w), ("touching", touching(w)))[first:]:
        if tier not in C.G.get("tiers", ["full"]):
            continue
        n = len(why)
        plan = _plan_place(wl, start5, held, dest, prefer_pitch, why)
        why[n:] = [f"[{tier}] {r}" for r in why[n:]]
        if plan is not None:
            plan.summary.update(place_tier=tier, tier_note=TIER_NOTES.get(tier, ""))
            plan.reasons = why[-6:]
            return plan
    return MotionPlan("place", False, reasons=why or ["no release pose"], summary=dict(destination=dest.to_dict()))


def _plan_place(w: K.World, start5, held: GR.Held, dest: Destination, prefer_pitch, why: list) -> MotionPlan | None:
    t = w.tuning
    c_delta = GR.contact(held.side)
    trim = _trim(w)
    grip = float(t.trims["grip_command_deg"])
    for rel in RL.candidates(w, start5, held, dest, prefer_pitch, why):
        o = rel.opening
        wd = RL.world_for(w, dest)          # a block stacked on: a precise box, not its map cells
        after = wd.with_extra(rel.placed, t.target_touch())
        if dest.mode == "set":
            a5, r5 = rel.above, rel.pose
            lower = K.move_check(wd, a5, r5, c_delta, held, table_margin=RT.table_allowance(wd, a5, r5, c_delta, held))
            back = K.move_check(after, r5, a5, o.delta, None, table_margin=RT.table_allowance(after, r5, a5, o.delta, None))
            if not (lower.ok and back.ok):
                why.append(f"pitch {rel.pitch} J5 {rel.j5}: {'lower' if not lower.ok else 'back off'}: "
                           f"{(lower if not lower.ok else back).failed()}")
                continue
            carry = RT.plan_route(w, start5, a5, c_delta, held)
            if not carry.ok:
                why.append(f"pitch {rel.pitch} J5 {rel.j5}: no way to above the spot ({carry.reason})")
                continue
            steps = _legs("carry", carry, grip, True, trim)
            steps.append(Step("move", "lower", r5, grip, True, lower, trim=trim))
            steps.append(Step("gripper", "release", r5, o.j6, False, rel.checks["open"], action=o.action,
                              note=f"pads move back and apart ({o.gap_m * 1000:.0f} mm); the block is set down "
                                   + {"table": "on the table", "stack": f"on {dest.support.name if dest.support else ''}",
                                      "bin": "on the bin's floor"}[dest.into], trim=trim))
            steps.append(Step("move", "back off", a5, o.j6, False, back, trim=trim))
        else:
            carry = RT.plan_route(w, start5, rel.pose, c_delta, held)
            if not carry.ok:
                why.append(f"pitch {rel.pitch} J5 {rel.j5}: no way to the drop ({carry.reason})")
                continue
            back = RT.plan_route(w, rel.pose, start5, o.delta, None)
            if not back.ok:
                why.append(f"pitch {rel.pitch} J5 {rel.j5}: no way back from the drop with the fingers open ({back.reason})")
                continue
            steps = _legs("carry", carry, grip, True, trim)
            steps.append(Step("gripper", "release", rel.pose, o.j6, False, rel.checks["open"], action=o.action,
                              note="pads move back and apart; the block drops into the bin", trim=trim))
            steps += _legs("back off", back, o.j6, False, trim)
        summary = dict(destination=dest.to_dict(), release=rel.to_dict(t.table_z),
                       target_base_cm=[round(rel.placed.centre[0] * 100, 2), round(rel.placed.centre[1] * 100, 2)],
                       block_level_deg=round(rel.level_deg, 1),
                       block_bottom_above_table_cm=round((rel.bottom_z - t.table_z) * 100, 2),
                       pitch=rel.pitch, j5=round(rel.j5, 1), into=dest.into, mode=dest.mode, opening=o.action,
                       open_j6=o.j6, open_gap_mm=round(o.gap_m * 1000, 1),
                       route=dict(method=carry.method, **carry.stats), **_site_flags(steps))
        return MotionPlan("place", True, steps, summary, why[-6:])
    return None
