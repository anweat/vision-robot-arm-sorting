"""Grasp solving: which (pitch, J5, opening, pad height) closes the pads on a target without touching
anything else. No motion here; planner.plan_pick turns the best candidate into a MotionPlan.

The TCP is the middle of the pads *closed on the block*: the four-bar pulls the pads 26 mm back along the
gripper axis when it opens, so the target comes from the closed state and the open fingers arrive higher
and wider. A candidate must
  * reach the pad target (block centre at trims.pad_centre_height_m, shifted by trims.grasp_offset_m)
    inside the joint limits, fingertips above the table closed on the block and on nothing;
  * keep the open fingers clear of the target by target_open (its xy/yaw and the arm's errors) while they
    come down from the hover and at the grasp pose, and clear of everything in the map by the neighbour
    clearance; the closing sweep is checked at the nominal pose against target_touch;
  * keep links 1-4 off the map and the gripper out of the platform;
  * square the jaw to the block faces (both face pairs, any half turn) without tilting it out of the table
    plane by more than max_jaw_tilt_deg.
Ranked: aligned, flat, residual, widest open, most room, nearest the preferred pitch, least rolled.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from . import body as B
from . import checks as K
from . import config as C
from . import gripper as GR
from . import scene as S
from . import solver as V

ROOM_LADDER = (0.0, 0.002, 0.004, 0.006, 0.008, 0.01, 0.015, 0.02)


@dataclass
class Target:
    """The block to pick, base frame. `label`: its cells in the collision map (left out of map checks)."""
    x: float
    y: float
    yaw_deg: float | None = None
    size: float = C.BLOCK
    label: int | None = None
    name: str = "target"

    def box(self, table_z: float) -> S.Box:
        return S.Box("block", self.name, (self.x, self.y, table_z + self.size / 2), (self.size / 2,) * 3,
                     self.yaw_deg or 0.0)

    @classmethod
    def from_map(cls, cmap, key) -> "Target":
        o = cmap.object(key)
        return cls(o["centre_m"][0], o["centre_m"][1], o.get("yaw_deg"), float(min(o["size_m"][:2])), o["id"],
                   o.get("name", str(o["id"])))

    def to_dict(self) -> dict:
        return dict(x_cm=round(self.x * 100, 2), y_cm=round(self.y * 100, 2), yaw_deg=self.yaw_deg,
                    size_mm=round(self.size * 1000, 1), label=self.label, name=self.name)


@dataclass
class Grasp:
    pitch: float
    j5: float
    grasp: list
    hover: list
    opening: GR.Opening
    pad: list
    rise: float
    aligned: bool
    residual_deg: float | None
    tilt_deg: float
    room_m: float
    checks: dict = field(default_factory=dict)        # open / close / descend

    def to_dict(self) -> dict:
        return dict(pitch=self.pitch, j5=round(self.j5, 1), grasp=[round(v, 2) for v in self.grasp],
                    hover=[round(v, 2) for v in self.hover], opening=self.opening.action, open_j6=self.opening.j6,
                    open_gap_mm=round(self.opening.gap_m * 1000, 1), pad_cm=[round(v * 100, 2) for v in self.pad],
                    hover_rise_cm=round(self.rise * 100, 1), aligned=self.aligned,
                    residual_deg=None if self.residual_deg is None else round(self.residual_deg, 1),
                    tilt_deg=round(self.tilt_deg, 1), room_mm=round(self.room_m * 1000, 1),
                    checks={k: c.to_dict() for k, c in self.checks.items()})


def world_for(w: K.World, target: Target) -> K.World:
    return w.with_target(target.box(w.tuning.table_z), target.label)


def offset(x: float, y: float, o) -> tuple[float, float, float]:
    """[radial, lateral-left, up] -> base dx, dy, dz at the bearing of (x, y)."""
    r = math.hypot(x, y) or 1.0
    ux, uy = x / r, y / r
    return o[0] * ux - o[1] * uy, o[0] * uy + o[1] * ux, o[2]


def candidates(w: K.World, target: Target, prefer: float | None = None, explain: list | None = None,
               limit: int = 8) -> list[Grasp]:
    """Feasible grasps, best first. `w` is the world without the target (it is added here)."""
    t = w.tuning
    prefer = float(t.trims["prefer_pitch_deg"]) if prefer is None else prefer
    why = explain if explain is not None else []
    wt = world_for(w, target)
    c_delta = GR.contact(target.size)
    dx, dy, dz = offset(target.x, target.y, t.trims["grasp_offset_m"])
    base = [target.x + dx, target.y + dy, t.table_z + float(t.trims["pad_centre_height_m"]) + dz]
    out: list[Grasp] = []
    sc = C.G["pitch_scan"]
    feasible = V.pitch_range(base, C.ROLL_ZERO, c_delta, sc["from_deg"], sc["to_deg"], sc["resolution_deg"])
    if not feasible:
        why.append(f"{math.hypot(target.x, target.y) * 100:.1f} cm from the base: no pitch "
                   f"{sc['from_deg']}-{sc['to_deg']} reaches it inside the joint limits")
        return []
    why.append(f"pitches {feasible[0]:g}-{feasible[-1]:g} reach it inside the joint limits")
    for pitch in V.spread(feasible, prefer, sc["step_deg"], sc["max_tried"]):
        q0 = V.solve_tcp(base, pitch, C.ROLL_ZERO, c_delta)
        if q0 is None:
            why.append(f"pitch {pitch}: out of reach")
            continue
        for roll in V.roll_candidates(q0, target.yaw_deg)[:4]:
            for opening in GR.OPENINGS:
                note: list = []
                g = _one(wt, target, base, pitch, roll, opening, c_delta, note)
                if g is not None:
                    out.append(g)
                    break
                why.append(f"pitch {pitch} J5 {roll['j5']} {opening.action}: " + ("; ".join(dict.fromkeys(note)) or "?"))
    tilt = C.G["max_jaw_tilt_deg"]
    out.sort(key=lambda g: (not g.aligned, g.tilt_deg > tilt, round(g.residual_deg or 0.0),
                            -g.opening.gap_m, -min(g.room_m, 0.01), abs(g.pitch - prefer), abs(g.j5 - C.ROLL_ZERO)))
    return out[:limit]


def _room(w: K.World, q5, delta) -> float:
    """Largest horizontal clearance on ROOM_LADDER the open gripper keeps from the map (ranking only)."""
    t = w.tuning
    pts, _ = B.gripper(q5, delta, None, 1)
    best = 0.0
    for m in ROOM_LADDER:
        c, _ = w.cmap.clearance(pts, t.table_z, m, 0.0, w.exclude, t.map.get("unknown_top_m"),
                                t.map.get("out_of_map_free", True))
        if c < 0:
            break
        best = m
    return best if best < ROOM_LADDER[-1] else math.inf


def _one(w: K.World, target: Target, base, pitch, roll, opening, c_delta, note) -> Grasp | None:
    t = w.tuning
    j5 = roll["j5"]
    t_open, t_touch = t.target_open(target.size), t.target_touch()
    sweep_end = GR.gg.delta_for_gap(target.size + 0.004)
    tip_margin = t.table(t.clear["fingertip_table_m"])
    closed_margin = t.table(t.clear["closed_fingertip_table_m"])
    q = close = open_c = None
    pad = base
    # lowest pads first; a tilted jaw may need them a few mm higher: one finger hangs lower, and the
    # finger above the pads leans in over the block's top edge
    for lift in C.G["pad_lifts_m"]:
        pad = [base[0], base[1], base[2] + lift]
        if pad[2] > t.table_z + target.size - C.G["pad_below_top_m"]:
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
        if (GR.lowest(g5, c_delta) < t.table_z + tip_margin
                or GR.lowest(g5, GR.CLOSED) < t.table_z + closed_margin):
            note.append(f"pads +{lift * 1000:.0f} mm: fingertips too near the table")
            q = None
            continue
        open_c = K.pose_check(w, g5, opening.delta, target_margin=t_open)
        close = K.sweep_check(w, g5, opening.delta, sweep_end, target_margin=t_touch, steps=4)
        if open_c.ok and close.ok:
            break
        bad = open_c if not open_c.ok else close
        note.append(f"pads +{lift * 1000:.0f} mm: " + ("open fingers: " if not open_c.ok else "closing: ") + bad.failed())
        q = None
    if q is None:                     # note[0] is the reason at the lowest pad height
        if not note:
            note.append("no grasp pose")
        return None
    g5 = q + [j5]
    for rise in C.G["hover_rises_m"]:
        h = V.solve_tcp([pad[0], pad[1], pad[2] + rise], pitch, j5, c_delta)
        if h is None:
            continue
        h5 = h + [j5]
        down = K.move_check(w, h5, g5, opening.delta, target_margin=t_open, table_margin=tip_margin)
        if not (K.limits_ok(h5) and down.ok):
            continue
        aligned = target.yaw_deg is None or (roll["residual_deg"] <= C.G["aligned_tolerance_deg"]
                                             and roll["tilt_deg"] <= C.G["max_jaw_tilt_deg"])
        return Grasp(pitch, j5, g5, h5, opening, pad, rise, aligned, roll["residual_deg"], roll["tilt_deg"],
                     _room(w, g5, opening.delta), dict(open=open_c, close=close, descend=down))
    note.append("no hover with a clear way down")
    return None
