"""Release solving: where the arm lets go of the held block. No motion; planner.plan_place adds the path.

The held block's pose in the fingers is kept from the grasp (`gripper.Held`). The release pose puts its
centre over the destination (shifted by trims.place_offset_m) and its bottom
  * "set" (the default): the release gap above the surface it is set on -- the table, the top of the block
    it is stacked on, or the floor of a bin (or what already lies there) -- coming straight down from a pose
    above it (over a bin: the held block clear of the rim);
  * "drop" (bins only): the bin clearance above the rim, and it falls in;
its faces at the requested yaw when one is given (J5 only turns one way from its zero, so most yaws are out
of reach without tilting the block; on a stack the carry's J5 is tried too), and level within
max_block_tilt_deg for where it goes. The fingers must be able to open there clear of everything: the
console's openings and every 10 deg of J6 between them are tried widest first (gripper.release_openings),
so inside a bin or beside a stack the jaw opens only as wide as there is room.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from . import checks as K
from . import config as C
from . import gripper as GR
from . import scene as S
from . import solver as V
from .grasp import offset


@dataclass
class Destination:
    x: float
    y: float
    yaw_deg: float | None = None         # None: keep the J5 the block was carried at
    rim_z: float | None = None           # a bin's rim (base z); None: no bin
    name: str = "destination"
    support_z: float | None = None       # base z of the surface it is set on; None: the table (or, in a bin,
                                         # its floor: rim_z - wall height is not known here, give it)
    mode: str | None = None              # "set" | "drop"; None: "drop" into a bin without support_z, else "set"
    support: S.Box | None = None         # the block it is stacked on (a precise box; its map label left out)
    support_label: int | None = None

    def __post_init__(self):
        if self.mode is None:
            self.mode = "drop" if self.rim_z is not None and self.support_z is None else "set"
        if self.mode not in ("set", "drop") or (self.mode == "drop" and self.rim_z is None):
            raise ValueError(f"release mode {self.mode!r}: 'set', or 'drop' into a bin")

    @property
    def into(self) -> str:
        return "bin" if self.rim_z is not None else "stack" if self.support is not None else "table"

    @property
    def tilt_key(self) -> str:
        return "bin" if self.mode == "drop" else "bin_set" if self.into == "bin" else self.into

    def to_dict(self) -> dict:
        d = dict(x_cm=round(self.x * 100, 2), y_cm=round(self.y * 100, 2), yaw_deg=self.yaw_deg,
                 rim_z_cm=None if self.rim_z is None else round(self.rim_z * 100, 2), into=self.into, mode=self.mode,
                 name=self.name)
        if self.support_z is not None:
            d["support_z_cm"] = round(self.support_z * 100, 2)
        if self.support is not None:
            d["on"] = self.support.name
        return d


def world_for(w: K.World, dest: Destination) -> K.World:
    """The world a release is checked in: a block stacked on is a precise box the held block comes down to
    (the release gap) and the fingers keep the touch margin from."""
    if dest.support is None:
        return w
    return w.without(dest.support_label).with_extra(dest.support, w.tuning.target_touch())


@dataclass
class Release:
    pitch: float
    j5: float
    pose: list
    above: list | None                   # the pose a table set-down comes down from and backs off to
    opening: GR.Opening
    placed: S.Box
    level_deg: float
    tilt_deg: float
    bottom_z: float
    checks: dict = field(default_factory=dict)
    fall_m: float = 0.0                  # how far the block falls when let go (bottom over what it lands on)

    def to_dict(self, table_z: float) -> dict:
        return dict(pitch=self.pitch, j5=round(self.j5, 1), pose=[round(v, 2) for v in self.pose],
                    above=None if self.above is None else [round(v, 2) for v in self.above],
                    opening=self.opening.action, open_j6=self.opening.j6,
                    block_centre_cm=[round(v * 100, 2) for v in self.placed.centre],
                    block_yaw_deg=round(self.placed.yaw_deg, 1), block_level_deg=round(self.level_deg, 1),
                    jaw_tilt_deg=round(self.tilt_deg, 1),
                    block_bottom_above_table_cm=round((self.bottom_z - table_z) * 100, 2),
                    fall_cm=round(self.fall_m * 100, 2),
                    checks={k: c.to_dict() for k, c in self.checks.items()})


def block_bottom(centre, rot, side) -> float:
    return centre[2] - side / 2 * (abs(rot[2][0]) + abs(rot[2][1]) + abs(rot[2][2]))


def candidates(w: K.World, carry5, held: GR.Held, dest: Destination, prefer_pitch: float | None = None,
               explain: list | None = None, limit: int = 4) -> list[Release]:
    """Feasible release poses, in the order tried (pitch spread around the carry's, then J5 candidates)."""
    t = w.tuning
    w = world_for(w, dest)
    why = explain if explain is not None else []
    side = held.side
    c_delta = GR.contact(side)
    prefer = V.pitch_of(carry5) if prefer_pitch is None else prefer_pitch
    dx, dy, dz = offset(dest.x, dest.y, t.trims["place_offset_m"])
    tx, ty = dest.x + dx, dest.y + dy
    surface = t.table_z if dest.support_z is None else dest.support_z
    gap = C.R.get("stack_gap_m", C.R["table_gap_m"]) if dest.into == "stack" else C.R["table_gap_m"]
    floor = (surface + gap + dz) if dest.mode == "set" else (dest.rim_z + C.R["bin_clearance_m"] + dz)
    floors = [floor]
    if dest.mode == "set" and dest.rim_z is not None:
        # into a bin: as low as the gripper fits between the walls, every bin_set_step_m up to the rim (where
        # a drop lets go); the block falls the rest (Release.fall_m)
        step, top = float(C.R.get("bin_set_step_m", 0.005)), dest.rim_z + C.R["bin_clearance_m"] + dz
        floors = [floor + i * step for i in range(int((top - floor) / step) + 1)]
    out: list[Release] = []
    for f in floors:
        n = len(why)
        out += _at(w, carry5, held, dest, tx, ty, f, surface, prefer, c_delta, why, limit)
        if out:
            if f > floor:
                why.append(f"let go {(f - floor) * 100:.1f} cm above the lowest set-down: nothing lower fits")
            return out
        if len(floors) > 1:
            why[n:] = [f"{(f - floor) * 100:.1f} cm up: {r}" for r in why[n:]][:3]
    return out


def _at(w, carry5, held, dest, tx, ty, floor, surface, prefer, c_delta, why, limit) -> list[Release]:
    side = held.side
    out: list[Release] = []
    for pitch in pitches(tx, ty, floor + side / 2, c_delta, prefer, dest, held, why):
        q0 = V.solve_tcp([tx, ty, floor + side / 2], pitch, C.ROLL_ZERO, c_delta)
        if q0 is None:
            why.append(f"pitch {pitch}: out of reach")
            continue
        keep = dict(j5=float(carry5[4]), residual_deg=None, tilt_deg=V.jaw_bearing_tilt(q0 + [carry5[4]])[1])
        rolls = V.roll_candidates(q0, dest.yaw_deg)[:4] if dest.yaw_deg is not None else [keep]
        if dest.yaw_deg is not None and dest.into == "stack":
            rolls = rolls + [keep]          # on a block, faces off its yaw still stand on it
        for roll in rolls:
            r = _one(w, held, tx, ty, floor, dest, pitch, roll, why)
            if r is not None:
                r.fall_m = max(0.0, r.bottom_z - surface)
                out.append(r)
                if len(out) >= limit:
                    return out
    return out


def pitches(tx, ty, z, c_delta, prefer: float, dest: Destination, held, why: list) -> list[float]:
    """Release pitches to try, nearest the carry's first. The approach angle open at the spot is solved
    (solver.pitch_range, as for a grasp): a block picked far out at a flat pitch is set down near the base
    only at a steep one. Turning the pitch turns the held block with it, so only pitches within the level
    limit for where it goes (plus the tilt the grasp already gave it) are kept; the level check decides."""
    sc = C.G["pitch_scan"]
    window = C.R["max_block_tilt_deg"][dest.tilt_key] + getattr(held, "tilt_deg", 0.0) + 2.0
    feasible = [p for p in V.pitch_range([tx, ty, z], C.ROLL_ZERO, c_delta, sc["from_deg"], sc["to_deg"],
                                         sc["resolution_deg"]) if abs(p - prefer) <= window]
    if not feasible:
        why.append(f"no pitch within {window:.0f} deg of the carry's {prefer:.0f} reaches "
                   f"({tx * 100:.1f}, {ty * 100:.1f}, {z * 100:.1f}) cm inside the joint limits")
        return []
    near = sorted(feasible, key=lambda p: abs(p - prefer))
    out = [near[0]] + [p for p in V.spread(feasible, prefer, sc["step_deg"], 99) if p != near[0]]
    return sorted(dict.fromkeys(out), key=lambda p: abs(p - prefer))[:int(C.R.get("max_pitches", 9))]


def _one(w: K.World, held, tx, ty, floor, dest, pitch, roll, why) -> Release | None:
    j5, side = roll["j5"], held.side
    c_delta = GR.contact(side)
    # solve so the *held block's* centre (not the pad middle) lands on the target and its lowest corner
    # sits on `floor`, then level-check it
    bx, by, z = tx, ty, floor + side / 2
    q = None
    for _ in range(3):
        q = V.solve_tcp([bx, by, z], pitch, j5, c_delta)
        if q is None:
            break
        centre, rot = GR.held_base(q + [j5], held)
        ddx, ddy = tx - centre[0], ty - centre[1]
        need = floor - block_bottom(centre, rot, side)
        if abs(ddx) < 3e-4 and abs(ddy) < 3e-4 and abs(need) < 3e-4:
            break
        bx, by, z = bx + ddx, by + ddy, z + need
    if q is None:
        why.append(f"pitch {pitch} J5 {j5}: no IK for the release")
        return None
    r5 = q + [j5]
    if dest.yaw_deg is not None and roll["residual_deg"] is not None and roll["residual_deg"] > C.G["aligned_tolerance_deg"]:
        why.append(f"pitch {pitch} J5 {j5}: the block would land {roll['residual_deg']:.0f} deg off the requested yaw "
                   f"(J5 {C.LIMITS[5][0]}-{C.LIMITS[5][1]})")
        return None
    centre, rot = GR.held_base(r5, held)
    level = math.degrees(math.acos(max(-1.0, min(1.0, rot[2][2]))))
    # set down on the table: near level, or no worse than the grasp left it (the same tilt the pads gave it
    # lifting it off a flat table; let go 3 mm up it rocks back onto its face)
    allowed = C.R["max_block_tilt_deg"][dest.tilt_key]
    if dest.tilt_key in ("table", "bin_set"):
        allowed = max(allowed, getattr(held, "tilt_deg", 0.0) + 1.0)
    if level > allowed:
        why.append(f"pitch {pitch} J5 {j5}: the block would come down {level:.0f} deg off level")
        return None
    if not K.limits_ok(r5):
        why.append(f"pitch {pitch} J5 {j5}: release pose outside the joint limits")
        return None
    hold = K.pose_check(w, r5, c_delta, held, table_margin=-0.0005)
    if not hold.ok:
        why.append(f"pitch {pitch} J5 {j5}: release pose: {hold.failed()}")
        return None
    placed = S.Box("block", "placed", tuple(centre), (side / 2,) * 3, math.degrees(math.atan2(rot[1][0], rot[0][0])))
    opening = open_c = None
    for o in GR.release_openings(side):
        c = K.sweep_check(w, r5, c_delta, o.delta)
        if c.ok:
            opening, open_c = o, c
            break
    if opening is None:
        why.append(f"pitch {pitch} J5 {j5}: the fingers cannot open there without touching something "
                   f"(down to {GR.release_openings(side)[-1].gap_m * 1000:.0f} mm)")
        return None
    checks = dict(hold=hold, open=open_c)
    above = None
    if dest.mode == "set":
        # over a bin the held block comes down from above the rim; elsewhere from the first rise that reaches
        need = 0.0 if dest.rim_z is None else dest.rim_z + C.R["bin_clearance_m"] - block_bottom(centre, rot, side)
        rises = [r for r in C.R.get("rises_m", [C.R["rise_m"]]) if r >= need] or []
        rises = rises if dest.rim_z is None else [need + e for e in C.R.get("over_rim_extra_m", [0.0, 0.01, 0.02])]
        for rise in rises:
            a = V.solve_tcp([bx, by, z + rise], pitch, j5, c_delta)
            if a is not None and K.limits_ok(a + [j5]):
                above = a + [j5]
                break
        if above is None:
            why.append(f"pitch {pitch} J5 {j5}: no pose above the release (rises {[round(r, 3) for r in rises]})")
            return None
    return Release(pitch, j5, r5, above, opening, placed, level, roll["tilt_deg"], block_bottom(centre, rot, side), checks)
