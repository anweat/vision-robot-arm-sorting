"""Release solving: where the arm lets go of the held block. No motion; planner.plan_place adds the path.

The held block's pose in the fingers is kept from the grasp (`gripper.Held`). The release pose puts its
centre over the destination (shifted by trims.place_offset_m) with its bottom the release gap above the
table, or the bin clearance above a rim; its faces at the requested yaw when one is given (J5 only turns
one way from its zero, so most yaws are out of reach without tilting the block), and level within
max_block_tilt_deg. The fingers must be able to open there (pads back and apart) clear of everything.
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
    rim_z: float | None = None           # None: set down on the table; else drop above this rim (base z)
    name: str = "destination"

    @property
    def into(self) -> str:
        return "table" if self.rim_z is None else "bin"

    def to_dict(self) -> dict:
        return dict(x_cm=round(self.x * 100, 2), y_cm=round(self.y * 100, 2), yaw_deg=self.yaw_deg,
                    rim_z_cm=None if self.rim_z is None else round(self.rim_z * 100, 2), into=self.into, name=self.name)


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

    def to_dict(self, table_z: float) -> dict:
        return dict(pitch=self.pitch, j5=round(self.j5, 1), pose=[round(v, 2) for v in self.pose],
                    above=None if self.above is None else [round(v, 2) for v in self.above],
                    opening=self.opening.action, open_j6=self.opening.j6,
                    block_centre_cm=[round(v * 100, 2) for v in self.placed.centre],
                    block_yaw_deg=round(self.placed.yaw_deg, 1), block_level_deg=round(self.level_deg, 1),
                    jaw_tilt_deg=round(self.tilt_deg, 1),
                    block_bottom_above_table_cm=round((self.bottom_z - table_z) * 100, 2),
                    checks={k: c.to_dict() for k, c in self.checks.items()})


def block_bottom(centre, rot, side) -> float:
    return centre[2] - side / 2 * (abs(rot[2][0]) + abs(rot[2][1]) + abs(rot[2][2]))


def candidates(w: K.World, carry5, held: GR.Held, dest: Destination, prefer_pitch: float | None = None,
               explain: list | None = None, limit: int = 4) -> list[Release]:
    """Feasible release poses, in the order tried (pitch spread around the carry's, then J5 candidates)."""
    t = w.tuning
    why = explain if explain is not None else []
    side = held.side
    c_delta = GR.contact(side)
    prefer = V.pitch_of(carry5) if prefer_pitch is None else prefer_pitch
    dx, dy, dz = offset(dest.x, dest.y, t.trims["place_offset_m"])
    tx, ty = dest.x + dx, dest.y + dy
    floor = (t.table_z + C.R["table_gap_m"] + dz) if dest.rim_z is None else (dest.rim_z + C.R["bin_clearance_m"] + dz)
    out: list[Release] = []
    for dp in C.R["pitch_spread_deg"]:
        pitch = round(prefer) + dp
        q0 = V.solve_tcp([tx, ty, floor + side / 2], pitch, C.ROLL_ZERO, c_delta)
        if q0 is None:
            why.append(f"pitch {pitch}: out of reach")
            continue
        rolls = (V.roll_candidates(q0, dest.yaw_deg)[:4] if dest.yaw_deg is not None else
                 [dict(j5=float(carry5[4]), residual_deg=None, tilt_deg=V.jaw_bearing_tilt(q0 + [carry5[4]])[1])])
        for roll in rolls:
            r = _one(w, held, tx, ty, floor, dest, pitch, roll, why)
            if r is not None:
                out.append(r)
                if len(out) >= limit:
                    return out
    return out


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
    allowed = C.R["max_block_tilt_deg"][dest.into]
    if dest.into == "table":
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
    for o in GR.OPENINGS:
        c = K.sweep_check(w, r5, c_delta, o.delta)
        if c.ok:
            opening, open_c = o, c
            break
    if opening is None:
        why.append(f"pitch {pitch} J5 {j5}: the fingers cannot open there without touching something")
        return None
    checks = dict(hold=hold, open=open_c)
    above = None
    if dest.rim_z is None:
        for rise in C.R.get("rises_m", [C.R["rise_m"]]):
            a = V.solve_tcp([bx, by, z + rise], pitch, j5, c_delta)
            if a is not None and K.limits_ok(a + [j5]):
                above = a + [j5]
                break
        if above is None:
            why.append(f"pitch {pitch} J5 {j5}: no pose above the release (rises {C.R.get('rises_m')})")
            return None
    return Release(pitch, j5, r5, above, opening, placed, level, roll["tilt_deg"], block_bottom(centre, rot, side), checks)
