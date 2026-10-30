"""Grasp, carry and place blocks through the running teleop console, plus the moves operators keep asking
for, so none of it is worked out by hand.

Everything is sent through the console API (src/tools/console_step.Session), so its limits, keep-out,
5 deg segments with readback checks and hold-at-readback on resistance all apply. The explore profile
does not check the table, so every path is checked here (lowest tip height along it).

    python3 src/tasks/console_grasp.py where             # blocks seen now: map/base cm, reach, grasp zone
    python3 src/tasks/console_grasp.py observe           # safely back to observe_v1 (rises first if low)
    python3 src/tasks/console_grasp.py retreat           # same as observe, named for "get out of the way"
    python3 src/tasks/console_grasp.py goto 0,13 [2]     # tip over map (x, y) cm, [height above the table, cm]
    python3 src/tasks/console_grasp.py locate green      # observe, localize (look again if the view is poor), plan
    python3 src/tasks/console_grasp.py hover             # open, go 2 cm above the block top
    python3 src/tasks/console_grasp.py grasp             # descend to the block centre, grip, lift back to hover
    python3 src/tasks/console_grasp.py put 43            # carry onto marker 43 (or map "x,y" cm), open, lift
    python3 src/tasks/console_grasp.py place             # back down where it was gripped, open, lift
    python3 src/tasks/console_grasp.py all green         # locate + hover + grasp (pauses at hover if the pitch is unverified)

Verified procedure (docs/05, 2026-09-26): board top-face localization, target moved 1 cm radially outwards
for the open gripper, pitch 145-155, hover 2 cm above the top, vertical descent to the block centre.
Beyond ~25 cm from the base only pitches <= 135 reach; two tries there missed (docs/05), so plans there stop.
"""
from __future__ import annotations

import json
import math
import sys
import time
from contextlib import nullcontext
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
for sub in ("spatial", "tools", "vision"):
    sys.path.insert(0, str(ROOT / "src" / sub))

import dofbot_kinematics as kin  # noqa: E402
import workspace  # noqa: E402
from console_step import BASE, ConsoleError, Session, status  # noqa: E402
from map_registration import DEFAULT_MAP  # noqa: E402

CONFIG = json.loads((ROOT / "config" / "arm_limits.json").read_text())
MAP = json.loads(DEFAULT_MAP.read_text())      # map_a4_v2 unless ARM_MAP names another sheet (map_registration.py)
TABLE_Z = CONFIG["keep_out"]["table_surface_z_m"]
KEEP_OUT = workspace.load_keep_out(CONFIG)   # the platform box (docs/05, refitted 2026-09-25)
RADIAL_OFFSET = CONFIG["gripper"].get("open_grasp_radial_offset_cm", 1.0) / 100
DX, DY = MAP["placement"]["base_from_map"]["dx"], MAP["placement"]["base_from_map"]["dy"]
BLOCK = 0.03
HOVER_ABOVE_TOP = 0.02
PITCHES = (150, 155, 145, 160, 140, 135, 130)
# The open fingers' midpoint sits ~1.15 cm off the gripper axis (1 cm horizontally at pitch 150, measured);
# its horizontal share changes little down to 135 (0.8 cm), so 135-155 counts as verified by geometry.
VERIFIED_PITCHES = range(135, 156)
HOVER_STEPS = (0.02, 0.015, 0.01, 0.005)   # above the block top; far blocks only reach a low hover
MAX_REACH = 0.28                     # absolute; within it, reachability is decided by IK (operator, 2026-09-26)
MIN_REACH = 0.175                    # nearer blocks need pitch ~160 and the camera cannot see markers there
ROLL_ZERO = CONFIG["gripper"].get("roll_zero_deg", 92)
ROLL_SIGN = CONFIG["gripper"].get("roll_sign", 1)
J5_RANGE = (90, 180)                 # course limit 90; above 180 the gripper servo turns up into the camera's view (operator, 2026-09-26)
ALIGN_ROLL = CONFIG["gripper"].get("align_roll", False)
TANGENT_SIGN = CONFIG["gripper"].get("finger_offset_tangent_sign", 1)
ROLL_WINDOW = (-2, 88)   # J5 - ROLL_ZERO allowed for alignment: the whole safe J5 span, 90-180.
                         # Was (-2, 45): a block at yaw 21 seen from J1 64 wanted J5 139, two degrees
                         # outside it, so the roll was dropped and the fingers met the block corner-on
                         # (43 deg off its faces) and flicked it away (2026-09-27). Rolls of 13-29 deg
                         # held in 4 of 4 grasps that night; the old caution came from J5 ~250, which
                         # is outside J5_RANGE anyway.
ROLL_90_MIN_PITCH = 165  # a 90 deg roll tips the finger-opening axis out of horizontal by (180 - pitch),
                         # so the fingers meet the block's top and bottom edges instead of its faces
                         # (2026-09-26, J5 167 at a flat pitch). Only near-vertical pitches may use one.
ROLL_ALIGNED_DEG = 12    # fingers this far off the faces still grip a 3 cm block
GRASP_HEIGHT = CONFIG["gripper"].get("grasp_height_cm", 1.2) / 100   # fingertips above the table when
                         # closing on a 3 cm block. Lower than the block's mid-height on purpose: the
                         # nearer the table the contact is, the less leverage a closing finger has to
                         # tip the block over or flick it out (operator, 2026-09-27). The block still
                         # spans 0-3 cm, so the fingers are well inside its faces.
OBSERVE = [90, 105, 5, 5]
EDGE_PX = 40            # a box this close to the image border may be a partial view
AGREE_M = 0.015         # board and phone must agree this well, else look again
CARRY_RADIUS = 0.17     # retract to this radius before turning the base, so a held block passes inside others
CARRY_ABOVE_TABLE = 0.07
RUNS = ROOT / "data" / "runs" / "console_grasp"
PLAN = RUNS / "plan.json"


class PathBlocked(ConsoleError):
    """A move would run the fingers into a bin (OBSTACLES). The arm has not been sent anywhere."""


class GripMissed(ConsoleError):
    """The fingers closed on nothing; the gripper works and the arm is back at the hover."""


def log(event: str, **data) -> None:
    RUNS.mkdir(parents=True, exist_ok=True)
    line = dict(t=time.time(), event=event, **data)
    with (RUNS / "log.jsonl").open("a") as f:
        f.write(json.dumps(line, ensure_ascii=False) + "\n")
    print(json.dumps(line, ensure_ascii=False))


def lease(session: Session | None):
    """Reuse the caller's lease, or take one just for this call."""
    return nullcontext(session) if session is not None else Session()


# -------------------------------------------------------------------------------------------- geometry

def tip(q) -> list[float]:
    return kin.forward(list(q)[:4])["tip"]


def map_to_base(mx: float, my: float) -> tuple[float, float]:
    return mx + DX, my + DY


def marker_centre(marker: str) -> tuple[float, float]:
    corners = MAP["layout_m"][str(marker)]
    return sum(c[0] for c in corners) / 4, sum(c[1] for c in corners) / 4


def parse_place(where: str) -> tuple[float, float]:
    """A marker id ("43") or map "x,y" in cm -> map metres."""
    if "," in where:
        x, y = (float(v) / 100 for v in where.split(","))
        return x, y
    return marker_centre(where)


def radial(bx: float, by: float, amount: float) -> tuple[float, float]:
    r = math.hypot(bx, by)
    return bx + amount * bx / r, by + amount * by / r


def finger_offset(bx: float, by: float, j5: float | None, pitch: float = 150) -> tuple[float, float]:
    """Where to put the tip so the open fingers' midpoint lands on (bx, by).

    The midpoint sits ~1 cm off the J5 axis and turns with J5: at J5 = ROLL_ZERO it is 1 cm nearer the
    base than the tip (hence 'move the tip 1 cm outwards', docs/05); after a 160 deg roll it is on the
    far side, and the same +1 cm missed twice (2026-09-26). Tangential sign unverified (TANGENT_SIGN)."""
    rho = math.radians((j5 if j5 is not None else ROLL_ZERO) - ROLL_ZERO)
    r = math.hypot(bx, by)
    ux, uy = bx / r, by / r
    # the offset is perpendicular to the gripper axis; its horizontal share is cos(tilt from vertical)
    size = RADIAL_OFFSET / math.cos(math.radians(30)) * math.cos(math.radians(180 - pitch))
    radial_m = size * math.cos(rho)
    tangent_m = size * math.sin(rho) * TANGENT_SIGN
    return bx + radial_m * ux - tangent_m * uy, by + radial_m * uy + tangent_m * ux


def roll_options(j1: float, block_yaw_deg: float | None, pitch: float) -> list[int]:
    """The J5 values that turn the fingers parallel to the block's faces, best first, for one pitch.

    Turning the base by (J1 - 90) turns the fingers with it, so the roll has to undo it. Half turns are
    always equivalent -- the fingers swap sides. Quarter turns grip the other pair of faces, which a
    square block is happy with, but they swing the finger-opening axis into the gripper's pitch plane,
    where it tilts by (180 - pitch) and the fingers meet the block's top and bottom edges instead of its
    faces; so those are offered only at a near-vertical pitch.

    J5 spans 90 deg (90-180: higher and the gripper servo turns up into the camera's view), which is
    exactly the half-turn period -- so half turns alone leave half of all orientations with no solution
    at all. That gap is what quarter turns close."""
    want = ROLL_ZERO + ROLL_SIGN * ((block_yaw_deg or 0.0) - (j1 - 90))
    steps = [180, -180, 0] if pitch < ROLL_90_MIN_PITCH else [180, -180, 90, -90, 0]
    seen, out = set(), []
    for step in sorted(steps, key=abs, reverse=True):          # 0 first once sorted back below
        for k in range(-2, 3):
            v = round(want + (step * k if step else 0))
            if not (J5_RANGE[0] <= v <= J5_RANGE[1]) or v in seen:
                continue
            if not (ROLL_WINDOW[0] <= v - ROLL_ZERO <= ROLL_WINDOW[1]):
                continue
            seen.add(v)
            out.append(v)
    return sorted(out, key=lambda v: abs(v - ROLL_ZERO))


def grip_roll(j1: float, block_yaw_deg: float | None, pitch: float) -> int:
    """The best J5 for this block at this pitch, or ROLL_ZERO when nothing aligns. `pitch` is required:
    whether a quarter turn may be used depends on it, and a quarter turn at a flat pitch is the roll that
    missed on 2026-09-26. Callers that care whether it aligned should use roll_options and check --
    gripping a block corner-on flicks it away rather than holding it (2026-09-27)."""
    options = roll_options(j1, block_yaw_deg, pitch)
    return options[0] if options else ROLL_ZERO


def plan_approach(bx: float, by: float, yaw_deg: float | None, prefer: int = 150) -> dict | None:
    """Pitch, wrist roll, tip target and hover for a block centred at base (bx, by): the pitch nearest
    `prefer` at which the grasp pose (tip at the block centre) is reachable, with the highest hover it allows."""
    if math.hypot(bx, by) > MAX_REACH:
        return None
    bearing = 90 + math.degrees(math.atan2(-bx, by))
    # a pitch that lets the fingers line up with the block beats a pitch nearer `prefer`: corner-on
    # fingers flick a block away instead of holding it (2026-09-27)
    pitches = sorted(range(130, 171, 5), key=lambda p: abs(p - prefer))
    if ALIGN_ROLL and yaw_deg is not None:
        pitches.sort(key=lambda p: (not roll_options(bearing, yaw_deg, p), abs(p - prefer)))
    for pitch in pitches:
        j5 = grip_roll(bearing, yaw_deg, pitch) if ALIGN_ROLL else None
        tx, ty = finger_offset(bx, by, j5, pitch)
        low = solve(tx, ty, TABLE_Z + GRASP_HEIGHT, [pitch], floor=-3)
        if low is None or not clear_of_platform(low[1]):
            continue
        for up in HOVER_STEPS:
            h = solve(tx, ty, TABLE_Z + BLOCK + up, [pitch], floor=-3)
            if h and clear_of_platform(h[1]):
                aligned = (not ALIGN_ROLL or yaw_deg is None
                           or bool(roll_options(bearing, yaw_deg, pitch)))
                return dict(pitch=pitch, pitch_verified=pitch in VERIFIED_PITCHES, j5=j5, hover=h[1],
                            aligned=aligned, hover_above_top_cm=up * 100,
                            tip_target_cm=[round(tx * 100, 2), round(ty * 100, 2)])
    return None


# Things on the table the gripper must not run into: (oriented box in the base frame: 2x2 rotation, centre xyz, half
# size xyz). run.use_bins fills it with the acrylic bins; empty means nothing is checked (the old marker-code layout).
OBSTACLES: list[tuple[list, list, list]] = []
HIGH_LEG = [90, 125, 0, 0]       # raised and leaning forward (the first field-scan view): the way over the bins
OBSTACLE_MARGIN = 0.01          # the fingers keep this far from a bin wall; a bin standing 1-2 mm off its seat is nothing next to it
FINGER_POINTS = 0.05            # the fingers start this far past the servo-5 axis (a gripper-servo housing is higher still)


def set_obstacles(boxes) -> None:
    OBSTACLES[:] = list(boxes)


def obstacle_depth(q) -> float:
    """How far into an obstacle (grown by OBSTACLE_MARGIN) the deepest point of the open fingers is, in metres;
    <= 0 when clear. The fingers are points along the last link, +-1.5 cm across it (both ways: J5 is not known here)."""
    if not OBSTACLES:
        return 0.0
    pose = kin.servo5_pose(list(q)[:4] + [90])
    o, x, y, z = (kin_col(pose, i) for i in (3, 0, 1, 2))
    deepest = 0.0
    for a in range(9):
        along = FINGER_POINTS + (kin.GRIPPER - FINGER_POINTS) * a / 8
        for u in (-0.015, 0.0, 0.015):
            for axis in (x, y):
                point = [o[i] + along * z[i] + u * axis[i] for i in range(3)]
                for rot, centre, half in OBSTACLES:
                    dx, dy = point[0] - centre[0], point[1] - centre[1]
                    local = (rot[0][0] * dx + rot[1][0] * dy, rot[0][1] * dx + rot[1][1] * dy, point[2] - centre[2])
                    inside = min(half[i] + OBSTACLE_MARGIN - abs(local[i]) for i in range(3))
                    deepest = max(deepest, inside)
    return deepest


def kin_col(pose, i):
    return [pose[0][i], pose[1][i], pose[2][i]]


def obstacle_on_path(a, b, samples: int = 20) -> float:
    return max(obstacle_depth([x + (y - x) * i / samples for x, y in zip(a[:4], b[:4])]) for i in range(samples + 1))


def clear_of_platform(pose) -> bool:
    return workspace.platform_clearance(pose, KEEP_OUT) >= 0


def platform_on_path(a, b, samples: int = 30) -> float:
    return min(workspace.platform_clearance([x + (y - x) * i / samples for x, y in zip(a[:4], b[:4])], KEEP_OUT)
               for i in range(samples + 1))


def lowest_on_path(a, b, samples: int = 40) -> float:
    return min(tip([x + (y - x) * i / samples for x, y in zip(a[:4], b[:4])])[2] for i in range(samples + 1))


def check_path(a, b) -> None:
    """The tip may not dip below both ends by more than 5 mm, nor come within 1 cm of the table; the
    gripper may not enter the platform box (a start already inside may only move out)."""
    plat = platform_on_path(a, b)
    if plat < 0 and plat < workspace.platform_clearance(a[:4], KEEP_OUT) - 1e-9:
        raise ConsoleError(f"path enters the platform keep-out by {-plat*100:.1f} cm; refused")
    if OBSTACLES:
        hit, was = obstacle_on_path(a, b), obstacle_depth(a)
        if hit > 0 and hit > was + 1e-9:        # a start already inside may only move out, as with the platform
            raise PathBlocked(f"path puts the fingers {hit*100:.1f} cm into a bin's keep-out; refused")
    low = lowest_on_path(a, b)
    ends = min(tip(a)[2], tip(b)[2])
    # a path may start or end low (a grip pose is 1-2 cm above the table); it may not dip in between
    # the destination may not be at or below the table; the start may (after a sag or a power cycle)
    if tip(b)[2] < TABLE_Z + 0.005 or low < ends - 0.005 or (low < TABLE_Z + 0.01 and low < ends - 0.001):
        raise ConsoleError(f"path dips to tip z {low*100:.1f} cm; refused (ends {tip(a)[2]*100:.1f} / {tip(b)[2]*100:.1f})")


def solve(tx: float, ty: float, z: float, pitches, floor: int = 0) -> tuple[int, list[int]] | None:
    for pitch in pitches:
        q = kin.inverse(tx, ty, z, pitch)
        if q is not None and min(q[1:4]) >= floor:
            return pitch, [round(v) for v in q]
    return None


# -------------------------------------------------------------------------------------------- motion

def wake_gripper(session: Session, steps: int = 10) -> bool:
    """Small J6 steps until the readback really rises (>= 3 deg), i.e. the servo is awake. A step the console
    accepts proves nothing (a sleeping servo stays within the 4 deg tolerance for two +2 steps), and one it
    refuses ("servo 6: expected 34, read 29") has disarmed the lease, which is re-armed; that is the sleeping
    servo, not a collision (2026-09-25 19:16 and 19:30; by hand, +2 steps woke it on the second step)."""
    start = status()["pose"][5]
    for _ in range(steps):
        try:
            session.jog(6, 2)
        except ConsoleError as e:
            log("gripper_wake_refused", reason=str(e))
            session.rearm()
        time.sleep(0.3)
        now = status()["pose"][5]
        if start is not None and now is not None and now >= start + 3:
            log("gripper_awake", start=start, readback=now)
            return True
    log("gripper_still_asleep", start=start, readback=status()["pose"][5])
    return False


def wake_and_grip(session: Session, tries: int = 3) -> dict:
    """Close the gripper the way that works on this servo: a single 30 -> 133 command is sometimes ignored
    (readback stays at 29, 2026-09-25) while small J6 steps wake it. The ignored command is visible in the
    readback ("not_closed"), so grip first and wake only after that, rather than waking every time: waking
    up front cost 7.3 s of every 35.5 s block (data/runs/console_grasp, 2026-09-27) even when the servo was
    awake, and an ignored command costs one extra grip command instead. The fingers are already around the
    block at this point, so a command that does nothing changes nothing."""
    result = {}
    for attempt in range(tries):
        if attempt:
            wake_gripper(session)
        session.grip("grip")
        result = status()["grip"]
        if result.get("result") != "not_closed":
            break
        log("gripper_not_closed", attempt=attempt + 1, readback=result.get("readback"))
    return result


def lift(session: Session, back_to: list[int]) -> None:
    """Rise by retracing to a pose above. A Cartesian +z from the grip pose can flip IK branches
    (2026-09-26: 44 deg jump refused at pitch 154), so it is only a fallback."""
    s = status()
    try:
        check_path(s["pose"], back_to)
        session.pose(list(back_to[:4]) + [s["pose"][4]])
        return
    except ConsoleError as e:
        log("lift_fallback", reason=str(e))
    for _ in range(3):
        session.cart("z", 1)


def rise_if_low(session: Session) -> None:
    s = status()
    t = tip(s["pose"])
    if t[2] >= TABLE_Z + BLOCK:   # only a tip down among the blocks (a grasp or release pose) must rise first
        return
    pitch = kin.forward(s["pose"][:4])["tip_pitch_deg"][0]
    up = None
    for rise in (0.03, 0.02, 0.01):
        up = solve(t[0], t[1], t[2] + rise, [round(pitch) + d for d in (0, -3, 3, -6, 6)], floor=-3)
        if up:
            break
    if up is None:
        raise ConsoleError("down among the blocks and no vertical way up; move up by hand")
    check_path(s["pose"], up[1])
    session.pose(up[1] + [s["pose"][4]])


def move_to(session: Session, target4: list[int], j5: int | None = None) -> None:
    s = status()
    j5 = s["pose"][4] if j5 is None else j5
    if max(abs(a - b) for a, b in zip(s["pose"][:4], target4)) <= 1 and abs(s["pose"][4] - j5) <= 1:
        return
    rise_if_low(session)
    s = status()
    try:
        check_path(s["pose"], target4)
    except PathBlocked:
        # round the bins: up to a raised, forward-leaning pose at the current bearing, over to the target's, then down
        legs = [[s["pose"][0], *HIGH_LEG[1:]], [target4[0], *HIGH_LEG[1:]], list(target4)]
        here = s["pose"]
        for leg in legs:
            check_path(here, leg)
            here = leg
        for leg in legs[:-1]:
            session.pose(leg + [j5])
        session.pose(list(target4) + [j5])
        return
    session.pose(list(target4) + [j5])


def descend_to(session: Session, tx: float, ty: float, z: float, pitch: int) -> list[int]:
    """One pose move to the tip at (tx, ty, z), then small corrections from the readback (the arm sags
    1-3 deg at J2 near full reach, which leaves the tip up to 1.5 cm high)."""
    low = solve(tx, ty, z, [pitch, pitch + 3, pitch - 3], floor=-3)
    if low is None:
        raise ConsoleError(f"no pose for tip ({tx*100:.1f}, {ty*100:.1f}, {z*100:.1f}) cm near pitch {pitch}")
    s = status()
    check_path(s["pose"], low[1])
    session.pose(low[1] + [s["pose"][4]])
    # 2026-09-26: correcting on a readback taken before J2 settled overshot by 0.9 cm; wait, then only
    # nudge when clearly high
    for _ in range(2):
        time.sleep(0.4)
        err = tip(status()["pose"])[2] - z
        if err <= 0.008:
            break
        session.cart("z", -0.5)
    # J2 sags 1-3 deg near full reach, which also pulls the tip towards the base: correct in the plane
    time.sleep(0.3)
    t = tip(status()["pose"])
    for axis, err in (("x", tx - t[0]), ("y", ty - t[1])):
        if abs(err) >= 0.004:
            try:
                session.cart(axis, math.copysign(0.5 if abs(err) < 0.0075 else 1.0, err))
            except ConsoleError as e:
                log("xy_correction_refused", axis=axis, error_cm=round(err * 100, 2), reason=str(e))
    return status()["pose"]


PROBE_STEP = 0.005        # m per step down; the console allows Cartesian steps of 0.5/1/2/3 cm only,
                          # so 0.5 cm is the finest probe available and the resolution of the result
PROBE_STALL_M = 0.002     # the readback height stopped following the command by this much: something is there
RESISTANCE = "遇到阻力或偏差"   # the console's wording when a servo did not reach its command
SLIP_DEG = 3              # J6 closing this much further across a lift means the block left the fingers


def close_for_probe(session: Session) -> int:
    """Shut the fingers and prove they shut, before anything measures a height with them.

    The forward-kinematics tip is the closed fingertip point, so a probe taken with the fingers open
    measures nothing: they straddle a 3 cm block and pass either side of it. The gripper servo ignores a
    large command often enough that this has to be checked rather than assumed -- on 2026-09-29 two of
    four probes ran with the fingers at 29 deg because the grip was ignored, and their heights were
    quietly wrong. Raises instead of guessing."""
    result = wake_and_grip(session)
    rb = status()["pose"][5]
    if rb is None or rb < CONFIG["gripper"]["holding_if_readback_below_deg"]:
        raise ConsoleError(f"探测前夹爪没有真正闭合（读回 {rb}，结果 {result.get('result')}）；"
                           "张开的手指会从方块两侧滑过去，量到的高度没有意义")
    return rb


def probe_down(session: Session, tx: float, ty: float, pitch: int, to_z: float) -> dict:
    """Lower the closed fingertips at (tx, ty) until they meet something or reach `to_z`, and report the
    height where they stopped. The descent starts from the height that comes back, not from one the caller
    believes: a readback above the commanded pose would otherwise read as contact on step one.

    Contact is read from the readback height, not from the console refusing a step: one 0.5 cm step is a
    few degrees of joint travel, inside the console's 4 deg tolerance, so a blocked step is accepted while
    the arm does not move -- the same trap as the gripper that accepted steps while its readback stayed at
    29 (2026-09-25). A refusal counts as contact ONLY when it is about resistance: a step the console
    rejects for its size, or for a stale readback, is not a measurement, and reading it as one invents a
    contact height that never happened (2026-09-29: four probes 'measured' a top face without descending
    a millimetre, because a rejected step size was taken for contact).

    `height_cm` is where the fingers stopped; the surface is between that and `last_free_cm`, one step
    above. The fingers must be closed: the tip of the forward kinematics is the closed fingertip point,
    which is what the table touch calibration measured (docs/05, 2026-09-25).

    Only useful downwards. A block is not fixed to the table, so a sideways probe pushes it away instead
    of meeting resistance; the table underneath is what makes a downward probe read a real height."""
    measured = z = tip(status()["pose"])[2]
    contact, stalls, last_free, steps = None, 0, measured, 0
    # only whole steps, and only while one more keeps the fingers at or above to_z: clamping the command
    # while still sending a full step would push them below the floor this limit exists to keep
    while z - PROBE_STEP >= to_z - 1e-9 and contact is None:
        z -= PROBE_STEP
        try:
            session.cart("z", -PROBE_STEP * 100)
        except ConsoleError as e:
            if RESISTANCE not in str(e):
                raise                       # a rejected step size or a stale readback is not a contact
            contact = "resistance: " + str(e)
            break
        steps += 1
        time.sleep(0.25)
        now = tip(status()["pose"])[2]
        if now - z >= PROBE_STALL_M:
            stalls += 1
            if stalls >= 2:                 # two in a row: not one late readback
                contact = "readback stalled"
        else:
            stalls, last_free = 0, now
        measured = now
    out = dict(contacted=contact is not None, why=contact, steps=steps,
               height_cm=round(measured * 100, 2),
               above_table_cm=round((measured - TABLE_Z) * 100, 2),
               last_free_cm=round((last_free - TABLE_Z) * 100, 2),
               floor_cm=round((to_z - TABLE_Z) * 100, 2),
               resolution_cm=round(PROBE_STEP * 100, 2), commanded_cm=round(z * 100, 2))
    log("probe_down", tx_cm=round(tx * 100, 1), ty_cm=round(ty * 100, 1), pitch=pitch, **out)
    return out


def wait_localization(after: float, timeout: float = 12.0) -> dict:
    """The second board result computed at rest after the move: the first may use a frame captured while
    the arm was still moving (2026-09-26: a block seen by hand at a look pose was 'not seen' by the task)."""
    end, first = time.time() + timeout, None
    while time.time() < end:
        s = status()
        board = s["localization"].get("board") or {}
        servos = board.get("servos") or []
        still = len(servos) == len(s["pose"]) and all(abs(x - y) <= 1 for x, y in zip(servos, s["pose"]) if x is not None and y is not None)
        if not s["busy"] and still and not board.get("error") and board.get("t", 0) > after:
            if first is None:
                first = board["t"]
            elif board["t"] > first:
                return s
        time.sleep(0.3)
    raise ConsoleError("no fresh board localization after the move (camera stale or no markers?)")


# -------------------------------------------------------------------------------------------- vision

def pick(objects: list[dict], colour: str | None = None, near_cm: list[float] | None = None,
         within_cm: float = 3.0) -> dict | None:
    """A block by colour label, or the one nearest a map position (cm): colour labels fail for blocks whose
    top is a sticker (2026-09-26: green, sticker up, read 'red' on the board), positions do not."""
    if near_cm is not None:
        close = [o for o in objects if math.dist(o["map_cm"], near_cm) <= within_cm]
        return min(close, key=lambda o: math.dist(o["map_cm"], near_cm)) if close else None
    found = [o for o in objects if o["label"] == colour]
    return found[0] if found else None


def quality(board_obj: dict | None, phone_obj: dict | None) -> list[str]:
    problems = []
    if board_obj is None:
        return ["not seen by the board camera"]
    x, y, w, h = board_obj["bbox"]
    if x < EDGE_PX or y < EDGE_PX or x + w > 640 - EDGE_PX or y + h > 480 - EDGE_PX:
        problems.append("box near the image border (partial view)")
    # 'edges' is the method for a sticker-up block (colour only on the sides); only a partial view is unusable
    if board_obj.get("method") not in ("top-face", "edges"):
        problems.append(f"method {board_obj.get('method')}")
    if phone_obj is not None:
        d = math.dist(board_obj["map_cm"], phone_obj["map_cm"]) / 100
        if d > AGREE_M:
            problems.append(f"board and phone differ by {d*100:.1f} cm")
    return problems


def look_pose(bx: float, by: float, current: list[int]) -> list[int]:
    """J1-J4 that put the block's top centre nearest the middle of the wrist camera image
    (base turn around the block's bearing, shoulder raised 95-125 as in observe_v1)."""
    import numpy as np
    from camera_model import ArmCamera
    from map_registration import load_intrinsics
    cam, (k, _) = ArmCamera(), load_intrinsics()
    bearing = 90 + math.degrees(math.atan2(-bx, by))
    point = np.array([bx, by, TABLE_Z + BLOCK, 1.0])
    best = None
    for j1 in range(round(bearing) - 20, round(bearing) + 21, 2):
        for j2 in range(95, 126, 5):
            q = [j1, j2, 5, 5]
            p = np.linalg.inv(cam.base_T_cam(q + [90])) @ point
            if p[2] <= 0.05:
                continue
            u, v = k[0, 0] * p[0] / p[2] + k[0, 2], k[1, 1] * p[1] / p[2] + k[1, 2]
            score = math.hypot(u - 320, v - 240)
            if 80 < u < 560 and 80 < v < 400 and (best is None or score < best[0]):
                try:
                    check_path(current, q)
                except ConsoleError:
                    continue
                best = (score, q)
    if best is None:
        raise ConsoleError(f"no look pose puts base ({bx*100:.1f}, {by*100:.1f}) cm in view")
    return best[1]


def save_images(run: Path, name: str) -> None:
    import urllib.request
    for source in ("board", "phone"):
        try:
            data = urllib.request.urlopen(f"{BASE}/api/annotated/{source}", timeout=5).read()
            (run / f"{name}_{source}.jpg").write_bytes(data)
        except Exception:  # images are a record, not a requirement
            pass


def seen(s: dict | None = None) -> dict[str, dict]:
    """Best current estimate per colour: phone (top-face corrected, sees the whole map) else board."""
    s = s or status()
    out = {}
    for source in ("board", "phone"):
        for o in (s["localization"].get(source) or {}).get("objects", []):
            if o["label"] in ("red", "green", "blue", "yellow"):
                out[o["label"]] = dict(o, source=source)
    return out


# -------------------------------------------------------------------------------------------- stages

def observe(session: Session | None = None) -> dict:
    with lease(session) as sess:
        move_to(sess, OBSERVE)
    return wait_localization(time.time() - 0.1)


def where() -> None:
    for colour, o in sorted(seen().items()):
        bx, by = map_to_base(o["map_cm"][0] / 100, o["map_cm"][1] / 100)
        reach = math.hypot(bx, by)
        zone = "ok" if reach <= MAX_REACH else "too far"
        print(f"{colour:6s} map ({o['map_cm'][0]:6.2f}, {o['map_cm'][1]:6.2f}) cm  base ({bx*100:5.1f}, {by*100:5.1f}) cm  "
              f"reach {reach*100:4.1f} cm  {zone:7s} [{o['source']}, {o.get('method')}]")


def goto(where_: str, above_cm: float = 2.0, session: Session | None = None) -> None:
    mx, my = parse_place(where_)
    bx, by = map_to_base(mx, my)
    found = solve(bx, by, TABLE_Z + above_cm / 100, PITCHES)
    if found is None:
        raise ConsoleError(f"map ({mx*100:.1f}, {my*100:.1f}) cm at {above_cm} cm is out of reach")
    with lease(session) as sess:
        move_to(sess, found[1])
    s = status()
    log("goto", target_map_cm=[round(mx * 100, 2), round(my * 100, 2)], pitch=found[0], pose=s["pose"],
        tip_cm=[round(v * 100, 1) for v in tip(s["pose"])])


def locate(colour: str | None, session: Session | None = None, near_cm: list[float] | None = None) -> dict:
    run = RUNS / time.strftime("%Y%m%d_%H%M%S")
    run.mkdir(parents=True, exist_ok=True)
    PLAN.unlink(missing_ok=True)   # a failed locate leaves no plan behind
    with lease(session) as sess:
        s = observe(sess)
        tries = iter((0, 6, -6))   # base offsets for the looks: a first estimate from a partial view can be 2 cm off
        for attempt in ("observe", "look", "look left", "look right"):
            board = pick(s["localization"]["board"].get("objects", []), colour, near_cm)
            phone = pick((s["localization"].get("phone") or {}).get("objects", []), colour, near_cm)
            if near_cm is not None and board is not None:
                near_cm = board["map_cm"]   # follow the better estimate into the close look
            problems = quality(board, phone)
            log("localized", colour=colour, attempt=attempt, pose=s["pose"], board=board, phone=phone, problems=problems,
                near_cm=near_cm, board_all=[(o["label"], o["map_cm"], o.get("method")) for o in s["localization"]["board"].get("objects", [])],
                board_t=s["localization"]["board"].get("t"))
            save_images(run, attempt)
            if not problems:
                break
            if attempt == "look right":
                raise ConsoleError(f"{colour or near_cm}: still unreliable after three looks: {problems}")
            source = board or phone or (dict(map_cm=near_cm) if near_cm is not None else None)
            if source is None:
                raise ConsoleError(f"{colour} not found by board or phone")
            bx, by = map_to_base(source["map_cm"][0] / 100, source["map_cm"][1] / 100)
            if math.hypot(bx, by) > MAX_REACH:
                raise ConsoleError(f"{colour} is {math.hypot(bx, by)*100:.1f} cm from the base (> {MAX_REACH*100:.0f}); move it closer")
            look = look_pose(bx, by, s["pose"])
            if board is None:   # nothing to re-centre on: sweep the base around the last estimate
                look = [look[0] + next(tries)] + look[1:]
            move_to(sess, look)
            s = wait_localization(time.time() - 0.1)
    bx, by = board["base_cm"][0] / 100, board["base_cm"][1] / 100
    if math.hypot(bx, by) > MAX_REACH:
        raise ConsoleError(f"{colour} is {math.hypot(bx, by)*100:.1f} cm from the base (> {MAX_REACH*100:.0f}); move it closer")
    approach = plan_approach(bx, by, board.get("yaw_deg"))
    if approach is not None:
        check_path(s["pose"], approach["hover"])
        plan = dict(colour=colour or (board or {}).get("label"), near_cm=near_cm, created=time.time(),
                    run=str(run.relative_to(ROOT)), block_base_cm=[round(bx * 100, 2), round(by * 100, 2)],
                    board=board, phone=phone, block_yaw_deg=board.get("yaw_deg"),
                    block_centre_z_cm=round((TABLE_Z + GRASP_HEIGHT) * 100, 2), **approach)
        PLAN.write_text(json.dumps(plan, indent=1, ensure_ascii=False))
        log("planned", **{k: plan[k] for k in ("colour", "block_base_cm", "tip_target_cm", "pitch", "pitch_verified",
                                                 "hover", "hover_above_top_cm", "block_yaw_deg", "j5")})
        return plan
    raise ConsoleError("no reachable grasp pose at pitch 130-160")


PLAN_MAX_AGE_S = 180


def fresh_plan() -> dict:
    """The plan from the last locate, refused when old: a failed locate must not leave an earlier plan in charge."""
    plan = json.loads(PLAN.read_text())
    age = time.time() - plan.get("created", 0)
    if age > PLAN_MAX_AGE_S:
        raise ConsoleError(f"plan for {plan.get('colour')} is {age:.0f} s old; run locate again")
    return plan


def hover(session: Session | None = None) -> None:
    plan = fresh_plan()
    with lease(session) as sess:
        sess.grip("open")
        move_to(sess, plan["hover"], plan.get("j5"))
    s = status()
    log("hovering", pose=s["pose"], tip_cm=[round(v * 100, 1) for v in tip(s["pose"])], colour=plan["colour"],
        aligned=plan.get("aligned"), block_yaw_deg=plan.get("block_yaw_deg"))


def grasp(session: Session | None = None) -> dict:
    plan = fresh_plan()
    tx, ty = plan["tip_target_cm"][0] / 100, plan["tip_target_cm"][1] / 100
    with lease(session) as sess:
        grip_pose = descend_to(sess, tx, ty, plan["block_centre_z_cm"] / 100, plan["pitch"])
        result = wake_and_grip(sess)
        log("gripped", pose=grip_pose, tip_cm=[round(v * 100, 1) for v in tip(grip_pose)], grip=result)
        plan["grip_pose"] = grip_pose
        PLAN.write_text(json.dumps(plan, indent=1, ensure_ascii=False))
        if result.get("result") != "holding":
            sess.grip("open")
            lift(sess, plan["hover"])
            if result.get("result") == "not_closed":
                # the fingers did not move: jammed on the block, or the servo latched its stall protection
                # (2026-09-26: after a jam it ignored every command, even in the air, until power-cycled)
                air = wake_and_grip(sess)
                sess.grip("open")
                if air.get("result") == "not_closed":
                    raise ConsoleError("GRIPPER FAULT: it does not close even in the air; power-cycle the servos, "
                                       "then check the fingers turn freely by hand")
                raise ConsoleError(f"fingers jammed on the block (readback {result.get('readback')}); gripper OK in the air; "
                                   "square the block or re-locate")
            raise GripMissed(f"grip missed (readback {result.get('readback')}); opened and lifted")
        lift(sess, plan["hover"])
    s = status()
    after = s["pose"][5]
    log("lifted", pose=s["pose"], gripper_readback=after, gripper_at_grip=result.get("readback"))
    # the fingers closing further across the lift means the block left them: a readback taken at the grip
    # says the fingers were around something, not that they still are
    if result.get("readback") is not None and after is not None and after - result["readback"] >= SLIP_DEG:
        with lease(session) as sess:
            sess.grip("open")
        raise GripMissed(f"块在抬起过程中滑脱（夹取读回 {result['readback']} -> 抬起后 {after}）")
    return result


BIN_CLEARANCE = 0.015     # the held block's underside clears a bin's rim by this much when it is let go


def bin_drop_tip_z(rim_above_table_m: float, clearance_m: float = BIN_CLEARANCE) -> float:
    """Fingertip height (FK frame) to let a block go over a bin: the fingers closed 1.2 cm above the block's
    underside (GRASP_HEIGHT), so the underside hangs that far below the tip."""
    return TABLE_Z + rim_above_table_m + GRASP_HEIGHT + clearance_m


def plan_put(grasp_pitch: float, where_: str, pose: list[int], drop_tip_z: float | None = None) -> dict | None:
    """The poses put() would use to place a held block on `where_`, starting from `pose`, or None when no
    pitch within 15 deg of the grasp pitch reaches it. With `drop_tip_z` the block is not set down: the
    hover is at that fingertip height and it is let go from there (over a bin; see bin_drop_tip_z).
    Separated from put() so a caller can ask before it
    grips: a block picked up for a place that turns out to be unreachable has to be put back down again,
    and until it is, the arm is carrying it (2026-09-27: three scatter moves failed this way)."""
    mx, my = parse_place(where_)
    bx, by = map_to_base(mx, my)
    # the block is square to the map when it goes down, so yaw 0; the pitch is the one it is carried at
    j5_put = grip_roll(90 + math.degrees(math.atan2(-bx, by)), 0.0, grasp_pitch) if ALIGN_ROLL else None
    tx, ty = finger_offset(bx, by, j5_put if j5_put is not None else pose[4], grasp_pitch)
    carry = None   # retracted and raised; near the grasp pitch so the held block tilts little
    for dp in (0, 5, -5, 10, -10):
        for radius in (CARRY_RADIUS, 0.16, 0.18, 0.15, 0.19):
            for above in (CARRY_ABOVE_TABLE, 0.06, 0.08, 0.05):
                carry = solve(0, radius, TABLE_Z + above, [grasp_pitch + dp])
                if carry:
                    break
            if carry:
                break
        if carry:
            break
    if carry is None:
        return None
    a = [pose[0]] + carry[1][1:]
    # a pitch unlike the grasp pitch tilts the held block by the difference; allow up to 15 deg (a far
    # grasp at 135 cannot place at 21 cm with 130-140, 2026-09-26) and release a little higher (below)
    soft = CONFIG["soft_limits_deg"]
    within_soft = lambda q: all(soft[str(i + 1)][0] <= q[i] <= soft[str(i + 1)][1] for i in range(4))   # noqa: E731
    # a drop over a bin first tries only poses inside the soft limits (the explore profile lets J2-J4 go to -10 and
    # J1 to its course limit, and a joint asked for -1..-8 reads back -2..-3, 2026-09-25), then any pose
    for strict in ((True, False) if drop_tip_z is not None else (False,)):
        for pitch in sorted({grasp_pitch + d for d in (0, 5, -5, 10, -10, 15, -15)}, key=lambda p: abs(p - grasp_pitch)):
            found = solve(tx, ty, TABLE_Z + BLOCK + HOVER_ABOVE_TOP if drop_tip_z is None else drop_tip_z, [pitch], floor=-3)
            if found is None or (drop_tip_z is None and solve(tx, ty, TABLE_Z + BLOCK / 2, [pitch], floor=-3) is None):
                continue
            hover_pose = found[1]
            if strict and not within_soft(hover_pose):
                continue
            b = [hover_pose[0]] + a[1:]
            try:
                check_path(pose, a)
                check_path(b, hover_pose)
            except ConsoleError:
                continue
            return dict(carry=a, turn=b, hover=hover_pose, pitch=pitch, tx=tx, ty=ty,
                        j5_put=pose[4] if j5_put is None else j5_put,
                        target_map_cm=[round(mx * 100, 2), round(my * 100, 2)], drop_tip_z=drop_tip_z,
                        within_soft_limits=within_soft(hover_pose))
    return None


def can_put(grasp_pitch: float, where_: str, pose: list[int], drop_tip_z: float | None = None) -> bool:
    return plan_put(grasp_pitch, where_, list(pose), drop_tip_z) is not None


def put(where_: str, session: Session | None = None, drop_tip_z: float | None = None) -> dict:
    """Carry the held block so its centre lands on a marker centre (or map x,y cm): retract and rise at the
    grasp pitch, turn only the base, reach out at a pitch within 15 deg of the grasp pitch (the block tilts
    with the gripper), descend, open, lift. With `drop_tip_z` (a bin) it stops at that height and lets go
    there: the block falls the last few centimetres."""
    plan = json.loads(PLAN.read_text())
    if "grip_pose" not in plan:
        raise ConsoleError("nothing gripped by this plan")
    s = status()
    steps = plan_put(plan["pitch"], where_, s["pose"], drop_tip_z)
    if steps is None:
        mx, my = parse_place(where_)
        raise ConsoleError(f"no hover within 15 deg of the grasp pitch {plan['pitch']} over map "
                           f"({mx*100:.1f}, {my*100:.1f}) cm")
    a, b, hover_pose, pitch = steps["carry"], steps["turn"], steps["hover"], steps["pitch"]
    tx, ty, j5_put = steps["tx"], steps["ty"], steps["j5_put"]
    j5 = s["pose"][4]
    log("put_planned", target_map_cm=steps["target_map_cm"], pitch=pitch, hover=hover_pose, j5=j5_put,
        **({} if drop_tip_z is None else dict(drop_tip_z_cm=round(drop_tip_z * 100, 2))))
    with lease(session) as sess:
        sess.pose(a + [j5])                # retract and rise where the block was picked
        sess.pose(b + [j5_put])            # turn the base (and the wrist, to square the block)
        sess.pose(hover_pose + [j5_put])   # reach out over the target
        if drop_tip_z is None:
            tilt = math.radians(abs(pitch - plan["pitch"]))
            corner = BLOCK / 2 * (math.sin(tilt) + math.cos(tilt) - 1)   # how much lower the tilted block's lowest corner is
            released_at = descend_to(sess, tx, ty, TABLE_Z + BLOCK / 2 + 0.003 + corner, pitch)
        else:
            time.sleep(0.4)                # let the arm settle (it sags near full reach) before letting go
            released_at = status()["pose"]
        sess.grip("open")
        lift(sess, hover_pose if drop_tip_z is None else b)   # a bin: back to the carry height, not straight up
    result = dict(target=where_, target_map_cm=steps["target_map_cm"], released_at=released_at)
    log("put", **result)
    return result


def holding_now() -> bool:
    """Whether the fingers are closed on something, by the readback alone."""
    rb = status()["pose"][5]
    g = CONFIG["gripper"]
    return rb is not None and g["holding_if_readback_above_deg"] <= rb <= g["holding_if_readback_below_deg"]


def require_empty_gripper() -> None:
    """Refuse to start a run while a block is in the fingers. A scan taken like that is missing that
    block, and the first hover of the run opens the gripper and drops it wherever the arm then stands
    (2026-09-27: a run scanned all eight views carrying one, and reported three blocks on the table)."""
    if holding_now():
        raise ConsoleError(
            f"夹爪里还夹着东西（J6 读回 {status()['pose'][5]}）。先松开再运行：\n"
            "  python3 -c \"import sys; sys.path.insert(0, 'src/tools'); "
            "from console_step import Session\\nwith Session() as s: s.grip('open')\"")


def back_off(session: Session | None = None) -> None:
    """Re-arm and retreat to the observe pose after a move was stopped by resistance.

    The console holds at its readback and disarms, which can leave the arm low, or just inside the
    platform envelope (2026-09-29: clearance -0.16 cm after J3 met resistance during an approach).
    Carrying on from there means the next approach plans its path from a pose the keep-out model would
    refuse. Rising and retreating first is what makes the next block's attempt a fresh one."""
    if session is not None:
        try:
            session.rearm()
        except ConsoleError as e:
            log("back_off_rearm_failed", reason=str(e))
    observe(session)


def release_safely(session: Session | None = None) -> str:
    """Get a held block onto the table without carrying it anywhere new, and say where it went.

    Two things this has to survive. The console disarms when a segment meets resistance, so it is
    re-armed first: on 2026-09-27 a place descent asked J3 for -9 deg (the joint reaches -4), the console
    held and disarmed, and the recovery was then refused for a stale readback -- the run stopped with the
    block still in the fingers. And when the tip is already down among the blocks, opening there drops the
    block a centimetre or two onto the table, which beats carrying it back across the map."""
    if session is not None:
        session.rearm()
    s = status()
    if tip(s["pose"])[2] <= TABLE_Z + BLOCK * 1.5:
        with lease(session) as sess:
            sess.grip("open")
        log("released_in_place", pose=s["pose"], tip_cm=[round(v * 100, 1) for v in tip(s["pose"])])
        return "在原地松手"
    place(session)
    return "放回抓取位置"


def place(session: Session | None = None) -> None:
    plan = json.loads(PLAN.read_text())
    if "grip_pose" not in plan:
        raise ConsoleError("no grip pose recorded; grasp first")
    with lease(session) as sess:
        check_path(status()["pose"], plan["grip_pose"])
        sess.pose(plan["grip_pose"][:5])
        sess.grip("open")
        lift(sess, plan["hover"])
    log("placed", pose=status()["pose"])


def main(argv: list[str]) -> None:
    commands = ("where", "observe", "retreat", "goto", "locate", "hover", "grasp", "put", "place", "all")
    if not argv or argv[0] not in commands:
        sys.exit(__doc__)
    try:
        if argv[0] == "where":
            where()
        elif argv[0] in ("observe", "retreat"):
            observe()
            where()
        elif argv[0] == "goto":
            goto(argv[1], float(argv[2]) if len(argv) > 2 else 2.0)
        elif argv[0] == "hover":
            hover()
        elif argv[0] == "grasp":
            grasp()
        elif argv[0] == "put":
            put(argv[1])
        elif argv[0] == "place":
            place()
        else:  # locate / all
            if len(argv) < 2:
                sys.exit("give a colour: red / green / blue / yellow")
            with Session() as sess:
                plan = locate(argv[1], sess)
                if argv[0] == "all":
                    hover(sess)
                    if not plan["pitch_verified"]:
                        log("paused", reason="pitch outside the verified range: check the alignment, then run 'grasp'")
                        return
                    grasp(sess)
    except ConsoleError as e:
        log("stopped", reason=str(e))
        sys.exit(1)


if __name__ == "__main__":
    main(sys.argv[1:])
