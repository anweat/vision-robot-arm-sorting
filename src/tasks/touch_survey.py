"""Touch the bare table with the closed fingertips at poses spread over reach, azimuth and pitch (doc 33).

Every touch is the console's fine probe (`/api/touch`): straight down in 1 mm steps at a fixed pitch, joints
read from the raw servo position, held the moment the tip stops following its command. The contact pose that
comes back says "the lowest finger point is on the table here", which is one equation in the joint zero
offsets, the arm's sag and the table height; many such poses at different reach and pitch separate them
(src/tools/fit_touch.py). The same run also tries the reach and pitch ends that only the model has seen
(doc 30 §11.2: ~12.5 cm near the base, ~29-31 cm fully stretched, steep pitches near the base).

    python3 src/tasks/touch_survey.py plan [--set core|edge|all]     # list the touch poses and checks, move nothing
    python3 src/tasks/touch_survey.py air                            # free-air probe high above the table: lag statistics
    python3 src/tasks/touch_survey.py run [--set core] [--only 0,3]  # touch, record, lift (moves the arm)

The table must be bare under the touch points (blocks taken off; the paper map may stay). Run on the board
next to the console (127.0.0.1:8765) with an operator watching. Results: data/runs/touch/<time>/touches.jsonl.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
for sub in ("tools", "spatial", ""):
    sys.path.insert(0, str(ROOT / "src" / sub))

import dofbot_kinematics as kin  # noqa: E402
import gripper_geometry as gg  # noqa: E402
from console_step import ConsoleError, Session, status  # noqa: E402
from planning import checks as CK  # noqa: E402
from planning import config as PC  # noqa: E402
from planning import tuning as TU  # noqa: E402
from planning.collision_map import CollisionMap  # noqa: E402

RUNS = ROOT / "data" / "runs" / "touch"
J6_CLOSED = 168            # the old touch calibration's closing (docs/05 2026-09-25): an empty gripper reaches it
J5 = PC.ROLL_ZERO          # jaw across the arm plane: both fingertips level
HOVER_CM = 2.5             # lowest finger point this far above the model table before a touch
PROBE_CM = 4.0             # how far a touch may go down: the model table may be off by a centimetre
OBSERVE = [90, 105, 5, 5]
WORLD = CK.World(CollisionMap.empty(), TU.load())
LIMITS = {j: PC.LIMITS[j] for j in (1, 2, 3, 4)}
DELTA = gg.delta_for_j6(J6_CLOSED)

SETS = {
    # spread for the fit: three azimuths x four reaches x up to three pitches
    "core": dict(j1=(60, 90, 120), r_cm=(15, 19, 23, 27), pitches=3),
    # the ends the model predicts and the arm has not done (doc 30 §11.2)
    "edge": dict(j1=(90,), r_cm=(13, 14, 29, 30), pitches=2),
}


def lowest(q4) -> float:
    return gg.lowest_z(list(q4) + [J5, J6_CLOSED], DELTA)


def pose_for(j1: float, r: float, z_low: float, pitch: float):
    """J1-J4 (float) with the FK tip on the ray (J1, r) and the lowest finger point at `z_low`."""
    a = math.radians(j1)
    x, y = r * math.cos(a), r * math.sin(a)
    zt = z_low + 0.01
    for _ in range(6):
        q = kin.inverse(x, y, zt, pitch)
        if q is None:
            return None
        err = lowest(q) - z_low
        if abs(err) < 2e-4:
            return q
        zt -= err
    return None


def locked_hover(p: dict, above_m: float = HOVER_CM / 100) -> list | None:
    """Hover for a J2-held touch: the touch pose's J2 (whole degrees, as goto sends it) kept, J3/J4 lifting the
    FK tip straight up by `above_m`; the console's lock_j2 descent then runs back down the same line."""
    qt = p["touch"]
    j2 = float(round(qt[1]))
    x, y, z = kin.forward([qt[0], j2, qt[2], qt[3]])["tip"]
    q = kin.inverse_j2_fixed(qt[0], j2, x, y, z + above_m, qt)
    if q is None or not all(LIMITS[j][0] <= v <= LIMITS[j][1] for j, v in zip((1, 2, 3, 4), q)):
        return None
    return [int(round(v)) for v in q] + [J5, J6_CLOSED]


def check(q4, table_margin: float = 0.0):
    c = CK.pose_check(WORLD, list(q4) + [J5], DELTA, table_margin=table_margin)
    return c


def candidates(set_name: str) -> list[dict]:
    spec = SETS[set_name]
    table = WORLD.tuning.table_z
    out = []
    for j1 in spec["j1"]:
        for r_cm in spec["r_cm"]:
            ok = []
            for p in range(95, 195):
                touch = pose_for(j1, r_cm / 100, table, p)
                hover = pose_for(j1, r_cm / 100, table + HOVER_CM / 100, p)
                if touch is None or hover is None:
                    continue
                if not all(LIMITS[j][0] + 1 <= v <= LIMITS[j][1] - 1 for j, v in zip((1, 2, 3, 4), touch)):
                    continue
                hv = [int(round(v)) for v in hover]
                ct, ch = check(touch, table_margin=-0.02), check(hv, table_margin=0.01)
                if not (ct.limits_ok and min(ct.platform_mesh, ch.platform_mesh) >= 0.002 and ch.ok
                        and ct.arm_body >= 0):
                    continue
                ok.append(dict(pitch=p, touch=[round(v, 2) for v in touch], hover=hv + [J5, J6_CLOSED],
                               mesh_cm=round(min(ct.platform_mesh, ch.platform_mesh) * 100, 2)))
            if not ok:
                out.append(dict(j1=j1, r_cm=r_cm, skipped="no pitch inside the limits and clear of the platform"))
                continue
            lo, hi = ok[0]["pitch"], ok[-1]["pitch"]
            n = spec["pitches"]
            want = [lo + 3, hi - 3] if n == 2 else [lo + 3, (lo + hi) / 2, hi - 3]
            if hi - lo < 8:
                want = [(lo + hi) / 2]
            seen = set()
            for w in want:
                best = min(ok, key=lambda o: abs(o["pitch"] - w))
                if best["pitch"] in seen:
                    continue
                seen.add(best["pitch"])
                out.append(dict(j1=j1, r_cm=r_cm, pitch_range=[lo, hi], **best))
    return out


def joint_path_ok(a4, b4) -> tuple[bool, str]:
    c = CK.move_check(WORLD, list(a4)[:4] + [J5], list(b4)[:4] + [J5], DELTA, table_margin=0.01)
    return c.ok, c.failed()


def show_plan(points: list[dict]) -> None:
    for i, p in enumerate(points):
        if "skipped" in p:
            print(f"--  J1 {p['j1']:>3}  r {p['r_cm']:>2} cm  {p['skipped']}")
            continue
        if "skipped_near_block_cm" in p:
            print(f"{i:>2}  J1 {p['j1']:>3}  r {p['r_cm']:>2} cm  pitch {p['pitch']:>3}  skipped: block {p['skipped_near_block_cm']} cm away")
            continue
        print(f"{i:>2}  J1 {p['j1']:>3}  r {p['r_cm']:>2} cm  pitch {p['pitch']:>3} (open {p['pitch_range'][0]}-{p['pitch_range'][1]})"
              f"  hover {p['hover'][:4]}  touch {p['touch']}  platform {p['mesh_cm']:+.2f} cm")


class Survey(Session):
    def goto(self, pose):
        for k in range(10):          # the board's readback takes 0.4-2 s: a goto right after a move can find it stale
            try:
                return self._post("/api/goto", {"pose": [int(round(v)) for v in pose]}, timeout=180)
            except ConsoleError as e:
                if "新鲜的回读" not in str(e) or k == 9:
                    raise
                time.sleep(0.5)

    def touch(self, max_cm=PROBE_CM, step_mm=1.0, stall_mm=3.0, lock_j2=False):
        return self._post("/api/touch", {"max_cm": max_cm, "step_mm": step_mm, "stall_mm": stall_mm,
                                         "lock_j2": lock_j2}, timeout=400)


def here() -> list:
    return status()["pose"]


def travel(s: Survey, target6: list) -> None:
    """Joint-linear to target if the gripper stays clear of the table and platform on the way, else via observe."""
    now = here()
    ok, why = joint_path_ok(now, target6)
    if not ok:
        print(f"   via observe ({why})", flush=True)
        if lowest(now[:4]) < WORLD.tuning.table_z + 0.03:          # low: straight up first
            q = kin.forward(now[:4])
            up = kin.inverse(*q["tip"][:2], q["tip"][2] + 0.04, q["tip_pitch_deg"][0])
            if up is not None:
                s.goto([int(round(v)) for v in up] + now[4:6])
        s.goto(OBSERVE + [J5, J6_CLOSED])
    s.goto(target6)


def close_gripper(s: Survey) -> float:
    s.goto(here()[:5] + [J6_CLOSED])
    rb = here()[5]
    if rb is None or rb < J6_CLOSED - 6:
        raise ConsoleError(f"夹爪没有合到 {J6_CLOSED}（读回 {rb}）：手指间有东西或指令被忽略，触地点不可信")
    return rb


def record(out: Path, row: dict) -> None:
    with out.open("a") as f:
        f.write(json.dumps(row, ensure_ascii=False) + "\n")


def run(points: list[dict], out_dir: Path, air: bool = False) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    out = out_dir / "touches.jsonl"
    print(f"writing {out}", flush=True)
    with Survey(owner="touch-survey") as s:
        close_gripper(s)
        for i, p in enumerate(points):
            if "skipped" in p or "skipped_near_block_cm" in p:
                continue
            began = time.time()
            print(f"[{i}] J1 {p['j1']} r {p['r_cm']} pitch {p['pitch']}{' J2 held' if p.get('lock_j2') else ''}: "
                  f"hover {(p['hover_locked'] if p.get('lock_j2') else p['hover'])[:4]}", flush=True)
            lock = bool(p.get("lock_j2"))
            hover = p["hover_locked"] if lock else p["hover"]
            travel(s, hover)
            j6 = here()[5]
            if j6 is None or j6 < J6_CLOSED - 6:
                raise ConsoleError(f"夹爪读回 {j6}：不再是闭合状态，停止")
            got = s.touch(max_cm=p.get("max_cm", PROBE_CM), lock_j2=lock)
            q = got["pose"][:4]
            row = dict(i=i, plan=p, result={k: v for k, v in got.items() if k != "trace"}, trace=got["trace"],
                       readback_int=here(), j6_readback=j6, lowest_model_m=round(lowest(q), 5),
                       fk_tip_m=[round(v, 5) for v in kin.forward(q)["tip"]],
                       seconds=round(time.time() - began, 1), air=air)
            record(out, row)
            print(f"    {'CONTACT' if got['contacted'] else 'no contact'}: {got['why']}; lowest finger "
                  f"{row['lowest_model_m'] * 100:+.2f} cm (model table {WORLD.tuning.table_z * 100:+.2f}), "
                  f"pose {[round(v, 2) for v in q]}, {got['steps']} steps, {row['seconds']} s", flush=True)
            s.goto(hover)                                          # straight back up the way it came
        travel(s, OBSERVE + [J5, J6_CLOSED])
        s.goto(OBSERVE + [J5, 30])                                 # open again: the usual observe pose


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd", choices=("plan", "air", "run", "repeat"))
    ap.add_argument("--lock-j2", action="store_true", help="hold J2 during each descent (doc 33 §3)")
    ap.add_argument("--n", type=int, default=3, help="repeat: touches per mode at each --only point")
    ap.add_argument("--avoid", type=Path, help="a located.json (flow scan): leave out touches near its blocks")
    ap.add_argument("--clear-cm", type=float, default=6.0, help="--avoid: least distance, FK tip to block centre")
    ap.add_argument("--set", default="core", choices=(*SETS, "all"))
    ap.add_argument("--only", default="", help="comma-separated indices from `plan`")
    a = ap.parse_args()
    names = list(SETS) if a.set == "all" else [a.set]
    points = [p for n in names for p in candidates(n)]
    if a.avoid:
        blocks = [o["centre_m"] for o in json.loads(a.avoid.read_text())["objects"] if o.get("kind") == "block"]
        for p in points:
            if "skipped" in p:
                continue
            tip = kin.forward(p["touch"])["tip"]
            near = min((math.dist(tip[:2], b) for b in blocks), default=9)
            if near < a.clear_cm / 100:
                p["skipped_near_block_cm"] = round(near * 100, 1)
    if a.only:
        keep = {int(x) for x in a.only.split(",")}
        points = [p for i, p in enumerate(points) if i in keep]
    for p in points:
        if "skipped" not in p:
            p["hover_locked"] = locked_hover(p)
            p["lock_j2"] = a.lock_j2 and p["hover_locked"] is not None
    if a.cmd == "plan":
        show_plan(points)
        print(f"model table {WORLD.tuning.table_z * 100:+.2f} cm (lowest finger point, FK frame); "
              f"hover {HOVER_CM} cm above it; probe up to {PROBE_CM} cm")
        return
    stamp = time.strftime("%Y%m%d_%H%M%S")
    if a.cmd == "repeat":
        # the same points n times each way, interleaved: repeatability and the two descent modes side by side
        seq = []
        for p in points:
            if "skipped" in p or "skipped_near_block_cm" in p or p["hover_locked"] is None:
                continue
            for k in range(a.n):
                for lock in (False, True):
                    seq.append(dict(p, lock_j2=lock, rep=k))
        run(seq, RUNS / f"{stamp}_repeat")
        return
    if a.cmd == "air":
        # well above the table: every step is free, so the lag statistics are the threshold's baseline; once
        # each way
        z = WORLD.tuning.table_z + 0.07
        touch = pose_for(90, 0.19, z, 150)
        p = dict(j1=90, r_cm=19, pitch=150, max_cm=2.0, touch=[round(v, 2) for v in touch],
                 hover=[int(round(v)) for v in touch] + [J5, J6_CLOSED])
        p["hover_locked"] = locked_hover(p, 0.0)
        run([dict(p, lock_j2=False), dict(p, lock_j2=True)], RUNS / f"{stamp}_air", air=True)
        return
    run(points, RUNS / stamp)


if __name__ == "__main__":
    main()
