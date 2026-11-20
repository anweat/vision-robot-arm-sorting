"""The flow stages. Each takes a FlowSession plus optional explicit inputs and writes its own output file.

Frames: everything a stage writes is in the arm base frame, metres (the forward-kinematics frame of
src/spatial/dofbot_kinematics.py). Map coordinates (map_cm) are only kept alongside for reference.
"""

from __future__ import annotations

import json
import math
import urllib.request
import sys
import time
from pathlib import Path
from typing import Callable, Sequence

ROOT = Path(__file__).resolve().parents[2]
for sub in ("", "pipeline", "tasks", "tools", "spatial", "vision"):
    p = str(ROOT / "src" / sub) if sub else str(ROOT / "src")
    if p not in sys.path:
        sys.path.insert(0, p)

import planning                                   # noqa: E402
from planning import config as PC                 # noqa: E402
from planning import planner as PP                # noqa: E402
from planning import scene as PS                  # noqa: E402
from planning.collision_map import CollisionMap   # noqa: E402
from planning.grasp import Target                 # noqa: E402
from planning.release import Destination          # noqa: E402

from .session import FlowSession, load_json       # noqa: E402

OBSERVE = [90, 105, 5, 5, 92]
SIM_CONSOLE = "http://127.0.0.1:8770"


def _use_console(url: str | None) -> str:
    """Point console_step (and the modules that copy its BASE at import) at a console."""
    import console_step
    if url:
        console_step.BASE = url.rstrip("/")
    return console_step.BASE


_SIM: dict = {}


def _is_sim(url: str) -> bool:
    """Whether the console at `url` is the simulator, by what it says it is (/api/bootstrap status.mode), not
    by its port: a simulator on another port was taken for the real arm (2026-11-20) and its scans written into
    the real arm's records. A console that does not answer counts as the real arm (on-site confirmation)."""
    url = url.rstrip("/")
    if url not in _SIM:
        try:
            with urllib.request.urlopen(url + "/api/bootstrap", timeout=5) as r:
                _SIM[url] = json.loads(r.read()).get("status", {}).get("mode") == "simulation"
        except (OSError, ValueError):
            return False
    return _SIM[url]


def _confirm_onsite(url: str, confirm: Callable[[str], str] | None, what: str) -> None:
    if _is_sim(url):
        return
    ask = confirm or input
    answer = ask(f"{what} on the real arm ({url}). Is the area around the arm clear and someone on site? [yes/no] ")
    if str(answer).strip().lower() not in ("yes", "y", "是"):
        raise RuntimeError("not confirmed on site; nothing was sent")


# ------------------------------------------------------------------------------------------------ scan

def scan(sess: FlowSession, console: str | None = None, bins_design: str | None = None, really: bool = False,
         confirm: Callable[[str], str] | None = None, refine: bool = False,
         refine_near: Sequence[float] | None = None) -> dict:
    """Visit the scan poses (src/pipeline/field_scan.py) and save the views to <session>/scan/.
    Moves the arm: needs really=True, and an on-site confirmation unless the console is the simulator.

    `refine`: then, as the verified pipeline does (run.refine), one close view of every block only partly seen
    or weakly detected, the bins kept out of the look poses. A position off by ~1.5 cm across the jaw closes
    the pads on nothing (2026-11-20 simulator rounds, doc 34 §8). `refine_near` (base x, y): a close view of the
    blocks within 4 cm of it however well they were seen -- the retry after an empty grip."""
    url = _use_console(console)
    if not really:
        raise RuntimeError("scan moves the arm: pass really=True (CLI --really)")
    _confirm_onsite(url, confirm, "scan")
    import console_grasp as G
    import field_scan as FS
    if _is_sim(url):                     # the simulator's own logs, never the real arm's
        sim = ROOT / "data" / "sim"
        G.RUNS, G.PLAN, FS.RUNS = sim, sim / "plan.json", sim / "pipeline"
    run = sess.file("scan")
    FS.scan(run)
    if bins_design and FS.has_bin_views(bins_design):
        FS.scan_bins(run, None, bins_design)
    refined = 0
    if refine or refine_near is not None:
        import run as R
        before = len(json.loads((run / "views.json").read_text(encoding="utf-8")))
        weak = R.needs_refine
        if refine_near is not None:
            def chosen(cube, _near=tuple(refine_near)):
                return math.dist(G.map_to_base(cube["map_cm"][0] / 100, cube["map_cm"][1] / 100), _near) < 0.04
            R.needs_refine = chosen
        try:
            with G.Session() as lease:
                if bins_design:
                    R.use_bins(R.locate_bins(run, bins_design))  # look poses keep off the bins; blocks in them are done
                scene = R.build_scene(run)
                R.use_registration(scene["registration"])
                R.refine(run, scene, lease)
        finally:
            R.needs_refine = weak
        refined = len(json.loads((run / "views.json").read_text(encoding="utf-8"))) - before
    views = json.loads((run / "views.json").read_text(encoding="utf-8"))
    out = dict(scan_dir=str(run), console=url, views=len(views), refine_views=refined,
               bin_views=(run / "bin_views.json").exists())
    sess.log("scan", **out)
    return out


# ---------------------------------------------------------------------------------------------- locate

def locate(sess: FlowSession, scan_dir: str | Path | None = None, bins_design: str | None = None,
           apply_registration_yaw: bool = False, tuning=None) -> dict:
    """Views -> located.json: blocks (and bins) in the base frame with a position sigma.

    Blocks: src/pipeline/run.py build_scene (orthophoto + detector + stickers) in map coordinates, placed
    in the base frame by this scan's own marker registration (x, y; the map moves between sessions,
    2026-11-15 camera check). Its yaw is applied only when asked (the pipeline never applies it).
    Bins: src/pipeline/bins.py from the bin views, else their planned places."""
    import run as R
    t = tuning or planning.tuning.default()
    run_dir = Path(scan_dir) if scan_dir else sess.file("scan")
    scene = R.build_scene(run_dir)
    reg = scene.get("registration") or {}
    if reg.get("views"):
        ox, oy = reg["map_origin_base_cm"][0] / 100, reg["map_origin_base_cm"][1] / 100
        spread = max(reg.get("spread_cm", [0, 0])[:2]) / 100
        placed_by = "scan registration"
    else:
        ox, oy = PC.map_placement()
        spread = 0.0
        placed_by = f"map placement ({PC.MAP_NAME}, touch calibration)"
    yaw = math.radians(reg.get("yaw_deg", 0.0)) if apply_registration_yaw and reg.get("views") else 0.0
    c, s = math.cos(yaw), math.sin(yaw)
    sigma = math.hypot(float(t.errors["target_xy_m"]), spread)
    objects = []
    for i, cube in enumerate(scene.get("cubes", []), start=1):
        mx, my = cube["map_cm"][0] / 100, cube["map_cm"][1] / 100
        objects.append(dict(
            id=i, name=f"b{i}", kind="block", **{"class": cube.get("category")},
            centre_m=[round(ox + c * mx - s * my, 5), round(oy + s * mx + c * my, 5)],
            yaw_deg=round(cube.get("yaw_deg", 0.0) + math.degrees(yaw), 2), size_m=[PC.BLOCK] * 3,
            sigma_xy_m=round(sigma, 4), sigma_yaw_deg=float(t.errors["target_yaw_deg"]),
            score=cube.get("score"), colour=cube.get("colour"), sticker=cube.get("sticker"),
            views=(cube.get("detector") or {}).get("views"),
            map_cm=cube["map_cm"], source=cube.get("found_by") or cube.get("source")))
    if bins_design:
        found = R.locate_bins(run_dir, bins_design)
        g = found.design["geometry_mm"]
        for b in found.bins.values():
            objects.append(dict(
                id=len(objects) + 1, name=f"bin{b.number}", kind="bin", **{"class": b.category}, design=bins_design,
                centre_m=[round(float(b.xy[0]), 5), round(float(b.xy[1]), 5)], yaw_deg=round(float(b.yaw_deg), 2),
                size_m=[g["outer"] / 1000, g["outer"] / 1000, g["rim_top_above_table"] / 1000],
                sigma_xy_m=round(float(t.errors["obstacle_xy_m"]) if b.source == "markers" else 0.01, 4),
                source=b.source))
    out = dict(scan_dir=str(run_dir), registration=reg, placed_by=placed_by,
               registration_yaw_applied=bool(yaw), table_z_m=t.table_z, objects=objects, t=time.time())
    sess.save("located.json", out)
    sess.log("locate", scan_dir=str(run_dir), blocks=sum(o["kind"] == "block" for o in objects),
             bins=sum(o["kind"] == "bin" for o in objects), placed_by=placed_by)
    return out


# ------------------------------------------------------------------------------------------------- map

def build_map(sess: FlowSession, located=None, extra_boxes: Sequence[dict] = (), cell_m: float | None = None,
              tuning=None) -> CollisionMap:
    """located.json (+ extra boxes {name, centre_m, size_m, yaw_deg}) -> collision_map.json.
    Blocks are upright boxes; bins are their four walls and floor plate under one label."""
    t = tuning or planning.tuning.default()
    loc = load_json(located) if located is not None else sess.load("located.json")
    m = CollisionMap.empty(cell_m=cell_m or float(t.map.get("cell_m", 0.0025)), table_z_m=t.table_z)
    for o in loc["objects"]:
        if o.get("noise"):                 # a scan-noise detection (steps.scan min_views): not an obstacle either
            continue
        meta = {k: v for k, v in o.items() if k in ("id", "name", "kind", "class", "centre_m", "size_m", "yaw_deg",
                                                    "sigma_xy_m", "sigma_yaw_deg", "design")}
        cx, cy = o["centre_m"]
        if o["kind"] == "bin":
            boxes = PS.bin_boxes((cx, cy), o.get("yaw_deg", 0.0), o["name"], o.get("design", PC.BINS_NAME))
        else:
            sx, sy, sz = o["size_m"]
            boxes = [PS.Box(o["kind"], o["name"], (cx, cy, t.table_z + sz / 2), (sx / 2, sy / 2, sz / 2),
                            o.get("yaw_deg", 0.0))]
        m.add_object(boxes, meta)
    for i, e in enumerate(extra_boxes):
        sx, sy, sz = e["size_m"]
        b = PS.Box(e.get("kind", "obstacle"), e.get("name", f"extra{i + 1}"),
                   (e["centre_m"][0], e["centre_m"][1], t.table_z + sz / 2), (sx / 2, sy / 2, sz / 2), e.get("yaw_deg", 0.0))
        m.add_object([b], dict(name=b.name, kind=b.kind, centre_m=list(e["centre_m"]), size_m=list(e["size_m"]),
                               yaw_deg=e.get("yaw_deg", 0.0)))
    m.source = dict(made_by="flow.build_map", located=str(sess.file("located.json")) if located is None else "given",
                    t=time.time())
    m.save(sess.file("collision_map.json"))
    sess.log("map", objects=len(m.objects), shape=list(m.shape), cell_m=m.cell)
    return m


# ------------------------------------------------------------------------------------------------ plan

def plan(sess: FlowSession, task, cmap=None) -> dict:
    """Collision map + task -> plan.json (pick, place, return MotionPlans).

    task: {"pick": <object id or name>, "to": {"bin": <object>} | {"table_m": [x, y], "yaw_deg": null},
           "start": [J1..J5] (default the observe pose), "start_j6": 30, "return_to": [J1..J5] | null,
           "tuning": {overrides of config/planning_tuning.json}}"""
    task = load_json(task)
    m = CollisionMap.load(cmap if cmap is not None else sess.file("collision_map.json"))
    w = planning.world(m, overrides=task.get("tuning"))
    t = w.tuning
    start = list(task.get("start") or OBSERVE)
    target = Target.from_map(m, task["pick"])
    to = task["to"]
    if "bin" in to:
        b = m.object(to["bin"])
        dest = Destination(b["centre_m"][0], b["centre_m"][1], None, t.table_z + b["size_m"][2], b.get("name", "bin"))
    else:
        dest = Destination(to["table_m"][0], to["table_m"][1], to.get("yaw_deg"), None, "table")
    pick, place = PP.plan_pick_and_place(w, start, target, dest, float(task.get("start_j6", 30)),
                                         avoid_j5=[float(v) for v in task.get("avoid_j5", ())])
    out = dict(task=task, ok=False, tuning=t.to_dict(), map=dict(objects=len(m.objects), shape=list(m.shape),
                                                                 source=m.source), plans=[pick.to_dict()])
    if pick.ok:
        wp = w.without(target.label)
        out["plans"].append(place.to_dict())
        if place.ok:
            ret = task.get("return_to", start)
            if ret:
                # the way back keeps the clearances the release was planned with (planner.plan_place tiers),
                # and the block just set down is between the open fingers at the back-off pose: it gets the
                # touch margin there, as plan_place's own back-off step gives it
                after = PP.touching(wp) if place.summary.get("place_tier") == "touching" else wp
                if dest.rim_z is None:
                    rel = place.summary["release"]
                    after = after.with_extra(PS.block(rel["block_centre_cm"][0] / 100, rel["block_centre_cm"][1] / 100,
                                                      rel["block_yaw_deg"], "placed", target.size), t.target_touch())
                back = PP.plan_transit(after, place.end, ret, place.steps[-1].j6)
                out["plans"].append(back.to_dict())
                out["ok"] = back.ok
            else:
                out["ok"] = True
    out["steps"] = [dict(plan=p["kind"], index=i, **s) for p in out["plans"] for i, s in enumerate(p["steps"])]
    out["reasons"] = [r for p in out["plans"] if not p["ok"] for r in p["reasons"]]
    sess.save("task.json", task)
    sess.save("plan.json", out)
    sess.log("plan", ok=out["ok"], steps=len(out["steps"]), pick=task["pick"], to=task["to"],
             reasons=out["reasons"][:3])
    return out


# --------------------------------------------------------------------------------------------- execute

class DryRun:
    """Pretends: the readback is the command. For checking a plan file end to end without an arm.
    `tracking(previous5, command5) -> reached5` stands in for servos that stop short (tests of the settle)."""
    name = "dry-run"

    def __init__(self, start6: Sequence[float], tracking: Callable | None = None):
        self.pose = [float(v) for v in start6]
        self.tracking = tracking

    def readback(self) -> list:
        return list(self.pose)

    def move(self, cmd5) -> None:
        cmd5 = [float(v) for v in cmd5]
        self.pose[:5] = self.tracking(self.pose[:5], cmd5) if self.tracking else cmd5

    def gripper(self, action: str | None, j6: float) -> str:
        self.pose[5] = float(j6)
        return "dry-run"

    def grip_result(self) -> str | None:
        return "holding"

    def close(self) -> None:
        pass


class Console:
    """The teleop console (real arm on :8765, simulator on :8770) through src/tools/console_step.Session.
    Moves go through /api/pose (the console plans its own 5-degree segments and keep-out); gripper actions
    the console has (wide / open / grip) are sent as such, other openings are reached by J6 jogs."""
    name = "console"
    ACTIONS = ("wide", "open", "grip")

    def __init__(self, url: str | None):
        self.url = _use_console(url)
        import console_step
        self.cs = console_step
        self.session = console_step.Session(owner="flow-execute").__enter__()

    def readback(self) -> list:
        return list(self.cs.status()["pose"])

    def move(self, cmd5) -> None:
        target = [int(round(v)) for v in cmd5]
        if all(abs(a - b) <= 1 for a, b in zip(self.readback()[:5], target)):
            return                       # already there: the console refuses a preview of no motion
        self.session.pose(target)

    def gripper(self, action: str | None, j6: float) -> str:
        if action in self.ACTIONS:
            self.session.grip(action)
            return action
        left = int(round(j6 - self.readback()[5]))
        while left:
            d = next(s for s in (10, 5, 2, 1) if abs(left) >= s) * (1 if left > 0 else -1)
            self.session.jog(6, d)
            left -= d
        return f"jog J6 to {round(j6)}"

    def grip_result(self) -> str | None:
        return (self.cs.status().get("grip") or {}).get("result")

    def close(self) -> None:
        self.session.__exit__(None, None, None)


def settle_settings(overrides: dict | None = None) -> dict:
    """planning_tuning.json `execute` (readback correction), with overrides."""
    from planning import tuning as TU
    s = dict(TU.load().raw.get("execute", {}))
    s = {k: v for k, v in s.items() if not k.startswith("_")}
    s.update(overrides or {})
    return dict(dict(settle_deg=1.5, extra_deg=3.0, tries=2, joints=[1, 2, 3, 4, 5]), **s)


def _settle(backend, cmd5: Sequence[float], rb: list, st: dict) -> tuple[list, list]:
    """Re-command each joint whose readback is >= settle_deg off by command + (command - readback), the extra
    capped at extra_deg, up to `tries` times. Servos 2-4 stop short when they lift against gravity (teleop
    readbacks 2026-09-24/25: J2 -1.3 / -2.0 deg on raises at 5-15 / >15 cm lever, J3 -1.0, J4 +1.0 lowering),
    which at 20 cm is 0.5-0.7 cm at the pads. The plan's checks are for `cmd5`; the corrections aim at it."""
    done = []
    for _ in range(int(st["tries"])):
        err = {j: cmd5[j - 1] - rb[j - 1] for j in st["joints"] if rb[j - 1] is not None}
        if not err or max(abs(e) for e in err.values()) < st["settle_deg"]:
            break
        nxt = [float(c) for c in cmd5]
        for j, e in err.items():
            if abs(e) >= st["settle_deg"]:
                nxt[j - 1] = cmd5[j - 1] + max(-st["extra_deg"], min(st["extra_deg"], e))
        backend.move(nxt)
        rb = backend.readback()
        done.append(dict(sent=[round(v, 1) for v in nxt], readback=rb))
    return rb, done


def execute(sess: FlowSession, plan_src=None, console: str | None = None, really: bool = False,
            confirm: Callable[[str], str] | None = None, backend=None, tolerance_deg: float = 4.0,
            start_tolerance_deg: float = 3.0, stop_after: int | None = None, settle: dict | bool | None = None,
            phases: Sequence[str] | None = None, grip_must_hold: bool = True) -> dict:
    """Run plan.json step by step and log each one to execution.jsonl.

    Without really=True (or with backend=DryRun) nothing moves. Before anything is sent: the plan must be ok
    with every step's checks ok, and the arm must stand at the plan's start (within start_tolerance_deg).
    After each move the readback is corrected towards the command (`settle`, planning_tuning.json `execute`;
    False turns it off), then compared with it; a grip must read 'holding'. Any miss stops the run where it is
    (no automatic recovery: use the console to back off).

    phases: run only the steps of these plan kinds (`pick`, `place`, `transit`), e.g. the grasp alone and the
    place later (docs/atlas-200i-dk-a2/32 §2); the arm must then stand where the step before the first selected
    one ends. grip_must_hold=False: a grip that does not hold is recorded, not a stop (the check step judges
    it). The returned dict carries the logged rows under `rows`."""
    p = load_json(plan_src) if plan_src is not None else sess.load("plan.json")
    if not p["ok"]:
        raise RuntimeError(f"the plan is not ok: {p.get('reasons', [])[:3]}")
    bad = [s["name"] for s in p["steps"] if not s["check"]["ok"]]
    if bad:
        raise RuntimeError(f"steps with failed checks: {bad}")
    start = list(p["task"].get("start") or OBSERVE)
    chosen = [i for i, s in enumerate(p["steps"]) if phases is None or s["plan"] in phases]
    if not chosen:
        raise RuntimeError(f"the plan has no steps of {list(phases)}")
    if chosen[0] > 0:
        start = list(p["steps"][chosen[0] - 1]["command"][:5])
    if backend is None:
        if not really:
            backend = DryRun(start + [float(p["task"].get("start_j6", 30))])
        else:
            url = _use_console(console)
            _confirm_onsite(url, confirm, f"execute {len(p['steps'])} steps")
            backend = Console(url)
    st = None if settle is False else settle_settings(settle if isinstance(settle, dict) else None)
    log_name = "execution.jsonl"
    result = dict(backend=backend.name, steps=len(chosen), done=0, ok=False, stopped=None,
                  phases=list(phases) if phases else None)
    rows = []
    try:
        rb = backend.readback()
        off = max(abs(a - b) for a, b in zip(rb[:5], start))
        if off > start_tolerance_deg:
            raise RuntimeError(f"the arm stands at {[round(v) for v in rb[:5]]}, the plan starts at {start} "
                               f"({off:.0f} deg off): plan again from here")
        sess.append(log_name, dict(event="start", backend=backend.name, readback=rb, plan=str(plan_src or "plan.json"),
                                   phases=result["phases"]))
        for n, i in enumerate(chosen):
            s = p["steps"][i]
            if stop_after is not None and n >= stop_after:
                result["stopped"] = f"stop_after {stop_after}"
                break
            began = time.time()
            if s["kind"] == "move":
                backend.move(s["command"][:5])
                sent = s["command"][:5]
            else:
                sent = backend.gripper(s.get("action"), s["command"][5])
            rb = backend.readback()
            fixes = []
            if s["kind"] == "move" and st is not None:
                first = rb
                rb, fixes = _settle(backend, s["command"][:5], rb, st)
            row = dict(event="step", i=i, plan=s["plan"], name=s["name"], kind=s["kind"], sent=sent,
                       command=s["command"], readback=rb, seconds=round(time.time() - began, 2))
            if fixes:
                row.update(first_readback=first, settle=fixes)
            if s["kind"] == "move":
                dev = max(abs(a - b) for a, b in zip(rb[:5], s["command"][:5]))
                row["deviation_deg"] = round(dev, 1)
                if dev > tolerance_deg:
                    row["problem"] = f"readback {dev:.0f} deg off the command"
            elif s.get("action") == "grip":
                g = backend.grip_result()
                row["grip_result"] = g
                if g not in ("holding", "dry-run") and grip_must_hold:
                    row["problem"] = f"grip reads {g!r}, not holding"
            sess.append(log_name, row)
            rows.append(row)
            if "problem" in row:
                result["stopped"] = f"step {i} {s['name']}: {row['problem']}"
                break
            result["done"] = n + 1
        else:
            result["ok"] = True
    finally:
        backend.close()
    sess.append(log_name, dict(event="end", **result))
    sess.log("execute", **result)
    return dict(result, rows=rows)


# ---------------------------------------------------------------------------------------------- verify

def verify(sess: FlowSession, after, before=None, plan_src=None, tolerance_m: float = 0.015) -> dict:
    """Compare a fresh located.json (`after`, from scan + locate after the run) with the one planned on:
    the picked block left its place; it is at the table destination (or no longer on the table, for a
    bin); every other block is where it was. Tolerance: tolerance_m on positions."""
    a = load_json(after)
    b = load_json(before) if before is not None else sess.load("located.json")
    p = load_json(plan_src) if plan_src is not None else sess.load("plan.json")
    key = p["task"]["pick"]
    picked = next(o for o in b["objects"] if o["id"] == key or o["name"] == key)
    blocks_after = [o for o in a["objects"] if o["kind"] == "block"]

    def near(xy, objs):
        best = min(objs, key=lambda o: math.dist(o["centre_m"], xy), default=None)
        return (best, math.dist(best["centre_m"], xy)) if best else (None, math.inf)

    checks = []
    o, d = near(picked["centre_m"], blocks_after)
    checks.append(dict(check="picked block left its place", ok=d > tolerance_m,
                       nearest_cm=None if o is None else round(d * 100, 2)))
    to = p["task"]["to"]
    place = next((q for q in p["plans"] if q["kind"] == "place"), None)
    if "table_m" in to and place and place["ok"]:
        want = [v / 100 for v in place["summary"]["target_base_cm"]]
        o, d = near(want, blocks_after)
        checks.append(dict(check="a block stands at the destination", ok=d <= tolerance_m,
                           offset_cm=None if o is None else round(d * 100, 2),
                           yaw_deg=None if o is None else o.get("yaw_deg")))
    else:
        n_before = sum(1 for q in b["objects"] if q["kind"] == "block")
        checks.append(dict(check="one block fewer on the table (it went into the bin)",
                           ok=len(blocks_after) == n_before - 1, before=n_before, after=len(blocks_after)))
    moved = []
    for q in b["objects"]:
        if q["kind"] != "block" or q is picked:
            continue
        o, d = near(q["centre_m"], blocks_after)
        if d > tolerance_m:
            moved.append(dict(name=q["name"], nearest_cm=None if o is None else round(d * 100, 2)))
    checks.append(dict(check="the other blocks stayed", ok=not moved, moved=moved))
    out = dict(ok=all(c["ok"] for c in checks), checks=checks, tolerance_cm=tolerance_m * 100,
               picked=picked["name"], to=to)
    sess.save("verify.json", out)
    sess.log("verify", ok=out["ok"], failed=[c["check"] for c in checks if not c["ok"]])
    return out
