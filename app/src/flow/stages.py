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
from planning import release as RL                # noqa: E402
from planning import scene as PS                  # noqa: E402
from planning.collision_map import CollisionMap   # noqa: E402
from planning.grasp import Target                 # noqa: E402
from planning.release import Destination          # noqa: E402

from . import moves as MV                          # noqa: E402
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
         refine_near: Sequence[float] | None = None, observe: str | dict = "field") -> dict:
    """Visit the poses of the observation script `observe` (config/observe/<name>.yaml, src/pipeline/observe.py;
    default the field scan) and save the views to <session>/scan/.
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
    FS.scan(run, script=observe)
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
               observe=observe if isinstance(observe, str) else observe.get("name"),
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
    # bins first, then blocks from the bottom up: what stands on something keeps it as the cell's "under"
    order = sorted(loc["objects"], key=lambda o: (o["kind"] != "bin", float(o.get("bottom_m", 0.0))))
    for o in order:
        if o.get("noise"):                 # a scan-noise detection (steps.scan min_views): not an obstacle either
            continue
        meta = {k: v for k, v in o.items() if k in ("id", "name", "kind", "class", "centre_m", "size_m", "yaw_deg",
                                                    "sigma_xy_m", "sigma_yaw_deg", "design", "bottom_m")}
        cx, cy = o["centre_m"]
        if o["kind"] == "bin":
            boxes = PS.bin_boxes((cx, cy), o.get("yaw_deg", 0.0), o["name"], o.get("design", PC.BINS_NAME))
        else:
            sx, sy, sz = o["size_m"]
            z0 = t.table_z + float(o.get("bottom_m", 0.0))     # bottom_m: on a block (0.03), in a bin (0.002)
            boxes = [PS.Box(o["kind"], o["name"], (cx, cy, z0 + sz / 2), (sx / 2, sy / 2, sz / 2),
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

    task: {"pick": <object id or name>,
           "to": {"bin": <object>, "mode": "set" | "drop"}     set (default): lowered inside the walls as far as
                                                                 the gripper fits; drop: let go above the rim
                 | {"stack_on": <block>}                         set down on that block, faces aligned with it
                 | {"table_m": [x, y], "yaw_deg": null},
           "start": [J1..J5] (default the observe pose), "start_j6": 30, "return_to": [J1..J5] | null,
           "tuning": {overrides of config/planning_tuning.json}}"""
    task = load_json(task)
    m = CollisionMap.load(cmap if cmap is not None else sess.file("collision_map.json"))
    w = planning.world(m, overrides=task.get("tuning"))
    t = w.tuning
    start = list(task.get("start") or OBSERVE)
    target = Target.from_map(m, task["pick"], t.table_z)
    dest = destination(m, t, task["to"], target)
    pick, place = PP.plan_pick_and_place(w, start, target, dest, float(task.get("start_j6", 30)),
                                         avoid_j5=[float(v) for v in task.get("avoid_j5", ())])
    out = dict(task=task, ok=False, tuning=t.to_dict(), map=dict(objects=len(m.objects), shape=list(m.shape),
                                                                 source=m.source,
                                                                 file=str(cmap) if isinstance(cmap, (str, Path))
                                                                 else str(sess.file("collision_map.json"))),
               plans=[pick.to_dict()])
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
                after = RL.world_for(after, dest)
                if dest.mode == "set":
                    rel = place.summary["release"]
                    c = rel["block_centre_cm"]
                    after = after.with_extra(PS.Box("block", "placed", (c[0] / 100, c[1] / 100, c[2] / 100),
                                                    (target.size / 2,) * 3, rel["block_yaw_deg"]), t.target_touch())
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


def destination(m: CollisionMap, t, to: dict, target: Target | None = None) -> Destination:
    """A task's "to" -> release.Destination (base frame, heights from the map)."""
    side = target.size if target is not None else PS.C.BLOCK
    if "bin" in to:
        b = m.object(to["bin"])
        mode = to.get("mode", "set")
        rim = t.table_z + b["size_m"][2]
        if mode == "drop":
            return Destination(b["centre_m"][0], b["centre_m"][1], None, rim, b.get("name", "bin"), mode="drop")
        x, y, top = bin_spot(m, b, side)
        return Destination(x, y, None, rim, b.get("name", "bin"), support_z=t.table_z + top, mode="set")
    if "stack_on" in to:
        o = m.object(to["stack_on"])
        if o.get("kind", "block") != "block":
            raise ValueError(f"stack_on: {o.get('name')} is a {o.get('kind')}, not a block")
        box = m.object_box(o["id"], t.table_z)
        return Destination(o["centre_m"][0], o["centre_m"][1], float(o.get("yaw_deg") or 0.0), None,
                           f"on {o.get('name', o['id'])}", support_z=box.top, mode="set", support=box,
                           support_label=o["id"])
    return Destination(to["table_m"][0], to["table_m"][1], to.get("yaw_deg"), None, "table")


BIN_SPOT_STEP_M = 0.01
BIN_WALL_M = 0.002          # acrylic bins: 2 mm walls (bins_acrylic_v2_pair.json geometry_mm.thickness)


def bin_spot(m: CollisionMap, b: dict, side: float) -> tuple[float, float, float]:
    """Where in a bin to set a block down: the spot inside the walls (a block's half side plus 5 mm off
    them) whose floor is lowest -- the bare plate before anything already in the bin, the middle first.
    Returns (x, y, surface height above the table)."""
    inner = min(b["size_m"][:2]) - 2 * BIN_WALL_M
    reach = max(0.0, inner / 2 - side / 2 - 0.005)
    yaw = math.radians(float(b.get("yaw_deg", 0.0)))
    best = None
    n = int(reach / BIN_SPOT_STEP_M)
    for i in range(-n, n + 1):
        for j in range(-n, n + 1):
            lx, ly = i * BIN_SPOT_STEP_M, j * BIN_SPOT_STEP_M
            x = b["centre_m"][0] + math.cos(yaw) * lx - math.sin(yaw) * ly
            y = b["centre_m"][1] + math.sin(yaw) * lx + math.cos(yaw) * ly
            top = m.top_under(x, y, side / 2, math.degrees(yaw))
            key = (round(top, 3), math.hypot(lx, ly))
            if best is None or key < best[0]:
                best = (key, x, y, top)
    return best[1], best[2], best[3]


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

    def trajectory(self, waypoints, stops=()) -> list:
        out = []
        for w in waypoints:
            if any(v is not None for v in w[:5]):
                self.move([self.pose[j] if v is None else v for j, v in enumerate(w[:5])])
            if w[5] is not None:
                self.pose[5] = float(w[5])
            out.append(list(self.pose))
        return out

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

    def trajectory(self, waypoints, stops=()) -> list:
        """/api/trajectory: the console checks the whole chain, then moves through it, stopping at `stops`.
        The readback at the end of every waypoint (None where the arm was already there)."""
        r = self.session.trajectory(waypoints, stops)
        return list((r.get("trajectory") or {}).get("reached") or [])

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
            phases: Sequence[str] | None = None, grip_must_hold: bool = True, correct: bool = True,
            motion: str = "steps") -> dict:
    """Run plan.json step by step and log each one to execution.jsonl.

    Without really=True (or with backend=DryRun) nothing moves. Before anything is sent: the plan must be ok
    with every step's checks ok, and the arm must stand at the plan's start (within start_tolerance_deg).
    After each move the readback is corrected towards the command (`settle`, planning_tuning.json `execute`;
    False turns it off), then compared with it; a grip must read 'holding'. Any miss stops the run where it is
    (no automatic recovery: use the console to back off).

    phases: run only the steps of these plan kinds (`pick`, `place`, `transit`), e.g. the grasp alone and the
    place later (docs/current/32 §2); the arm must then stand where the step before the first selected
    one ends. grip_must_hold=False: a grip that does not hold is recorded, not a stop (the check step judges
    it). correct: after a place's "lower" (and its settle), where the held block is by the readback is
    compared with where the plan puts it, and up to planning.correct.TRIES small moves take it there before
    the release, each only if it passes the planner's checks (planning/correct.py).
    motion: `steps` (each step its own move, stopped and checked) or `continuous` (the steps between a grip, a
    place's lower and the end go to the console as one trajectory, stopping only at gripper openings and the
    run's end; the settle and the checks above apply at the run's end: flow/moves.py, docs/current/42 §4).
    The returned dict carries the logged rows under `rows`."""
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
                  phases=list(phases) if phases else None, motion=motion)
    groups = MV.runs(p["steps"], chosen, motion, correct)
    rows = []
    grip_rb = None
    try:
        rb = backend.readback()
        off = max(abs(a - b) for a, b in zip(rb[:5], start))
        if off > start_tolerance_deg:
            raise RuntimeError(f"the arm stands at {[round(v) for v in rb[:5]]}, the plan starts at {start} "
                               f"({off:.0f} deg off): plan again from here")
        sess.append(log_name, dict(event="start", backend=backend.name, readback=rb, plan=str(plan_src or "plan.json"),
                                   phases=result["phases"], motion=motion,
                                   runs=[[g[0], g[-1]] for g in groups if len(g) > 1]))
        n = 0
        for group in groups:
            if stop_after is not None and n >= stop_after:
                result["stopped"] = f"stop_after {stop_after}"
                break
            if stop_after is not None:
                group = group[:stop_after - n]
            if len(group) > 1:
                rows_g, rb = _run_continuous(backend, p, group, st, correct, grip_rb, tolerance_deg)
                for row in rows_g:
                    sess.append(log_name, row)
                    rows.append(row)
                bad = next((row for row in rows_g if "problem" in row), None)
                if bad is not None:
                    result["stopped"] = f"step {bad['i']} {bad['name']}: {bad['problem']}"
                    break
                n += len(group)
                result["done"] = n
                continue
            i = group[0]
            s = p["steps"][i]
            began = time.time()
            fix = None
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
            if correct and s["kind"] == "move" and s["plan"] == "place" and s["name"] == "lower":
                rb, fix = _correct_held(backend, p, s["command"], rb, grip_rb)
            if s.get("action") == "grip":
                grip_rb = rb              # where the pads really closed: the held block's model is rebuilt from it
            row = dict(event="step", i=i, plan=s["plan"], name=s["name"], kind=s["kind"], sent=sent,
                       command=s["command"], readback=rb, seconds=round(time.time() - began, 2))
            if fixes:
                row.update(first_readback=first, settle=fixes)
            if fix is not None:
                row["correction"] = fix
            if s["kind"] == "move" and not (fix and fix.get("nudges")):   # a nudged pose is off on purpose
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
            n += 1
            result["done"] = n
        else:
            result["ok"] = result["stopped"] is None
    finally:
        backend.close()
    sess.append(log_name, dict(event="end", **result))
    sess.log("execute", **result)
    return dict(result, rows=rows)


def _run_continuous(backend, p: dict, group: Sequence[int], st: dict | None, correct: bool, grip_rb,
                    tolerance_deg: float) -> tuple[list[dict], list]:
    """One run of steps as one trajectory (flow/moves.py); the settle and the readback correction at its end.
    Returns (rows, readback at the end)."""
    began = time.time()
    steps = [p["steps"][i] for i in group]
    stop_at = MV.stops(p["steps"], group)
    reached = backend.trajectory([MV.waypoint(s) for s in steps], stop_at)
    rb = backend.readback()
    last = steps[-1]
    fixes, fix, first = [], None, rb
    if last["kind"] == "move" and st is not None:
        rb, fixes = _settle(backend, last["command"][:5], rb, st)
    if correct and last["kind"] == "move" and last["plan"] == "place" and last["name"] == "lower":
        rb, fix = _correct_held(backend, p, last["command"], rb, grip_rb)
    seconds = round(time.time() - began, 2)
    rows = []
    for k, (i, s) in enumerate(zip(group, steps)):
        final = k == len(group) - 1
        at = rb if final else (reached[k] if k < len(reached) and reached[k] is not None else None)
        row = dict(event="step", i=i, plan=s["plan"], name=s["name"], kind=s["kind"], sent=MV.waypoint(s),
                   command=s["command"], readback=at, run=[group[0], group[-1]], stop=k in stop_at,
                   seconds=seconds if final else 0.0)
        if final and fixes:
            row.update(first_readback=first, settle=fixes)
        if final and fix is not None:
            row["correction"] = fix
        if s["kind"] == "move" and at is not None and not (final and fix and fix.get("nudges")):
            dev = max(abs(a - b) for a, b in zip(at[:5], s["command"][:5]))
            row["deviation_deg"] = round(dev, 1)
            tol = tolerance_deg if (final or k in stop_at) else max(tolerance_deg, MV.PASS_TOLERANCE_DEG)
            if dev > tol:
                row["problem"] = f"readback {dev:.0f} deg off the command"
        rows.append(row)
    return rows, rb


def _correct_held(backend, p: dict, cmd, rb, grip_rb=None) -> tuple[list, dict]:
    """At a set-down pose: nudge until the held block is where the plan puts it by the readback
    (planning/correct.py), each move checked against the map first. Returns (readback, record).

    Where the block is in the fingers comes from the grip's readback (`grip_rb`) when there is one: the pads
    closed where the servos really stood, not where the plan put them, and the block sits that much higher or
    lower between them. The planned set-down pose meant the planned model's block centre; the target is that
    point, reached with the block as it is really held."""
    from planning import correct as CR
    pick = next((q for q in p["plans"] if q["kind"] == "pick"), None)
    place = next((q for q in p["plans"] if q["kind"] == "place"), None)
    held_d = ((pick or {}).get("summary") or {}).get("held_block")
    if held_d is None:
        return rb, dict(applied=False, reason="the plan has no held block model")
    planned = CR.held_from_summary(held_d)
    held, model = planned, "planned grasp"
    tg = pick["summary"].get("target") or {}
    if grip_rb is not None and "x_cm" in tg:
        table_z = pick["summary"].get("table_z_cm", 0.0) / 100
        side = tg.get("size_mm", 30.0) / 1000
        centre = [tg["x_cm"] / 100, tg["y_cm"] / 100, table_z + tg.get("bottom_cm", 0.0) / 100 + side / 2]
        held, model = CR.GR.held_from_grasp(list(grip_rb[:5]), side, centre), "grip readback"
    target_centre = CR.centre(cmd, planned)
    try:
        w = planning.world(p["map"]["file"], overrides=p["task"].get("tuning"))
    except (KeyError, OSError, ValueError) as e:
        return rb, dict(applied=False, reason=f"no map to check a nudge against ({e})")
    label = pick["summary"].get("target_label")
    w = w.without(label) if label else w
    dest = (place or {}).get("summary", {}).get("destination", {})
    if dest.get("on"):
        o = w.cmap.object(dest["on"])
        w = w.without(o["id"]).with_extra(w.cmap.object_box(o["id"], w.tuning.table_z), w.tuning.target_touch())
    record = dict(first=None, nudges=[], applied=False, reason=None, held_model=model)
    tight = dict(settle_settings(), settle_deg=CR.SETTLE_DEG, tries=CR.SETTLE_TRIES)
    for _ in range(CR.TRIES):
        r = CR.reach(target_centre, rb, held)
        record["first"] = record["first"] or r["error_mm"]
        if not r["applied"]:
            record["reason"] = r["reason"]
            break
        # where the joints must stand for the block to be on target, reached as the settle reaches a command:
        # re-sent past it by the remaining error (servos that stop short do not move for a fraction of a degree)
        goal = [b + m for b, m in zip(rb[:4], r["moved_deg"])] + [float(rb[4])]
        over = [g + max(-tight["extra_deg"], min(tight["extra_deg"], g - b)) for g, b in zip(goal[:4], rb[:4])] + [goal[4]]
        bad = next((c for c in (CR.lower_ok(w, rb, goal, held), CR.lower_ok(w, rb, over, held)) if not c.ok), None)
        if bad is not None:
            record["reason"] = f"nudge refused: {bad.failed()}"
            break
        rb, fx = _settle(backend, goal, rb, tight)
        record["nudges"].append(dict(goal=[round(v, 2) for v in goal], moved_deg=r["moved_deg"], settle=fx, readback=rb))
        record["applied"] = True
    record["last_error_mm"] = CR.reach(target_centre, rb, held)["error_mm"]
    e0, e1 = record["first"], record["last_error_mm"]
    record["note"] = (f"held block {math.hypot(e0[0], e0[1]):.1f} mm / {e0[2]:+.1f} mm off at the readback -> "
                      f"{math.hypot(e1[0], e1[1]):.1f} mm / {e1[2]:+.1f} mm after {len(record['nudges'])} nudge(s)")
    return rb, record


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
