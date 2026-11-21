"""The steps of docs/current/32-操作逻辑简化与分步任务.md §2, each callable on its own.

Every step is `fn(ctx, **params) -> Outcome`. `Outcome.result` is a short word the flow script branches on
(`on_result:` in config/flows/*.yaml); `summary` is the Chinese line a page shows. The steps reuse
src/flow/stages.py (scan / locate / map / plan / execute / verify) and keep their files in the session
directory, so any step can also be run alone from the CLI or the gateway, picking up the previous step's
files (`orchestrator.py --step grasp --session S --block b3`).

| step     | moves the arm | results                                   |
|----------|---------------|-------------------------------------------|
| scan     | yes           | done, empty                               |
| relocate | no            | found, gone                               |
| plan     | no            | ok, rejected, no_destination, gone, base  |
| grasp    | yes           | done, failed                              |
| check    | no            | holding, empty, slipped, jammed           |
| back_off | yes           | done, failed                              |
| place    | yes           | done, failed                              |
| verify   | no            | ok, mismatch                              |
| home     | yes           | done, failed                              |
"""
from __future__ import annotations

import copy
import json
import math
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable

from . import stages as ST
from .session import FlowSession, load_json

SLIP_DEG = 3            # J6 closing this much further across the lift: the block left the fingers (console_grasp)
FIND_RADIUS_M = 0.03    # a block found again after a rescan must be this close to where it was
# to: table with no table_m -- a free spot on the page (docs/current/35 §10.1, carry.yaml)
TABLE_AREA_M = (-0.13, 0.09, 0.13, 0.265)   # x0, y0, x1, y1, base frame: the A4 page of map_a4_v4 (long edge across,
                                            # base = map + (0, 6.75 cm)) less 1.5-2 cm at the edges
SPOT_STEP_M = 0.02
SPOT_CLEAR_M = 0.015    # between the block's footprint circle and any other object's
SPOT_MIN_MOVE_M = 0.06  # a carry that moves the block less than this is not worth the trip
SPOT_TRIES = 6          # spots planned, nearest first, before `rejected`
TOWER_LEVELS = 3        # stack with on: top -- the tower stops growing at this many blocks
SUCCESS = dict(scan="done", relocate="found", plan="ok", grasp="done", check="holding", back_off="done",
               place="done", verify="ok", home="done")


@dataclass
class Outcome:
    result: str
    summary: str = ""
    data: dict = field(default_factory=dict)


@dataclass
class Context:
    """What the steps share within one run (or one standalone step)."""
    sess: FlowSession
    console: str | None = None          # console URL; None = console_step's default (the real arm)
    really: bool = False                # False: nothing moves (execute runs as a dry run, scan refuses)
    backend: Callable | None = None     # () -> execute backend; tests pass a DryRun-like object
    sink: Callable | None = None        # (event, **fields): contract log + session log
    scene: dict | None = None           # located.json of the latest scan
    map_path: Path | None = None
    block: dict | None = None           # the block being handled
    first_scene: dict | None = None     # the scene the round started from (verify compares with it)
    sorted: list = field(default_factory=list)
    scans: int = 0
    motion: str = "steps"               # how grasp / place send their steps (flow/moves.py); a script's top-level `motion`
    tower: list = field(default_factory=list)   # plan to: stack, on: top -- scene names, base first

    # ---- helpers ------------------------------------------------------------------------------------
    def emit(self, event: str, **fields) -> None:
        if self.sink:
            self.sink(event, **fields)

    def block_session(self) -> FlowSession:
        if not self.block:
            raise RuntimeError("no block chosen: run inside for_each, or pass --block")
        return FlowSession(self.sess.path / "blocks" / self.block["name"])

    def make_backend(self):
        return self.backend() if self.backend else None

    def load_latest(self) -> None:
        """For a standalone step: the session's newest located.json and collision map."""
        if self.scene is not None:
            return
        scans = sorted(self.sess.path.glob("scan_*/located.json")) or \
            ([self.sess.file("located.json")] if self.sess.has("located.json") else [])
        if not scans:
            raise RuntimeError(f"{self.sess.path} has no located.json: run the scan step first")
        latest = scans[-1]
        self.scene = load_json(latest)
        self.map_path = latest.parent / "collision_map.json"
        self.first_scene = self.first_scene or load_json(scans[0])
        if not self.sorted:                  # blocks this session already put away (their place step ran ok)
            for d in sorted((self.sess.path / "blocks").glob("*/execution.jsonl")):
                ends = [json.loads(ln) for ln in d.read_text(encoding="utf-8").splitlines() if '"end"' in ln]
                if any(e.get("ok") and "place" in (e.get("phases") or []) for e in ends):
                    named = next((b for b in blocks(self.first_scene) if b["name"] == d.parent.name), None)
                    if named:
                        self.sorted.append(named)


def in_bin(o: dict, scene: dict) -> str | None:
    """Name of the bin whose outer footprint holds this block's centre (a block already put away), else None."""
    for b in bin_objects(scene):
        a = math.radians(-b.get("yaw_deg", 0.0))
        dx, dy = o["centre_m"][0] - b["centre_m"][0], o["centre_m"][1] - b["centre_m"][1]
        x, y = dx * math.cos(a) - dy * math.sin(a), dx * math.sin(a) + dy * math.cos(a)
        if abs(x) <= b["size_m"][0] / 2 and abs(y) <= b["size_m"][1] / 2:
            return b["name"]
    return None


def blocks(scene: dict) -> list[dict]:
    """Blocks on the table: a block seen inside a bin's footprint is already sorted and is not offered again;
    one marked as scan noise (scan's min_views) is not a block."""
    return [o for o in scene.get("objects", []) if o.get("kind") == "block" and not o.get("noise")
            and not in_bin(o, scene)]


def mark_noise(scene: dict, min_views: int) -> list[str]:
    """Detections seen from fewer than `min_views` views and carrying no sticker are scan noise (PR #7,
    2026-11-19: real blocks are seen from 5-8 views; run 20261119_175630 dropped 3 phantoms, no real block).
    A block whose view count is unknown (an older located.json) is kept."""
    out = []
    for o in scene.get("objects", []):
        v = o.get("views")
        if o.get("kind") == "block" and min_views and v is not None and v < min_views and not o.get("sticker"):
            o["noise"] = True
            out.append(o["name"])
    return out


def bin_objects(scene: dict) -> list[dict]:
    return [o for o in scene.get("objects", []) if o.get("kind") == "bin"]


def map_cm(o: dict) -> list:
    """The contract's `cube` field: map centimetres; base-frame cm when the scene has no map coordinates."""
    if o.get("map_cm"):
        return [round(float(v), 1) for v in o["map_cm"][:2]]
    return [round(o["centre_m"][0] * 100, 1), round(o["centre_m"][1] * 100, 1)]


def _find_any(scene: dict, near: dict) -> dict | None:
    """As _find, also among blocks inside bins."""
    pool = [o for o in scene.get("objects", []) if o.get("kind") == "block"]
    best = min(pool, key=lambda o: math.dist(o["centre_m"], near["centre_m"]), default=None)
    return best if best is not None and math.dist(best["centre_m"], near["centre_m"]) <= FIND_RADIUS_M else None


def _find(scene: dict, near: dict) -> dict | None:
    best = min(blocks(scene), key=lambda o: math.dist(o["centre_m"], near["centre_m"]), default=None)
    if best is None or math.dist(best["centre_m"], near["centre_m"]) > FIND_RADIUS_M:
        return None
    return best


def free_spots(scene: dict, block: dict, area=TABLE_AREA_M, step: float = SPOT_STEP_M,
               clear: float = SPOT_CLEAR_M, min_move: float = SPOT_MIN_MOVE_M) -> list[list[float]]:
    """Spots on the page where `block` could be set down: its footprint circle `clear` away from every other
    object's (blocks and bins, noise left out), at least `min_move` from where it is; nearest first.
    Whether the arm reaches one, and the way there, is the planner's to say."""
    def radius(o):
        return math.hypot(*o["size_m"][:2]) / 2
    others = [o for o in scene.get("objects", []) if o.get("name") != block.get("name") and not o.get("noise")]
    r0 = radius(block)
    x0, y0, x1, y1 = area
    out = []
    nx, ny = int((x1 - x0) / step + 1e-9), int((y1 - y0) / step + 1e-9)   # floor: no spot past the page's edge
    for i in range(nx + 1):
        for j in range(ny + 1):
            c = (x0 + i * step, y0 + j * step)
            d = math.dist(c, block["centre_m"])
            if d < min_move:
                continue
            if all(math.dist(c, o["centre_m"]) >= r0 + radius(o) + clear for o in others):
                out.append((d, [round(c[0], 3), round(c[1], 3)]))
    return [s for _, s in sorted(out)]


def _scene_after_place(ctx: Context) -> None:
    """After a block was set down on the table or on another block: the scene as the plan left it, so the next
    block's plan sees it where it now stands (a free spot taken, a tower one level higher) without a rescan --
    the ordinary scan takes every block to stand on the table (stack_scan, docs/current/41, is not a step yet).
    A block put into a bin keeps the old handling (plan drops it from the first scene)."""
    dest = ctx.block.get("to") or {}
    if "bin" in dest:
        return
    try:
        p = ctx.block_session().load("plan.json")
        rel = p["plans"][1]["summary"]["release"]
    except (OSError, KeyError, IndexError, TypeError, ValueError):
        return
    scene = copy.deepcopy(ctx.scene)
    if ctx.scene is ctx.first_scene:               # blocks already put into bins leave the copy, as plan does
        binned = {f["name"] for o in ctx.sorted if "bin" in (o.get("to") or {"bin": 1})
                  and (f := _find_any(scene, o)) is not None}
        scene["objects"] = [o for o in scene["objects"] if o["name"] not in binned]
    obj = next((o for o in scene["objects"] if o.get("name") == p["task"]["pick"]), None)
    if obj is None:
        return
    c = rel["block_centre_cm"]
    new = [c[0] / 100, c[1] / 100]
    if obj.get("map_cm"):                           # map and base frame differ by a shift (map_a4: no rotation)
        obj["map_cm"] = [round(obj["map_cm"][0] + (new[0] - obj["centre_m"][0]) * 100, 2),
                         round(obj["map_cm"][1] + (new[1] - obj["centre_m"][1]) * 100, 2)] + list(obj["map_cm"][2:])
    bottom = 0.0
    if "stack_on" in dest:
        under = next((o for o in scene["objects"] if o.get("name") == dest["stack_on"]), None)
        if under is not None:
            bottom = float(under.get("bottom_m", 0.0)) + float(under["size_m"][2])
    obj.update(centre_m=[round(v, 4) for v in new], yaw_deg=rel["block_yaw_deg"], bottom_m=round(bottom, 4),
               placed_by_flow=True)
    after = FlowSession(ctx.block_session().path / "after")
    after.save("located.json", scene)
    ST.build_map(after)
    ctx.scene, ctx.map_path = scene, after.file("collision_map.json")
    if "stack_on" in dest:
        ctx.tower.append(obj["name"])


# ---------------------------------------------------------------------------------------------- steps

def scan(ctx: Context, bins: str | None = None, source: str | None = None, refine=False, min_views: int = 0,
         observe: str = "field", **_) -> Outcome:
    """S1: scan poses -> views -> back to the observe pose -> located.json -> collision map, in
    <session>/scan_<n>/. `source`: use this located.json instead of scanning (no arm motion; for replaying a
    recorded scene)."""
    ctx.scans += 1
    sub = FlowSession(ctx.sess.path / f"scan_{ctx.scans:02d}")
    if source:
        sub.save("located.json", load_json(source))
    else:
        if not ctx.really:
            raise RuntimeError("scan moves the arm: run with --really (or give `source:` a located.json)")
        # refine: true -- a close view of the weakly seen blocks; "block" -- of the current block (a retry)
        near = ctx.block["centre_m"] if refine == "block" and ctx.block else None
        ST.scan(sub, console=ctx.console, bins_design=bins, really=True, confirm=lambda _q: "yes",
                refine=refine is True, refine_near=near, observe=observe)
        back = home(ctx)                 # plans start at the observe pose; the scan ends at its last view
        if back.result != "done":
            raise RuntimeError(f"扫描后回观察位失败：{back.summary}")
        ST.locate(sub, bins_design=bins)
    noise = []
    if min_views:
        located = sub.load("located.json")
        noise = mark_noise(located, int(min_views))
        sub.save("located.json", located)
    ST.build_map(sub)
    ctx.scene = sub.load("located.json")
    ctx.map_path = sub.file("collision_map.json")
    if ctx.first_scene is None:
        ctx.first_scene = ctx.scene
    n = len(blocks(ctx.scene))
    return Outcome("done" if n else "empty", f"找到 {n} 块、{len(bin_objects(ctx.scene))} 个桶"
                   + (f"；{len(noise)} 处只被不到 {min_views} 个视角看到、无贴纸，按扫描噪声去掉（{','.join(noise)}）" if noise else ""),
                   dict(blocks=n, dir=str(sub.path), noise=noise))


def relocate(ctx: Context, **_) -> Outcome:
    """Find the current block again in the latest scene (it may have been nudged by a missed grasp)."""
    ctx.load_latest()
    found = _find(ctx.scene, ctx.block)
    if not found:
        return Outcome("gone", f"重扫后在 {FIND_RADIUS_M * 100:.0f} cm 内找不到 {ctx.block['name']}")
    moved = math.dist(found["centre_m"], ctx.block["centre_m"])
    keep = {k: ctx.block[k] for k in ("failed_j5", "bin") if k in ctx.block}     # what the attempts learnt
    ctx.block = dict(found, name=ctx.block["name"], **keep)
    return Outcome("found", f"{ctx.block['name']} 偏移 {moved * 100:.1f} cm", dict(moved_cm=round(moved * 100, 1)))


def plan(ctx: Context, to="by_class", table_m=None, start=None, tuning=None, on=None, bin_mode="set",
         levels: int = TOWER_LEVELS, **_) -> Outcome:
    """S2: grasp + place + return, the whole path checked against the collision map. Nothing moves.

    to=table without table_m: the free spots of the page (free_spots), nearest first, the first that plans.
    to=stack, on=top: onto the tower this run is building (ctx.tower); with no tower yet the current block is
    its base and stays where it is (result `base`)."""
    ctx.load_latest()
    current = _find(ctx.scene, ctx.block)          # the latest scan names blocks afresh: match by position
    if current is None:
        return Outcome("gone", f"当前场景里找不到 {ctx.block['name']}")
    name = current["name"]
    if to == "by_class":
        cls = ctx.block.get("class")
        target = next((b for b in bin_objects(ctx.scene) if b.get("class") == cls), None) if cls else None
        if target is None:
            return Outcome("no_destination", f"{ctx.block['name']}（{cls or '类别未知'}）没有对应的桶")
        dests = [{"bin": target["name"], "mode": bin_mode}]
    elif to == "table":
        spots = [list(table_m)] if table_m else free_spots(ctx.scene, current)[:SPOT_TRIES]
        if not spots:
            return Outcome("no_destination", f"桌面上找不到放 {ctx.block['name']} 的空位")
        dests = [{"table_m": s} for s in spots]
    elif to == "stack":
        if on == "top":
            if not ctx.tower:
                ctx.tower.append(name)
                return Outcome("base", f"{ctx.block['name']} 作塔底，不动", dict(tower=list(ctx.tower)))
            if len(ctx.tower) >= int(levels):
                return Outcome("no_destination", f"塔已经 {len(ctx.tower)} 层（上限 {levels}）")
            on = ctx.tower[-1]
        under = next((o for o in ctx.scene["objects"] if o.get("kind") == "block" and o.get("name") == on), None)
        if under is None:
            return Outcome("no_destination", f"场景里找不到要叠在下面的块 {on!r}")
        dests = [{"stack_on": under["name"]}]
    else:
        raise ValueError(f"plan: to 只允许 by_class / table / stack，收到 {to!r}")
    cmap = ctx.map_path
    gone = [o["name"] for o in ctx.sorted if (f := _find_any(ctx.scene, o)) is not None and f["name"] != name] \
        if ctx.scene is ctx.first_scene else []
    if gone:                                       # this scan still shows blocks already carried away: drop them
        bs = ctx.block_session()
        bs.save("located.json", dict(ctx.scene, objects=[o for o in ctx.scene["objects"] if o["name"] not in gone]))
        ST.build_map(bs)
        cmap = bs.file("collision_map.json")
    for dest in dests:
        task = dict(pick=name, to=dest)
        if start:
            task["start"] = list(start)
        if tuning:
            task["tuning"] = tuning
        if ctx.block.get("failed_j5"):             # a retry: close the jaw across the other faces
            task["avoid_j5"] = list(ctx.block["failed_j5"])
        p = ST.plan(ctx.block_session(), task, cmap=cmap)
        if p["ok"]:
            break
    if not p["ok"]:
        tried = f"（试了 {len(dests)} 个空位）" if len(dests) > 1 else ""
        return Outcome("rejected", "解算不通过" + tried + "：" + "；".join(str(r) for r in p["reasons"][:2]),
                       dict(reasons=p["reasons"][:5], tried=len(dests)))
    ctx.block["bin"] = dest.get("bin") or (f"on {dest['stack_on']}" if "stack_on" in dest else "table")
    ctx.block["to"] = dest
    # which solving tier each part needed (planner.grasp_tiers): anything past "full" is said in the summary
    pick_s = p["plans"][0].get("summary", {})
    place_s = p["plans"][1].get("summary", {}) if len(p["plans"]) > 1 else {}
    tiers = dict(grasp=pick_s.get("grasp_tier", "full"), place=place_s.get("place_tier", "full"))
    names = {"touching": "贴边", "pipeline": "按旧流水线口径（不避邻块）"}
    note = "；".join(f"{'抓取' if k == 'grasp' else '放置'}{names[v]}" for k, v in tiers.items() if v in names)
    aligned = (pick_s.get("grasp") or {}).get("aligned", True)
    if not aligned:
        note = "；".join(filter(None, [note, f"爪面未对正 {(pick_s.get('grasp') or {}).get('residual_deg')}°，靠合爪扳正"]))
    return Outcome("ok", f"{len(p['steps'])} 步，去 {ctx.block['bin']}" + (f"（{note}）" if note else ""),
                   dict(steps=len(p["steps"]), tiers=tiers, aligned=aligned))


def _execute(ctx: Context, phases, grip_must_hold=True, motion: str | None = None) -> dict:
    return ST.execute(ctx.block_session(), console=ctx.console, really=ctx.really, confirm=lambda _q: "yes",
                      backend=ctx.make_backend(), phases=phases, grip_must_hold=grip_must_hold,
                      motion=motion or ctx.motion)


def grasp(ctx: Context, motion: str | None = None, **_) -> Outcome:
    """S3: descend, close, lift (the plan's `pick` steps). A grip that does not hold is left to `check`."""
    r = _execute(ctx, ("pick",), grip_must_hold=False, motion=motion)
    ctx.block_session().save("grasp.json", r)
    if not r["ok"]:
        return Outcome("failed", f"抓取中途停止：{r['stopped']}", dict(stopped=r["stopped"]))
    return Outcome("done", f"抓取 {r['done']} 步完成")


def _remember_failed_jaw(ctx: Context) -> None:
    """The J5 of the grasp that closed on nothing (its plan.json), so the retry's plan turns the jaw."""
    try:
        j5 = ctx.block_session().load("plan.json")["plans"][0]["summary"]["j5"]
    except (OSError, KeyError, IndexError, TypeError, ValueError):
        return
    ctx.block.setdefault("failed_j5", []).append(float(j5))


def check(ctx: Context, slip_deg: float = SLIP_DEG, **_) -> Outcome:
    """S4: read the grasp's own log: what the grip read back, and whether J6 closed further during the lift."""
    g = ctx.block_session().load("grasp.json")
    rows = g.get("rows", [])
    grip = next((r for r in rows if r.get("kind") == "gripper" and "grip_result" in r), None)
    if grip is None:
        return Outcome("empty", "抓取记录里没有合爪这一步")
    result = grip["grip_result"]
    at_grip = grip["readback"][5]
    after = rows[-1]["readback"][5] if rows else at_grip
    data = dict(grip_result=result, j6_at_grip=at_grip, j6_after_lift=after)
    if result == "not_closed":
        return Outcome("jammed", f"夹爪没有合拢（读回 {at_grip}°）", data)
    if result not in ("holding", "dry-run"):
        _remember_failed_jaw(ctx)
        return Outcome("empty", f"空爪（读回 {at_grip}°）", data)
    if at_grip is not None and after is not None and after - at_grip >= slip_deg:
        return Outcome("slipped", f"抬起时滑脱（{at_grip}° → {after}°）", data)
    return Outcome("holding", f"夹住（读回 {at_grip}°）", data)


def back_off(ctx: Context, **_) -> Outcome:
    """Open, rise to the grasp's hover pose, go to the observe pose. Used before a retry and after a jam."""
    p = ctx.block_session().load("plan.json")
    pick = [s for s in p["steps"] if s["plan"] == "pick"]
    backend = ctx.make_backend()
    owned = backend is None
    if owned:
        if not ctx.really:
            return Outcome("done", "空跑：未动")
        backend = ST.Console(ctx.console)
    try:
        backend.gripper("open", 30)
        if pick:
            backend.move(pick[0]["command"][:5])
        backend.move(list(p["task"].get("start") or ST.OBSERVE))
    except Exception as exc:
        return Outcome("failed", f"退回失败：{exc}")
    finally:
        if owned:
            backend.close()
    return Outcome("done", "已张爪退回观察位")


def place(ctx: Context, motion: str | None = None, **_) -> Outcome:
    """S6: carry, release, return (the plan's `place` and `transit` steps)."""
    r = _execute(ctx, ("place", "transit"), motion=motion)
    if not r["ok"]:
        return Outcome("failed", f"放置中途停止：{r['stopped']}", dict(stopped=r["stopped"]))
    ctx.sorted.append(ctx.block)
    _scene_after_place(ctx)
    return Outcome("done", f"已放到 {ctx.block.get('bin', '目标位')}")


def verify(ctx: Context, scope: str = "round", tolerance_m: float = 0.015, **_) -> Outcome:
    """S7: compare the latest scan with the one the round started from: every placed block left the table
    and nothing else moved further than tolerance_m. scope=block: stages.verify for the current block."""
    ctx.load_latest()
    if scope == "block":
        out = ST.verify(ctx.block_session(), after=ctx.scene, before=ctx.first_scene)
        return Outcome("ok" if out["ok"] else "mismatch", "；".join(c["check"] for c in out["checks"] if not c["ok"]) or "通过", out)
    before, after = blocks(ctx.first_scene), blocks(ctx.scene)
    placed = {b["name"] for b in ctx.sorted}
    left = [b["name"] for b in before if b["name"] in placed and _find(ctx.scene, b)]
    moved = [b["name"] for b in before if b["name"] not in placed and not _find(ctx.scene, b)]
    ok = not left and not moved and len(after) == len(before) - len(placed)
    summary = "通过" if ok else "；".join(filter(None, [
        f"仍在桌上：{','.join(left)}" if left else "", f"被碰动或不见：{','.join(moved)}" if moved else "",
        f"桌上 {len(before)} → {len(after)} 块（放走 {len(placed)}）"]))
    return Outcome("ok" if ok else "mismatch", summary, dict(left=left, moved=moved, before=len(before), after=len(after)))


def home(ctx: Context, **_) -> Outcome:
    backend = ctx.make_backend()
    owned = backend is None
    if owned:
        if not ctx.really:
            return Outcome("done", "空跑：未动")
        backend = ST.Console(ctx.console)
    try:
        backend.move(ST.OBSERVE)
    except Exception as exc:
        return Outcome("failed", f"回观察位失败：{exc}")
    finally:
        if owned:
            backend.close()
    return Outcome("done", "已回观察位")


STEPS: dict[str, Callable[..., Outcome]] = dict(scan=scan, relocate=relocate, plan=plan, grasp=grasp, check=check,
                                                back_off=back_off, place=place, verify=verify, home=home)


def order_blocks(scene: dict, order: str = "reach") -> list[dict]:
    bs = blocks(scene)
    if order == "reach":
        return sorted(bs, key=lambda o: math.hypot(*o["centre_m"]))
    if order == "scene":
        return bs
    raise ValueError(f"order 只允许 reach / scene，收到 {order!r}")


def now() -> float:
    return time.time()
