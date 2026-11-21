"""What every check against the simulator needs, in one place: connect (refusing anything that is not the simulator),
lay out the table, read the truth, and apply parameter overrides.

Before 2026-11-21 each script (flow_check, vertical_check, stack_bench, roi_bench, pose_bench...) had its own copy of
the bootstrap, the reset, the block placing and the fault calls; new checks use this, and dev/sim/bench.py runs
scenario files (dev/sim/scenarios/*.yaml) through it. Results only say the geometry and the interfaces agree with each
other (docs/current/16).

Overrides (`--set key=value` in bench.py; `apply_sets` here) -- one prefix per place a parameter lives:

  fault.<name>       the simulator's faults (/api/sim/faults): stop_short_deg, grip_miss, pose_jitter_deg, servo_bias_deg
  tuning.<a.b>       planning_tuning.json, as a task's `tuning` overrides (errors, trims, execute...)
  plan.<a.b>         motion_planning.json, patched in this process (planning.config.P; the sub-sections G / R / PATH
                     are the same dicts). A value some module copied at import is not reached: the report shows
                     what each override resolved to, check it took
  console.speed      the console's speed (deg/s, /api/speed)
  execute.<k>        flow.stages.execute: motion (steps / continuous), correct (true / false), tolerance_deg, and the
                     settle keys of planning_tuning.json `execute` (settle_deg, extra_deg, tries)
"""
from __future__ import annotations

import copy
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
for sub in ("app/src", "app/src/tools", "app/src/flow"):
    if str(ROOT / sub) not in sys.path:
        sys.path.insert(0, str(ROOT / sub))

from flow import stages as ST          # noqa: E402

FAULT_KEYS = ("stop_short_deg", "grip_miss", "pose_jitter_deg", "servo_bias_deg")
NO_FAULTS = dict(stop_short_deg=0.0, grip_miss=0.0, pose_jitter_deg=[0.0] * 5, servo_bias_deg=[0.0] * 5)
EXECUTE_KEYS = ("motion", "correct", "tolerance_deg", "settle_deg", "extra_deg", "tries")


class NotSimulator(SystemExit):
    pass


class Sim:
    """The simulator console at `url` (default flow.stages.SIM_CONSOLE). Every call that edits the scene needs the
    console idle; none moves the arm."""

    def __init__(self, url: str | None = None):
        self.url = ST._use_console(url or ST.SIM_CONSOLE)
        import console_step as cs
        self.cs = cs
        boot = cs.call("/api/bootstrap")
        if boot.get("status", {}).get("mode") != "simulation":
            raise NotSimulator(f"{self.url} is not the simulator; refusing")
        self.token = boot["token"]
        self.options = cs.call("/api/sim/options")

    # ---- calls ---------------------------------------------------------------------------------------
    def post(self, path: str, body: dict | None = None) -> dict:
        r = self.cs.call(path, body or {}, token=self.token)
        if isinstance(r, dict) and r.get("error"):
            raise RuntimeError(f"{path}: {r['error']}")
        return r

    def world(self) -> dict:
        return self.cs.call("/api/sim/world")

    def pose(self) -> list:
        return self.cs.status()["pose"]

    @property
    def speed(self) -> float:
        """The simulator's --speed: its waits are divided by this (real-arm seconds ~ simulated seconds x speed)."""
        return float(self.options.get("speed") or 1.0)

    # ---- the table -----------------------------------------------------------------------------------
    def reset(self, seed: int = 1, count: int = 0) -> None:
        self.post("/api/sim/reset", {"count": count, "seed": seed})

    def faults(self, **kw) -> dict:
        """Set every fault: those not given to none (faults persist in the simulator across runs)."""
        unknown = set(kw) - set(FAULT_KEYS)
        if unknown:
            raise ValueError(f"unknown faults {sorted(unknown)}; have {FAULT_KEYS}")
        body = dict(NO_FAULTS, **{k: v for k, v in kw.items() if v is not None})
        self.post("/api/sim/faults", body)
        return body

    def bins(self, name: str | None) -> None:
        """Stand a design's bins (replacing the last set); None: take every prop off the table."""
        if name:
            self.post("/api/sim/bins", {"name": name})
        else:
            for p in self.world()["props"]:
                self.post("/api/sim/prop/remove", {"id": p["id"]})

    def map(self, name: str | None) -> None:
        if name:
            self.post("/api/sim/map", {"name": name})

    def block(self, category: str, map_cm, yaw_deg: float = 0.0, up=None) -> str:
        """Put a block at map_cm (on whatever stands there: a block put where one is stands on it); its id."""
        before = {b["id"] for b in self.world()["blocks"]}
        body = {"category": category, "map_cm": list(map_cm), "yaw_deg": yaw_deg}
        if up is not None:
            body["up"] = up
        self.post("/api/sim/block", body)
        return next(b["id"] for b in self.world()["blocks"] if b["id"] not in before)

    def lay_out(self, blocks, seed: int = 1) -> dict:
        """reset, then [{tag, category, map_cm, yaw, up, on}] in order (`on`: another tag -- put at its place, so it
        stands on it); {tag: id}."""
        self.reset(seed)
        ids, at = {}, {}
        for b in blocks:
            xy = at[b["on"]] if b.get("on") else b["map_cm"]
            ids[b["tag"]] = self.block(b["category"], xy, b.get("yaw", 0.0), b.get("up"))
            at[b["tag"]] = xy
        return ids

    # ---- truth -----------------------------------------------------------------------------------------
    def table_z(self) -> float:
        return self.world()["table_z_cm"] / 100

    def table_trim(self) -> dict:
        """The tuning override that puts the simulator's table where the planner's is (doc 40 §2): the simulator's
        table is where its map stands (the 12 cm FK tip's -1.42 cm), the planner's where the mesh fingers touch
        (motion_planning.json table_z_model_m); a touch calibration would write the difference to trims.table_z_m."""
        from planning import config as PC
        return {"trims": {"table_z_m": round(self.table_z() - PC.TABLE_Z, 5)}}

    def located(self, bins_design: str | None = None, world: dict | None = None, sigma_m: float = 0.002) -> dict:
        """located.json from the truth: blocks with their bottom height (stacks included), bins at their poses."""
        w = world or self.world()
        tz = w["table_z_cm"] / 100
        objs = []
        for b in w["blocks"]:
            if b["held"]:
                continue
            objs.append(dict(id=len(objs) + 1, name=b["id"], kind="block", **{"class": b["category"]},
                             centre_m=[v / 100 for v in b["base_cm"][:2]], yaw_deg=b["yaw_deg"], size_m=[0.03] * 3,
                             sigma_xy_m=sigma_m, bottom_m=round(max(0.0, b["base_cm"][2] / 100 - 0.015 - tz), 4)))
        for p in w["props"]:
            if p.get("number") is None:
                continue
            T = p["base_T_prop"]
            objs.append(dict(id=len(objs) + 1, name=f"bin{p['number']}", kind="bin", **{"class": p["category"]},
                             design=bins_design, centre_m=[T[0][3], T[1][3]], yaw_deg=p["yaw_deg"],
                             size_m=[0.084, 0.084, 0.042], sigma_xy_m=sigma_m))
        return dict(objects=objs, table_z_m=tz, placed_by="simulator truth (dev/sim/harness.py)")

    def block_truth(self, block_id: str, world: dict | None = None) -> dict:
        return next(b for b in (world or self.world())["blocks"] if b["id"] == block_id)

    def bin_of(self, category: str, world: dict | None = None) -> str | None:
        """The located name (bin<n>) of the bin standing for a category."""
        for p in (world or self.world())["props"]:
            if p.get("number") is not None and p.get("category") == category:
                return f"bin{p['number']}"
        return None

    @staticmethod
    def counts(world: dict) -> dict:
        return {k: world[k]["count"] for k in ("contacts", "block_contacts", "platform_contacts")}


# ------------------------------------------------------------------------------------------------- overrides

def parse_value(text: str):
    """JSON when it parses (numbers, true/false/null, lists), else the plain string."""
    try:
        return json.loads(text)
    except ValueError:
        return text


def parse_sets(items) -> dict:
    """["a.b=1", ...] -> {"a.b": 1}."""
    out = {}
    for it in items or ():
        if "=" not in it:
            raise ValueError(f"--set {it!r}: key=value")
        k, v = it.split("=", 1)
        out[k.strip()] = parse_value(v.strip())
    return out


def _nest(path: str, value) -> dict:
    out = cur = {}
    keys = path.split(".")
    for k in keys[:-1]:
        cur[k] = {}
        cur = cur[k]
    cur[keys[-1]] = value
    return out


def _merge(a: dict, b: dict) -> dict:
    out = copy.deepcopy(a)
    for k, v in b.items():
        out[k] = _merge(out[k], v) if isinstance(v, dict) and isinstance(out.get(k), dict) else v
    return out


class Overrides:
    """Parameter overrides by prefix (module docstring). `apply` patches what lives in this process and returns
    an undo; the faults and the speed go to the simulator through `sim_*`."""

    def __init__(self, sets: dict | None = None):
        self.fault: dict = {}
        self.tuning: dict = {}
        self.plan: dict = {}
        self.console: dict = {}
        self.execute: dict = {}
        for k, v in (sets or {}).items():
            self.add(k, v)

    def add(self, key: str, value) -> None:
        prefix, _, rest = key.partition(".")
        if not rest:
            raise ValueError(f"{key}: prefix.name (fault / tuning / plan / console / execute)")
        if prefix == "fault":
            if rest not in FAULT_KEYS:
                raise ValueError(f"{key}: faults are {FAULT_KEYS}")
            self.fault[rest] = value
        elif prefix == "tuning":
            self.tuning = _merge(self.tuning, _nest(rest, value))
        elif prefix == "plan":
            self.plan[rest] = value
        elif prefix == "console":
            if rest != "speed":
                raise ValueError(f"{key}: only console.speed")
            self.console[rest] = value
        elif prefix == "execute":
            if rest not in EXECUTE_KEYS:
                raise ValueError(f"{key}: execute takes {EXECUTE_KEYS}")
            self.execute[rest] = value
        else:
            raise ValueError(f"{key}: unknown prefix {prefix!r}")

    def merged(self, other: "Overrides") -> "Overrides":
        out = Overrides()
        out.fault = dict(self.fault, **other.fault)
        out.tuning = _merge(self.tuning, other.tuning)
        out.plan = dict(self.plan, **other.plan)
        out.console = dict(self.console, **other.console)
        out.execute = dict(self.execute, **other.execute)
        return out

    def apply_plan(self):
        """Patch motion_planning.json values in this process; returns undo()."""
        from planning import config as PC
        undo = []
        for path, value in self.plan.items():
            keys = path.split(".")
            cur = PC.P
            for k in keys[:-1]:
                if k not in cur or not isinstance(cur[k], dict):
                    raise KeyError(f"plan.{path}: motion_planning.json has no section {k!r}")
                cur = cur[k]
            if keys[-1] not in cur:
                raise KeyError(f"plan.{path}: motion_planning.json has no {keys[-1]!r} there")
            undo.append((cur, keys[-1], cur[keys[-1]]))
            cur[keys[-1]] = value
        consts = {"table_z_model_m": "TABLE_Z", "block_side_m": "BLOCK"}
        saved = {}
        for path, const in consts.items():
            if path in self.plan:
                saved[const] = getattr(PC, const)
                setattr(PC, const, float(self.plan[path]))

        def restore():
            for d, k, v in reversed(undo):
                d[k] = v
            for const, v in saved.items():
                setattr(PC, const, v)
        return restore

    def execute_kwargs(self) -> dict:
        """stages.execute keyword arguments: motion / correct / tolerance_deg, and `settle` from the settle keys."""
        kw = {k: self.execute[k] for k in ("motion", "correct", "tolerance_deg") if k in self.execute}
        settle = {k: self.execute[k] for k in ("settle_deg", "extra_deg", "tries") if k in self.execute}
        if settle:
            kw["settle"] = settle
        return kw

    def describe(self) -> dict:
        return {k: v for k, v in dict(fault=self.fault, tuning=self.tuning, plan=self.plan, console=self.console,
                                      execute=self.execute).items() if v}


def set_speed(sim: Sim, speed) -> None:
    if speed is not None:
        sim.post("/api/speed", {"speed": speed})
