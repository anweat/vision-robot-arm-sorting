"""Observation scripts: a scan's fixed poses as an editable file, and the one loop that visits them.

A scan is a fixed series of moves and pictures, so it is written down (app/config/observe/<name>.yaml, or JSON)
instead of living in the code: which poses, in what order, how long to wait at each, whether to wait for the
console's board result, and what to do when a pose cannot be reached. The field scan is `field.yaml` (what
field_scan.SCAN_POSES was); the bin views, the close looks and the stack scan's height views are built from
other inputs and run through the same loop (`run` takes a script dict as well as a name).

Script keys (docs/current/42 §2):
  name, title          labels
  j5                   roll for every view: `roll_zero` (arm_limits.json gripper.roll_zero_deg) or degrees
  settle_s             two readbacks this far apart must agree within 1 deg before the picture (capture.py)
  pause_s              and then this much longer (a slow camera, a swaying arm)
  board                wait for the console's localization result of the pose (map codes, official detections)
  on_blocked           skip (default): a pose the way to which a bin blocks is left out; stop: the scan stops
  event                the log event written per picture (scan_view: docs/current/18)
  end                  null, or a pose [J1..J4] / `observe` to go to after the last view
  views                [{pose: [J1, J2, J3, J4] or [J1..J5], name, settle_s, pause_s, board, j5, ...}]; any
                       other key of a view is copied into its views.json entry (e.g. `bin: 2`)

Every move goes through console_grasp.move_to (its path check, the way round over the bins) and the console's
own checks; every picture through src/adapters/capture.py.

    python3 src/pipeline/observe.py list
    python3 src/pipeline/observe.py check field [--start 90,105,5,5]     # offline: limits, platform, paths
    python3 src/pipeline/observe.py run field --out DIR [--console URL] --really   # moves the arm
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
for _sub in ("tools", "tasks", "adapters", "flow", "teleop"):
    if str(ROOT / "src" / _sub) not in sys.path:
        sys.path.insert(0, str(ROOT / "src" / _sub))

SCRIPTS = ROOT / "config" / "observe"
TOP = ("name", "title", "j5", "settle_s", "pause_s", "board", "on_blocked", "event", "end", "views", "note")
VIEW_KEYS = ("pose", "name", "settle_s", "pause_s", "board", "j5", "note")
DEFAULTS = dict(j5="roll_zero", settle_s=0.8, pause_s=0.0, board=True, on_blocked="skip", event="scan_view", end=None)


def _parse(path: Path):
    text = path.read_text(encoding="utf-8")
    if path.suffix == ".json":
        return json.loads(text)
    from miniyaml import load
    return load(text)


def names() -> list[str]:
    return sorted(p.stem for p in SCRIPTS.glob("*.yaml")) + sorted(p.stem for p in SCRIPTS.glob("*.json"))


def problems(script) -> list[str]:
    """What is wrong with a script's shape ([] if nothing); limits and paths are `check`'s."""
    out = []
    if not isinstance(script, dict):
        return ["not a mapping"]
    out += [f"unknown key {k!r}" for k in script if k not in TOP]
    views = script.get("views")
    if not isinstance(views, list) or not views:
        return out + ["views: a non-empty list is needed"]
    if script.get("on_blocked", "skip") not in ("skip", "stop"):
        out.append("on_blocked: skip or stop")
    j5 = script.get("j5", "roll_zero")
    if j5 != "roll_zero" and not _num(j5):
        out.append("j5: roll_zero or a number")
    end = script.get("end")
    if end not in (None, "observe") and not _pose(end, (4,)):
        out.append("end: null, observe or [J1, J2, J3, J4]")
    for k in ("settle_s", "pause_s"):
        if k in script and not (_num(script[k]) and script[k] >= 0):
            out.append(f"{k}: a number >= 0")
    for i, v in enumerate(views):
        where = f"views[{i}]"
        if not isinstance(v, dict):
            out.append(f"{where}: a mapping with pose")
            continue
        if not _pose(v.get("pose"), (4, 5)):
            out.append(f"{where}.pose: [J1, J2, J3, J4] or [J1..J5], numbers")
        for k in ("settle_s", "pause_s"):
            if k in v and not (_num(v[k]) and v[k] >= 0):
                out.append(f"{where}.{k}: a number >= 0")
        if "j5" in v and v["j5"] != "roll_zero" and not _num(v["j5"]):
            out.append(f"{where}.j5: roll_zero or a number")
    return out


def _num(v) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def _pose(v, lengths) -> bool:
    return isinstance(v, list) and len(v) in lengths and all(_num(x) for x in v)


def load(script) -> dict:
    """A script by name (config/observe/<name>.yaml|json), path, or dict; checked and with defaults filled in."""
    if isinstance(script, dict):
        raw, source = script, "<dict>"
    else:
        p = Path(script)
        if not p.exists():
            p = next((SCRIPTS / f"{script}{ext}" for ext in (".yaml", ".json") if (SCRIPTS / f"{script}{ext}").exists()), p)
        if not p.exists():
            raise FileNotFoundError(f"no observation script {script!r} (have: {', '.join(names())})")
        raw, source = _parse(p), str(p)
    bad = problems(raw)
    if bad:
        raise ValueError(f"{source}: " + "; ".join(bad))
    out = dict(DEFAULTS, **{k: v for k, v in raw.items() if k != "views"}, source=source)
    out["views"] = []
    for i, v in enumerate(raw["views"]):
        view = dict(v)
        view.setdefault("name", f"view_{i:02d}")
        for k in ("settle_s", "pause_s", "board", "j5"):
            view.setdefault(k, out[k])
        out["views"].append(view)
    return out


def roll(j5) -> int:
    import console_grasp as G
    return G.ROLL_ZERO if j5 == "roll_zero" else int(round(j5))


def command(view: dict) -> list[int]:
    """[J1..J5] the view is taken at."""
    p = [int(round(x)) for x in view["pose"]]
    return p if len(p) == 5 else p + [roll(view["j5"])]


# ------------------------------------------------------------------------------------------------- run

def run(script, run_dir: Path, session=None, views_file: str = "views.json", image: str = "view_{i:02d}.jpg",
        append: bool = False, rel_to: Path = ROOT) -> list[dict]:
    """Visit the views and take a picture at each; <run_dir>/<views_file> is written after every picture (a scan
    stopped halfway keeps what it took). `image` names the files ({i}: the running index in views_file, {name}:
    the view's name, and any key of the view). append: add to an existing views_file. Moves the arm."""
    import capture as CAP
    import console_grasp as G
    sc = load(script)
    run_dir.mkdir(parents=True, exist_ok=True)
    path = run_dir / views_file
    views = json.loads(path.read_text(encoding="utf-8")) if append and path.exists() else []
    rel = str(run_dir.relative_to(rel_to)) if run_dir.is_relative_to(rel_to) else str(run_dir)
    extra_keys = lambda v: {k: x for k, x in v.items() if k not in VIEW_KEYS}   # noqa: E731
    with G.lease(session) as sess:
        for n, v in enumerate(sc["views"]):
            cmd = command(v)
            try:
                G.move_to(sess, cmd[:4], cmd[4])
            except G.PathBlocked as e:       # a bin stands where this view puts the fingers: take the others
                G.log("scan_view_skipped", run=rel, view=n, name=v["name"], pose=cmd, reason=str(e))
                if sc["on_blocked"] == "stop":
                    raise
                continue
            if v["pause_s"]:
                time.sleep(float(v["pause_s"]))
            try:
                shot = CAP.shot(None, board=bool(v["board"]), settle_s=float(v["settle_s"]))
            except CAP.CaptureError as e:
                raise G.ConsoleError(str(e)) from e
            i = len(views)
            name = image.format(i=i, **{k: x for k, x in v.items() if k != "pose"})
            (run_dir / name).write_bytes(shot.jpeg)
            views.append(shot.record(name, command=cmd, name=v["name"], script=sc["name"] if "name" in sc else None,
                                     **extra_keys(v)))
            path.write_text(json.dumps(views, indent=1, ensure_ascii=False), encoding="utf-8")
            b = shot.board or {}
            G.log(sc["event"], run=rel, view=i, pose=shot.pose, name=v["name"], markers=b.get("markers"),
                  official=[(d.get("label"), d.get("conf")) for d in b.get("official", [])], **extra_keys(v))
        end = sc.get("end")
        if end is not None:
            G.move_to(sess, list(G.OBSERVE) if end == "observe" else [int(round(x)) for x in end])
    return views


# ----------------------------------------------------------------------------------------------- check

def check(script, start=None, obstacles=None) -> list[dict]:
    """Offline, nothing moves: each view's limits (the console's runtime limits), its platform clearance, and the
    way to it from the one before (console_grasp.check_path; `around`: blocked straight, taken over HIGH_LEG as
    move_to would). Rows {view, name, pose, ok, platform_cm, path, note}."""
    import console_grasp as G
    import safe_arm
    from motion_policy import runtime_config
    sc = load(script)
    cfg = runtime_config(safe_arm.load_config(safe_arm.DEFAULT_CONFIG))
    j6 = int(cfg["gripper"]["open_deg"])
    saved = list(G.OBSTACLES)
    if obstacles is not None:
        G.set_obstacles(obstacles)
    rows = []
    try:
        here = [int(v) for v in (start or G.OBSERVE)][:4]
        prev6 = here + [G.ROLL_ZERO, j6]
        for n, v in enumerate(sc["views"]):
            cmd = command(v)
            row = dict(view=n, name=v["name"], pose=cmd, ok=True, path="direct", note="")
            try:
                safe_arm.validate_target(prev6, cmd + [j6], cfg)
            except safe_arm.MotionError as e:
                row.update(ok=False, note=f"limits: {e}")
            row["platform_cm"] = round(G.workspace.platform_clearance(cmd[:4], G.KEEP_OUT) * 100, 2)
            try:
                G.check_path(here, cmd[:4])
            except G.PathBlocked:
                legs = [[here[0], *G.HIGH_LEG[1:]], [cmd[0], *G.HIGH_LEG[1:]], cmd[:4]]
                try:
                    a = here
                    for leg in legs:
                        G.check_path(a, leg)
                        a = leg
                    row["path"] = "around"
                except G.ConsoleError as e:
                    row.update(ok=False, path="blocked", note=str(e))
            except G.ConsoleError as e:
                row.update(ok=False, path="refused", note=str(e))
            rows.append(row)
            if row["ok"]:
                here, prev6 = cmd[:4], cmd + [j6]
    finally:
        G.set_obstacles(saved)
    return rows


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("list")
    c = sub.add_parser("check")
    c.add_argument("script")
    c.add_argument("--start", help="J1-J4 the arm starts from (default the observe pose)")
    r = sub.add_parser("run")
    r.add_argument("script")
    r.add_argument("--out", type=Path, required=True)
    r.add_argument("--console")
    r.add_argument("--really", action="store_true", help="it moves the arm")
    a = ap.parse_args(argv)
    if a.cmd == "list":
        for n in names():
            sc = load(n)
            print(f"{n:16s} {len(sc['views']):2d} views  {sc.get('title', '')}")
        return 0
    if a.cmd == "check":
        rows = check(a.script, [int(v) for v in a.start.split(",")] if a.start else None)
        for row in rows:
            print(f"{row['view']:2d} {row['name']:14s} {row['pose']}  platform {row['platform_cm']:+5.1f} cm  "
                  f"{row['path']:8s} {'ok' if row['ok'] else 'NO'}  {row['note']}")
        return 0 if all(r["ok"] for r in rows) else 1
    if not a.really:
        raise SystemExit("run moves the arm: pass --really (and be on site with the real arm)")
    sys.path.insert(0, str(ROOT / "src"))
    from flow import stages as ST
    url = ST._use_console(a.console)
    ST._confirm_onsite(url, None, f"observation script {a.script}")
    views = run(a.script, a.out)
    print(json.dumps(dict(out=str(a.out), views=len(views)), ensure_ascii=False))
    return 0


if __name__ == "__main__":
    sys.exit(main())
