"""Per-view pose solving against the simulator's truth: the old per-view PnP and the joint bundle on the same images.

    uv run --with 'opencv-python<5' --with numpy --with onnxruntime python dev/tools/pose_bench.py \
        [--console http://127.0.0.1:8770] [--jitter 0.7,0.8,0.8,0.8,0] [--seeds 1-6] [--blocks 4]

The simulator (dev/sim/sim_server.py) draws a fresh servo error at every move (`pose_jitter_deg`, sigma per joint)
and renders the camera where the arm really is, while the readback says the command -- as on the real arm,
whose views are off by ~1.5 deg / ~10 px each (doc 30 §6). Each seed: a random layout, one scan, then
app/src/pipeline/run.build_scene twice on the same views (scene.POSES "pnp" and "bundle"). Scored per block
against the truth: centre error (cm, map frame), yaw error, found; and the map placement against the
simulator's (exact) placement. Results: data/sim/pose_bench.json.
"""
from __future__ import annotations

import argparse
import contextlib
import io
import json
import math
import shutil
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
for sub in ("tools", "tasks", "pipeline", "spatial", "vision", "adapters"):
    sys.path.insert(0, str(ROOT / "app" / "src" / sub))
sys.path.insert(0, str(ROOT / "app" / "src"))

import console_step as CS  # noqa: E402


def seeds(text: str) -> list[int]:
    out = []
    for part in text.split(","):
        a, _, b = part.partition("-")
        out += list(range(int(a), int(b or a) + 1))
    return out


def yaw_err(a: float, b: float) -> float:
    return abs((a - b + 45) % 90 - 45)


def score(scene: dict, truth: list[dict], table_offset) -> dict:
    cubes = scene["cubes"]
    used, rows = set(), []
    for t in truth:
        free = [i for i in range(len(cubes)) if i not in used]
        near = min(free, key=lambda i: math.dist(cubes[i]["map_cm"], t["map_cm"]), default=None)
        if near is None or math.dist(cubes[near]["map_cm"], t["map_cm"]) > 2.5:
            rows.append(dict(found=False))
            continue
        used.add(near)
        c = cubes[near]
        rows.append(dict(found=True, err_cm=round(math.dist(c["map_cm"], t["map_cm"]), 3),
                         yaw_err=round(yaw_err(c["yaw_deg"], t["yaw_deg"]), 1)))
    reg = scene["registration"]
    place_err = math.dist([v / 100 for v in reg["map_origin_base_cm"][:2]], list(table_offset[:2])) * 100
    return dict(blocks=rows, extra=len(cubes) - len(used), placement_err_cm=round(place_err, 3),
                placement_yaw_deg=reg.get("yaw_deg"), bundle=reg.get("bundle"))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--console", default="http://127.0.0.1:8770")
    ap.add_argument("--jitter", default="0.7,0.8,0.8,0.8,0")
    ap.add_argument("--seeds", default="1-6")
    ap.add_argument("--blocks", type=int, default=4)
    ap.add_argument("--out", type=Path, default=ROOT / "data" / "sim" / "pose_bench.json")
    a = ap.parse_args()
    CS.BASE = a.console.rstrip("/")
    from flow import stages as ST
    from flow.session import FlowSession
    import scene as SC
    import run as R
    tok = CS.call("/api/bootstrap")["token"]
    jitter = [float(v) for v in a.jitter.split(",")]
    CS.call("/api/sim/faults", {"pose_jitter_deg": jitter}, token=tok)
    work = ROOT / "data" / "sim" / "pose_bench"
    rows = []
    try:
        for seed in seeds(a.seeds):
            world = CS.call("/api/sim/reset", {"count": a.blocks, "seed": seed}, token=tok)
            truth = [dict(map_cm=b["map_cm"], yaw_deg=b["yaw_deg"]) for b in world["blocks"]]
            sess = FlowSession(work / f"s{seed}")
            shutil.rmtree(sess.path, ignore_errors=True)
            sess = FlowSession(work / f"s{seed}")
            with contextlib.redirect_stdout(io.StringIO()):
                ST.scan(sess, a.console, really=True)
            offset = SC.TableMap().offset
            for mode in ("pnp", "bundle"):
                SC.POSES = mode
                with contextlib.redirect_stdout(io.StringIO()):
                    scene = R.build_scene(sess.file("scan"))
                r = dict(seed=seed, mode=mode, **score(scene, truth, offset))
                rows.append(r)
                errs = [b["err_cm"] for b in r["blocks"] if b["found"]]
                print(f"seed {seed} {mode:6s}: found {len(errs)}/{len(truth)}, centre err cm {errs}, yaw err "
                      f"{[b['yaw_err'] for b in r['blocks'] if b['found']]}, placement off {r['placement_err_cm']} cm"
                      + (f", bundle {r['bundle']}" if r.get('bundle') else ""), flush=True)
    finally:
        CS.call("/api/sim/faults", {"pose_jitter_deg": [0, 0, 0, 0, 0]}, token=tok)
    for mode in ("pnp", "bundle"):
        errs = sorted(b["err_cm"] for r in rows if r["mode"] == mode for b in r["blocks"] if b["found"])
        yaws = sorted(b["yaw_err"] for r in rows if r["mode"] == mode for b in r["blocks"] if b["found"])
        n = sum(len(r["blocks"]) for r in rows if r["mode"] == mode)
        if errs:
            q = lambda v, p: v[min(len(v) - 1, int(p * len(v)))]   # noqa: E731
            print(f"{mode}: found {len(errs)}/{n}; centre cm median {q(errs, .5):.2f} p90 {q(errs, .9):.2f} max "
                  f"{errs[-1]:.2f}; yaw median {q(yaws, .5):.1f} p90 {q(yaws, .9):.1f}; >1 cm {sum(e > 1 for e in errs)}; "
                  f"placement off cm median {sorted(r['placement_err_cm'] for r in rows if r['mode'] == mode)[len(rows) // 4]}")
    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(json.dumps(rows, ensure_ascii=False, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
