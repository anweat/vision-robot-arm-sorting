"""The step flow (app/config/flows/sort.yaml) and the verified pipeline (app/src/pipeline/run.py), on the same
random layouts of the simulator with bins, judged by its ground truth (doc 34 §8).

    ARM_MAP=map_a4_v4 uv run --with 'opencv-contrib-python<5' --with numpy --with onnxruntime \
        python dev/sim/compare_sort.py --console http://127.0.0.1:8771 --seeds 41-47 [--only flow|pipeline]

Needs dev/sim/sim_server.py with the map and bins of the real table (`--map map_a4_v4 --bins bins_acrylic_v2_pair`,
launch config arm-sim-bins). Each seed: /api/sim/reset count 4, then one round; a block counts when the truth
has it inside the bin of its category. Bin and block contacts are the simulator's records during the round.
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "app" / "src" / "tools"))
import console_step as CS  # noqa: E402

RUNNERS = {
    "flow": lambda base: [sys.executable, str(ROOT / "app" / "src" / "flow" / "orchestrator.py"), "--script",
                          str(ROOT / "app" / "config" / "flows" / "sort.yaml"), "--console", base, "--really",
                          "--log", str(ROOT / "data" / "sim" / "compare_log.jsonl")],
    "pipeline": lambda base: [sys.executable, str(ROOT / "dev" / "sim" / "run_pipeline.py"), "--base", base,
                              "--bins", "bins_acrylic_v2_pair", "--keep-scene", "--rounds", "1"],
}


def seeds(text: str) -> list[int]:
    out = []
    for part in text.split(","):
        a, _, b = part.partition("-")
        out += list(range(int(a), int(b or a) + 1))
    return out


def judge(world: dict, before: dict) -> dict:
    cat = {p["id"]: p["category"] for p in world["props"]}
    able = [b for b in world["blocks"] if b["category"] in cat.values()]
    return dict(binnable=len(able), right=sum(1 for b in able if b["inside"] and cat[b["inside"]] == b["category"]),
                wrong=sum(1 for b in world["blocks"] if b["inside"] and cat[b["inside"]] != b["category"]),
                bin_contacts=world["contacts"]["count"] - before["contacts"]["count"],
                block_contacts=world["block_contacts"]["count"] - before["block_contacts"]["count"],
                platform_contacts=world["platform_contacts"]["count"] - before["platform_contacts"]["count"])


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--console", default="http://127.0.0.1:8771")
    ap.add_argument("--seeds", default="41-47")
    ap.add_argument("--only", choices=tuple(RUNNERS))
    ap.add_argument("--out", type=Path, default=ROOT / "data" / "sim" / "compare_sort.json")
    a = ap.parse_args()
    CS.BASE = a.console.rstrip("/")
    tok = CS.call("/api/bootstrap")["token"]
    rows = []
    for name in ([a.only] if a.only else list(RUNNERS)):
        for seed in seeds(a.seeds):
            CS.call("/api/sim/reset", {"count": 4, "seed": seed}, token=tok)
            before = CS.call("/api/sim/world")
            t0 = time.time()
            done = subprocess.run(RUNNERS[name](a.console), cwd=ROOT, capture_output=True, text=True)
            r = dict(runner=name, seed=seed, exit=done.returncode, seconds=round(time.time() - t0),
                     **judge(CS.call("/api/sim/world"), before))
            rows.append(r)
            print(json.dumps(r, ensure_ascii=False), flush=True)
    for name in sorted({r["runner"] for r in rows}):
        rs = [r for r in rows if r["runner"] == name]
        print(f"{name}: right bin {sum(r['right'] for r in rs)}/{sum(r['binnable'] for r in rs)}, wrong "
              f"{sum(r['wrong'] for r in rs)}, bin contacts {sum(r['bin_contacts'] for r in rs)}, block contacts "
              f"{sum(r['block_contacts'] for r in rs)}, platform {sum(r['platform_contacts'] for r in rs)}")
    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(json.dumps(rows, ensure_ascii=False, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
