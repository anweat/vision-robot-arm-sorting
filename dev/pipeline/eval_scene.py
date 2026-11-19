"""Score the whole scene pipeline against the placed rounds: found, category, centre and yaw.

`build_scene` (app/src/pipeline/run.py) is what the sorting run acts on, so it is what has to be verified,
not the detector alone: a block the detector names but the orthophoto squares onto a side face is still
a failed grasp. For every round recorded by dev/pipeline/collect.py in "placed" mode the labelled blocks
are the truth (the operator put them on named markers, the centre refined on the orthophoto), and each
is matched to the nearest scene block.

    python3 dev/pipeline/eval_scene.py data/datasets/blocks4/<ds>                 # every placed round
    python3 dev/pipeline/eval_scene.py data/datasets/blocks4/<ds> --rounds round_017,round_021 \
        --model data/models/blocks4/v2                                            # compare a version
"""
from __future__ import annotations

import argparse
import contextlib
import io
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
for sub in ("pipeline", "tasks", "tools", "spatial", "vision"):
    sys.path.insert(0, str(ROOT / "dev" / sub))
    sys.path.insert(0, str(ROOT / "app" / "src" / sub))

MATCH_CM = 2.0          # a scene block this near the truth is that block
GRASP_CM = 1.0          # the fingers are 3 cm apart on a 3 cm block: more than ~1 cm off misses
GRASP_YAW_DEG = 12.0


def yaw_error(a: float, b: float) -> float:
    """Degrees between two square orientations (a square repeats every 90)."""
    return abs((a - b + 45) % 90 - 45)


def evaluate(dataset: Path, rounds: list[str] | None, model_dir: Path | None, verbose: bool) -> dict:
    import blocks_detector as B
    if model_dir is not None:
        B.MODEL_DIR = model_dir
    import run as R

    records = [json.loads(line) for line in (dataset / "track.jsonl").read_text().splitlines()]
    records = [r for r in records if r.get("mode") == "placed" and r.get("labelled")]
    if rounds:
        records = [r for r in records if r["round"] in rounds]
    total = dict(blocks=0, found=0, right=0, wrong=0, no_category=0, missed=0, extra=0, graspable=0)
    centre_errs: list[float] = []
    yaw_errs: list[float] = []
    problems: list[str] = []
    for rec in records:
        with contextlib.redirect_stdout(io.StringIO()):
            scene = R.build_scene(dataset / rec["round"])
        cubes = scene["cubes"]
        used: set[int] = set()
        for truth in rec["labelled"]:
            total["blocks"] += 1
            free = [i for i in range(len(cubes)) if i not in used]
            near = min(free, key=lambda i: math.dist(cubes[i]["map_cm"], truth["map_cm"]), default=None)
            if near is None or math.dist(cubes[near]["map_cm"], truth["map_cm"]) > MATCH_CM:
                total["missed"] += 1
                problems.append(f"{rec['round']} MISSED   {truth['category']:10s} at {truth['map_cm']}")
                continue
            used.add(near)
            cube = cubes[near]
            total["found"] += 1
            err = math.dist(cube["map_cm"], truth["map_cm"])
            dyaw = yaw_error(cube["yaw_deg"], truth["yaw_deg"])
            centre_errs.append(err)
            yaw_errs.append(dyaw)
            if cube["category"] == truth["category"]:
                total["right"] += 1
                if err <= GRASP_CM and dyaw <= GRASP_YAW_DEG:
                    total["graspable"] += 1
                elif verbose:
                    problems.append(f"{rec['round']} OFF      {truth['category']:10s} "
                                    f"{err:.1f} cm, yaw {dyaw:.0f} deg")
            elif cube["category"] is None:
                total["no_category"] += 1
                problems.append(f"{rec['round']} NO CLASS {truth['category']:10s} ({cube['source'][:50]})")
            else:
                total["wrong"] += 1
                problems.append(f"{rec['round']} WRONG    {truth['category']:10s} -> {cube['category']:10s} "
                                f"({cube['source'][:50]})")
        for i, cube in enumerate(cubes):
            if i not in used:
                total["extra"] += 1
                problems.append(f"{rec['round']} EXTRA    at {cube['map_cm']} -> {cube['category']}")
    for line in problems:
        print(line)
    n = max(1, total["blocks"])
    summary = dict(total, rounds=len(records), model=str(model_dir or "default"),
                   found_pct=round(100 * total["found"] / n, 1),
                   right_pct=round(100 * total["right"] / n, 1),
                   graspable_pct=round(100 * total["graspable"] / n, 1),
                   centre_err_cm=dict(median=round(sorted(centre_errs)[len(centre_errs) // 2], 2) if centre_errs else None,
                                      p90=round(sorted(centre_errs)[int(len(centre_errs) * 0.9)], 2) if centre_errs else None,
                                      max=round(max(centre_errs), 2) if centre_errs else None),
                   yaw_err_deg=dict(median=round(sorted(yaw_errs)[len(yaw_errs) // 2], 1) if yaw_errs else None,
                                    max=round(max(yaw_errs), 1) if yaw_errs else None))
    print(json.dumps(summary, ensure_ascii=False))
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("dataset", type=Path)
    parser.add_argument("--rounds", help="comma-separated rounds (default: every placed round)")
    parser.add_argument("--model", type=Path, help="model directory for blocks_detector (default: its own)")
    parser.add_argument("--out", type=Path, help="write the summary here as JSON")
    parser.add_argument("-v", "--verbose", action="store_true", help="also list blocks found but off-centre")
    args = parser.parse_args()
    summary = evaluate(args.dataset.resolve(), args.rounds.split(",") if args.rounds else None,
                       args.model.resolve() if args.model else None, args.verbose)
    if args.out:
        args.out.write_text(json.dumps(summary, indent=1, ensure_ascii=False))


if __name__ == "__main__":
    main()
