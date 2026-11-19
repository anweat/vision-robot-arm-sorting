"""Round after round of sort-then-scatter, unattended, so success rates come from counted rounds.

    python3 src/pipeline/loop.py --rounds 10          # sort, scatter, sort, ... (moves the arm)
    python3 src/pipeline/loop.py --rounds 2 --seed 4  # repeatable scatter
    python3 src/pipeline/loop.py --status             # the tracked blocks, no motion

The loop alternates: when every block is on its bin the next phase scatters them over free markers,
otherwise the next phase sorts them. Both phases are grasps and places, so both measure the grip; only
the sorting phases measure the sorting.

**Where the truth comes from.** The categories are not taken from the detector -- that is what is being
measured. The session starts from a sorted table, where each block's identity is the bin it stands on,
and from then on identity travels with the block: the loop moved it, so it knows which block is where.
A detection is then scored against a label the detector had no part in. If a tracked block cannot be
matched to anything in a scan, the loop stops instead of guessing: a wrong identity would quietly
poison every round after it.

**What it cannot do.** It never turns a block over (the arm drops a tilted block, 2026-09-25), so one
session sees one face per block. Turn them by hand between sessions, or the numbers only cover four faces.
"""
from __future__ import annotations

import argparse
import json
import math
import random
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
for sub in ("pipeline", "tasks", "tools", "spatial", "vision"):
    sys.path.insert(0, str(ROOT / "src" / sub))

import console_grasp as G  # noqa: E402
import run as R  # noqa: E402
import scramble as SCR  # noqa: E402
import sort_blocks as SB  # noqa: E402

RUNS = ROOT / "data" / "runs" / "loop"
MATCH_CM = 4.0        # a tracked block and its detection are this near; blocks stand >= 8.5 cm apart
ON_BIN_CM = 2.5       # SORTING["in_place_cm"] plus the place error seen on 2026-09-26 (2.2 cm)


def bin_of(category: str) -> str:
    return SB.SORTING["category_bins"][category]


def bin_centre(category: str) -> list[float]:
    return [round(v * 100, 1) for v in G.marker_centre(bin_of(category))]


def match(tracked: list[dict], cubes: list[dict]) -> tuple[list[dict], list[dict]]:
    """Pair each tracked block with the nearest detection within MATCH_CM. Returns the pairings and the
    detections nothing claimed (extras: a side face squared onto the plane, or a block that moved)."""
    pairs, free = [], list(cubes)
    for t in tracked:
        near = sorted(free, key=lambda c: math.dist(c["map_cm"], t["map_cm"]))
        got = near[0] if near and math.dist(near[0]["map_cm"], t["map_cm"]) <= MATCH_CM else None
        if got is not None:
            free.remove(got)
        pairs.append(dict(block=t, cube=got,
                          error_cm=round(math.dist(got["map_cm"], t["map_cm"]), 2) if got else None))
    return pairs, free


def score(pairs: list[dict], extras: list[dict]) -> dict:
    """What this scan says about the detector, against identities it did not supply."""
    found = [p for p in pairs if p["cube"]]
    right = [p for p in found if p["cube"]["category"] == p["block"]["category"]]
    return dict(
        blocks=len(pairs), found=len(found), right=len(right),
        wrong=[dict(truth=p["block"]["category"], said=p["cube"]["category"], at=p["block"]["map_cm"])
               for p in found if p["cube"]["category"] != p["block"]["category"]],
        no_category=sum(p["cube"]["category"] is None for p in found),
        missed=[p["block"]["category"] for p in pairs if not p["cube"]],
        extras=[c["map_cm"] for c in extras],
        centre_err_cm=sorted(p["error_cm"] for p in found) if found else [],
    )


def bootstrap(cubes: list[dict]) -> list[dict]:
    """Identities from a sorted table: the block on the kitchen bin is the kitchen block. Anything else
    is refused -- an identity guessed here is an identity wrong in every later round."""
    tracked = []
    for category in SB.SORTING["category_bins"]:
        centre = bin_centre(category)
        near = [c for c in cubes if math.dist(c["map_cm"], centre) <= ON_BIN_CM]
        if len(near) != 1:
            raise SystemExit(
                f"起始状态不对：{category} 的格子上有 {len(near)} 块。\n"
                f"先让桌面回到已分拣状态（python3 src/pipeline/run.py），确认四块各就各位后再开始。")
        tracked.append(dict(category=category, map_cm=list(near[0]["map_cm"])))
    return tracked


def on_bins(tracked: list[dict]) -> bool:
    return all(math.dist(t["map_cm"], bin_centre(t["category"])) <= ON_BIN_CM for t in tracked)


def scan_and_look(sess, session_dir: Path):
    import field_scan as SC
    run = SC.scan(session=sess)
    scene = R.build_scene(run)
    R.use_registration(scene["registration"])
    return run, scene


def phase(tracked: list[dict], scene: dict, sess, rng: random.Random) -> dict:
    """One sorting phase or one scattering phase; returns what happened, with the tracked positions
    updated to where the arm put things."""
    pairs, extras = match(tracked, scene["cubes"])
    result = dict(detection=score(pairs, extras))
    for p in pairs:                                    # a detection is this block's, whatever it is called
        if p["cube"]:
            p["block"]["map_cm"] = list(p["cube"]["map_cm"])

    if on_bins(tracked):
        result["phase"] = "scatter"
        cubes = [p["cube"] for p in pairs if p["cube"]]
        moved = []
        markers = SCR.draw(len(cubes), rng)
        order = sorted(range(len(cubes)), key=lambda i: -math.dist(cubes[i]["map_cm"], [0, 0]))
        pool, placed = SCR.candidates(), [c["map_cm"] for c in cubes]
        for i, marker in zip(order, markers):
            block = next(p["block"] for p in pairs if p["cube"] is cubes[i])
            here = [p for p in placed if p is not cubes[i]["map_cm"]]
            try:
                got = SCR.move(cubes[i], SCR.alternatives(marker, here, pool), R.plan_for, sess)
                moved.append(got)
                block["map_cm"] = list(got["target_map_cm"])
                placed.append(got["target_map_cm"])
            except SCR.StillHolding:
                raise                       # the arm is holding a block: stop, do not open the fingers
            except (G.ConsoleError, G.GripMissed) as e:
                moved.append(dict(marker=marker, frm=cubes[i]["map_cm"], error=str(e)))
                G.back_off(sess)          # a stop leaves the arm where it stopped; start the next fresh
        result["moved"] = moved
        result["failed"] = sum("error" in m for m in moved)
        result["moved_count"] = len(moved) - result["failed"]
        return result

    result["phase"] = "sort"
    jobs = R.decide(scene)
    result["counts"] = R.execute(scene.get("run", "loop"), scene, jobs, sess, rng=rng)
    result["moved_count"] = result["counts"]["sorted"] + result["counts"]["cleared"]
    for p in pairs:                                    # where the sort put each block, by its own identity
        if p["cube"] and p["cube"].get("category") and any(
                j["cube"] is p["cube"] and not j["skip"] for j in jobs):
            p["block"]["map_cm"] = bin_centre(p["cube"]["category"])
    result["failed"] = result["counts"]["missed"]
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--rounds", type=int, default=4)
    parser.add_argument("--seed", type=int)
    parser.add_argument("--status", action="store_true", help="print the tracked blocks and stop")
    parser.add_argument("--stop-after-failures", type=int, default=2,
                        help="consecutive phases with a failed move before the loop stops")
    args = parser.parse_args()
    rng = random.Random(args.seed)

    session_dir = RUNS / time.strftime("%Y%m%d_%H%M%S")
    state_file = RUNS / "state.json"
    if args.status:
        print(state_file.read_text() if state_file.exists() else "还没有跟踪状态")
        return

    G.require_empty_gripper()
    session_dir.mkdir(parents=True, exist_ok=True)
    records = session_dir / "rounds.jsonl"
    tracked, consecutive = None, 0
    with G.Session() as sess:
        for n in range(args.rounds):
            run, scene = scan_and_look(sess, session_dir)
            if tracked is None:
                tracked = bootstrap(scene["cubes"])
                print("起始身份：", ", ".join(f"{t['category']}@{t['map_cm']}" for t in tracked))
            scene["run"] = str(run)
            out = phase(tracked, scene, sess, rng)
            out.update(round=n + 1, run=str(run), at=time.time(),
                       tracked=[dict(t) for t in tracked])
            with records.open("a") as f:
                f.write(json.dumps(out, ensure_ascii=False) + "\n")
            state_file.write_text(json.dumps(dict(tracked=tracked, at=time.time()), ensure_ascii=False, indent=1))

            d = out["detection"]
            print(f"\n第 {n + 1} 轮 [{out['phase']}]  识别 {d['found']}/{d['blocks']} 找到、"
                  f"{d['right']}/{d['found']} 类别正确"
                  + (f"，判错 {d['wrong']}" if d["wrong"] else "")
                  + (f"，漏检 {d['missed']}" if d["missed"] else "")
                  + (f"，多报 {len(d['extras'])}" if d["extras"] else "")
                  + f"，动作失败 {out['failed']}")

            if d["missed"]:
                print("有跟踪中的块没有在扫描里找到，停止：身份可能已经对不上。")
                break
            if not out["moved_count"]:
                # nothing moved and nothing can: alternating scans forever would look like progress
                print("这一阶段一块也没动（都够不着或都已就位），停止。原因见日志的 pipeline_skip。")
                break
            consecutive = consecutive + 1 if out["failed"] else 0
            if consecutive >= args.stop_after_failures:
                print(f"连续 {consecutive} 个阶段有动作失败，停止。")
                break
        G.observe(sess)
    print("\n记录：", records)


if __name__ == "__main__":
    main()
