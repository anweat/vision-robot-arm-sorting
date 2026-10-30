"""Run the 4-class block detector on simulated wrist-camera frames and score it against the scene.

    uv run --with 'opencv-python<5' --with numpy --with onnxruntime src/sim/evaluate.py -n 50

Each scene is a random set of blocks seen from the six scan poses of src/tasks/scan.py. A block counts
as expected when at least 150 of its pixels are visible and 10 % of its box is inside the frame.

This is a consistency check, not a measure of real accuracy: the blocks are textured with the same 20
face photographs that produced 800 of the detector's training frames (src/vision/synth_blocks.py), and
the picture has no arm, no real lighting and no real background.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
for sub in ("vision", "sim"):
    sys.path.insert(0, str(ROOT / "src" / sub))

import render as R  # noqa: E402
import world as W  # noqa: E402

SCAN_POSES = [[s1, 105, 5, 5, 95, 30] for s1 in (70, 90, 110)] + [[s1, 95, 10, 5, 95, 30] for s1 in (105, 90, 75)]
MATCH_IOU = 0.5


def iou(a, b) -> float:
    w = min(a[2], b[2]) - max(a[0], b[0])
    h = min(a[3], b[3]) - max(a[1], b[1])
    if w <= 0 or h <= 0:
        return 0.0
    return w * h / ((a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - w * h)


def compare(truth: list[dict], dets: list[dict]) -> dict:
    """Match detections to blocks by box overlap, best overlap first, whatever the class.

    Each block: `result` is correct / wrong_class / missed (expected blocks) or ignored (too little of it
    in view to count either way). Detections left over are false positives unless they sit on an
    ignored block."""
    pairs = sorted(((iou(t["bbox"], d["bbox"]), i, j) for i, t in enumerate(truth) for j, d in enumerate(dets)),
                   reverse=True)
    block_of, det_of = {}, {}
    for overlap, i, j in pairs:
        if overlap < MATCH_IOU:
            break
        if i not in det_of and j not in block_of:
            det_of[i], block_of[j] = (j, overlap), i
    blocks = []
    for i, t in enumerate(truth):
        item = dict(t, detection=None, iou=None)
        if i in det_of:
            j, overlap = det_of[i]
            item.update(detection=dets[j], iou=round(overlap, 3))
        if not t["expected"]:
            item["result"] = "ignored"
        elif i not in det_of:
            item["result"] = "missed"
        else:
            item["result"] = "correct" if dets[det_of[i][0]]["category"] == t["category"] else "wrong_class"
        blocks.append(item)
    loose = [d for j, d in enumerate(dets) if j not in block_of]
    # an unmatched box over a block too little in view to count is not held against the detector
    ignored = [t["bbox"] for t in truth if not t["expected"]]
    false_positives = [d for d in loose if not any(iou(d["bbox"], b) > 0.1 for b in ignored)]
    count = {k: sum(b["result"] == k for b in blocks) for k in ("correct", "wrong_class", "missed", "ignored")}
    return dict(blocks=blocks, false_positives=false_positives, summary=dict(count, false_positives=len(false_positives)))


def annotate(image: np.ndarray, result: dict) -> np.ndarray:
    """Truth boxes thin white, matched detections green (right class) or orange (wrong), false positives red."""
    out = image.copy()

    def box(b, colour, width, text=None):
        x0, y0, x1, y1 = [int(round(v)) for v in b]
        cv2.rectangle(out, (x0, y0), (x1, y1), colour, width)
        if text:
            cv2.putText(out, text, (x0 + 2, max(12, y0 - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, colour, 1, cv2.LINE_AA)

    for b in result["blocks"]:
        box(b["bbox"], (255, 255, 255), 1)
        if b["detection"]:
            d = b["detection"]
            box(d["bbox"], (0, 200, 0) if d["category"] == b["category"] else (0, 140, 255), 2, f"{d['category']} {d['conf']:.2f}")
        elif b["result"] == "missed":
            box(b["bbox"], (255, 255, 255), 1, f"missed {b['category']}")
    for d in result["false_positives"]:
        box(d["bbox"], (0, 0, 255), 2, f"FP {d['category']} {d['conf']:.2f}")
    return out


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("-n", "--scenes", type=int, default=50)
    parser.add_argument("--blocks", type=int, default=4)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--model", type=Path, help="ONNX model (default: the detector's own)")
    parser.add_argument("--map", default=W.DEFAULT_MAP.stem, help="config/map_*.json name, or 'none'")
    parser.add_argument("--bins", help="config/bins_*.json name: bins standing at their planned places")
    parser.add_argument("--out", type=Path, help="default data/sim/eval_<time>")
    args = parser.parse_args()
    from blocks_detector import BlocksDetector
    detector = BlocksDetector(args.model, "onnx")
    out = args.out or ROOT / "data" / "sim" / time.strftime("eval_%Y%m%d_%H%M%S")
    out.mkdir(parents=True, exist_ok=True)
    world = W.World(args.seed, args.map)
    if args.bins:
        world.load_bins(args.bins)
    renderer = R.Renderer(world.items)
    renderer.set_map(world.table, world.map_data)
    noise = np.random.default_rng(args.seed)
    total = dict(correct=0, wrong_class=0, missed=0, ignored=0, false_positives=0)
    per_class = {c: dict(correct=0, wrong_class=0, missed=0) for c in W.CATEGORIES}
    confusion = {c: {d: 0 for d in W.CATEGORIES} for c in W.CATEGORIES}
    tiles = []
    with (out / "frames.jsonl").open("w") as log:
        for scene in range(args.scenes):
            world.reset(args.blocks)
            blocks = world.render_list()[1]
            for view, pose in enumerate(SCAN_POSES):
                frame, truth = renderer.render(pose, blocks, world.solids(), world.glass(), world.decals())
                image = R.sensor(frame, noise)
                result = compare(truth, detector.detect(image, conf=args.conf))
                for k in total:
                    total[k] += result["summary"][k]
                for b in result["blocks"]:
                    if b["result"] in per_class[b["category"]]:
                        per_class[b["category"]][b["result"]] += 1
                    if b["result"] in ("correct", "wrong_class"):
                        confusion[b["category"]][b["detection"]["category"]] += 1
                log.write(json.dumps(dict(scene=scene, view=view, servos=pose, **result["summary"])) + "\n")
                bad = result["summary"]["wrong_class"] + result["summary"]["missed"] + result["summary"]["false_positives"]
                if len(tiles) < 12 and (bad or scene == 0):
                    tiles.append(cv2.resize(annotate(image, result), (320, 240)))
    expected = total["correct"] + total["wrong_class"] + total["missed"]
    detections = total["correct"] + total["wrong_class"] + total["false_positives"]
    summary = dict(model=str(detector.model), scenes=args.scenes, views_per_scene=len(SCAN_POSES), blocks_per_scene=args.blocks,
                   seed=args.seed, conf=args.conf, map=world.map_name, bins=args.bins, match_iou=MATCH_IOU, counts=total, expected_blocks=expected,
                   recall=round(total["correct"] / expected, 4) if expected else None,
                   precision=round(total["correct"] / detections, 4) if detections else None,
                   per_class=per_class, confusion_truth_to_detected=confusion,
                   evidence="simulated frames; same face artwork as the synthetic training set; not a real-camera result")
    (out / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2))
    if tiles:
        while len(tiles) % 4:
            tiles.append(np.zeros_like(tiles[0]))
        cv2.imwrite(str(out / "sheet.jpg"), np.vstack([np.hstack(tiles[i:i + 4]) for i in range(0, len(tiles), 4)]))
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    print("written to", out)


if __name__ == "__main__":
    main()
