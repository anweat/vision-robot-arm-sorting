"""Check the labels of a scan round against the current block detector (a wrong hand placement shows up).

For each labelled block, over the views where it has a box: how often the detector finds a box there
(IoU >= 0.4) and with which class. A block the detector never finds where the label says it is, or finds
as another class, is reported: on 2026-09-25 blocks were left on the previous round's markers, or two
were swapped, and the labels went wrong without the colour check noticing (sticker faces up).

    python3 dev/vision/check_round.py data/datasets/blocks4/<ds> round_019 [--model data/models/blocks4/v3/best.pt]
"""
from __future__ import annotations

import argparse
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
CLASSES = ["recyclable", "kitchen", "hazardous", "other"]


def iou(a, b) -> float:
    x0, y0, x1, y1 = max(a[0], b[0]), max(a[1], b[1]), min(a[2], b[2]), min(a[3], b[3])
    inter = max(0, x1 - x0) * max(0, y1 - y0)
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union > 0 else 0.0


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("dataset", type=Path)
    parser.add_argument("round")
    parser.add_argument("--model", type=Path, default=ROOT / "data" / "models" / "blocks4" / "v3" / "best.pt")
    args = parser.parse_args()
    from ultralytics import YOLO
    model = YOLO(str(args.model))
    per_class: dict[int, dict] = {}
    for img in sorted((args.dataset / "images").glob(f"{args.round}_*.jpg")):
        lab = args.dataset / "labels" / f"{img.stem}.txt"
        boxes = []
        for line in lab.read_text().splitlines():
            c, x, y, w, h = line.split()
            x, y, w, h = (float(v) for v in (x, y, w, h))
            boxes.append((int(c), ((x - w / 2) * 640, (y - h / 2) * 480, (x + w / 2) * 640, (y + h / 2) * 480)))
        pred = model.predict(str(img), imgsz=640, conf=0.25, verbose=False)[0]
        dets = [(int(b.cls), b.xyxy[0].tolist(), float(b.conf)) for b in pred.boxes]
        for c, box in boxes:
            s = per_class.setdefault(c, dict(views=0, found=0, classes={}))
            s["views"] += 1
            best = max(dets, key=lambda d: iou(d[1], box), default=None)
            if best and iou(best[1], box) >= 0.4:
                s["found"] += 1
                s["classes"][CLASSES[best[0]]] = s["classes"].get(CLASSES[best[0]], 0) + 1
    if not per_class:
        print("CHECK: no labelled images for", args.round)
        raise SystemExit(1)
    bad = False
    for c, s in sorted(per_class.items()):
        agree = s["classes"].get(CLASSES[c], 0)
        flag = ""
        if s["views"] >= 2 and s["found"] == 0:
            flag, bad = "  <- nothing detected where the label is: placed elsewhere?", True
        elif s["found"] and agree < s["found"] / 2:
            flag, bad = f"  <- detected as {max(s['classes'], key=s['classes'].get)}: swapped?", True
        print(f"{CLASSES[c]:10s} labelled in {s['views']} views, detected in {s['found']}, classes {s['classes']}{flag}")
    print("CHECK:", "look at this round" if bad else "ok")


if __name__ == "__main__":
    main()
