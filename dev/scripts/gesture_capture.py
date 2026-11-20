"""Capture hand-gesture frames from the board camera server and classify them with hrnet.om.

The camera server (app/src/adapters/board_camera_server.py) owns /dev/video0, so this polls its
`/snapshot.jpg` endpoint instead of opening the device -- two processes cannot hold a V4L2 device.

Runs on the board (needs ais_bench + numpy + cv2 + requests/urllib):

    python3 dev/scripts/gesture_capture.py --label fist --seconds 5

Writes frames to <out>/<label>/, prints per-frame predictions, and appends a summary line to
<out>/results.json. Serves only the model; it never imports Arm_Lib and never moves the arm.
"""
from __future__ import annotations

import argparse
import json
import time
import urllib.request
from pathlib import Path

import cv2
import numpy as np

SNAPSHOT = "http://127.0.0.1:8080/snapshot.jpg"
LABELS = ("fist", "like", "ok", "palm")
SIZE = 224


def grab(url: str, timeout: float = 5.0) -> np.ndarray | None:
    try:
        with urllib.request.urlopen(url, timeout=timeout) as response:
            buf = np.frombuffer(response.read(), dtype=np.uint8)
        return cv2.imdecode(buf, cv2.IMREAD_COLOR)
    except Exception:                                   # noqa: BLE001 - a dropped frame is not fatal
        return None


def to_tensor(image: np.ndarray) -> np.ndarray:
    """Match the course notebook: Resize(224) + ToTensor(), i.e. RGB / 255, no mean/std."""
    resized = cv2.resize(image, (SIZE, SIZE), interpolation=cv2.INTER_LINEAR)
    rgb = resized[..., ::-1].astype(np.float32) / 255.0
    return np.ascontiguousarray(rgb.transpose(2, 0, 1)[None])


def softmax(x: np.ndarray) -> np.ndarray:
    e = np.exp(x - x.max())
    return e / e.sum()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--label", required=True, choices=LABELS, help="gesture being held")
    parser.add_argument("--seconds", type=float, default=5.0)
    parser.add_argument("--dir", default="/root/model-smoke/gesture_hrnet")
    parser.add_argument("--out", default="/root/model-smoke/captures")
    parser.add_argument("--threshold", type=float, default=0.6, help="course notebook's accept threshold")
    args = parser.parse_args()

    from ais_bench.infer.interface import InferSession

    session = InferSession(0, str(Path(args.dir) / "hrnet.om"))
    out_dir = Path(args.out) / args.label
    out_dir.mkdir(parents=True, exist_ok=True)

    frames, rows = [], []
    deadline = time.time() + args.seconds
    while time.time() < deadline:
        image = grab(SNAPSHOT)
        if image is None:
            time.sleep(0.05)
            continue
        logits = np.asarray(session.infer([to_tensor(image)])[0]).reshape(-1)
        probs = softmax(logits)
        pred = int(probs.argmax())
        rows.append({
            "t": round(time.time(), 3),
            "pred": LABELS[pred],
            "confidence": round(float(probs[pred]), 4),
            "accepted": bool(probs[pred] >= args.threshold),
            "probs": [round(float(p), 4) for p in probs],
        })
        frames.append(image)
        time.sleep(0.03)

    for i, image in enumerate(frames):
        cv2.imwrite(str(out_dir / f"{args.label}_{i:03d}.jpg"), image)

    accepted = [r for r in rows if r["accepted"]]
    tally: dict[str, int] = {}
    for r in accepted:
        tally[r["pred"]] = tally.get(r["pred"], 0) + 1

    summary = {
        "label": args.label,
        "frames": len(rows),
        "seconds": args.seconds,
        "threshold": args.threshold,
        "accepted_frames": len(accepted),
        "accepted_rate": round(len(accepted) / len(rows), 4) if rows else 0.0,
        "tally_accepted": tally,
        "correct_accepted": tally.get(args.label, 0),
        "accuracy_over_accepted": (round(tally.get(args.label, 0) / len(accepted), 4)
                                   if accepted else None),
        "accuracy_over_all_frames": (round(tally.get(args.label, 0) / len(rows), 4) if rows else None),
        "mean_confidence": (round(float(np.mean([r["confidence"] for r in accepted])), 4)
                            if accepted else None),
        "frames_saved_to": str(out_dir),
    }
    print(json.dumps({"summary": summary, "per_frame": rows}, ensure_ascii=False, indent=2))

    results = Path(args.out) / "results.json"
    history = json.loads(results.read_text()) if results.exists() else []
    history.append(summary)
    results.write_text(json.dumps(history, ensure_ascii=False, indent=2), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
