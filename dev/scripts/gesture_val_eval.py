"""Confusion matrix for hrnet.om over the course's own validation images.

Decides whether the model's index->name mapping matches the course notebook
(0=fist, 1=like, 2=ok, 3=palm) and how well it does inside its own training domain.

    python3 dev/scripts/gesture_val_eval.py --images val/gesture_val --dir gesture_hrnet

Run on the board (ais_bench). Does not import Arm_Lib.
"""
from __future__ import annotations

import argparse
import glob
import os

import cv2
import numpy as np

CLASSES = ("fist", "like", "ok", "palm")


def to_tensor(path: str) -> np.ndarray:
    image = cv2.resize(cv2.imread(path), (224, 224), interpolation=cv2.INTER_LINEAR)
    rgb = image[..., ::-1].astype(np.float32) / 255.0
    return np.ascontiguousarray(rgb.transpose(2, 0, 1)[None])


def softmax(x: np.ndarray) -> np.ndarray:
    e = np.exp(x - x.max())
    return e / e.sum()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--images", default="val/gesture_val")
    parser.add_argument("--dir", default="gesture_hrnet")
    parser.add_argument("--write", default=None, help="also write the matrices here as JSON")
    args = parser.parse_args()

    from ais_bench.infer.interface import InferSession

    session = InferSession(0, os.path.join(args.dir, "hrnet.om"))

    counts = {}
    cm = np.zeros((4, 4), dtype=int)
    mean_p = np.zeros((4, 4))
    for ai, cls in enumerate(CLASSES):
        files = sorted(glob.glob(os.path.join(args.images, cls, "*.jpg")))
        counts[cls] = len(files)
        probs = np.array([softmax(np.asarray(session.infer([to_tensor(f)])[0]).reshape(-1))
                          for f in files])
        for pi in range(4):
            cm[ai, pi] = int((probs.argmax(1) == pi).sum())
            mean_p[ai, pi] = probs[:, pi].mean()

    print("样本数/类:", counts)
    print()
    print("混淆矩阵（行=真实类别，列=模型 idx）")
    print("        idx0  idx1  idx2  idx3   行正确率")
    for ai, cls in enumerate(CLASSES):
        total = cm[ai].sum()
        print(f"  {cls:<5} {cm[ai,0]:>5} {cm[ai,1]:>5} {cm[ai,2]:>5} {cm[ai,3]:>5}"
              f"   {cm[ai, ai] / total * 100:>6.1f}%")
    print()
    print("平均概率（行=真实类别，列=idx）")
    for ai, cls in enumerate(CLASSES):
        print(f"  {cls:<5} " + " ".join(f"{mean_p[ai, pi]:.3f}" for pi in range(4)))
    accuracy = float(sum(cm[i, i] for i in range(4)) / cm.sum())
    print()
    print("总体准确率: %.1f%%" % (accuracy * 100))

    if args.write:
        import json
        with open(args.write, "w", encoding="utf-8") as fh:
            json.dump({"classes": CLASSES, "counts": counts,
                       "confusion": cm.tolist(), "mean_prob": mean_p.round(4).tolist(),
                       "accuracy": round(accuracy, 4)}, fh, ensure_ascii=False, indent=2)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
