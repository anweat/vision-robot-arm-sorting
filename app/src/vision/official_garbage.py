"""The official Atlas 200I DK A2 E2E robot-arm garbage detector (YOLOv5s OM), without ROS or torch.

Model: data/models/official_arm/yolov5s_bs1.om (see SOURCE.md there). Read on the board 2026-09-26:
input `images` [1, 3, 640, 640] float16, output [1, 25200, 8] float16, i.e. 3 classes, matching the
upstream coco_names.txt: Fish_bone, Newspaper, Syringe. Of the 16 sticker kinds on the project's blocks
(docs/atlas-200i-dk-a2/07) it can therefore name at most these three; "other" has no class at all.

Pre/post-processing mirror upstream npu_utils.py / det_utils.py (letterbox 640 with grey 114 padding,
BGR->RGB, CHW, /255, float16; obj x class confidence, NMS), with numpy + OpenCV instead of torch.
Runs where ais_bench and the Ascend runtime are available (the board, after sourcing set_env.sh).

    python3 src/vision/official_garbage.py image.jpg [more.jpg ...]   # print detections, write *_official.jpg
"""
from __future__ import annotations

import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
MODEL = ROOT / "models" / "official_arm" / "yolov5s_bs1.om"          # board layout (/root/arm-app)
if not MODEL.exists():
    MODEL = ROOT / "data" / "models" / "official_arm" / "yolov5s_bs1.om"
LABELS = ("Fish_bone", "Newspaper", "Syringe")
CATEGORY = {"Fish_bone": "kitchen", "Newspaper": "recyclable", "Syringe": "hazardous"}
SIZE = 640


def letterbox(image: np.ndarray, size: int = SIZE) -> tuple[np.ndarray, float, tuple[float, float]]:
    h, w = image.shape[:2]
    r = min(size / h, size / w)
    nw, nh = round(w * r), round(h * r)
    dw, dh = (size - nw) / 2, (size - nh) / 2
    out = cv2.resize(image, (nw, nh), interpolation=cv2.INTER_LINEAR)
    top, bottom, left, right = round(dh - 0.1), round(dh + 0.1), round(dw - 0.1), round(dw + 0.1)
    out = cv2.copyMakeBorder(out, top, bottom, left, right, cv2.BORDER_CONSTANT, value=(114, 114, 114))
    return out, r, (left, top)


def decode(output: np.ndarray, ratio: float, pad: tuple[float, float], shape: tuple[int, int],
           conf: float = 0.25, iou: float = 0.45) -> list[dict]:
    """[25200, 5 + classes] (cx, cy, w, h, obj, cls...) in letterbox pixels -> detections in the image."""
    pred = output.reshape(-1, output.shape[-1]).astype(np.float32)
    scores = pred[:, 4:5] * pred[:, 5:]
    cls = scores.argmax(1)
    best = scores[np.arange(len(cls)), cls]
    keep = best > conf
    if not keep.any():
        return []
    pred, cls, best = pred[keep], cls[keep], best[keep]
    xy = (pred[:, :2] - np.array(pad)) / ratio
    wh = pred[:, 2:4] / ratio
    boxes = np.column_stack([xy - wh / 2, wh])                       # x, y, w, h for NMSBoxes
    out = []
    for c in np.unique(cls):
        idx = np.where(cls == c)[0]
        kept = cv2.dnn.NMSBoxes(boxes[idx].tolist(), best[idx].tolist(), conf, iou)
        for k in np.array(kept).reshape(-1):
            x, y, w, h = boxes[idx[k]]
            x0, y0 = max(0.0, x), max(0.0, y)
            x1, y1 = min(shape[1] - 1.0, x + w), min(shape[0] - 1.0, y + h)
            label = LABELS[int(c)]
            out.append(dict(label=label, category=CATEGORY[label], conf=round(float(best[idx[k]]), 3),
                            bbox=[round(x0), round(y0), round(x1), round(y1)]))
    return sorted(out, key=lambda d: -d["conf"])


class OfficialGarbage:
    def __init__(self, model: Path = MODEL, device: int = 0):
        from ais_bench.infer.interface import InferSession
        self.session = InferSession(device, str(model))

    def detect(self, image_bgr: np.ndarray, conf: float = 0.25, iou: float = 0.45) -> list[dict]:
        img, ratio, pad = letterbox(image_bgr)
        tensor = np.ascontiguousarray(img[:, :, ::-1].transpose(2, 0, 1), dtype=np.float16)[None] / np.float16(255)
        output = self.session.infer([tensor])[0]
        return decode(np.asarray(output), ratio, pad, image_bgr.shape[:2], conf, iou)


def draw(image: np.ndarray, dets: list[dict]) -> np.ndarray:
    out = image.copy()
    for d in dets:
        x0, y0, x1, y1 = d["bbox"]
        cv2.rectangle(out, (x0, y0), (x1, y1), (0, 255, 0), 2)
        cv2.putText(out, f"{d['label']} {d['conf']:.2f}", (x0, max(14, y0 - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    return out


if __name__ == "__main__":
    model = OfficialGarbage()
    for name in sys.argv[1:]:
        image = cv2.imread(name)
        dets = model.detect(image, conf=0.1)
        print(name, image.shape[1], "x", image.shape[0], dets)
        cv2.imwrite(str(Path(name).with_name(Path(name).stem + "_official.jpg")), draw(image, dets))
