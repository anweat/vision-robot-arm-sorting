"""The project's 4-class block detector (YOLOv8n trained by dev/vision/train_blocks.py), on the NPU or in ONNX.

Classes are garbage categories, one per block whatever face is up: recyclable, kitchen, hazardous, other.
Input `images` [1, 3, 640, 640] float32 RGB /255, letterboxed with grey 114 (as Ultralytics exports it);
output [1, 4 + 4, 8400]: cx, cy, w, h in letterbox pixels, then one score per class (no objectness).

Backends: `om` (ais_bench on the Atlas 200I DK A2, after sourcing set_env.sh) for
data/models/blocks4/<version>/blocks4_yolov8n.om, or `onnx` (onnxruntime, anywhere) for best.onnx -- the same
pre/post-processing, so the two can be compared on the same frames.

    python3 src/vision/blocks_detector.py image.jpg [...] [--onnx data/models/blocks4/best.onnx]
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
CLASSES = ("recyclable", "kitchen", "hazardous", "other")
MODEL_DIR = ROOT / "data" / "models" / "blocks4" / "v3"     # v3: 2026-09-26, 60 epochs on 136 real + 800 synthetic frames
SIZE = 640


def letterbox(image: np.ndarray, size: int = SIZE) -> tuple[np.ndarray, float, tuple[int, int]]:
    h, w = image.shape[:2]
    r = min(size / h, size / w)
    nw, nh = round(w * r), round(h * r)
    dw, dh = (size - nw) / 2, (size - nh) / 2
    out = cv2.resize(image, (nw, nh), interpolation=cv2.INTER_LINEAR)
    top, bottom, left, right = round(dh - 0.1), round(dh + 0.1), round(dw - 0.1), round(dw + 0.1)
    out = cv2.copyMakeBorder(out, top, bottom, left, right, cv2.BORDER_CONSTANT, value=(114, 114, 114))
    return out, r, (left, top)


def preprocess(image_bgr: np.ndarray) -> tuple[np.ndarray, float, tuple[int, int]]:
    img, ratio, pad = letterbox(image_bgr)
    tensor = np.ascontiguousarray(img[:, :, ::-1].transpose(2, 0, 1), dtype=np.float32)[None] / 255.0
    return tensor, ratio, pad


def decode(output: np.ndarray, ratio: float, pad: tuple[int, int], shape: tuple[int, int],
           conf: float = 0.25, iou: float = 0.5) -> list[dict]:
    """[1, 4 + nc, N] -> detections in image pixels, class-wise NMS."""
    pred = np.asarray(output, np.float32).reshape(4 + len(CLASSES), -1).T      # N x (4 + nc)
    scores = pred[:, 4:]
    cls = scores.argmax(1)
    best = scores[np.arange(len(cls)), cls]
    keep = best > conf
    if not keep.any():
        return []
    pred, cls, best = pred[keep], cls[keep], best[keep]
    xy = (pred[:, :2] - np.array(pad, np.float32)) / ratio
    wh = pred[:, 2:4] / ratio
    boxes = np.column_stack([xy - wh / 2, wh])
    out = []
    for c in np.unique(cls):
        idx = np.where(cls == c)[0]
        for k in np.array(cv2.dnn.NMSBoxes(boxes[idx].tolist(), best[idx].tolist(), conf, iou)).reshape(-1):
            x, y, w, h = boxes[idx[k]]
            x0, y0 = max(0.0, x), max(0.0, y)
            x1, y1 = min(shape[1] - 1.0, x + w), min(shape[0] - 1.0, y + h)
            out.append(dict(category=CLASSES[int(c)], conf=round(float(best[idx[k]]), 3),
                            bbox=[round(x0), round(y0), round(x1), round(y1)]))
    return sorted(out, key=lambda d: -d["conf"])


def _ais_bench_available() -> bool:
    import importlib.util
    return importlib.util.find_spec("ais_bench") is not None


def choose_model(model: Path | None = None, backend: str | None = None) -> tuple[Path, str]:
    """The model file and backend to use. The .om needs ais_bench, which only the board has, so off the
    board an .om sitting in MODEL_DIR must not shadow the ONNX: that would silently leave the caller with
    no detector at all. An explicit backend is obeyed as given."""
    if model is None:
        om = MODEL_DIR / "blocks4_yolov8n.om"
        use_om = om.exists() and backend != "onnx" and (backend == "om" or _ais_bench_available())
        model = om if use_om else MODEL_DIR / "best.onnx"
    model = Path(model)
    return model, backend or ("om" if model.suffix == ".om" else "onnx")


class BlocksDetector:
    def __init__(self, model: Path | None = None, backend: str | None = None, device: int = 0):
        self.model, self.backend = choose_model(model, backend)
        if self.backend == "om":
            from ais_bench.infer.interface import InferSession
            self.session = InferSession(device, str(self.model))
        else:
            import onnxruntime as ort
            self.session = ort.InferenceSession(str(self.model), providers=["CPUExecutionProvider"])
            self.input = self.session.get_inputs()[0].name

    def raw(self, tensor: np.ndarray) -> np.ndarray:
        if self.backend == "om":
            return np.asarray(self.session.infer([tensor])[0])
        return self.session.run(None, {self.input: tensor})[0]

    def detect(self, image_bgr: np.ndarray, conf: float = 0.25, iou: float = 0.5) -> list[dict]:
        tensor, ratio, pad = preprocess(image_bgr)
        return decode(self.raw(tensor), ratio, pad, image_bgr.shape[:2], conf, iou)


def draw(image: np.ndarray, dets: list[dict]) -> np.ndarray:
    out = image.copy()
    colours = {"recyclable": (255, 128, 0), "kitchen": (0, 180, 0), "hazardous": (0, 0, 255), "other": (0, 200, 255)}
    for d in dets:
        x0, y0, x1, y1 = d["bbox"]
        cv2.rectangle(out, (x0, y0), (x1, y1), colours[d["category"]], 2)
        cv2.putText(out, f"{d['category']} {d['conf']:.2f}", (x0, max(12, y0 - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    colours[d["category"]], 1)
    return out


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("images", nargs="+", type=Path)
    parser.add_argument("--model", type=Path)
    parser.add_argument("--backend", choices=("om", "onnx"))
    parser.add_argument("--bench", type=int, default=0, help="time this many inferences on the first image")
    parser.add_argument("--dump", type=Path, help="save the raw output of the first image (.npy) for comparison")
    args = parser.parse_args()
    det = BlocksDetector(args.model, args.backend)
    for i, path in enumerate(args.images):
        img = cv2.imread(str(path))
        dets = det.detect(img)
        print(path.name, json.dumps(dets))
        cv2.imwrite(str(path.with_name(path.stem + "_blocks.jpg")), draw(img, dets))
        if i == 0 and args.dump:
            np.save(args.dump, det.raw(preprocess(img)[0]))
    if args.bench:
        import time
        tensor = preprocess(cv2.imread(str(args.images[0])))[0]
        det.raw(tensor)
        t = time.perf_counter()
        for _ in range(args.bench):
            det.raw(tensor)
        print(f"{det.backend}: {(time.perf_counter() - t) / args.bench * 1000:.1f} ms per inference")


if __name__ == "__main__":
    main()
