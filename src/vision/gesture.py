"""手势识别（课程任务 3 的推理侧）：hrnet.om 四分类，`fist / like / ok / palm`。

与 `src/audio/asr.py` 一样，只做“加载 + 预处理 + 推理 + 取标签”，不碰机械臂：
识别结果要变成指令时走控制台的 `POST /api/interact/gesture`（`src/interact/intents.py`
负责连续帧、阈值、冷却），这里不重复那套判定。

    python3 src/vision/gesture.py --image hand.jpg        # 单张图
    python3 src/vision/gesture.py --camera 5              # 从板载相机抓 5 秒，逐帧打印
    python3 src/vision/gesture.py --camera 3 --json       # 机器可读输出

预处理与课程 notebook 一致：Resize(224) + ToTensor，即 RGB/255，无 mean/std；
输入 `1x3x224x224` float32，输出 `1x4` logits（README 里与 ONNX 的 argmax 逐张一致）。
"""

from __future__ import annotations

import argparse
import json
import sys
import time
import urllib.request
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_DIR = ROOT / "data" / "models" / "gesture_hrnet"
SIZE = 224
SNAPSHOT = "http://127.0.0.1:8080/snapshot.jpg"
DEFAULT_LABELS = ("fist", "like", "ok", "palm")


class GestureUnavailable(RuntimeError):
    """模型目录不完整，或这台机器上没有 ais_bench。"""


def labels_for(model_dir: Path) -> tuple[str, ...]:
    path = model_dir / "labels.txt"
    try:
        names = [line.split()[-1] for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
    except OSError:
        return DEFAULT_LABELS
    return tuple(names) if len(names) == 4 else DEFAULT_LABELS


def to_tensor(image: np.ndarray) -> np.ndarray:
    resized = cv2.resize(image, (SIZE, SIZE), interpolation=cv2.INTER_LINEAR)
    rgb = resized[..., ::-1].astype(np.float32) / 255.0
    return np.ascontiguousarray(rgb.transpose(2, 0, 1)[None])


def softmax(x: np.ndarray) -> np.ndarray:
    e = np.exp(x - x.max())
    return e / e.sum()


class Gesture:
    def __init__(self, path: str | Path | None = None):
        d = Path(path) if path else DEFAULT_DIR
        om = d / "hrnet.om"
        if not om.is_file():
            raise GestureUnavailable(f"找不到 {om}（Release assets-v4 的 gesture_hrnet_4cls.zip）")
        try:
            from ais_bench.infer.interface import InferSession
        except Exception as exc:                               # noqa: BLE001
            raise GestureUnavailable(f"没有 ais_bench：{type(exc).__name__}: {exc}") from exc
        self.dir = d
        self.labels = labels_for(d)
        self.session = InferSession(0, str(om))

    def classify(self, image: np.ndarray) -> dict:
        logits = np.asarray(self.session.infer([to_tensor(image)])[0]).reshape(-1)
        probs = softmax(logits)
        i = int(probs.argmax())
        return dict(label=self.labels[i], confidence=round(float(probs[i]), 4),
                    probs={n: round(float(p), 4) for n, p in zip(self.labels, probs)})


def grab(url: str = SNAPSHOT, timeout: float = 5.0) -> np.ndarray | None:
    try:
        with urllib.request.urlopen(url, timeout=timeout) as r:
            buf = np.frombuffer(r.read(), dtype=np.uint8)
        return cv2.imdecode(buf, cv2.IMREAD_COLOR)
    except Exception:                                          # noqa: BLE001
        return None


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--dir", help="模型目录（默认 data/models/gesture_hrnet）")
    src = p.add_mutually_exclusive_group(required=True)
    src.add_argument("--image", help="单张图片")
    src.add_argument("--camera", type=float, metavar="SECONDS", help="从板载相机抓取若干秒")
    p.add_argument("--url", default=SNAPSHOT, help="相机快照地址")
    p.add_argument("--threshold", type=float, default=0.6, help="课程 notebook 的接受阈值")
    p.add_argument("--json", action="store_true", help="只输出 JSON")
    args = p.parse_args(argv)

    try:
        model = Gesture(args.dir)
    except GestureUnavailable as exc:
        print(f"手势识别不可用：{exc}", file=sys.stderr)
        return 2

    if args.image:
        image = cv2.imread(args.image, cv2.IMREAD_COLOR)
        if image is None:
            print(f"读不到图片：{args.image}", file=sys.stderr)
            return 2
        result = model.classify(image)
        result["accepted"] = result["confidence"] >= args.threshold
        print(json.dumps(result, ensure_ascii=False, indent=1))
        return 0

    deadline = time.time() + args.camera
    rows = []
    while time.time() < deadline:
        image = grab(args.url)
        if image is None:
            time.sleep(0.05)
            continue
        r = model.classify(image)
        r["accepted"] = r["confidence"] >= args.threshold
        r["t"] = round(time.time(), 3)
        rows.append(r)
        if not args.json:
            print(f"{r['t']}  {r['label']:5} {r['confidence']:.3f}  {'接受' if r['accepted'] else '拒绝'}")
        time.sleep(0.03)
    accepted = [r for r in rows if r["accepted"]]
    summary = dict(frames=len(rows), accepted=len(accepted),
                   accepted_rate=round(len(accepted) / len(rows), 4) if rows else 0.0,
                   labels={n: sum(1 for r in accepted if r["label"] == n) for n in model.labels})
    if args.json:
        print(json.dumps(dict(summary=summary, frames=rows), ensure_ascii=False, indent=1))
    else:
        print(json.dumps(summary, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
