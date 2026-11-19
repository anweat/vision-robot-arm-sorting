# /// script
# requires-python = ">=3.9"
# dependencies = ["opencv-python>=4.8,<5", "numpy"]
# ///
"""Watch the board camera and an optional phone RTSP/HTTP stream side by side on the Mac.

  uv run dev/tools/multi_view.py --phone http://PHONE_IP:8080/video
  uv run dev/tools/multi_view.py --phone rtsp://PHONE_IP:8080/h264_ulaw.sdp --record-every 2

The two phone URLs are the raw streams of the Android "IP Webcam" app, whose
root page is only an HTML wrapper; /shot.jpg gives a single frame.

Keys: s = save the current frames of every source, r = toggle periodic recording, q = quit.
Frames go to data/captures/<date>/ with a JSON sidecar. Timestamps are Mac receive
times, not camera exposure times, so treat cross-source alignment as approximate.
"""

from __future__ import annotations

import argparse
import json
import os
import threading
import time
from datetime import datetime
from pathlib import Path

os.environ.setdefault("OPENCV_FFMPEG_CAPTURE_OPTIONS", "rtsp_transport;tcp|fflags;nobuffer|flags;low_delay")

import cv2  # noqa: E402
import numpy as np  # noqa: E402

BOARD_STREAM = "http://192.168.137.100:8080/stream.mjpg"
ROOT = Path(__file__).resolve().parents[2]


class Source(threading.Thread):
    """Keeps only the newest frame so a slow display never builds latency."""

    def __init__(self, name: str, url: str):
        super().__init__(daemon=True)
        self.name, self.url = name, url
        self.lock = threading.Lock()
        self.frame: np.ndarray | None = None
        self.stamp = 0.0
        self.count = 0

    def run(self) -> None:
        while True:
            cap = cv2.VideoCapture(self.url, cv2.CAP_FFMPEG)
            while cap.isOpened():
                ok, frame = cap.read()
                if not ok:
                    break
                with self.lock:
                    self.frame, self.stamp, self.count = frame, time.time(), self.count + 1
            cap.release()
            time.sleep(1.0)

    def latest(self) -> tuple[np.ndarray | None, float, int]:
        with self.lock:
            return self.frame, self.stamp, self.count


def tile(sources: list[Source], height: int) -> np.ndarray:
    panels = []
    now = time.time()
    for src in sources:
        frame, stamp, _ = src.latest()
        if frame is None:
            panel = np.zeros((height, height * 4 // 3, 3), np.uint8)
            label = f"{src.name}: waiting for {src.url}"
        else:
            panel = cv2.resize(frame, (round(frame.shape[1] * height / frame.shape[0]), height))
            label = f"{src.name}  age {now - stamp:.2f}s"
        cv2.putText(panel, label, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        panels.append(panel)
    return np.hstack(panels)


def save(sources: list[Source], out_dir: Path, note: str) -> Path | None:
    out_dir.mkdir(parents=True, exist_ok=True)
    tag = datetime.now().strftime("%H%M%S_%f")[:-3]
    meta = {"saved_at": time.time(), "note": note, "frames": {}}
    for src in sources:
        frame, stamp, count = src.latest()
        if frame is None:
            continue
        name = f"{tag}_{src.name}.jpg"
        cv2.imwrite(str(out_dir / name), frame)
        meta["frames"][src.name] = {"file": name, "received_at": stamp, "seq": count, "url": src.url,
                                    "size": [frame.shape[1], frame.shape[0]]}
    if not meta["frames"]:
        return None
    path = out_dir / f"{tag}.json"
    path.write_text(json.dumps(meta, ensure_ascii=False, indent=2))
    return path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--board", default=BOARD_STREAM, help="board MJPEG URL; empty string disables it")
    parser.add_argument("--phone", default="", help="phone RTSP or HTTP MJPEG URL")
    parser.add_argument("--height", type=int, default=480, help="display height per panel")
    parser.add_argument("--out", type=Path, default=ROOT / "data" / "captures" / datetime.now().strftime("%Y-%m-%d"))
    parser.add_argument("--record-every", type=float, default=0, help="seconds between automatic saves while recording")
    parser.add_argument("--note", default="", help="free text stored with every saved frame")
    args = parser.parse_args()

    sources = [Source(n, u) for n, u in (("board", args.board), ("phone", args.phone)) if u]
    if not sources:
        parser.error("give at least one of --board / --phone")
    for src in sources:
        src.start()

    recording = args.record_every > 0
    last_save = 0.0
    while True:
        view = tile(sources, args.height)
        if recording:
            cv2.circle(view, (view.shape[1] - 20, 20), 8, (0, 0, 255), -1)
            if time.time() - last_save >= max(args.record_every, 0.2):
                save(sources, args.out, args.note)
                last_save = time.time()
        cv2.imshow("multi view (s save, r record, q quit)", view)
        key = cv2.waitKey(30) & 0xFF
        if key == ord("q"):
            break
        if key == ord("s"):
            print("saved", save(sources, args.out, args.note))
        if key == ord("r"):
            recording = not recording
            args.record_every = args.record_every or 1.0
            print("recording", "on" if recording else "off")
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
