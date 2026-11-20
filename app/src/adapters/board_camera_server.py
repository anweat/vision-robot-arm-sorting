#!/usr/bin/env python3
"""Board USB camera as a small HTTP service: the single owner of /dev/video*.

Endpoints:
  /              browser page with the live stream
  /stream.mjpg   multipart MJPEG stream
  /snapshot.jpg  latest frame; headers X-Frame-Time (unix seconds) and X-Frame-Seq
  /health        JSON: frame count, measured fps, age of the latest frame

Run with the ros-arm-env interpreter, e.g.
  /root/miniconda3/envs/ros-arm-env/bin/python board_camera_server.py --port 8080
Other programs on the board should read /snapshot.jpg instead of opening the camera.
"""

from __future__ import annotations

import argparse
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2

PAGE = b"""<!doctype html><meta charset="utf-8"><title>Board camera</title>
<body style="margin:0;background:#111;color:#ddd;font:14px sans-serif">
<img src="/stream.mjpg" style="max-width:100%;display:block;margin:auto">
<p style="text-align:center"><a style="color:#8cf" href="/snapshot.jpg">snapshot</a> |
<a style="color:#8cf" href="/health">health</a></p></body>"""


class Camera(threading.Thread):
    def __init__(self, device: int, width: int, height: int, fps: int, quality: int):
        super().__init__(daemon=True)
        self.device, self.width, self.height, self.fps = device, width, height, fps
        self.encode_params = [cv2.IMWRITE_JPEG_QUALITY, quality]
        self.cond = threading.Condition()
        self.jpeg: bytes | None = None
        self.stamp = 0.0
        self.seq = 0
        self.opened_at = 0.0
        self.errors = 0

    def _open(self) -> cv2.VideoCapture:
        cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.opened_at = time.time()
        return cap

    def run(self) -> None:
        cap = self._open()
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                self.errors += 1
                cap.release()
                time.sleep(1.0)
                cap = self._open()
                continue
            stamp = time.time()
            ok, buf = cv2.imencode(".jpg", frame, self.encode_params)
            if not ok:
                continue
            with self.cond:
                self.jpeg, self.stamp, self.seq = buf.tobytes(), stamp, self.seq + 1
                self.cond.notify_all()

    def wait_newer(self, seq: int, timeout: float = 2.0) -> tuple[bytes | None, float, int]:
        with self.cond:
            self.cond.wait_for(lambda: self.seq > seq, timeout=timeout)
            return self.jpeg, self.stamp, self.seq

    def health(self) -> dict:
        elapsed = max(time.time() - self.opened_at, 1e-6)
        return {
            "device": f"/dev/video{self.device}",
            "frames": self.seq,
            "fps_since_open": round(self.seq / elapsed, 1),
            "latest_age_s": round(time.time() - self.stamp, 3) if self.seq else None,
            "reopen_count": self.errors,
            "size": [self.width, self.height],
        }


def make_handler(camera: Camera):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):  # keep stdout quiet
            pass

        def _send(self, body: bytes, ctype: str, extra: dict | None = None) -> None:
            self.send_response(200)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            for key, value in (extra or {}).items():
                self.send_header(key, value)
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = self.path.split("?", 1)[0]
            if path == "/":
                self._send(PAGE, "text/html; charset=utf-8")
            elif path == "/health":
                self._send(json.dumps(camera.health()).encode(), "application/json")
            elif path == "/snapshot.jpg":
                jpeg, stamp, seq = camera.wait_newer(0)
                if jpeg is None:
                    self.send_error(503, "no frame yet")
                    return
                self._send(jpeg, "image/jpeg", {"X-Frame-Time": f"{stamp:.3f}", "X-Frame-Seq": str(seq)})
            elif path == "/stream.mjpg":
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                seq = 0
                try:
                    while True:
                        jpeg, stamp, seq = camera.wait_newer(seq)
                        if jpeg is None:
                            continue
                        self.wfile.write(
                            b"--frame\r\nContent-Type: image/jpeg\r\n"
                            + f"Content-Length: {len(jpeg)}\r\nX-Frame-Time: {stamp:.3f}\r\n\r\n".encode()
                            + jpeg + b"\r\n"
                        )
                except (BrokenPipeError, ConnectionResetError):
                    pass
            else:
                self.send_error(404)

    return Handler


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--device", type=int, default=0)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30, help="camera supports 5/10/15/20/25/30 at 640x480 YUYV")
    parser.add_argument("--quality", type=int, default=80)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8080)
    args = parser.parse_args()

    camera = Camera(args.device, args.width, args.height, args.fps, args.quality)
    camera.start()
    server = ThreadingHTTPServer((args.host, args.port), make_handler(camera))
    server.daemon_threads = True
    print(f"serving /dev/video{args.device} on http://{args.host}:{args.port}/", flush=True)
    server.serve_forever()


if __name__ == "__main__":
    main()
