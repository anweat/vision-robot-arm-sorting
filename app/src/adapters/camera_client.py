"""Frames from board_camera_server with their capture time.

Motion code must only use frames captured after the arm settled: snapshot_after(t)
waits until the server's X-Frame-Time is later than t.
"""

from __future__ import annotations

import time
import urllib.request

import cv2
import numpy as np

DEFAULT_URL = "http://127.0.0.1:8080/snapshot.jpg"


def snapshot(url: str = DEFAULT_URL, timeout: float = 3.0) -> tuple[np.ndarray, float]:
    with urllib.request.urlopen(url, timeout=timeout) as resp:
        stamp = float(resp.headers.get("X-Frame-Time", time.time()))
        data = resp.read()
    image = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError("camera returned an undecodable frame")
    return image, stamp


def snapshot_after(t_ready: float, url: str = DEFAULT_URL, timeout: float = 5.0) -> tuple[np.ndarray, float]:
    deadline = time.time() + timeout
    while time.time() < deadline:
        image, stamp = snapshot(url)
        if stamp > t_ready:
            return image, stamp
        time.sleep(0.05)
    raise RuntimeError(f"no camera frame newer than {t_ready:.3f} within {timeout} s")
