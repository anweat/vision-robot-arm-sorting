"""Narrow follow-up: is the threaded ACL failure about *sharing* a session across threads?

The main concurrency test created three InferSessions in the main thread and then hammered them
from three worker threads -- every thread died with '[-1][ACL: general failure]'. ACL contexts are
normally thread-bound, so the question is whether a thread works if it creates its own session.

Variants, each for --seconds:
  A  sessions created in main thread, infer from main thread   (control)
  B  sessions created in main thread, infer from 3 threads     (reproduces the failure)
  C  each thread creates its own session, then infers          (the candidate fix)
"""
from __future__ import annotations

import argparse
import json
import statistics
import threading
import time
from pathlib import Path

import numpy as np

MODELS = {
    "asr": "wenet_asr/offline_encoder.om",
    "gesture": "gesture_hrnet/hrnet.om",
    "detector": "detector/blocks4_yolov8n.om",
}


def build_inputs(session) -> list[np.ndarray]:
    tensors = []
    for entry in session.get_inputs():
        shape, dtype = list(entry.shape), str(entry.datatype)
        if "int32" in dtype:
            tensors.append(np.ones(shape, dtype=np.int32))
        elif "int64" in dtype:
            tensors.append(np.ones(shape, dtype=np.int64))
        else:
            tensors.append(np.zeros(shape, dtype=np.float32))
    if len(tensors) == 2 and tensors[1].dtype == np.int32:
        tensors[1] = np.array([tensors[0].shape[1]], dtype=np.int32)
    return tensors


def hammer(session, tensors, seconds: float) -> dict:
    latencies, errors = [], []
    for _ in range(2):
        try:
            session.infer(tensors)
        except Exception:                                       # noqa: BLE001
            pass
    deadline = time.time() + seconds
    while time.time() < deadline:
        t = time.perf_counter()
        try:
            session.infer(tensors)
            latencies.append((time.perf_counter() - t) * 1000)
        except Exception as exc:                                # noqa: BLE001
            errors.append(f"{type(exc).__name__}: {exc}")
            break
    out = {"iterations": len(latencies), "errors": len(errors)}
    if latencies:
        out["median_ms"] = round(statistics.median(latencies), 3)
        out["per_second"] = round(len(latencies) / seconds, 2)
    if errors:
        out["first_error"] = errors[0]
    return out


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dir", default="/root/model-smoke")
    parser.add_argument("--seconds", type=float, default=5.0)
    args = parser.parse_args()

    from ais_bench.infer.interface import InferSession

    paths = {n: Path(args.dir) / p for n, p in MODELS.items()}
    report: dict = {}

    # ---- A: control, main thread only
    sessions = {n: InferSession(0, str(p)) for n, p in paths.items()}
    tensors = {n: build_inputs(s) for n, s in sessions.items()}
    report["A_main_thread_shares_sessions_created_in_main"] = {
        n: hammer(sessions[n], tensors[n], args.seconds) for n in sessions}

    # ---- B: same sessions, used from three threads
    rows: dict[str, dict] = {}

    def run_shared(name: str) -> None:
        rows[name] = hammer(sessions[name], tensors[name], args.seconds)

    threads = [threading.Thread(target=run_shared, args=(n,)) for n in sessions]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    report["B_shared_sessions_used_from_threads"] = rows

    # ---- C: each thread builds its own session
    rows_c: dict[str, dict] = {}

    def run_own(name: str) -> None:
        try:
            own = InferSession(0, str(paths[name]))
            rows_c[name] = hammer(own, build_inputs(own), args.seconds)
        except Exception as exc:                                # noqa: BLE001
            rows_c[name] = {"iterations": 0, "error_on_create": f"{type(exc).__name__}: {exc}"}

    threads = [threading.Thread(target=run_own, args=(n,)) for n in MODELS]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    report["C_thread_local_sessions"] = rows_c

    print("\n=== JSON ===")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
