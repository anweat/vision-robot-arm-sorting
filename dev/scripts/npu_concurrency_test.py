"""Can the three OM models live on the NPU at once, and how much do they slow each other down?

Three models: WeNet ASR encoder, HRNet gesture classifier, and the project's YOLOv8n block detector.
Answers three questions in order:

  1. co-residency  -- do three InferSession instances load on device 0 at the same time?
  2. contention    -- with all three running concurrently (threads), what happens to each one's rate?
  3. separate procs -- same question when each model runs in its own process (the real service shape)

    python3 dev/scripts/npu_concurrency_test.py --seconds 6
    python3 dev/scripts/npu_concurrency_test.py --worker asr --seconds 6     # used internally

Run on the board. Imports no Arm_Lib and never touches a servo.
"""
from __future__ import annotations

import argparse
import json
import os
import statistics
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np

MODELS = {
    "asr": ("wenet_asr/offline_encoder.om", {}),
    "gesture": ("gesture_hrnet/hrnet.om", {}),
    "detector": ("detector/blocks4_yolov8n.om", {}),
}


def build_inputs(session) -> list[np.ndarray]:
    """Make one correctly-shaped, correctly-typed tensor per model input."""
    tensors = []
    for entry in session.get_inputs():
        shape = list(entry.shape)
        dtype = str(entry.datatype)
        if "int32" in dtype:
            tensors.append(np.ones(shape, dtype=np.int32))
        elif "int64" in dtype:
            tensors.append(np.ones(shape, dtype=np.int64))
        elif "float16" in dtype:
            tensors.append(np.zeros(shape, dtype=np.float16))
        else:
            tensors.append(np.zeros(shape, dtype=np.float32))
    if len(tensors) == 2 and tensors[1].dtype == np.int32:      # WeNet speech_lengths
        tensors[1] = np.array([tensors[0].shape[1]], dtype=np.int32)
    return tensors


def describe(session, name: str) -> dict:
    return {
        "name": name,
        "inputs": [{"shape": list(i.shape), "dtype": str(i.datatype)} for i in session.get_inputs()],
        "outputs": [{"shape": list(o.shape), "dtype": str(o.datatype)} for o in session.get_outputs()],
    }


def npu_memory() -> dict | None:
    """Parse the NPU memory cell out of `npu-smi info`.

    The table packs several 'used / total' pairs into single cells (hugepages '15 / 392' on the
    Chip row, memory '619 / 3513' on the Device row), so take the pair with the largest total --
    that is the HBM figure we want.
    """
    import re

    try:
        out = subprocess.run(["npu-smi", "info"], capture_output=True, text=True, timeout=20).stdout
    except Exception:                                           # noqa: BLE001
        return None
    pairs = [(int(a), int(b)) for a, b in re.findall(r"(\d+)\s*/\s*(\d+)", out)]
    if not pairs:
        return None
    used, total = max(pairs, key=lambda p: p[1])
    return {"used_mb": used, "total_mb": total}


# ------------------------------------------------------------------------------- worker mode


def worker(args) -> int:
    """Hammer one model for N seconds and report iterations + latencies. Used as a child process."""
    from ais_bench.infer.interface import InferSession

    rel, _ = MODELS[args.worker]
    session = InferSession(0, str(Path(args.dir) / rel))
    tensors = build_inputs(session)
    for _ in range(3):                                          # warmup
        session.infer(tensors)

    latencies, deadline = [], time.time() + args.seconds
    while time.time() < deadline:
        t = time.perf_counter()
        session.infer(tensors)
        latencies.append((time.perf_counter() - t) * 1000)

    print(json.dumps({
        "worker": args.worker,
        "iterations": len(latencies),
        "per_second": round(len(latencies) / args.seconds, 2),
        "median_ms": round(statistics.median(latencies), 3),
        "min_ms": round(min(latencies), 3),
        "max_ms": round(max(latencies), 3),
    }))
    return 0


# --------------------------------------------------------------------------------- main mode


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dir", default="/root/model-smoke")
    parser.add_argument("--seconds", type=float, default=6.0)
    parser.add_argument("--worker", choices=list(MODELS), default=None)
    args = parser.parse_args()
    if args.worker:
        return worker(args)

    from ais_bench.infer.interface import InferSession

    report: dict = {"dir": args.dir, "seconds": args.seconds, "npu_memory": {}}
    report["npu_memory"]["before_load"] = npu_memory()

    sessions, tensors, specs, failures = {}, {}, [], {}
    for name, (rel, _) in MODELS.items():
        path = Path(args.dir) / rel
        if not path.exists():
            failures[name] = f"missing {path}"
            continue
        try:
            session = InferSession(0, str(path))
            sessions[name] = session
            tensors[name] = build_inputs(session)
            specs.append(describe(session, name))
        except Exception as exc:                                # noqa: BLE001
            failures[name] = f"{type(exc).__name__}: {exc}"

    report["loaded"] = sorted(sessions)
    report["failed_to_load"] = failures
    report["model_specs"] = specs
    report["npu_memory"]["after_load"] = npu_memory()
    print(f"co-resident: {sorted(sessions)}  failed: {failures}", file=sys.stderr)

    # ---- baseline: each model alone, back to back
    baseline = {}
    for name, session in sessions.items():
        for _ in range(3):
            session.infer(tensors[name])
        lat = []
        deadline = time.time() + args.seconds
        while time.time() < deadline:
            t = time.perf_counter()
            session.infer(tensors[name])
            lat.append((time.perf_counter() - t) * 1000)
        baseline[name] = {
            "iterations": len(lat),
            "per_second": round(len(lat) / args.seconds, 2),
            "median_ms": round(statistics.median(lat), 3),
        }
    report["baseline_alone"] = baseline

    # ---- concurrent in one process (threads)
    results: dict[str, dict] = {}
    lock = threading.Lock()

    def hammer(name: str) -> None:
        session = sessions[name]
        lat, deadline = [], time.time() + args.seconds
        while time.time() < deadline:
            t = time.perf_counter()
            session.infer(tensors[name])
            lat.append((time.perf_counter() - t) * 1000)
        with lock:
            results[name] = {
                "iterations": len(lat),
                "per_second": round(len(lat) / args.seconds, 2),
                "median_ms": round(statistics.median(lat), 3),
            }

    threads = [threading.Thread(target=hammer, args=(n,)) for n in sessions]
    started = time.time()
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    wall = time.time() - started

    for name in results:
        base = baseline[name]
        results[name]["slowdown_vs_alone"] = round(
            results[name]["median_ms"] / base["median_ms"], 3) if base["median_ms"] else None
        results[name]["throughput_ratio"] = round(
            results[name]["per_second"] / base["per_second"], 3) if base["per_second"] else None
    report["threads_concurrent"] = results
    report["threads_wall_seconds"] = round(wall, 3)
    report["threads_total_iterations"] = sum(r["iterations"] for r in results.values())
    report["npu_memory"]["after_threads"] = npu_memory()

    # ---- one process per model
    procs = []
    for name in sessions:
        procs.append((name, subprocess.Popen(
            [sys.executable, __file__, "--worker", name, "--dir", args.dir,
             "--seconds", str(args.seconds)],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)))
    per_process = {}
    for name, proc in procs:
        out, _ = proc.communicate(timeout=args.seconds + 90)
        line = [ln for ln in out.splitlines() if ln.strip().startswith("{")]
        per_process[name] = json.loads(line[-1]) if line else {"error": "no output"}
    for name, row in per_process.items():
        base = baseline.get(name)
        if base and "median_ms" in row:
            row["slowdown_vs_alone"] = round(row["median_ms"] / base["median_ms"], 3)
            row["throughput_ratio"] = round(row["per_second"] / base["per_second"], 3)
    report["separate_processes"] = per_process
    report["npu_memory"]["after_processes"] = npu_memory()

    print("\n=== JSON ===")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
