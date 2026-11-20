"""Board-side smoke test for the speech and gesture OM models from Release assets-v4.

One-off tool: it only answers "does this .om load and run on the NPU, what does it output, and how
long does it take". It imports **no** Arm_Lib and never touches a servo.

Runs where ais_bench and the Ascend runtime are available -- the board, after
`source /usr/local/Ascend/ascend-toolkit/set_env.sh`.

    # speech: transcribe the bundled sample and time each stage
    python3 dev/scripts/model_smoke_test.py asr --dir models/wenet_asr --runs 3

    # gesture: synthetic input (seeded), then optionally a real image, plus an ONNX cross-check
    python3 dev/scripts/model_smoke_test.py gesture --dir models/gesture_hrnet --runs 20
    python3 dev/scripts/model_smoke_test.py gesture --dir models/gesture_hrnet --image hand.jpg

Prints a JSON block at the end so the numbers can be pasted into the hardware log.
Nothing is written except the optional temp file for the ONNX comparison.
"""
from __future__ import annotations

import argparse
import json
import statistics
import sys
import time
from pathlib import Path

import numpy as np


def _load_session(model: Path):
    from ais_bench.infer.interface import InferSession

    return InferSession(0, str(model))


def _describe(session) -> dict:
    def shape_of(entry):
        shape = getattr(entry, "shape", None)
        return list(shape) if shape is not None else str(entry)

    return {
        "inputs": [{"name": getattr(i, "name", "?"), "shape": shape_of(i), "dtype": str(getattr(i, "datatype", "?"))}
                   for i in session.get_inputs()],
        "outputs": [{"name": getattr(o, "name", "?"), "shape": shape_of(o), "dtype": str(getattr(o, "datatype", "?"))}
                    for o in session.get_outputs()],
    }


def _stats(samples: list[float]) -> dict:
    if not samples:
        return {}
    return {
        "runs": len(samples),
        "min_ms": round(min(samples), 3),
        "median_ms": round(statistics.median(samples), 3),
        "max_ms": round(max(samples), 3),
    }


# ------------------------------------------------------------------------------------- speech


def run_asr(args) -> dict:
    sys.path.insert(0, str(Path(args.dir).resolve()))
    from model import WeNetASR          # official pre/post-processing, shipped next to the .om

    model_path = Path(args.dir) / "offline_encoder.om"
    vocab_path = Path(args.dir) / "vocab.txt"
    wav = Path(args.wav) if args.wav else Path(args.dir) / "sample.wav"
    for path in (model_path, vocab_path, wav):
        if not path.exists():
            raise SystemExit(f"missing {path}")

    t0 = time.perf_counter()
    asr = WeNetASR(str(model_path), str(vocab_path))
    load_ms = (time.perf_counter() - t0) * 1000

    result: dict = {
        "model": str(model_path),
        "model_bytes": model_path.stat().st_size,
        "max_frames": int(asr.max_len),
        "max_audio_seconds": round(asr.max_len * 0.01, 2),
        "load_ms": round(load_ms, 3),
        "descriptors": _describe(asr.model),
        "wav": str(wav),
    }

    transcripts, pre_ms, infer_ms, post_ms = [], [], [], []
    feats_pad = feats_lengths = None
    for run in range(args.runs):
        t = time.perf_counter()
        feats_pad, feats_lengths = asr.preprocess(str(wav))
        pre_ms.append((time.perf_counter() - t) * 1000)

        t = time.perf_counter()
        output = asr.model.infer([feats_pad, feats_lengths])
        infer_ms.append((time.perf_counter() - t) * 1000)

        t = time.perf_counter()
        transcripts.append(asr.post_process(output))
        post_ms.append((time.perf_counter() - t) * 1000)

    result["tensors"] = {
        "speech": list(np.asarray(feats_pad).shape),
        "speech_dtype": str(np.asarray(feats_pad).dtype),
        "speech_lengths": np.asarray(feats_lengths).tolist(),
        "output_shapes": [list(np.asarray(o).shape) for o in output],
        "output_dtypes": [str(np.asarray(o).dtype) for o in output],
    }
    result["transcripts"] = transcripts
    result["stable"] = len(set(transcripts)) == 1
    result["preprocess"] = _stats(pre_ms)
    result["infer"] = _stats(infer_ms)
    result["postprocess"] = _stats(post_ms)
    return result


# ------------------------------------------------------------------------------------ gesture

LABELS = ("fist", "like", "ok", "palm")


def _tensor_from_image(path: Path) -> np.ndarray:
    import cv2

    image = cv2.imread(str(path))
    if image is None:
        raise SystemExit(f"cannot read image {path}")
    image = cv2.resize(image, (224, 224), interpolation=cv2.INTER_LINEAR)
    rgb = image[..., ::-1].astype(np.float32) / 255.0
    return np.ascontiguousarray(rgb.transpose(2, 0, 1)[None])


def _tensor_synthetic(seed: int) -> np.ndarray:
    rng = np.random.default_rng(seed)
    return rng.random((1, 3, 224, 224), dtype=np.float32)


def run_gesture(args) -> dict:
    import cv2  # noqa: F401  (imported here so the failure mode is obvious)

    model = Path(args.dir) / "hrnet.om"
    labels_file = Path(args.dir) / "labels.txt"
    if not model.exists():
        raise SystemExit(f"missing {model}")

    session = _load_session(model)
    result: dict = {
        "model": str(model),
        "model_bytes": model.stat().st_size,
        "descriptors": _describe(session),
        "labels": labels_file.read_text().strip().splitlines() if labels_file.exists() else list(LABELS),
    }

    def infer(tensor: np.ndarray, runs: int) -> tuple[np.ndarray, list[float]]:
        logits, timings = None, []
        for _ in range(runs):
            t = time.perf_counter()
            logits = session.infer([tensor])
            timings.append((time.perf_counter() - t) * 1000)
        first = logits[0] if isinstance(logits, (list, tuple)) else logits
        return np.asarray(first).reshape(-1), timings

    # The .onnx is loaded once, and every case is cross-checked against *its own* input.
    onnx_session, onnx_version, onnx_error = None, None, None
    onnx_path = Path(args.dir) / "hrnet.onnx"
    if onnx_path.exists():
        try:
            import onnxruntime as ort

            onnx_session = ort.InferenceSession(str(onnx_path), providers=["CPUExecutionProvider"])
            onnx_version = ort.__version__
        except ImportError:
            onnx_error = "onnxruntime not installed on this device"
        except Exception as exc:                       # noqa: BLE001 - report, do not mask
            onnx_error = f"{type(exc).__name__}: {exc}"
    else:
        onnx_error = f"no {onnx_path}"

    def add_case(name: str, tensor: np.ndarray, extra: dict) -> np.ndarray:
        out, timings = infer(tensor, args.runs)
        entry = {
            "input_shape": list(tensor.shape),
            "output_shape": list(out.shape),
            "output_dtype": str(out.dtype),
            "logits": [round(float(v), 6) for v in out],
            "argmax": int(out.argmax()),
            "latency": _stats(timings),
        }
        if onnx_session is not None:
            ref = np.asarray(
                onnx_session.run(None, {onnx_session.get_inputs()[0].name: tensor})[0]
            ).reshape(-1)
            entry["onnx_cross_check"] = {
                "onnxruntime": onnx_version,
                "max_abs_diff": round(float(np.abs(ref - out).max()), 8),
                "argmax_match": bool(ref.argmax() == out.argmax()),
                "onnx_logits": [round(float(v), 6) for v in ref],
            }
        elif onnx_error:
            entry["onnx_cross_check"] = {"skipped": onnx_error}
        entry.update(extra)
        cases[name] = entry
        return out

    cases: dict = {}
    tensor = _tensor_synthetic(args.seed)
    add_case("synthetic", tensor, {})
    np.save("/tmp/gesture_smoke_input.npy", tensor)

    if args.image:
        add_case("image", _tensor_from_image(Path(args.image)), {"path": str(args.image)})

    result["cases"] = cases
    return result


# ----------------------------------------------------------------------------------------- main


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="mode", required=True)

    asr = sub.add_parser("asr", help="WeNet offline encoder")
    asr.add_argument("--dir", required=True, help="directory holding offline_encoder.om, vocab.txt, model.py")
    asr.add_argument("--wav", default=None, help="default: <dir>/sample.wav")
    asr.add_argument("--runs", type=int, default=3)
    asr.add_argument("--out", default=None, help="write the clean JSON here (stdout also gets ais_bench [INFO] noise)")
    asr.set_defaults(func=run_asr)

    ges = sub.add_parser("gesture", help="HRNet 4-class classifier")
    ges.add_argument("--dir", required=True, help="directory holding hrnet.om")
    ges.add_argument("--image", default=None, help="optional real image; synthetic input is always run")
    ges.add_argument("--runs", type=int, default=20)
    ges.add_argument("--seed", type=int, default=20261016)
    ges.add_argument("--out", default=None, help="write the clean JSON here (stdout also gets ais_bench [INFO] noise)")
    ges.set_defaults(func=run_gesture)

    args = parser.parse_args()
    payload = {"mode": args.mode, "argv": sys.argv[1:], "result": args.func(args)}
    text = json.dumps(payload, ensure_ascii=False, indent=2)
    # ais_bench writes [INFO] lines to stdout while the model loads/unloads, so a redirected stdout is
    # not valid JSON. --out writes the clean copy; the marker below keeps stdout readable for humans.
    if args.out:
        Path(args.out).write_text(text, encoding="utf-8")
        print(f"wrote {args.out}", file=sys.stderr)
    print("\n=== JSON ===")
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
