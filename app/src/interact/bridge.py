"""Recognisers -> the gateway (E1 gesture, E7 voice). Runs on the board, next to the models.

    wrist camera frame -> src/vision/gesture.py (hrnet.om) -> (label, confidence) -> POST <gateway>/api/interact/gesture
    microphone (arecord) -> src/audio/asr.py (WeNet .om) -> text           -> POST <gateway>/api/interact/voice

This file only feeds recognitions. Whether one is a command -- armed switch, keyword match, threshold,
N frames in a row, cooldown -- is decided in one place, src/interact/intents.py, held by the gateway
(src/web/serve.py), which also runs the command (start / stop / gripper / run_preset) and writes the
`interaction` event. A flow script's `wait_gesture` reads the same recognitions from the gateway.
(PR #7 ran the rules here and posted /api/cmd, because the real console had no /api/interact; the gateway
now holds them for the real arm and the simulator alike.)

    python3 src/interact/bridge.py gesture --seconds 30               # wrist camera, 30 s
    python3 src/interact/bridge.py gesture --label ok --times 3       # no model: three frames of "ok"
    python3 src/interact/bridge.py voice --text 开始分拣               # no model: one sentence
    python3 src/interact/bridge.py voice --wav data/models/wenet_asr/sample.wav
    python3 src/interact/bridge.py listen                             # gesture until stopped (the gateway's 监听)
    python3 src/interact/bridge.py listen --voice --mic 3             # and record 3 s at a time

Models: data/models/gesture_hrnet/hrnet.om, data/models/wenet_asr/ (Release assets-v4); both need ais_bench,
so `gesture`/`listen` only work on the board. The board has no confirmed audio input yet (2026-11-19), so
`--voice` is off by default.
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
for sub in ("vision", "audio"):
    sys.path.insert(0, str(ROOT / "src" / sub))

SNAPSHOT = os.environ.get("ARM_CAMERA", "http://127.0.0.1:8080/snapshot.jpg")
GESTURE_HZ = 6.0           # frames per second offered to the rules (hrnet ~9 ms on the NPU; the camera ~7.6 fps)


class Gateway:
    def __init__(self, base: str, token: str | None = None):
        self.base = base.rstrip("/")
        self.token = token or ""

    def post(self, path: str, body: dict) -> dict:
        req = urllib.request.Request(self.base + path, data=json.dumps(body).encode(),
                                     method="POST", headers={"Content-Type": "application/json", "X-Token": self.token})
        try:
            with urllib.request.urlopen(req, timeout=30) as r:
                return json.loads(r.read())
        except urllib.error.HTTPError as exc:
            try:
                return json.loads(exc.read())
            except ValueError:
                return dict(error=f"HTTP {exc.code}")

    def gesture(self, label: str, confidence: float) -> dict:
        return self.post("/api/interact/gesture", dict(label=label, confidence=round(float(confidence), 4)))

    def voice(self, text: str) -> dict:
        return self.post("/api/interact/voice", dict(text=text))


def say(result: dict, what: str) -> None:
    status = result.get("status") or ("error" if result.get("error") else "?")
    if status != "ignored" or result.get("command"):
        print(f"[{time.strftime('%H:%M:%S')}] {what} → {status}"
              f"{(' / ' + result['command']) if result.get('command') else ''}"
              f"{('（' + str(result.get('reason') or result.get('error')) + '）') if result.get('reason') or result.get('error') else ''}",
              flush=True)


def run_gesture(args, gw: Gateway) -> int:
    if args.label:                        # no model: the rules and the command path only
        for _ in range(max(1, args.times)):
            say(gw.gesture(args.label, args.confidence), f"手势 {args.label}")
            time.sleep(0.05)
        return 0
    import gesture as G
    model = G.Gesture(args.gesture_dir)
    deadline = time.time() + args.seconds if args.seconds else float("inf")
    while time.time() < deadline:
        began = time.time()
        frame = G.grab(args.url)
        if frame is not None:
            c = model.classify(frame)
            say(gw.gesture(c["label"], c["confidence"]), f"手势 {c['label']} {c['confidence']:.2f}")
        time.sleep(max(0.0, 1.0 / GESTURE_HZ - (time.time() - began)))
    return 0


class Ears:
    """arecord + WeNet, the model loaded once (the first load takes ~3 s of ACL set-up)."""

    def __init__(self, asr_dir=None, seconds: float = 3.0):
        import asr
        self.model = asr.load(asr_dir)
        self.seconds = seconds

    def listen(self) -> str:
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as f:
            wav = f.name
        try:
            subprocess.run(["arecord", "-q", "-f", "S16_LE", "-r", "16000", "-c", "1",
                            "-d", str(max(1, int(round(self.seconds)))), wav], timeout=self.seconds + 10, check=False)
            if not Path(wav).exists() or Path(wav).stat().st_size <= 1000:
                return ""
            return self.model.transcribe(wav) or ""
        finally:
            Path(wav).unlink(missing_ok=True)


def run_voice(args, gw: Gateway) -> int:
    if args.text:
        text = args.text
    else:
        import asr
        text = asr.transcribe(args.wav, args.asr_dir)["text"]
    say(gw.voice(text), f"语音「{text}」")
    return 0


def run_listen(args, gw: Gateway) -> int:
    """The gateway's 监听 switch runs this until it is stopped: gesture frames at GESTURE_HZ; with --voice, a
    recording every --mic seconds in a second thread (recording blocks)."""
    import threading
    stop = threading.Event()
    if args.voice:
        ears = Ears(args.asr_dir, args.mic)

        def hear():
            while not stop.is_set():
                text = ears.listen()
                if text:
                    say(gw.voice(text), f"语音「{text}」")
        threading.Thread(target=hear, daemon=True).start()
    print(f"监听中：手势{'、语音' if args.voice else ''}；结果交给 {gw.base}，Ctrl-C 退出", flush=True)
    try:
        args.label, args.seconds = None, 0
        return run_gesture(args, gw)
    except KeyboardInterrupt:
        return 0
    finally:
        stop.set()


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--gateway", default=os.environ.get("ARM_GATEWAY", "http://127.0.0.1:8080"), help="网关地址")
    p.add_argument("--token", default=os.environ.get("ARM_GATEWAY_TOKEN"), help="网关的 --token")
    sub = p.add_subparsers(dest="cmd", required=True)
    g = sub.add_parser("gesture", help="从腕部相机连续识别，或 --label 直接给标签")
    v = sub.add_parser("voice", help="转写一段 wav，或 --text 直接给一句话")
    L = sub.add_parser("listen", help="常驻：手势（可选语音），给网关的「监听」开关用")
    for q in (g, L):
        q.add_argument("--url", default=SNAPSHOT, help="相机快照地址")
        q.add_argument("--gesture-dir", help="手势模型目录（默认 data/models/gesture_hrnet）")
    g.add_argument("--seconds", type=float, default=30.0, help="识别多少秒（0 = 一直）")
    g.add_argument("--label", help="跳过识别：直接给标签")
    g.add_argument("--times", type=int, default=3, help="配合 --label：连发几帧")
    g.add_argument("--confidence", type=float, default=0.95, help="配合 --label：置信度")
    v.add_argument("--wav", default=str(ROOT / "data" / "models" / "wenet_asr" / "sample.wav"))
    v.add_argument("--text", help="跳过识别：直接给一句话")
    for q in (v, L):
        q.add_argument("--asr-dir", help="语音模型目录（默认 data/models/wenet_asr）")
    L.add_argument("--voice", action="store_true", help="同时录音识别（板上音频输入未确认，默认关）")
    L.add_argument("--mic", type=float, default=3.0, help="每段录音秒数")
    args = p.parse_args(argv)
    gw = Gateway(args.gateway, args.token)
    return {"gesture": run_gesture, "voice": run_voice, "listen": run_listen}[args.cmd](args, gw)


if __name__ == "__main__":
    raise SystemExit(main())
