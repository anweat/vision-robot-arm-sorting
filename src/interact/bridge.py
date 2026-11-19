"""Recogniser -> command bridge for E1 (gesture) and E7 (voice).

The rules live in exactly one place, `src/interact/intents.py` (armed switch, keyword match,
confidence threshold, N consecutive frames, cooldown). This file is the missing piece between a
recogniser and those rules:

    wav / camera frame  ->  src/audio/asr.py | src/vision/gesture.py  ->  text / (label, prob)
                        ->  Intents.voice() / Intents.gesture()
                        ->  POST <gateway>/api/cmd   (start / stop / run_preset)

Run it on the board (needs ais_bench for both models):

    python3 src/interact/bridge.py arm                          # 武装（一条指令只触发一次）
    python3 src/interact/bridge.py voice --wav data/models/wenet_asr/sample.wav
    python3 src/interact/bridge.py voice --text 跳舞             # 跳过识别，只测链路
    python3 src/interact/bridge.py gesture --seconds 10          # 从腕部相机抓 10 秒
    python3 src/interact/bridge.py gesture --label palm --times 3

Why it posts to /api/cmd instead of the console's /api/interact/*: the console (src/teleop/server.py)
does not mount those routes yet — only the simulator does — so the gateway's /api/interact proxy
answers 501 on the real arm. `docs/atlas-200i-dk-a2/25-语音手势接口.md` records the intended design;
when the console gains the routes, this bridge can be pointed at them with `--interact`.

The `start` command is the only one whose `interaction` event the gateway writes today
(`State.start_task`); stop / run_preset do not, which is why the log gap is listed as unfinished.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
for sub in ("interact", "audio", "vision"):
    sys.path.insert(0, str(ROOT / "src" / sub))

import intents as I  # noqa: E402


def append_event(path: str | Path, event: dict) -> None:
    """把受理的 interaction 写进网关的 log.jsonl（契约里 seq 由读取端按行号生成，追加即可）。"""
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    with p.open("a", encoding="utf-8") as f:
        f.write(json.dumps(event, ensure_ascii=False) + "\n")


class Gateway:
    """The gateway's /api/cmd, with its token."""

    def __init__(self, base: str, token: str | None = None):
        self.base = base.rstrip("/")
        self.token = token

    def _request(self, path: str, body: dict | None = None) -> dict:
        data = json.dumps(body).encode() if body is not None else None
        req = urllib.request.Request(self.base + path, data=data,
                                     method="POST" if body is not None else "GET")
        req.add_header("Content-Type", "application/json")
        if self.token:
            req.add_header("X-Token", self.token)
        try:
            with urllib.request.urlopen(req, timeout=240) as r:
                return json.loads(r.read())
        except urllib.error.HTTPError as exc:
            try:
                return json.loads(exc.read())
            except Exception:                                  # noqa: BLE001
                raise RuntimeError(f"{path}: HTTP {exc.code}") from exc

    def cmd(self, **body) -> dict:
        return self._request("/api/cmd", body)


def make_executor(gateway: Gateway, source: str):
    """Intents calls this once per accepted command; raising means 'not done, tell the user'."""

    def execute(command: str, preset: str | None) -> str:
        body = dict(cmd=command, source=source)
        if command == "gripper":          # 配置里写的是 gripper:open，网关要的是 cmd+action
            body["action"] = preset
        elif preset:
            body["preset"] = preset
        result = gateway.cmd(**body)
        if not result.get("ok"):
            raise ValueError(result.get("error") or "网关拒绝")
        return f"网关已执行 {command}" + (f" / {preset}" if preset else "")

    return execute


def run_voice(args, gw: Gateway) -> int:
    if args.text:
        text, seconds = args.text, 0.0
    else:
        import asr

        got = asr.transcribe(args.wav, args.asr_dir)
        text, seconds = got["text"], got["seconds_median"] if "seconds_median" in got else got["seconds"]
    it = I.Interaction(execute=make_executor(gw, "voice"))
    if args.armed:
        it.arm(True)
    result = it.voice(text)
    result["_recognised"] = text
    result["_asr_seconds"] = seconds
    if args.log and result.get("event"):
        append_event(args.log, result["event"])
    print(json.dumps(result, ensure_ascii=False, indent=1))
    return 0


def run_gesture(args, gw: Gateway) -> int:
    it = I.Interaction(execute=make_executor(gw, "gesture"))
    if args.armed:
        it.arm(True)
    if args.label:                       # 跳过识别，只测判定 + 执行
        labels = [(args.label, 1.0)] * max(1, args.times)
        for label, prob in labels:
            out = it.gesture(label, prob)
            print(json.dumps(out, ensure_ascii=False))
            if args.log and out.get("event"):
                append_event(args.log, out["event"])
            time.sleep(0.05)
        return 0

    import gesture as G

    model = G.Gesture(args.gesture_dir)
    deadline = time.time() + args.seconds
    fired = 0
    while time.time() < deadline:
        frame = G.grab(args.url)
        if frame is None:
            time.sleep(0.05)
            continue
        r = model.classify(frame)
        out = it.gesture(r["label"], r["confidence"])
        if out.get("status") == "accepted":
            fired += 1
            print(json.dumps(out, ensure_ascii=False))
            if args.log and out.get("event"):
                append_event(args.log, out["event"])
        time.sleep(0.03)
    print(json.dumps(dict(frames_seconds=args.seconds, accepted=fired), ensure_ascii=False))
    return 0


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--gateway", default="http://127.0.0.1:8081", help="对外服务地址")
    p.add_argument("--token", default="armtest", help="对外服务的 --token")
    p.add_argument("--armed", action="store_true", help="本次运行前先武装（否则除 stop 外都会被拒）")
    p.add_argument("--log", default=str(ROOT / "data" / "runs" / "web" / "log.jsonl"),
                   help="把受理的 interaction 追加到这个日志（与网关 --log 一致）")
    sub = p.add_subparsers(dest="cmd", required=True)

    sub.add_parser("arm", help="只武装一次，然后退出")

    v = sub.add_parser("voice", help="转写一段 wav（或 --text）并交给规则层")
    v.add_argument("--wav", default=str(ROOT / "data" / "models" / "wenet_asr" / "sample.wav"))
    v.add_argument("--text", help="跳过识别，直接给一句话")
    v.add_argument("--asr-dir", help="语音模型目录")

    g = sub.add_parser("gesture", help="从相机抓若干秒，或直接给 label")
    g.add_argument("--seconds", type=float, default=10.0)
    g.add_argument("--label", help="跳过识别，直接给标签")
    g.add_argument("--times", type=int, default=3, help="配合 --label：模拟连续帧数")
    g.add_argument("--url", default="http://127.0.0.1:8080/snapshot.jpg")
    g.add_argument("--gesture-dir", help="手势模型目录")

    args = p.parse_args(argv)
    gw = Gateway(args.gateway, args.token)
    if args.cmd == "arm":
        it = I.Interaction(execute=make_executor(gw, "voice"))
        print(json.dumps(it.arm(True), ensure_ascii=False))
        return 0
    return run_voice(args, gw) if args.cmd == "voice" else run_gesture(args, gw)


if __name__ == "__main__":
    raise SystemExit(main())
