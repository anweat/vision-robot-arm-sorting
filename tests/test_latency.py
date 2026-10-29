"""验收补充：事件发生到网页拿到，实测端到端延迟。

对外服务按 SSE 推送，事件自带 t（写入时刻）。这里在服务同机订阅 /api/log/stream，
用"收到时刻 - 事件 t"当作这一层的延迟上界。W1 要求 ≤ 1 s。
"""
import argparse
import json
import sys
import threading
import time
import urllib.request
from http.client import HTTPConnection
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "web"))
sys.path.insert(0, str(ROOT / "src" / "tools"))

SAMPLE = ROOT / "docs" / "samples" / "log_sim_8rounds.jsonl"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", type=int, default=8099)
    ap.add_argument("--seconds", type=float, default=25.0)
    ap.add_argument("--speed", type=float, default=20.0)
    args = ap.parse_args()

    import serve as S

    log = ROOT / "data" / "latency_probe.jsonl"
    log.parent.mkdir(parents=True, exist_ok=True)
    log.write_text("", encoding="utf-8")
    state = S.State(log)
    S.Replayer(SAMPLE, state, speed=args.speed, loop=True).start()

    from http.server import ThreadingHTTPServer

    httpd = ThreadingHTTPServer(("127.0.0.1", args.port), S.Handler)
    httpd.state = state
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    time.sleep(1.0)

    conn = HTTPConnection("127.0.0.1", args.port, timeout=30)
    conn.request("GET", "/api/log/stream?after=0")
    resp = conn.getresponse()
    subscribed = time.time()

    # 只统计"订阅之后才发生"的事件。连接那一刻之前写下的属于首屏回填，它们的年龄取决于
    # 页面多晚才打开，不是这一层的推送延迟；把它们算进来会把指标测成 1 s 以上。
    gaps, begun, backfilled = [], time.time(), 0
    buffer = b""
    while time.time() - begun < args.seconds:
        chunk = resp.read1(65536)
        if not chunk:
            break
        buffer += chunk
        while b"\n\n" in buffer:
            frame, buffer = buffer.split(b"\n\n", 1)
            for line in frame.split(b"\n"):
                if not line.startswith(b"data: "):
                    continue
                event = json.loads(line[6:])
                t = event.get("t")
                if not isinstance(t, (int, float)):
                    continue
                if t < subscribed:
                    backfilled += 1
                    continue
                gaps.append(time.time() - t)
    conn.close()
    httpd.shutdown()

    print(f"首屏回填 {backfilled} 条不计入；实时事件 {len(gaps)} 条")
    if not gaps:
        print("没有收到实时事件，无法测量")
        return 2
    gaps.sort()
    n = len(gaps)
    print(f"延迟：中位 {gaps[n // 2] * 1000:.0f} ms  "
          f"p95 {gaps[min(n - 1, int(n * 0.95))] * 1000:.0f} ms  最大 {gaps[-1] * 1000:.0f} ms")
    verdict = "通过" if gaps[-1] <= 1.0 else "不通过"
    print(f"验收口径 ≤ 1 s：{verdict}（最大 {gaps[-1]:.3f} s）")
    return 0 if gaps[-1] <= 1.0 else 1


if __name__ == "__main__":
    sys.exit(main())
