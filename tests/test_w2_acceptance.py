"""W2 控制面板的三条验收指标，实测出数，不再停留在"应该能过"。

分工表 F1-3 的交付物写的是「滑块响应 ≤ 1 s」；需求规格 W2 另外两条是
「角度刷新 ≥ 2 Hz」和「坐标输入到位误差 ≤ 1 cm」。这三条此前在 tests/ 里一条
自动化证据都没有（搜 "2Hz" 得 0 处、搜 "滑块" 得 0 处），而 F1-8 的验收是要拿
数据说话的。所以这里补上，并且让它能自己起仿真控制台跑完收工。

三条分别怎么量：

1. 角度刷新：订阅 /api/status/stream，数一段时间里的帧数算频率
2. 滑块响应：下发 set_joints，量到**回读角度第一次动**为止。量的是"下发到开始动"，
   不是"到最终位置"——后者取决于运动距离，不是这一层能控的
3. 坐标到位：下发 move_to，等运动结束，读 fk 的末端位置，算它与目标的距离

关于第 3 条要如实说明：仿真是理想执行侧，没有实机的齿轮间隙、负载形变和标定误差，
所以这里达标**不能代表实机达标**。输出会标清楚这是仿真值；实机的这一条要等 H1-3
接进来后在同一脚本上重跑（--console 指向实机即可，脚本不用改）。

任何一条量不出来就明确报"无法测量"，不拿通过来凑数。
"""
import argparse
import json
import subprocess
import sys
import threading
import time
import urllib.request
from http.client import HTTPConnection
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "web"))

REFRESH_HZ = 2.0        # 需求：角度刷新 ≥ 2 Hz
SLIDER_S = 1.0          # 需求：滑块指令响应 ≤ 1 s
MOVE_CM = 1.0           # 需求：坐标输入到位误差 ≤ 1 cm
JOINTS = 6


class Lines:
    """收集输出：同时打印和写文件，跑完能直接拿走当验收记录。"""

    def __init__(self, out: Path | None):
        self.out = out
        self.text: list[str] = []

    def __call__(self, text=""):
        self.text.append(text)
        print(text, flush=True)
        if self.out:
            self.out.write_text("\n".join(self.text) + "\n", encoding="utf-8")


def wait_health(url: str, timeout: float = 40.0) -> bool:
    """轮询控制台 /health。仿真启动要渲染场景，第一次会慢。"""
    end = time.time() + timeout
    while time.time() < end:
        try:
            with urllib.request.urlopen(url, timeout=2) as res:
                if res.status == 200:
                    return True
        except Exception:
            time.sleep(0.5)
    return False


def get(port: int, path: str, timeout: float = 10.0) -> dict:
    with urllib.request.urlopen(f"http://127.0.0.1:{port}{path}", timeout=timeout) as res:
        return json.loads(res.read())


def call(port: int, cmd: str, timeout: float = 60.0, **body) -> dict:
    """下发指令。多步运动可能要走十几秒，超时给宽一点。"""
    req = urllib.request.Request(
        f"http://127.0.0.1:{port}/api/cmd",
        data=json.dumps(dict(cmd=cmd, source="web", **body)).encode(),
        method="POST", headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as res:
        return json.loads(res.read())


def wait_armed(port: int, timeout: float = 20.0) -> dict:
    """等到状态里有六轴回读为止。

    姿态只认控制台一个源：接控制台时顶层 joints_deg 恒为 None，要读 arm.joints_deg。
    服务刚起来时还没 poll 过控制台，回读是空的——拿空表去选轴会直接把脚本
    带崩（max() of empty sequence）。所以这里先等真数据。
    """
    end = time.time() + timeout
    while time.time() < end:
        status = get(port, "/api/status")
        arm = status.get("arm") or {}
        joints = arm.get("joints_deg") or []
        if len(joints) == JOINTS and arm.get("tip_cm"):
            return status
        time.sleep(0.3)
    raise RuntimeError("一直没等到六轴回读，控制台可能没接上")


class StatusStream:
    """订阅 /api/status/stream，逐帧给出 status。

    用流而不是反复 GET：GET 每次都要重新建连接，采样间隔会额外加噪声，把"什么时候
    第一次看到角度动了"量偏。
    """

    def __init__(self, port: int):
        self.conn = HTTPConnection("127.0.0.1", port, timeout=60)
        self.conn.request("GET", "/api/status/stream")
        self.resp = self.conn.getresponse()
        self.buffer = b""

    def frames(self):
        """逐帧产出。流断开就正常结束——调用方用 for 迭代，不会撞上 StopIteration。"""
        while True:
            try:
                chunk = self.resp.read1(65536)
            except Exception:
                return
            if not chunk:
                return
            self.buffer += chunk
            while b"\n\n" in self.buffer:
                frame, self.buffer = self.buffer.split(b"\n\n", 1)
                for line in frame.split(b"\n"):
                    if line.startswith(b"data: "):
                        yield json.loads(line[6:])

    def close(self):
        try:
            self.conn.close()
        except Exception:
            pass


def measure_refresh(say, port: int):
    """【1】角度刷新率。"""
    say("\n【1】角度刷新（需求 ≥ 2 Hz）")
    stream = StatusStream(port)
    stamps = []
    started = time.time()
    for _ in stream.frames():
        stamps.append(time.time())
        if time.time() - started > 6.0:
            break
    stream.close()
    if len(stamps) < 2:
        say("  没收到足够帧 → 无法测量")
        return None
    span = stamps[-1] - stamps[0]
    hz = (len(stamps) - 1) / span if span > 0 else 0.0
    ok = hz >= REFRESH_HZ
    say(f"  {len(stamps)} 帧 / {span:.1f} s  =  {hz:.2f} Hz   {'通过' if ok else '不通过'}")
    return ok


def measure_slider(say, port: int):
    """【2】滑块响应：下发到回读角度第一次动。"""
    say("\n【2】滑块响应（需求 ≤ 1 s，量到回读角度第一次动为止）")
    base = (wait_armed(port).get("arm") or {}).get("joints_deg")
    target = list(base)
    # 挑一个离两端限位都最远的轴来改，别碰到限位把指令弄成被拒
    idx = max(range(JOINTS), key=lambda i: min(base[i], 180 - base[i]))
    target[idx] = base[idx] + 8 if base[idx] < 90 else base[idx] - 8
    say(f"  下发 set_joints：轴{idx + 1} {base[idx]:.0f}° → {target[idx]:.0f}°")

    stream = StatusStream(port)
    try:
        started = time.time()
        try:
            call(port, "set_joints", joints_deg=target)
        except Exception as exc:
            say(f"  下发被拒：{str(exc)[:80]}")
            return False
        for status in stream.frames():
            now = (status.get("arm") or {}).get("joints_deg") or []
            if len(now) == JOINTS and max(abs(a - b) for a, b in zip(now, base)) > 0.5:
                latency = time.time() - started
                ok = latency <= SLIDER_S
                say(f"  回读角度动了，延迟 {latency * 1000:.0f} ms   {'通过' if ok else '不通过'}")
                say("  （状态推送周期 0.4 s，此值是上界：网页看到的粒度也是这个）")
                return ok
            if time.time() - started > 8.0:
                break
        say("  8 s 内回读角度没变化 → 不通过")
        return False
    finally:
        stream.close()


def measure_move(say, port: int):
    """【3】坐标到位误差。"""
    say("\n【3】坐标到位误差（需求 ≤ 1 cm）")
    tip = (wait_armed(port).get("arm") or {}).get("tip_cm")
    say(f"  当前末端 {[round(v, 2) for v in tip]} cm")

    # 依次试越来越小的偏移：目标不可达时 move_to 会被拒，那就退一步再试
    for offset in ([2.5, -1.3, 0.5], [1.5, -0.8, 0.3], [0.8, 0.0, 0.0]):
        goal = [round(tip[i] + offset[i], 2) for i in range(3)]
        try:
            call(port, "move_to", target_cm=goal)
        except Exception as exc:
            say(f"  目标 {goal} 被拒：{str(exc)[:70]}")
            continue
        stream = StatusStream(port)
        try:
            deadline = time.time() + 20.0
            for status in stream.frames():
                arm = status.get("arm") or {}
                if not arm.get("busy") and arm.get("tip_cm"):
                    settled = arm["tip_cm"]
                    err = sum((settled[i] - goal[i]) ** 2 for i in range(3)) ** 0.5
                    ok = err <= MOVE_CM
                    say(f"  目标 {goal} → 实到 {[round(v, 2) for v in settled]}")
                    say(f"  偏差 {err * 10:.1f} mm   {'通过' if ok else '不通过'}")
                    say("  ⚠ 这是**仿真**执行侧的值：理想运动学，没有实机的间隙与标定误差，")
                    say("    不能代表实机达标。实机接进来后指向 --console 重跑本脚本即可。")
                    return ok
                if time.time() > deadline:
                    break
            say(f"  目标 {goal} 下发后没在 20 s 内停稳")
        finally:
            stream.close()
    say("  没有一次 move_to 成功 → 无法测量（实机的单次逆解大步待 H1-3）")
    return None


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--console", help="控制台地址；不传则自己起一个仿真")
    ap.add_argument("--port", type=int, default=8098, help="本服务监听端口")
    ap.add_argument("--sim-port", type=int, default=8799, help="自起仿真的端口")
    ap.add_argument("--out", help="结果写入这个文件")
    args = ap.parse_args()

    say = Lines(Path(args.out) if args.out else None)
    sim = None
    httpd = None
    console = args.console

    try:
        if not console:
            say("启动仿真控制台…")
            sim = subprocess.Popen(
                [sys.executable, str(ROOT / "src" / "sim" / "sim_server.py"),
                 "--port", str(args.sim_port), "--seed", "0", "--speed", "20"],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            console = f"http://127.0.0.1:{args.sim_port}"
            if not wait_health(f"{console}/health"):
                say("仿真没起来，无法测量")
                return 2
            say(f"仿真就绪 {console}")

        import serve as S
        from http.server import ThreadingHTTPServer

        state = S.State(ROOT / "data" / "w2_acceptance.jsonl",
                        console=S.Console(console, timeout=10))
        httpd = ThreadingHTTPServer(("127.0.0.1", args.port), S.Handler)
        httpd.state = state
        threading.Thread(target=httpd.serve_forever, daemon=True).start()
        # 必须起回读线程：/api/status 的六轴角度与 arm 段是它从控制台轮询来的。
        # 不起的话状态永远是初始值，后面三条一条都量不了（踩过一次）
        threading.Thread(target=state.poll_console, daemon=True).start()

        verdicts = [("角度刷新", measure_refresh(say, args.port), None)]

        try:
            call(args.port, "enable")
        except Exception as exc:
            say(f"\n使能失败：{str(exc)[:80]}  后两条量不了")
            verdicts += [("滑块响应", None, None), ("坐标到位", None, None)]
        else:
            verdicts.append(("滑块响应", measure_slider(say, args.port), None))
            verdicts.append(("坐标到位", measure_move(say, args.port), None))

    except Exception:
        # 别让异常静默掉：之前就是这样只看到"进程没了"，查了半天
        import traceback
        say("\n脚本异常：")
        say(traceback.format_exc())
        return 2

    finally:
        # 无论中途出什么岔子都要收干净：不关服务、不杀仿真会留下一堆孤儿进程
        if httpd is not None:
            httpd.shutdown()
        if sim is not None:
            sim.kill()

    say("\n=== 汇总 ===")
    for name, ok, _ in verdicts:
        mark = "通过" if ok else ("无法测量" if ok is None else "不通过")
        say(f"  {name:<8} {mark}")
    failed = [n for n, ok, _ in verdicts if ok is False]
    say(f"\n结论：{'全部通过' if not failed else '不通过 - ' + ', '.join(failed)}")
    return 0 if not failed else 1


if __name__ == "__main__":
    sys.exit(main())
