#!/usr/bin/env python3
"""对外 HTTP 服务（F1-1）：电脑端网页与手机端唯一的入口。

    python3 src/web/serve.py --replay samples/log_sim_8rounds.jsonl
    python3 src/web/serve.py --log data/log.jsonl --lan --port 8080

只依赖标准库。日志的读取、逐块记录与统计一律复用 src/tools/log_contract.py，
与日志契约 v1（docs/current/18-日志契约.md）保持一致；这里的路径与
仿真后端（dev/sim/sim_server.py）同名同义，页面从仿真切到实机不用改。

    GET  /api/log            ?after&limit     事件分页（seq 为 1 起行号）
    GET  /api/log/stream     ?after           SSE 推送，id 是 seq，断线按 Last-Event-ID 续传
    GET  /api/log/records                     逐块记录（时间/类别/来源/目标/耗时/结果）
    GET  /api/log/summary                     统计
    GET  /api/scene | /api/scene/image        最近一次扫描的 scene.json 与重建图
    GET  /api/blocks                          桌上物块基座坐标与禁入区尺寸（三维视图用）
    GET  /api/archive | /api/archive/<name>   E6 未知物体告警列表与存档图片
    GET  /api/status                          当前模式与角度
    GET  /api/status/stream                   ≥ 2 Hz 角度推送（SSE）
    POST /api/cmd                             需求规格 ④ 的控制指令
    GET  /api/interact                        语音/手势状态（武装、词表、阈值、最近识别、监听进程）
    POST /api/interact/{arm,voice,gesture}    E1/E7：识别端（src/interact/bridge.py）报来的一句话 / 一帧
    GET  /api/health                          健康检查

语音/手势的判定**只有一份**：本服务持有的 src/interact/intents.py（武装、连续帧、阈值、冷却、
accepted/ignored/rejected 三态，口径见 25-语音手势接口.md）。受理的指令由本服务执行（开始、停止、
夹爪、预设动作）并写 interaction 事件；流程脚本的「等手势」也读这里的识别记录。实机控制台没有
这组端点（2026-11-20 起不再代理），仿真控制台自己那一份只给它的独立页面用。

实机接入前用 --replay 回放样例日志：页面、事件名、字段与实机相同，可以先验收
“条数一致、延迟 ≤ 1 s”。运动类指令需要控制台，未接控制台时按 501 返回，不假装执行。
"""
from __future__ import annotations

import argparse
import datetime
import json
import os
import secrets
import shlex
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, unquote, urlsplit

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src" / "tools"))
sys.path.insert(0, str(ROOT / "src"))
import log_contract as L  # noqa: E402
sys.path.insert(0, str(ROOT / "src" / "interact"))
import intents as I  # noqa: E402
from flow import script as FS  # noqa: E402   纯标准库：校验与存储，不导入 numpy
from flow.capabilities import catalog as capability_catalog  # noqa: E402

WEB = Path(__file__).resolve().parent

MODES = ("manual", "preset", "auto")
PRESETS = ("dance", "stack", "carry")   # 预设动作 = config/flows/<名字>.yaml 流程脚本（三份都有，2026-11-21）
SOURCES = ("web", "mobile", "voice", "gesture", "timer", "button")
CMDS = ("start", "stop", "set_mode", "run_preset", "set_joints", "move_to", "unlock_bin",
        "enable", "gripper", "heartbeat", "takeover", "speed", "drive", "goto", "flow_step", "alarm_ack", "release",
        "run_script", "listen", "led", "beep")
FLOW_STEPS = ("scan", "relocate", "plan", "grasp", "check", "retry", "back_off", "place", "verify", "home")
FLOW_SCRIPT = "config/flows/sort.yaml"
MOTION_CMDS = ("start", "run_preset", "set_joints", "move_to", "unlock_bin", "gripper", "drive", "goto")
CLIENT_LEASE_S = 2.5     # the gateway keeps the console lease only while a page heartbeats this recently
GRIPPER_ACTIONS = ("grip", "open", "wide")

MIME = {".html": "text/html; charset=utf-8", ".css": "text/css; charset=utf-8",
        ".js": "text/javascript; charset=utf-8", ".json": "application/json; charset=utf-8",
        ".svg": "image/svg+xml", ".png": "image/png", ".jpg": "image/jpeg",
        ".jpeg": "image/jpeg", ".webp": "image/webp", ".ico": "image/x-icon"}


def load_keep_out_cm() -> dict | None:
    """标准禁入区的尺寸，米转厘米，给三维视图画平台与禁入区用。

    读不到就返回 None，页面跳过这一层——不因为一个可视化的要素让整个接口挂掉。
    单位判断靠 `_m` 后缀：只有以米计的字段才换算。
    """
    path = Path(__file__).resolve().parents[2] / "config" / "arm_limits.json"
    try:
        cfg = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None
    keep = cfg.get("keep_out") or {}
    out = {}
    for key, value in keep.items():
        if key.startswith("_") or not isinstance(value, (int, float)):
            continue
        out[key[:-2] + "_cm" if key.endswith("_m") else key] = round(float(value) * (100 if key.endswith("_m") else 1), 3)
    return out or None


def safe_under(root: Path, name: str) -> Path | None:
    """把请求里的文件名限制在 root 之内；越界返回 None。

    文件名是日志事件带进来的，属于外部输入，所以这里不做任何"信任"假设：绝对路径、
    ../ 穿越、盘符都会被 join + resolve 挡在外面。允许子目录，但结果必须仍在 root 里。
    """
    if not name or "\x00" in name:
        return None
    try:
        base = root.resolve()
        candidate = (root / name).resolve()
    except (OSError, ValueError):
        return None
    if candidate == base or base not in candidate.parents:
        return None
    return candidate


class ConsoleError(ValueError):
    """控制台返回的业务错误。status 是 HTTP 状态码；reason 是控制台对错误的分类。
    一律原样上抛：控制台给的中文原因（画面过期、另一端持有操作权、正在运动……）就是页面该显示的。"""

    def __init__(self, message: str, status: int | None = None, reason: str | None = None):
        super().__init__(message)
        self.status = status
        self.reason = reason


class MotionError(ValueError):
    """多步运动中途失败：带上已走的步数，页面能告诉用户停在中间姿态。"""

    def __init__(self, message: str, steps_done: int = 0, steps_total: int = 0):
        super().__init__(message)
        self.steps_done = steps_done
        self.steps_total = steps_total


class Console:
    """把运动指令转发给控制台，并回读它的状态（视图层不自己实现任何安全边界）。

    控制台可以是仿真（dev/sim/sim_server.py，默认 8770）也可以是实机（src/teleop/server.py）；
    两边答同一套 /api/*（接口文档 §4），所以换目标这里一行都不用改。

    自文档 32 起控制台没有使能和档位：第一条运动指令就取得操作权，本服务用固定 owner 代表所有页面，
    只在页面还在发心跳时续租（State.keep_alive）。六轴绝对角度一次交给 /api/goto（整条路径先检查再动）；
    坐标移动仍拆成直角小步（core.cartesian 每步 ≤ 5 cm）。限位、禁区、堵转全由控制台把关。
    """

    CARTESIAN_STEPS = (5.0, 3.0, 2.0, 1.0, 0.5)  # core.cartesian 允许 0–5 cm
    CARTESIAN_DEADBAND_CM = 0.25
    MAX_MOVES = 60                               # 一条指令最多拆多少小步，防失控

    def __init__(self, base: str, owner: str = "web", timeout: float = 30.0):
        self.base = base.rstrip("/")
        self.owner = owner
        self.timeout = timeout
        self.token: str | None = None

    @property
    def target(self) -> str:
        return self.base

    def _fetch_token(self) -> None:
        """控制台每次启动生成一次性凭证（teleop.App.token），所有请求都要带 X-Console-Token。

        网页那边是 crypto.randomUUID 的 owner 加这个 token；本服务作为唯一客户端代理，
        用固定 owner 托管同一份凭证。
        """
        self.token = self._raw("/api/bootstrap").get("token")

    def _raw(self, path: str, body: dict | None = None):
        data = json.dumps(body, ensure_ascii=False).encode() if body is not None else None
        req = urllib.request.Request(self.base + path, data=data, method="POST" if data else "GET")
        if data:
            req.add_header("Content-Type", "application/json")
        if self.token:
            req.add_header("X-Console-Token", self.token)
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as res:
                raw, mime = res.read(), res.headers.get("Content-Type", "")
        except urllib.error.HTTPError as exc:
            raw = exc.read()
            try:
                data = json.loads(raw)
                raise ConsoleError(data.get("error") or f"控制台 {exc.code}",
                                   status=exc.code, reason=data.get("reason")) from exc
            except json.JSONDecodeError:
                raise ConsoleError(f"控制台 {exc.code} {path}", status=exc.code) from exc
        except urllib.error.URLError as exc:
            raise ConsoleError(f"连不上控制台 {self.base}：{exc.reason}") from exc
        if not raw:
            return {}
        return json.loads(raw) if "json" in mime else raw

    def request(self, path: str, body: dict | None = None, retry: bool = True):
        """GET/POST 控制台；业务错误以 ValueError 抛出，消息是控制台给出的中文原因。

        控制台重启会让旧凭证失效（"页面凭证过期"），这里自动重取一次再试。
        """
        if not self.token and path != "/api/bootstrap":
            self._fetch_token()
        try:
            return self._raw(path, body)
        except ValueError as exc:
            if not retry or "凭证" not in str(exc):
                raise
        self._fetch_token()
        return self._raw(path, body)

    def status(self) -> dict:
        return self.request("/api/status")

    def world(self) -> dict:
        """仿真世界状态：物块的基座坐标、类别、颜色。只有仿真有这个端点。

        实机控制台没有它，调用方按 ValueError 降级——没有位置就显示没有，不假造。
        """
        return self.request("/api/sim/world")

    def move(self, path: str, body: dict) -> dict:
        """运动指令带上 owner 转给控制台；不再需要 revision，错误原样上抛。"""
        return self.request(path, dict(body, owner=self.owner))

    def enable(self) -> dict:
        """旧接口：等同于取得操作权。"""
        return self.request("/api/enable", dict(owner=self.owner))

    def takeover(self) -> dict:
        return self.request("/api/takeover", dict(owner=self.owner))

    def speed(self, value) -> dict:
        return self.request("/api/speed", dict(owner=self.owner, speed=value))

    def drive(self, mode, v, wrist=0) -> dict:
        return self.request("/api/drive", dict(owner=self.owner, mode=mode, v=v, wrist=wrist))

    def goto(self, pose=None, named=None, dry=False) -> dict:
        return self.request("/api/goto", dict(owner=self.owner, pose=pose, named=named, dry=dry))

    def heartbeat(self) -> dict:
        return self.request("/api/heartbeat", dict(owner=self.owner))

    def stop(self) -> dict:
        return self.request("/api/stop", dict(owner=self.owner))

    def gripper(self, action: str) -> dict:
        """wide / open / grip，见 core.gripper。"""
        return self.move("/api/gripper", dict(action=action))

    @staticmethod
    def _chunks(total: float, steps: tuple, deadband: float) -> list[float]:
        """把 total 拆成允许的步长序列，从大到小；不足最小步长的零头丢弃。"""
        out: list[float] = []
        while abs(total) >= deadband:
            step = next((s for s in steps if s <= abs(total) + 1e-9), steps[-1])
            signed = step if total > 0 else -step
            out.append(signed)
            total = round(total - signed, 3)
        return out

    def set_joints(self, targets, pose) -> int:
        """六个绝对角度 -> 一次 /api/goto。返回控制台分了几段。"""
        if not pose or len(pose) != 6:
            raise ValueError("控制台还没有六轴回读，无法下发角度")
        target = [int(round(t)) for t in targets]
        if all(abs(t - p) < 1 for t, p in zip(target, pose)):
            return 0
        plan = self.goto(pose=target, dry=True)
        self.goto(pose=target)
        return len(plan.get("segments") or [])

    def move_to(self, target_cm, status: dict) -> int:
        """目标坐标（厘米）-> 一串直角点动。当前位置取控制台回读的 fk.tip（米）。"""
        tip = (status.get("fk") or {}).get("tip")
        if not tip or len(tip) < 3:
            raise ValueError("控制台没有回读末端位置，无法做坐标移动")
        moves = [(axis, d) for axis, want, now in zip("xyz", target_cm, tip)
                 for d in self._chunks(round(want - now * 100.0, 2),
                                       self.CARTESIAN_STEPS, self.CARTESIAN_DEADBAND_CM)]
        if len(moves) > self.MAX_MOVES:
            raise ValueError(f"这一步要拆 {len(moves)} 个直角点动，超过 {self.MAX_MOVES} 步上限")
        done = 0
        try:
            for axis, amount in moves:
                self.move("/api/cartesian", dict(axis=axis, amount=amount))
                done += 1
        except ValueError as exc:
            raise MotionError(f"{exc}（已走 {done}/{len(moves)} 步，机械臂可能停在中间姿态）",
                              steps_done=done, steps_total=len(moves)) from exc
        return done


class Timers:
    """W4 定时启动：网页配置时间，到点自动启动一整轮分拣。

    需求规格定的兜底是"到点先提示 5 s，可在页面取消，默认关闭"，所以这里：

    - **默认关闭**：服务起来不排任何任务；不传 --timers 文件时压根不启用这个功能
    - 到点前 LEAD 秒进入"待发"，`/api/status` 的 `timer.pending` 变 True，页面这时还能取消
    - 到点才真正下发 `start`；偏差 ≤ 5 s 靠 0.5 s 的检查周期保证
    - 取消=删除任务，不留"已禁用"的僵尸任务

    任务存在 JSON 文件里（`repeat=daily` 的会存下来），服务重启不会丢还没有跑的任务。
    """

    LEAD = 5.0          # 提前多少秒进入"待发"并开始提示
    TICK = 0.5          # 检查周期；启动时刻偏差就在这个量级
    GRACE = 5.0         # 到点后的宽限：错过超过这个窗口就视为"错过"，不补跑

    def __init__(self, path: Path, fire):
        self.path = path
        self.fire = fire                       # 到点回调：(timer) -> dict
        self.list: list[dict] = []
        self.pending: dict | None = None       # 已显示待发、还没扣扳机的任务
        self.last_result: dict | None = None
        self.running = True
        self.lock = threading.Lock()
        self.load()

    def load(self) -> None:
        try:
            data = json.loads(self.path.read_text(encoding="utf-8"))
            if isinstance(data, list):
                self.list = [t for t in data if isinstance(t, dict) and t.get("at")]
        except (OSError, ValueError):
            self.list = []

    def save(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.path.write_text(json.dumps(self.list, ensure_ascii=False, indent=1), encoding="utf-8")

    def add(self, at: float, repeat: str = "once") -> dict:
        timer = dict(id=f"t{int(time.time() * 1000)}", at=round(at, 3), repeat=repeat,
                     created_at=round(time.time(), 3))
        with self.lock:
            self.list.append(timer)
            self.save()
        return timer

    def remove(self, timer_id: str) -> bool:
        with self.lock:
            return self._remove(timer_id)

    def _remove(self, timer_id: str) -> bool:
        """删任务。**不加锁**——tick() 已经拿着锁时要调它，加进去会死锁。调用方负责持锁。"""
        before = len(self.list)
        self.list = [t for t in self.list if t.get("id") != timer_id]
        if len(self.list) == before:
            return False
        self.save()
        return True

    def next(self) -> dict | None:
        live = [t for t in self.list if t.get("at")]
        return min(live, key=lambda t: t["at"]) if live else None

    def run(self) -> None:
        """后台检查线程：到点前 LEAD 秒标记待发，到点真正下发。"""
        while self.running:
            try:
                self.tick()
            except Exception:                  # 定时器不能因为一次出错就整个停掉
                pass
            time.sleep(self.TICK)

    def stop(self) -> None:
        """停下检查线程。单测与关停都要能叫住它，否则进程退不干净。"""
        self.running = False

    def _settle(self, timer: dict, now: float) -> None:
        """触发（或错过）之后收尾：daily 重排到下一个未来时刻，once 删除。**不加锁**。"""
        if timer.get("repeat") == "daily":
            while timer["at"] <= now:
                timer["at"] += 86400
        else:
            self._remove(timer["id"])

    def tick(self, now: float | None = None) -> None:
        """推进一次。now 可注入，测试里不必真的等墙钟（生产不传，取 time.time）。

        错过不补跑：只触发 `now-GRACE < at <= now` 的任务；过期更久的 once 直接删除、
        daily 重排到下一个未来时刻（停机几天不会重启后连续补触发）。
        """
        now = time.time() if now is None else now
        with self.lock:
            due = [t for t in sorted(self.list, key=lambda t: t["at"])
                   if now - self.GRACE < t["at"] <= now]
            missed = [t for t in self.list if t["at"] <= now - self.GRACE]
            upcoming = [t for t in self.list if now < t["at"] <= now + self.LEAD]
            self.pending = upcoming[0] if upcoming else None
        # 触发放在锁外：fire 最终要向控制台下发，慢的话几秒钟，捧着锁会把 /api/status 一起堵住
        results = []
        for timer in due:
            try:
                results.append((timer, self.fire(timer)))
            except Exception as exc:             # 启动失败要留下来痕，任务照样收尾，不反复补触发
                results.append((timer, dict(ok=False, error=f"启动失败：{exc}")))
        with self.lock:
            for timer, result in results:
                self.last_result = dict(at=round(now, 3), **result)
                self._settle(timer, now)
            for timer in missed:
                self.last_result = dict(at=round(now, 3), ok=False,
                                        error="到点时服务未在运行，这次不补跑")
                self._settle(timer, now)
            if results or missed:
                self.save()

    def status(self, now: float | None = None) -> dict:
        now = time.time() if now is None else now
        with self.lock:
            nxt = self.next()
            pending = bool(self.pending)
            at = self.pending["at"] if self.pending else (nxt["at"] if nxt else None)
        # id 一并给出：页面在待发窗口里要能直接取消这一个，不用再拿时间去比对
        which = self.pending or nxt
        return dict(lead_s=self.LEAD, pending=pending, at=at,
                    id=which.get("id") if which else None,
                    countdown_s=round(at - now, 1) if at else None,
                    count=len(self.list), last_result=self.last_result)


class State:
    """服务端的少量状态：当前模式、最近一次回读的角度、日志与场景文件的位置。"""

    def __init__(self, log_path: Path, token: str | None = None,
                 scene_json: Path | None = None, scene_image: Path | None = None,
                 console: Console | None = None, timers: Timers | None = None,
                 archive: Path | None = None, flow_root: Path | None = None):
        self.log_path = log_path
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        self.token = token
        self.scene_json = scene_json
        self.scene_image = scene_image
        self.console = console
        self.timers = timers
        self.archive = archive
        self.mode = "manual"
        self.joints: list | None = None
        self.joints_t: float | None = None
        self.console_status: dict | None = None
        self.console_error: str | None = None
        self.console_observed_at: float | None = None   # 最近一次成功回读的本地墙钟，算 age_s 用
        self.keep_alive_on = False
        self.client_seen = 0.0                   # last page heartbeat (monotonic)
        self.motion_lock = threading.Lock()      # 运动串行化：同一时刻只执行一条运动指令
        self.replay = False                      # --replay 时为真：页面据此标"回放"，不冒充实时
        self.flow = None                         # 编排器子进程（src/flow/orchestrator.py）
        self.flow_lock = threading.Lock()
        self.flow_root = Path(flow_root) if flow_root is not None else ROOT / "data" / "flow"
        self.flow_python = [sys.executable]      # 跑编排器的解释器（要 numpy 等；--flow-python 改）
        self.task = "pipeline"                   # 「开始」跑什么：pipeline 已验证的旧流水线（默认）/ flow 分步编排器
        self.pipeline_args: list[str] = []       # 传给 src/pipeline/task.py → run.py 的参数（如 --bins DESIGN）
        self.flow_out = None
        self.flow_script: str | None = None      # 正在/最近跑的流程脚本名（run_script；「开始」跑 flow 时为 sort）
        # 流程脚本：config/flows 随应用发布（只读），data/flows 是各端保存的（文档 35）
        self.scripts = FS.Store(ROOT / "config" / "flows", ROOT / "data" / "flows")
        self.lock = threading.Lock()
        self.base_url = "http://127.0.0.1:8080"  # 本服务自己的地址（main 里按端口改）：编排器、识别端回连用
        # 语音/手势的判定（唯一一份）：受理的指令由 run_command 执行
        self.interaction = I.Interaction(execute=self.run_command, pass_source=True)
        self.listener = None                     # 识别端子进程（src/interact/bridge.py listen）
        self.listener_out: Path | None = None

    def console_sim(self) -> bool:
        """控制台是不是仿真：看它回读里自报的 mode（teleop/core.py：simulation / hardware），与
        flow/stages._is_sim 同一个口径。

        以前是额外请求一次 /api/sim/world、探到就永久记住——那一次请求慢或失败（Windows 上启动时）
        就一直当成实机，作业台画成实机骨架（俯视 + 侧视两个平面图），仿真三维再也不出来。
        """
        if self.console is None or self.console_status is None:
            return False                        # 还没连上，先按实机处理，别乱开；页面隔几秒会再问
        return self.console_status.get("mode") == "simulation"

    def append(self, obj: dict) -> None:
        with self.lock:
            with self.log_path.open("a", encoding="utf-8") as f:
                f.write(json.dumps(obj, ensure_ascii=False) + "\n")
                f.flush()

    def truncate_log(self) -> None:
        with self.lock:
            self.log_path.write_text("", encoding="utf-8")

    def note_pose(self, pose, t) -> None:
        if isinstance(pose, list) and len(pose) == 6:
            self.joints, self.joints_t = pose, t

    def keep_alive(self) -> None:
        """控制台的操作权租约只有 2 s。运动期间由后台线程每隔 0.5 s 续一次——但只在页面最近
        CLIENT_LEASE_S 内发过心跳（cmd=heartbeat）时才续：关页、断网、手机退后台后页面不再发，
        这里也就停续，控制台 2 s 后自停。死人开关因此一直通到页面，而不是停在本服务。
        """
        while self.keep_alive_on and self.console is not None:
            if time.monotonic() - self.client_seen > CLIENT_LEASE_S:
                self.keep_alive_on = False
                return
            try:
                self.console.heartbeat()
            except Exception:
                self.keep_alive_on = False
                return
            time.sleep(0.5)

    def client_beat(self) -> None:
        self.client_seen = time.monotonic()

    def arm_on(self) -> None:
        if self.console is None or self.keep_alive_on:
            return
        self.keep_alive_on = True
        threading.Thread(target=self.keep_alive, daemon=True).start()

    def arm_off(self) -> None:
        self.keep_alive_on = False

    def poll_console(self, every: float = 0.4) -> None:
        """后台回读控制台。≥ 2 Hz，满足“实时显示 6 个舵机角度（每秒 ≥ 2 次）”。

        只兜 ValueError 的话，连接被拒抛的 RemoteDisconnected（不是 ValueError）会把线程
        永久杀死，console_status 冻在最后一个好值、console_error 一直是 None——页面看到的
        是一条活着的机械臂，实际已失去监控。这里兜底捕获全部异常，线程永远不退出；
        age_s 用本地墙钟算，冻住时它会自己涨上去，前端一眼能看出过期。
        """
        while True:
            if self.console is not None:
                try:
                    status = self.console.status()
                    self.console_status, self.console_error = status, None
                    self.console_observed_at = time.time()
                    self.note_pose(status.get("pose"), status.get("observed_at") or time.time())
                except Exception as exc:
                    self.console_error = str(exc)
                    if "操作权" in str(exc) or "另一端" in str(exc):
                        self.keep_alive_on = False
            time.sleep(every)

    def arm(self) -> dict:
        """控制台给机器人的那一部分状态，未接控制台时为空 dict。"""
        status = self.console_status or {}
        if not status:
            return dict(attached=self.console is not None, error=self.console_error)
        grip = status.get("grip") or {}
        sim = status.get("sim") or {}
        # 正运动学结果直接转给页面：末端一串骨架点，厘米。页面按它画三维视图，不在前端
        # 再实现一遍运动学——这样图上看到的位置和控制台拿去做禁入区检查的是同一份数。
        fk = status.get("fk") or {}
        chain = {k: [round(v * 100, 2) for v in p]
                 for k, p in fk.items() if isinstance(p, list) and len(p) == 3}
        # age_s 不透传控制台自报的值，用本地墙钟减最近一次成功回读：控制台掉线、线程
        # 没拿到新值时，这个数会持续涨，前端就能看出回读已过期，而不是显示一个冻住的 0.88 s
        if self.console_observed_at is not None:
            age_s = round(max(0.0, time.time() - self.console_observed_at), 3)
        else:
            age_s = None
        return dict(
            attached=True, error=self.console_error,
            joints_deg=status.get("pose"),
            target_deg=status.get("command"),
            armed=bool(status.get("armed")), busy=bool(status.get("busy")),
            revision=status.get("revision"), owner=status.get("owner"),
            speed=status.get("speed"), drive=status.get("drive"),
            age_s=age_s, observed_at=status.get("observed_at"),
            clearance_cm=status.get("clearance_cm"),
            tip_cm=chain.get("tip"),
            chain_cm=dict(chain, base=[0.0, 0.0, 0.0]) if chain else None,
            tip_pitch_deg=(fk.get("tip_pitch_deg") or [None])[0],
            grip_action=grip.get("action"), grip_readback=grip.get("readback"),
            holding=grip.get("holding"), grip_result=grip.get("result"),
            error_text=status.get("error") or "",
            blocks=sim.get("blocks"), holding_block=sim.get("holding"),
            limits=status.get("limits"),
            signal=status.get("signal"),
        )

    def _spawn_flow(self, args: list[str], source: str, command: str, script: str = "flow", log: bool = True) -> dict:
        with self.flow_lock:
            return self._spawn_flow_locked(args, source, command, script, log)

    def _spawn_flow_locked(self, args: list[str], source: str, command: str, script: str, log: bool) -> dict:
        """Run the task executor against the console as a child process: src/flow/orchestrator.py
        (文档 32 §3), or with script="pipeline" src/pipeline/task.py（已验证的旧流水线，文档 34 §4.2）。

        同一时刻只跑一个流程；页面持有的操作权先放掉（流程用自己的 owner 取得），
        事件按日志契约写进本服务的活日志，页面经 SSE 直接看到 step / alarm / pipeline_* 。"""
        console = self.console
        if console is None:
            return dict(ok=False, error="未接控制台，没有可启动的任务执行侧")
        if self.flow and self.flow.poll() is None:
            return dict(ok=False, error="已有流程在运行；先停止")
        try:
            console.stop()
        except ValueError:
            pass
        self.arm_off()
        if script == "pipeline":
            cmd = [*self.flow_python, str(ROOT / "src" / "pipeline" / "task.py"),
                   "--console", console.base, "--log", str(self.log_path), *args]
        else:
            cmd = [*self.flow_python, str(ROOT / "src" / "flow" / "orchestrator.py"), *args,
                   "--console", console.base, "--really", "--log", str(self.log_path)]
        if script != "pipeline" and "--session" not in args:
            cmd += ["--session", str(self.flow_root / f"{time.strftime('%Y%m%d_%H%M%S')}_{secrets.token_hex(4)}")]
        self.flow_out = self.flow_root / "gateway_last.out"
        self.flow_out.parent.mkdir(parents=True, exist_ok=True)
        env = dict(os.environ, ARM_GATEWAY=self.base_url, ARM_GATEWAY_TOKEN=self.token or "")
        with self.flow_out.open("w", encoding="utf-8") as output:
            self.flow = subprocess.Popen(cmd, cwd=ROOT, env=env, stdout=output, stderr=subprocess.STDOUT)
        self.flow_script = {"flow": "sort", "pipeline": None, "step": None}.get(script, script)
        if log:                                  # 语音/手势触发的由 intents 的事件记（带 confidence / detail）
            try:
                self.append(dict(t=time.time(), event="interaction", source=source, command=command))
            except ValueError:
                pass
        return dict(ok=True, started=True, pid=self.flow.pid, args=args, task=script)

    def start_task(self, source: str = "web", log: bool = True) -> dict:
        """启动一整轮分拣（按钮、语音、手势、定时都走这里）：默认 `--task pipeline` 跑已在实机验证的
        旧流水线（src/pipeline/task.py → run.py）；`--task flow` 按 config/flows/sort.yaml 跑分步编排器
        （新规划器对实机扫描太保守，文档 34 §4.2，优化前只作试验）。
        有没确认的报警时，只有人在页面上点的「开始」能启动；语音、手势、定时一律拒绝（文档 32 §3.2）。"""
        if source not in ("web", "mobile", "button") and self.alarms():
            return dict(ok=False, error="有未确认的报警，先在页面上点“知道了”")
        if self.task == "pipeline":
            return self._spawn_flow(list(self.pipeline_args), source, "start", script="pipeline", log=log)
        return self._spawn_flow(["--script", FLOW_SCRIPT], source, "start", log=log)

    def run_script(self, name: str, source: str = "web", log: bool = True) -> dict:
        """按名字跑一份流程脚本（内置或各端保存的）。先在这里查一遍：有错误、用到板上还没有的能力，
        直接拒绝并给原因，不起子进程（编排器自己在动臂前也会再查一次）。"""
        if source not in ("web", "mobile", "button") and self.alarms():
            return dict(ok=False, error="有未确认的报警，先在页面上点“知道了”")
        got = self.scripts.get(name) if isinstance(name, str) else None
        if got is None:
            return dict(ok=False, error=f"没有脚本 {name!r}")
        c = got["check"]
        if not c["ok"]:
            return dict(ok=False, error="脚本有错误：" + c["errors"][0]["message"])
        if c["unavailable"]:
            return dict(ok=False, error="脚本用到了还不可用的能力：" + "、".join(c["unavailable"]) + "（见能力表）")
        return self._spawn_flow(["--script", got["path"]], source, f"script:{name}", script=name, log=log)

    # ---- 语音 / 手势 ---------------------------------------------------------------------------------
    def stop_all(self, source: str) -> None:
        """停止：写 stopped、放开操作权、停流程、停控制台。页面、语音、手势都走这里。"""
        self.append(dict(t=time.time(), event="stopped", reason=f"{source} 下发停止"))
        self.arm_off()
        self.stop_flow()
        if self.console is not None:
            try:
                self.console.stop()
            except ValueError:
                pass

    def run_command(self, command: str, preset: str | None, source: str = "voice") -> str:
        """intents 受理的一条语音/手势指令：在这里执行；做不了就抛 ValueError / NotImplementedError，原因回给识别端。"""
        if command == "stop":
            self.stop_all(source)
            return "已停止"
        if command == "start":
            r = self.start_task(source=source, log=False)
            if not r.get("ok"):
                raise ValueError(r.get("error") or "启动失败")
            return "已启动一整轮分拣"
        if command == "run_preset":
            if preset not in PRESETS or self.scripts.get(preset) is None:
                raise NotImplementedError(f"预设动作“{preset}”还没有写成流程脚本（config/flows/{preset}.yaml）")
            r = self.run_script(preset, source=source, log=False)
            if not r.get("ok"):
                raise ValueError(r.get("error") or "启动失败")
            return f"已开始预设动作 {preset}"
        if command == "gripper":
            if self.console is None:
                raise ValueError("未接控制台")
            if self.flow and self.flow.poll() is None:
                raise ValueError("流程正在运行，夹爪交给流程")
            if not self.motion_lock.acquire(blocking=False):
                raise ValueError("上一条运动指令还在执行")
            try:
                self.console.enable()
                self.console.gripper(preset)
            finally:
                self.motion_lock.release()
                try:
                    self.console.stop()
                except ValueError:
                    pass
            return f"夹爪 {preset}"
        raise ValueError(f"未知指令 {command}")

    def start_listener(self, voice: bool = False) -> dict:
        """识别端常驻进程（src/interact/bridge.py listen）：腕部相机逐帧手势（可选录音识别），结果回报本服务。"""
        if self.listener and self.listener.poll() is None:
            return dict(ok=True, already=True, pid=self.listener.pid)
        cmd = [*self.flow_python, str(ROOT / "src" / "interact" / "bridge.py"), "--gateway", self.base_url,
               *(["--token", self.token] if self.token else []), "listen", *(["--voice"] if voice else [])]
        self.listener_out = ROOT / "data" / "web" / "listen.out"
        self.listener_out.parent.mkdir(parents=True, exist_ok=True)
        self.listener = subprocess.Popen(cmd, cwd=ROOT, stdout=self.listener_out.open("w"), stderr=subprocess.STDOUT)
        return dict(ok=True, started=True, pid=self.listener.pid, voice=voice)

    def stop_listener(self) -> bool:
        p = self.listener
        if p is None or p.poll() is not None:
            return False
        p.terminate()
        try:
            p.wait(timeout=5)
        except subprocess.TimeoutExpired:
            p.kill()
        return True

    def listener_state(self) -> dict:
        p = self.listener
        tail: list[str] = []
        if self.listener_out is not None:
            try:
                tail = [ln[:300] for ln in self.listener_out.read_text(encoding="utf-8", errors="replace").splitlines()[-6:]]
            except OSError:
                pass
        return dict(running=bool(p and p.poll() is None), exit_code=None if p is None else p.poll(), output=tail)

    def interact(self, path: str, body: dict) -> dict:
        it = self.interaction
        if path == "/api/interact/arm":
            result = it.arm(body.get("armed") is True)
        elif path == "/api/interact/voice":
            result = it.voice(body.get("text"))
        elif path == "/api/interact/gesture":
            result = it.gesture(body.get("label"), body.get("confidence"))
        else:
            raise KeyError(path)
        event = result.pop("event", None)
        if event:
            self.append(event)
        return result

    def run_step(self, step: str, block: str | None, session: str | None, source: str = "web") -> dict:
        """单独执行一步（文档 32 §2）：不给 session 时用最近一次流程的会话目录。"""
        if step not in FLOW_STEPS:
            return dict(ok=False, error=f"step 应为 {'/'.join(FLOW_STEPS)}")
        session = session or self.flow_session or None
        args = ["--step", step] + (["--session", session] if session else []) + (["--block", block] if block else [])
        out = self._spawn_flow(args, source, f"step:{step}", script="step")
        return out

    def flow_state(self) -> dict:
        """编排器子进程：在不在跑、退出码、输出最后几行（起不来时页面能看到原因，不只是没动静）。"""
        p = self.flow
        tail: list[str] = []
        if self.flow_out is not None:
            try:
                tail = [ln[:300] for ln in self.flow_out.read_text(encoding="utf-8", errors="replace").splitlines()[-8:]
                        if not ln.startswith('{"t"')]
            except OSError:
                pass
        return dict(running=bool(p and p.poll() is None), exit_code=None if p is None else p.poll(),
                    pid=None if p is None else p.pid, output=tail, task=self.task, script=self.flow_script)

    def alarms(self) -> list[dict]:
        """还没确认的报警：日志里的 alarm 减去之后的 alarm_ack（同 kind，或 kind="*" 全部）。"""
        try:
            events, _ = L.read(self.log_path)
        except OSError:
            return []
        open_: list[dict] = []
        for e in events:
            if e.get("event") == "alarm" and e.get("needs_ack", True):
                open_.append(dict(kind=e.get("kind"), summary=e.get("summary"), block=e.get("block"),
                                  t=e.get("t"), seq=e.get("seq")))
            elif e.get("event") == "alarm_ack":
                open_ = [a for a in open_ if e.get("kind") not in ("*", a["kind"])]
        return open_

    def flow_scene(self) -> dict:
        """最近一次流程会话的最新扫描：桌上的块（单步执行要选块）和桶。只读文件，不依赖 numpy。"""
        session = self.flow_session
        out = dict(session=Path(session).name if session else None, scan=None, blocks=[], bins=[])
        if not session:
            return out
        scans = sorted(Path(session).glob("scan_*/located.json"))
        if not scans:
            return out
        try:
            located = json.loads(scans[-1].read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return out
        objs = located.get("objects", [])
        bins = [o for o in objs if o.get("kind") == "bin"]

        def in_bin(o):
            import math
            for b in bins:
                a = math.radians(-b.get("yaw_deg", 0.0))
                dx, dy = o["centre_m"][0] - b["centre_m"][0], o["centre_m"][1] - b["centre_m"][1]
                x, y = dx * math.cos(a) - dy * math.sin(a), dx * math.sin(a) + dy * math.cos(a)
                if abs(x) <= b["size_m"][0] / 2 and abs(y) <= b["size_m"][1] / 2:
                    return b.get("name")
            return None
        out["scan"] = scans[-1].parent.name
        out["bins"] = [dict(name=b.get("name"), category=b.get("class"),
                            centre_cm=[round(v * 100, 1) for v in b["centre_m"]]) for b in bins]
        out["blocks"] = [dict(name=o.get("name"), category=o.get("class"), in_bin=in_bin(o),
                              centre_cm=[round(v * 100, 1) for v in o["centre_m"]])
                         for o in objs if o.get("kind") == "block"]
        return out

    def stop_flow(self) -> bool:
        with self.flow_lock:
            if self.flow and self.flow.poll() is None:
                self.flow.terminate()
                try:
                    self.flow.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self.flow.kill()
                    self.flow.wait(timeout=5)
                return True
        return False

    @property
    def flow_session(self) -> str | None:
        """最近一次流程的会话目录（编排器在 step 事件里带 session 名）。"""
        try:
            for line in reversed(self.log_path.read_text(encoding="utf-8").splitlines()[-400:]):
                e = json.loads(line)
                if e.get("event") == "step" and e.get("session"):
                    return str(self.flow_root / e["session"])
        except (OSError, ValueError):
            return None
        return None

    def status(self) -> dict:
        arm = self.arm()
        # 姿态只认一个源：接了控制台就以控制台回读（arm.joints_deg）为准，顶层 joints_deg
        # 只在无控制台时兜底（日志事件里的 pose）。两个源混用会让页面在两组值之间闪烁。
        attached = self.console is not None
        return dict(mode=self.mode,
                    joints_deg=None if attached else self.joints,
                    t=None if attached else self.joints_t,
                    contract_version=L.VERSION, log=str(self.log_path), arm=arm,
                    replay=self.replay, flow=self.flow_state(), listen=self.listener_state(),
                    timer=self.timers.status() if self.timers else dict(enabled=False))


class Replayer(threading.Thread):
    """把一份样例日志按原速（可加速）写进活日志，供没有实机时开发页面。

    只改 `t`（改成写入时刻），其余字段原样写入；事件名、字段与顺序与实机一致。
    """

    def __init__(self, src: Path, state: State, speed: float = 1.0, loop: bool = False):
        super().__init__(daemon=True)
        self.src, self.state, self.speed, self.loop = src, state, max(speed, 0.1), loop

    def run(self) -> None:
        try:
            events = [json.loads(line) for line in self.src.read_text(encoding="utf-8").splitlines() if line.strip()]
        except OSError:
            return
        while True:
            base = events[0]["t"]
            started = time.time()
            for event in events:
                due = (event["t"] - base) / self.speed
                wait = due - (time.time() - started)
                if wait > 0:
                    time.sleep(wait)
                out = dict(event, t=time.time())
                self.state.append(out)
                self.state.note_pose(out.get("pose"), out["t"])
            if not self.loop:
                return
            time.sleep(3.0)


class Handler(BaseHTTPRequestHandler):
    server_version = "arm-web/1.0"

    def log_message(self, fmt, *args):          # 静音默认的逐请求日志
        pass

    # ---- helpers -------------------------------------------------------
    @property
    def state(self) -> State:
        return self.server.state                 # type: ignore[attr-defined]

    def _send(self, payload, status=200, mime="application/json; charset=utf-8", extra=()):
        body = payload if isinstance(payload, bytes) else json.dumps(payload, ensure_ascii=False).encode()
        self.send_response(status)
        self.send_header("Content-Type", mime)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        for key, value in extra:
            self.send_header(key, value)
        self.end_headers()
        self.wfile.write(body)

    def _timers(self, body: dict):
        """POST /api/timers：新增或取消一个定时任务（W4）。"""
        timers = self.state.timers
        if timers is None:
            return self._fail("没有启用定时任务：启动时加 --timers data/web/timers.json", status=404)
        if body.get("cancel"):
            return self._send(dict(ok=timers.remove(str(body["cancel"])), cancelled=body["cancel"]))
        at = body.get("at")
        if isinstance(at, str):                                    # 网页给的时间字符串
            try:
                at = datetime.datetime.fromisoformat(at).timestamp()
            except ValueError:
                return self._fail("at 应为 ISO 时间或时间戳")
        if not isinstance(at, (int, float)):
            return self._fail("缺少 at：到点的时刻（ISO 时间或时间戳）")
        if at <= time.time():
            return self._fail("at 必须是将来的时刻")
        repeat = body.get("repeat", "once")
        if repeat not in ("once", "daily"):
            return self._fail("repeat 应为 once 或 daily")
        timer = timers.add(at, repeat)
        self.state.append(dict(t=time.time(), event="interaction", source="web",
                               command="add_timer", detail=repeat))
        return self._send(dict(ok=True, timer=timer, **timers.status()))

    def _interact(self, path: str, body: dict | None = None):
        """语音/手势：判定在本服务的 intents（见文件头）。GET 给状态与最近识别，POST 一句话 / 一帧。"""
        st = self.state
        if body is None:
            return self._send(dict(st.interaction.describe(), listen=st.listener_state()))
        try:
            return self._send(st.interact(path, body))
        except KeyError:
            return self._fail("没有这个接口", status=404)
        except ValueError as exc:                # 输入不合法（label 不在四类、confidence 不在 0–1）
            return self._fail(str(exc), status=409)

    def _fail(self, message, status=400, **extra):
        self._send(dict(ok=False, error=message,
                        **{k: v for k, v in extra.items() if v is not None}), status=status)

    @staticmethod
    def _is_static(path: str) -> bool:
        return path in ("/", "/index.html", "/style.css") or path.startswith("/js/")

    def _authorized(self) -> bool:
        token = self.state.token
        if not token:
            return True
        given = self.headers.get("X-Token") or parse_qs(urlsplit(self.path).query).get("token", [None])[0]
        return given == token

    def _write_allowed(self) -> bool:
        """写操作的边界：同源 + JSON 请求体。

        不开 ACAO，跨源 fetch 本来就发不出来；但表单/fetch 的"简单请求"可以不带 CORS 预检
        直接打到端口上，所以 Origin 对不上 Host 的一律拒绝（curl 这类不带 Origin 的客户端
        不受影响——它们拿不到浏览器里的页面上下文，风险模型不同）。
        """
        origin = self.headers.get("Origin")
        if origin and urlsplit(origin).netloc != self.headers.get("Host", ""):
            self._fail("跨源写操作被拒绝", status=403)
            return False
        ctype = (self.headers.get("Content-Type") or "").split(";")[0].strip().lower()
        if ctype != "application/json":
            self._fail("Content-Type 应为 application/json", status=415)
            return False
        return True

    def _sse_headers(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream; charset=utf-8")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "keep-alive")
        self.send_header("X-Accel-Buffering", "no")
        self.end_headers()

    def _events_after(self, after: int):
        events, total = L.read(self.state.log_path, after=after)
        for event in events:
            self.state.note_pose(event.get("pose"), event.get("t"))
        return events, total

    # ---- routes --------------------------------------------------------
    def do_OPTIONS(self):
        # 不放开跨源：页面与 API 同端口同源，浏览器不需要预检；这里只回一个空 204
        self.send_response(204)
        self.end_headers()

    def do_GET(self):
        url = urlsplit(self.path)
        path, query = url.path, parse_qs(url.query)
        # 静态资源放行：开了 --token 时页面本身也要能加载，进来之后由页面带上 token 再调 API
        if not self._is_static(path) and not self._authorized():
            return self._fail("token 无效", status=401)

        if path in ("/", "/index.html"):
            return self._send((WEB / "index.html").read_bytes(), mime=MIME[".html"])
        if path.startswith("/js/") or path == "/style.css":
            target = (WEB / path.lstrip("/")).resolve()
            allowed = target.is_file() and target.parent in (WEB, WEB / "js")
            if not allowed:
                return self._fail("没有这个文件", status=404)
            return self._send(target.read_bytes(), mime=MIME.get(target.suffix, "application/octet-stream"))

        if path == "/api/health":
            console = self.state.console
            return self._send(dict(ok=True, contract_version=L.VERSION, mode=self.state.mode,
                                   console=console.target if console else None,
                                   console_ready=self.state.console_status is not None,
                                   console_sim=self.state.console_sim(),
                                   console_error=self.state.console_error))

        if path == "/api/log":
            after = int(query.get("after", ["0"])[0])
            limit = min(int(query.get("limit", ["500"])[0]), 2000)
            events, total = self._events_after(after)
            events = events[:limit]
            return self._send(dict(contract_version=L.VERSION, events=events, total=total,
                                   next=events[-1]["seq"] if events else after))

        if path == "/api/log/stream":
            after = int(query.get("after", ["0"])[0]) or int(self.headers.get("Last-Event-ID") or 0)
            self._sse_headers()
            last_send = time.time()
            try:
                while True:
                    events, _ = self._events_after(after)
                    for event in events:
                        self.wfile.write(f"id: {event['seq']}\n".encode())
                        self.wfile.write(f"data: {json.dumps(event, ensure_ascii=False)}\n\n".encode())
                        after = event["seq"]
                    if events:
                        self.wfile.flush()
                        last_send = time.time()
                    elif time.time() - last_send > 15:
                        # 长静默时发注释行保活：前置代理常以 30~60 s 无数据为由掐断 SSE
                        self.wfile.write(b": ka\n\n")
                        self.wfile.flush()
                        last_send = time.time()
                    time.sleep(0.15)
            except (BrokenPipeError, ConnectionResetError, OSError):
                return

        if path == "/api/flow":
            scene = self.state.flow_scene()
            return self._send(dict(process=self.state.flow_state(), alarms=self.state.alarms(),
                                   steps=list(FLOW_STEPS), script=FLOW_SCRIPT, **scene))

        if path == "/api/scripts":
            st = self.state
            return self._send(dict(scripts=st.scripts.list(), running=st.flow_script if st.flow_state()["running"] else None,
                                   start_runs="sort" if st.task == "flow" else None, task=st.task))
        if path == "/api/scripts/capabilities":
            return self._send(capability_catalog())
        if path.startswith("/api/scripts/"):
            got = self.state.scripts.get(unquote(path[len("/api/scripts/"):]))
            if got is None:
                return self._fail("没有这个脚本", status=404)
            got.pop("path", None)
            return self._send(got)

        if path == "/api/log/records":
            events, _ = L.read(self.state.log_path)
            return self._send(dict(contract_version=L.VERSION, records=L.records(events)))

        if path == "/api/log/summary":
            events, _ = L.read(self.state.log_path)
            return self._send(L.summary(events))

        if path in ("/api/scene", "/api/scene/image"):
            target = self.state.scene_json if path == "/api/scene" else self.state.scene_image
            if not target or not Path(target).is_file():
                return self._fail("还没有扫描结果", status=404)
            data = Path(target).read_bytes()
            return self._send(data, mime=MIME[".json"] if path == "/api/scene" else "image/png")

        # 桌上物块的基座坐标：给三维视图画方块用。数据来自仿真的世界真值，不是识别结果；
        # 实机上这一层要由视觉侧给出（scene.json）。接不上时返回空列表，页面照常画臂。
        if path == "/api/blocks":
            world = None
            if self.state.console is not None:
                try:
                    world = self.state.console.world()
                except Exception:
                    world = None          # 仿真没起或实机没有这个端点：降级为空，不让接口挂掉
            rows = []
            for b in (world or {}).get("blocks") or []:
                pos = b.get("base_cm")
                if not pos or len(pos) != 3:
                    continue
                rows.append(dict(id=b.get("id"), category=b.get("category"), colour=b.get("colour"),
                                 held=bool(b.get("held")),
                                 centre_cm=[round(float(v), 2) for v in pos]))
            return self._send(dict(count=len(rows), block_side_cm=(world or {}).get("block_side_cm"),
                                   blocks=rows, keep_out=load_keep_out_cm()))

        # E6 未知物体存档。判定与图片由 H1-6 的视觉侧写进 --archive 目录并记 unknown_object
        # 事件；这里只负责把图片安全地取出来给页面看，自己不做任何识别。
        if path == "/api/archive":
            events, _ = L.read(self.state.log_path)
            found = [e for e in events if e.get("event") == "unknown_object"]
            return self._send(dict(archive=str(self.state.archive) if self.state.archive else None,
                                   count=len(found), events=found[::-1]))

        if path.startswith("/api/archive/"):
            if self.state.archive is None:
                return self._fail("没有启用图片存档：启动时加 --archive <目录>", status=404)
            # 事件里的图片名可以带中文与空格，URL 里是百分号编码，先解码再做边界校验
            target = safe_under(self.state.archive, unquote(path[len("/api/archive/"):]))
            if target is None or not target.is_file():
                return self._fail("存档里没有这张图", status=404)
            return self._send(target.read_bytes(),
                              mime=MIME.get(target.suffix.lower(), "image/jpeg"))

        # 语音/手势：武装状态、词表、阈值与最近识别记录，页面直接显示控制台给的那一份
        if path == "/api/interact":
            return self._interact(path)

        if path == "/api/status":
            return self._send(self.state.status())

        # W4 定时任务：列表、新增、取消。不传 --timers 时整个功能不启用（默认关闭）
        if path == "/api/timers":
            if self.state.timers is None:
                return self._fail("没有启用定时任务：启动时加 --timers data/web/timers.json", status=404)
            return self._send(dict(timers=self.state.timers.list, **self.state.timers.status()))

        # 腕部相机：直接转控制台的画面。仿真时是合成渲染，实机时是真相机，路径同名同义。
        if path in ("/api/frame", "/api/frame/stream"):
            console = self.state.console
            if console is None:
                return self._fail("未接控制台，没有画面", status=404)
            upstream = "/api/frame" if path == "/api/frame" else "/stream.mjpg"
            try:
                return self._send(console.request(upstream), mime="image/jpeg")
            except ValueError as exc:
                return self._fail(str(exc), status=502)

        if path == "/api/status/stream":
            self._sse_headers()
            try:
                while True:
                    status = self.state.status()
                    status["t"] = time.time()
                    self.wfile.write(f"data: {json.dumps(status, ensure_ascii=False)}\n\n".encode())
                    self.wfile.flush()
                    time.sleep(0.4)          # 2.5 Hz，满足“≥ 2 Hz”
            except (BrokenPipeError, ConnectionResetError, OSError):
                return

        return self._fail("没有这个接口", status=404)

    def do_POST(self):
        url = urlsplit(self.path)
        if not self._authorized():
            return self._fail("token 无效", status=401)
        supported = url.path in ("/api/cmd", "/api/timers") or url.path.startswith(("/api/interact/", "/api/scripts/"))
        if supported and not self._write_allowed():
            return
        try:
            raw = self.rfile.read(int(self.headers.get("Content-Length") or 0))
        except (ValueError, OSError):
            return self._fail("请求体不是 JSON")
        # Drain the request before closing an unknown route; unread bytes can turn the 404 into a TCP reset
        # on Windows (the client observes WinError 10053 rather than the HTTP response).
        if not supported:
            return self._fail("没有这个接口", status=404)
        try:
            body = json.loads(raw or b"{}")
        except ValueError:
            return self._fail("请求体不是 JSON")

        if url.path == "/api/timers":
            return self._timers(body)
        if url.path.startswith("/api/interact/"):
            return self._interact(url.path, body)
        if url.path.startswith("/api/scripts/"):
            return self._scripts(url.path, body)
        return self._cmd(body)

    def _scripts(self, path: str, body: dict):
        """流程脚本的校验 / 保存 / 删除（文档 35）。保存、删除写一条 script_changed，别的端从事件流看到就重拉；
        带 rev 时做乐观并发：文件在这之间被另一端改过就 409，并把现在的内容一并给回去。"""
        store = self.state.scripts
        source = body.get("source", "web")
        if source not in SOURCES:
            return self._fail(f"source 应为 {'/'.join(SOURCES)}")
        if path == "/api/scripts/check":
            text = body.get("text")
            if not isinstance(text, str):
                return self._fail("text 应为脚本文本")
            return self._send(FS.check_text(text))
        if path not in ("/api/scripts/save", "/api/scripts/delete"):
            return self._fail("没有这个接口", status=404)
        name = body.get("name")
        try:
            if path == "/api/scripts/save":
                got = store.save(name, body.get("text"), base_rev=body.get("rev"))
                got.pop("path", None)
                action, out = "saved", got
            else:
                if not store.delete(name, base_rev=body.get("rev")):
                    return self._fail("没有这个脚本", status=404)
                action, out = "deleted", dict(ok=True, name=name)
        except FS.ConflictError as exc:
            return self._fail("脚本已被另一端改过：先看看对方的改动再存", status=409, current=exc.current)
        except ValueError as exc:
            return self._fail(str(exc))
        self.state.append(dict(t=time.time(), event="script_changed", name=name, action=action, by=source,
                               rev=out.get("rev")))
        return self._send(out)

    def _cmd(self, body: dict):
        cmd = body.get("cmd")
        if cmd not in CMDS:
            return self._fail(f"cmd 应为 {'/'.join(CMDS)}")
        source = body.get("source", "web")
        if source not in SOURCES:
            return self._fail(f"source 应为 {'/'.join(SOURCES)}")

        if cmd == "set_mode":
            mode = body.get("mode")
            if mode not in MODES:
                return self._fail(f"mode 应为 {'/'.join(MODES)}")
            previous = self.state.mode
            self.state.mode = mode
            # 拟定事件，字段按日志契约 §3
            self.state.append(dict(t=time.time(), event="mode_changed",
                                   mode=mode, source=source, previous=previous))
            return self._send(dict(ok=True, mode=mode, source=source))

        if cmd == "set_joints":
            joints = body.get("joints_deg")
            if not isinstance(joints, list) or len(joints) != 6:
                return self._fail("joints_deg 应为 6 个角度")
        if cmd == "move_to":
            target = body.get("target_cm")
            if not isinstance(target, list) or len(target) != 3:
                return self._fail("target_cm 应为 [x, y, z] 厘米")

        console = self.state.console

        if cmd == "stop":                              # 写 stopped、放开操作权、停流程、停控制台
            self.state.stop_all(source)
            return self._send(dict(ok=True, stopped=True, via_console=console is not None))

        if cmd == "listen":                            # 识别端开关：{on: true/false, voice: false}
            if body.get("on") is False:
                return self._send(dict(ok=True, stopped=self.state.stop_listener()))
            return self._send(self.state.start_listener(voice=body.get("voice") is True))

        if cmd in ("led", "beep"):                     # 声光：控制台独占 I2C，经它的 /api/led、/api/beep
            if console is None:
                return self._fail("未接控制台，没有灯和蜂鸣器", status=501)
            try:
                reply = console.request("/api/led" if cmd == "led" else "/api/beep",
                                        dict(owner=console.owner, colour=body.get("colour")) if cmd == "led"
                                        else dict(owner=console.owner, ms=body.get("ms", 120)))
            except ValueError as exc:
                return self._fail(str(exc), status=409)
            return self._send(dict(ok=True, **{k: v for k, v in reply.items() if k in ("led", "beep")}))

        if cmd == "start":
            result = self.state.start_task(source=source)
            return self._send(result) if result.get("ok") else self._fail(
                result.get("error"), status=501)

        if cmd == "run_script":
            result = self.state.run_script(body.get("name"), source=source)
            return self._send(result) if result.get("ok") else self._fail(result.get("error"), status=409)

        if cmd == "flow_step":
            result = self.state.run_step(body.get("step"), body.get("block"), body.get("session"), source=source)
            return self._send(result) if result.get("ok") else self._fail(result.get("error"), status=409)

        if cmd == "alarm_ack":
            kind = body.get("kind")
            if not isinstance(kind, str) or not kind:
                return self._fail("kind 应为报警类型")
            self.state.append(dict(t=time.time(), event="alarm_ack", kind=kind, by=source))
            return self._send(dict(ok=True))

        if cmd == "release":                          # 页面切后台/关闭：只停止续租，不停流程、不停控制台
            self.state.arm_off()
            return self._send(dict(ok=True))

        if cmd == "heartbeat":                        # 页面可见期间每 ~0.8 s 一次（文档 32 §1.2）
            self.state.client_beat()
            return self._send(dict(ok=True))

        if cmd in ("enable", "takeover", "speed", "drive", "goto") and console is None:
            return self._fail(f"{cmd} 需要控制台（--console http://127.0.0.1:8770）", status=501)

        if cmd in ("enable", "takeover"):              # enable：旧页面的按钮，等同取得操作权
            self.state.client_beat()
            try:
                console.takeover() if cmd == "takeover" else console.enable()
            except ValueError as exc:
                return self._fail(str(exc), status=409)
            self.state.console_status = console.status()
            self.state.arm_on()
            return self._send(dict(ok=True, enabled=True))

        if cmd == "speed":
            value = body.get("speed")
            try:
                status = console.speed(value)
            except ValueError as exc:
                return self._fail(str(exc))
            self.state.console_status = status
            try:
                self.state.append(dict(t=time.time(), event="speed", speed=float(value), source=source))
            except ValueError:
                pass
            return self._send(dict(ok=True, speed=status.get("speed")))

        if cmd == "drive":                             # 摇杆：10 Hz 刷新，不占 motion_lock（控制台自己串行）
            self.state.client_beat()
            try:
                status = console.drive(body.get("mode", "cartesian"), body.get("v"), body.get("wrist", 0))
            except ValueError as exc:
                return self._fail(str(exc), status=409)
            self.state.arm_on()
            return self._send(dict(ok=True, drive=status.get("drive")))

        if cmd == "gripper":
            action = body.get("action")
            if action not in GRIPPER_ACTIONS:
                return self._fail(f"action 应为 {'/'.join(GRIPPER_ACTIONS)}")
            if console is None:
                return self._fail("未接控制台，无法控制夹爪", status=501)
            if not self.state.motion_lock.acquire(blocking=False):
                return self._fail("上一条运动指令还在执行，等它结束或用 stop 中止", status=409)
            self.state.client_beat()
            self.state.arm_on()
            try:
                try:
                    result = console.gripper(action)
                except ValueError as exc:
                    return self._fail(str(exc), steps_done=getattr(exc, "steps_done", None),
                                      steps_total=getattr(exc, "steps_total", None))
                self.state.console_status = console.status()
            finally:
                self.state.motion_lock.release()
            grip = result.get("grip") or {}
            return self._send(dict(ok=True, action=action, holding=grip.get("holding"),
                                   result=grip.get("result"), readback=grip.get("readback")))

        # 以下运动都要控制台（H1-3）。预设动作是同名流程脚本（dance / stack / carry.yaml），没有脚本就 501，不假造。
        if cmd == "run_preset":                        # 预设动作就是同名的流程脚本（dance.yaml）
            preset = body.get("preset")
            if preset not in PRESETS:
                return self._fail(f"preset 应为 {'/'.join(PRESETS)}")
            if self.state.scripts.get(preset) is None:
                return self._fail(f"预设动作“{preset}”还没有写成流程脚本（config/flows/{preset}.yaml）", status=501)
            result = self.state.run_script(preset, source=source)
            return self._send(result) if result.get("ok") else self._fail(result.get("error"), status=409)

        if console is None:
            return self._fail(f"{cmd} 需要控制台执行侧，尚未接入（--console 指定仿真或实机）", status=501)

        # 运动串行化：同一时刻只执行一条；上一条没走完时新指令按 409 拒绝，不排队、不混姿态
        if not self.state.motion_lock.acquire(blocking=False):
            return self._fail("上一条运动指令还在执行，等它结束或用 stop 中止", status=409)
        self.state.client_beat()
        self.state.arm_on()                            # 多段运动期间续租，页面停发心跳即停
        try:
            status = console.status()                  # 最新的回读
            if status.get("busy"):
                return self._fail("控制台正在运动中，等这一步结束再下发", status=409)
            try:
                if cmd == "set_joints":
                    steps = console.set_joints(joints, status.get("pose"))
                elif cmd == "goto":
                    named, pose = body.get("named"), body.get("pose")
                    plan = console.goto(pose=pose, named=named, dry=True)
                    console.goto(pose=pose, named=named)
                    steps = len(plan.get("segments") or [])
                elif cmd == "move_to":
                    steps = console.move_to(target, status)
                else:
                    return self._fail(f"{cmd} 需要执行侧，本服务不代管运动", status=501)
                self.state.console_status = console.status()
            except ValueError as exc:
                return self._fail(str(exc), steps_done=getattr(exc, "steps_done", None),
                                  steps_total=getattr(exc, "steps_total", None))
        finally:
            self.state.motion_lock.release()
        return self._send(dict(ok=True, cmd=cmd, steps=steps, executed=True, via_console=True))


def split_command(text: str, windows: bool | None = None) -> list[str]:
    """Split a command line given as one option (--flow-python, --pipeline-args). POSIX rules eat the
    backslashes of a Windows path (C:\\Users\\... -> C:Users...), so on Windows split without escapes and only
    drop the quotes around a word ("opencv-python<5")."""
    if not (os.name == "nt" if windows is None else windows):
        return shlex.split(text)
    out = []
    for word in shlex.split(text, posix=False):
        if len(word) >= 2 and word[0] == word[-1] and word[0] in "\"'":
            word = word[1:-1]
        out.append(word)
    return out


class Server(ThreadingHTTPServer):
    """浏览器关页面时 SSE/长连接会被掐断，socketserver 默认会为此打整段 traceback。
    那不是错误，是正常的断开，静默掉，免得日志里全是噪音。"""

    def handle_error(self, request, client_address) -> None:
        import sys as _sys
        exc = _sys.exc_info()[1]
        if isinstance(exc, (ConnectionAbortedError, ConnectionResetError, BrokenPipeError)):
            return
        super().handle_error(request, client_address)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="对外 HTTP 服务（F1-1）")
    parser.add_argument("--log", default="data/web/log.jsonl", help="活日志文件（只追加）")
    parser.add_argument("--replay", help="回放一份样例日志到活日志，供无实机时开发")
    parser.add_argument("--speed", type=float, default=4.0, help="回放倍速，默认 4")
    parser.add_argument("--loop", action="store_true", help="回放结束后循环")
    parser.add_argument("--scene-json", help="最近一次扫描的 scene.json")
    parser.add_argument("--scene-image", help="最近一次扫描的重建图")
    parser.add_argument("--console", help="控制台地址：仿真 http://127.0.0.1:8770，或实机控制台；不接则运动类指令按 501 返回")
    parser.add_argument("--timers", help="启用 W4 定时启动并把任务存在这个文件里；不传则整个功能关闭（默认关闭）")
    parser.add_argument("--archive", default="data/archive",
                        help="E6 未知物体图片的存档目录（H1-6 往这里写，页面从这里读）")
    parser.add_argument("--flow-python", default=None,
                        help="跑编排器的命令（它要 numpy/opencv/onnxruntime），如 "
                             "\"uv run --with 'opencv-python<5' --with numpy --with onnxruntime python\"；默认本解释器")
    parser.add_argument("--task", choices=("pipeline", "flow"), default="pipeline",
                        help="「开始」跑什么：pipeline 已在实机验证的旧流水线（默认）；flow 分步编排器（试验，文档 34 §4.2）")
    parser.add_argument("--pipeline-args", default="",
                        help="--task pipeline 时传给 run.py 的参数，如 \"--bins bins_acrylic_v2_pair --bin-side left\"")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--lan", action="store_true", help="监听所有网卡，手机可访问（强制开启 token 鉴权）")
    parser.add_argument("--token", help="开启简单鉴权，请求带 X-Token 或 ?token=")
    args = parser.parse_args(argv)

    token = args.token
    if args.lan and not token:
        # 开到局域网就必须有口令：不设防的写接口等于把机械臂暴露给同网段任何人
        token = secrets.token_urlsafe(8)
        print(f"--lan 已强制开启鉴权，本次访问口令：{token}（页面用 ?token={token} 打开）")

    console = Console(args.console) if args.console else None
    # 定时到点是让 Timer 回调 state.start_task：管自己和管页面走的是同一条启动路径
    timers = Timers(ROOT / args.timers, lambda _t: state.start_task(source="timer")) if args.timers else None
    state = State(ROOT / args.log, token=token, console=console, timers=timers,
                  archive=ROOT / args.archive if args.archive else None,
                  scene_json=ROOT / args.scene_json if args.scene_json else None,
                  scene_image=ROOT / args.scene_image if args.scene_image else None,
)
    state.base_url = f"http://127.0.0.1:{args.port}"
    if args.flow_python:
        state.flow_python = split_command(args.flow_python)
    state.task = args.task
    state.pipeline_args = split_command(args.pipeline_args)
    if console is not None:
        threading.Thread(target=state.poll_console, daemon=True).start()
    if timers is not None:
        threading.Thread(target=timers.run, daemon=True).start()

    if args.replay:
        sample = ROOT / args.replay
        if not sample.is_file():
            print(f"找不到样例日志：{sample}", file=sys.stderr)
            return 2
        state.truncate_log()
        state.replay = True
        Replayer(sample, state, speed=args.speed, loop=args.loop).start()
        print(f"回放 {sample}（{args.speed}×）")

    server = Server(("0.0.0.0" if args.lan else "127.0.0.1", args.port), Handler)
    server.state = state                                        # type: ignore[attr-defined]
    host = "0.0.0.0" if args.lan else "127.0.0.1"
    print(f"对外服务 http://{host}:{args.port}/   日志 {state.log_path}")
    if console is not None:
        print(f"控制台 {console.target}（运动指令转发过去，由它把关限位与禁区）")
    else:
        print("未接控制台：运动类指令返回 501，只有日志与模式可用")
    if timers is not None:
        print(f"定时启动 已启用 任务表 {timers.path}（默认无任务，到点前 "
              f"{Timers.LEAD:.0f} s 提示且可取消）")
    else:
        print("定时启动 未启用（默认关闭；加 --timers <文件> 打开）")
    print("Ctrl+C 退出")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n已停止")
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
