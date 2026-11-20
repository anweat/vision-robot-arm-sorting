"""What a flow script can ask the arm to do: the one table every end reads (docs/current/35).

Standard library only: the gateway (src/web/serve.py) serves this table and checks scripts with it without
numpy; the orchestrator (src/flow/orchestrator.py) runs the same names. Each entry says

  * group / label / help   how a page shows it;
  * params                 the `with:` keys: type (str / num / int / bool / enum), default, range, unit;
  * results / success      the words `on_result:` can branch on, and the ones that count as "went fine";
  * moves / block          moves the arm; needs a block (only inside `for_each: blocks`);
  * status                 ready       runs on the console today (real arm and simulator);
                           partial     the console side is there, the input that feeds it is not wired up
                                       (a script using it is accepted, a page says what is missing);
                           unavailable the board has no interface for it yet: a script using it is valid but
                                       refused before anything moves, with `needs` saying what is missing.

Adding a capability: an entry here, an implementation in src/flow/steps.py or src/flow/actions.py under the
same name (tests/test_script.py checks the two match), and the doc table.
"""
from __future__ import annotations

STATUSES = ("ready", "partial", "unavailable")
GROUPS = {
    "vision": "视觉",
    "grasp": "抓放",
    "motion": "运动",
    "interact": "交互",
    "signal": "声光",
    "flow": "流程",
}


def P(type_: str, label: str, default=None, *, enum=None, min=None, max=None, unit=None, help=None,
      required=False, advanced=False) -> dict:
    out = dict(type=type_, label=label, default=default)
    for k, v in dict(enum=enum, min=min, max=max, unit=unit, help=help).items():
        if v is not None:
            out[k] = v
    if required:
        out["required"] = True
    if advanced:
        out["advanced"] = True
    return out


BINS_HELP = "桶的设计名（app/config/bins_*.json 去掉 .json），如 bins_acrylic_v2_pair；不填就不定位桶"

CAPABILITIES: dict[str, dict] = {
    # ---- vision ----------------------------------------------------------------------------------------
    "scan": dict(
        group="vision", label="扫描识别", moves=True, block=False, status="ready",
        help="到扫描位姿逐视角拍照 → 识别 → 定位物块与桶 → 碰撞地图；refine 近看补拍看不全的块",
        params=dict(bins=P("str", "桶设计", None, help=BINS_HELP),
                    refine=P("enum", "近看补拍", False, enum=[False, True, "block"],
                             help="true：补拍看不全/得分低的块；block：只近看当前这一块"),
                    min_views=P("int", "幻影门槛", 0, min=0, max=8,
                                help="少于这么多视角看到、又没有贴纸的检测按扫描噪声去掉；0 不过滤。只用在开头那次带近看补拍的扫描："
                                     "实机 2026-11-19 用 3 去掉 3 个幻影、没误伤真块；仿真 2026-11-20 不近看的扫描里真块只有 1–2 个视角，"
                                     "用 3 会把真块当噪声（近端带 y < 11 cm 带补拍也会）"),
                    source=P("str", "离线场景", None, advanced=True,
                             help="用这份 located.json 代替真扫描（回放、测试用）")),
        results=["done", "empty", "failed"], success=["done"]),
    "relocate": dict(
        group="vision", label="找回当前块", moves=False, block=True, status="ready",
        help="在最新一次扫描里按位置找回当前处理的块（重扫后用）",
        params={}, results=["found", "gone"], success=["found"]),
    "verify": dict(
        group="vision", label="核对", moves=False, block=False, status="ready",
        help="把最新扫描和本轮起始扫描比：放走的块离开了桌面、其余没被碰动",
        params=dict(scope=P("enum", "范围", "round", enum=["round", "block"]),
                    tolerance_m=P("num", "容差", 0.015, min=0.002, max=0.1, unit="m")),
        results=["ok", "mismatch"], success=["ok"]),
    # ---- grasp -----------------------------------------------------------------------------------------
    "plan": dict(
        group="grasp", label="解算验证", moves=False, block=True, status="ready",
        help="抓取 + 投放 + 返回整条路径先检查，不动臂",
        params=dict(to=P("enum", "放到", "by_class", enum=["by_class", "table"],
                         help="by_class：按类别投对应的桶；table：放回桌面")),
        results=["ok", "rejected", "no_destination", "gone"], success=["ok"]),
    "grasp": dict(
        group="grasp", label="抓取", moves=True, block=True, status="ready",
        help="下探、合爪、抬起（按解算好的路径）", params={},
        results=["done", "failed"], success=["done"]),
    "check": dict(
        group="grasp", label="抓后检查", moves=False, block=True, status="ready",
        help="按夹爪回读判定：holding 夹住 / empty 空爪 / slipped 抬起时滑脱 / jammed 卡住",
        params=dict(slip_deg=P("num", "滑脱阈值", 3, min=1, max=20, unit="°", advanced=True)),
        results=["holding", "empty", "slipped", "jammed"], success=["holding"]),
    "back_off": dict(
        group="grasp", label="张爪退回", moves=True, block=True, status="ready",
        help="张爪、抬回悬停位、回观察位（重试前、卡住后）", params={},
        results=["done", "failed"], success=["done"]),
    "place": dict(
        group="grasp", label="放置", moves=True, block=True, status="ready",
        help="搬到目标位、松手、回观察位", params={},
        results=["done", "failed"], success=["done"]),
    # ---- motion ----------------------------------------------------------------------------------------
    "home": dict(
        group="motion", label="回观察位", moves=True, block=False, status="ready",
        help="回流程的观察位（和扫描、退回用的同一个）", params={},
        results=["done", "failed"], success=["done"]),
    "pose": dict(
        group="motion", label="到命名位", moves=True, block=False, status="ready",
        help="到控制台的命名位姿（app/config/arm_limits.json named_poses），整条路径先检查再动",
        params=dict(named=P("enum", "位姿", "observe_v1", enum=["observe_v1", "observe_v0", "upright", "gesture_view"],
                            required=True, help="gesture_view：平视位，腕部相机正对前方看操作员的手（2026-11-19 实机）")),
        results=["done", "failed"], success=["done"]),
    "joints": dict(
        group="motion", label="到关节角", moves=True, block=False, status="ready",
        help="六轴绝对角度（null 表示该轴不动），控制台整条路径先检查再按速度分段走；舞蹈等预设动作就是一串这个",
        params=dict(pose=P("pose", "关节角", None, required=True, help="[J1, J2, J3, J4, J5, J6]，度；null 不动")),
        results=["done", "failed"], success=["done"]),
    "gripper": dict(
        group="motion", label="夹爪", moves=True, block=False, status="ready",
        help="张开 / 合拢 / 张到最大；合拢后按回读给出 holding 或 empty",
        params=dict(action=P("enum", "动作", "open", enum=["open", "grip", "wide"], required=True)),
        results=["done", "holding", "empty", "failed"], success=["done", "holding", "empty"]),
    "speed": dict(
        group="motion", label="速度", moves=False, block=False, status="ready",
        help="改控制台速度，从下一段运动起生效",
        params=dict(value=P("num", "速度", 20, min=3, max=40, unit="°/s", required=True)),
        results=["done", "failed"], success=["done"]),
    # ---- interaction -----------------------------------------------------------------------------------
    "wait_gesture": dict(
        group="interact", label="等手势", moves=False, block=False, status="ready",
        help="等识别端报来指定手势（连续帧、阈值、冷却由网关的 intents 判）；识别端是 src/interact/bridge.py "
             "（板上 hrnet.om，网页「手动操作」里可开关），也可在调试面板代发。没人比划就到点给 timeout",
        params=dict(label=P("enum", "手势", "ok", enum=["ok", "palm", "fist", "like"], required=True),
                    timeout_s=P("num", "超时", 60, min=1, max=3600, unit="s")),
        results=["matched", "timeout", "failed"], success=["matched"]),
    "wait_voice": dict(
        group="interact", label="等语音", moves=False, block=False, status="unavailable",
        needs="语音识别目前不可用：板上收音与识别进程未接，/api/interact/voice 只有调试输入",
        help="等识别到含指定词的一句话，超时给 timeout",
        params=dict(words=P("str", "指令词", "开始", required=True, help="多个词用 / 分隔"),
                    timeout_s=P("num", "超时", 60, min=1, max=3600, unit="s")),
        results=["matched", "timeout", "failed"], success=["matched"]),
    # ---- signal ----------------------------------------------------------------------------------------
    "led": dict(
        group="signal", label="灯光", moves=False, block=False, status="ready",
        help="扩展板 RGB 灯颜色，经控制台 /api/led（I2C 只能由控制台进程写）；2026-11-19 实机目视绿/黄/红/灭正确。"
             "约定：绿=放好，黄=夹空重试，红=失败，蓝=等操作员",
        params=dict(color=P("enum", "颜色", "green", enum=["off", "red", "green", "blue", "yellow", "white"],
                            required=True)),
        results=["done", "failed"], success=["done"]),
    "buzzer": dict(
        group="signal", label="蜂鸣", moves=False, block=False, status="ready",
        help="蜂鸣一声，经控制台 /api/beep；2026-11-19 调用成功，是否真的出声还没用耳朵确认。约定：短 120 ms=成功，长 400 ms=失败",
        params=dict(ms=P("int", "时长", 120, min=20, max=1000, unit="ms")),
        results=["done", "failed"], success=["done"]),
    "say": dict(
        group="signal", label="语音播报", moves=False, block=False, status="unavailable",
        needs="语音目前不可用：板上音频输出与播报未核对、未接",
        help="板上扬声器播报一句话",
        params=dict(text=P("str", "内容", "", required=True)),
        results=["done", "failed"], success=["done"]),
    # ---- flow helpers ----------------------------------------------------------------------------------
    "wait": dict(
        group="flow", label="等待", moves=False, block=False, status="ready",
        help="原地等一会儿（“停止”随时有效）",
        params=dict(seconds=P("num", "时长", 2, min=0, max=3600, unit="s", required=True)),
        results=["done"], success=["done"]),
    "notify": dict(
        group="flow", label="提示", moves=False, block=False, status="ready",
        help="给各端发一条提示（写一条 step 事件，页面弹出）",
        params=dict(text=P("str", "内容", "", required=True),
                    level=P("enum", "级别", "info", enum=["info", "warn"])),
        results=["done"], success=["done"]),
}

# Control actions (not hardware): what a script line may be besides `step:`. Keys each one allows.
CONTROL = {
    "step": dict(keys=["step", "with", "on_result", "note"], label="执行一项能力"),
    "for_each": dict(keys=["for_each", "order", "do", "note"], label="对每一块",
                     help="对扫描到的每一块依次执行 do；order：reach 由近及远 / scene 按扫描顺序"),
    "retry": dict(keys=["retry", "note"], label="重试",
                  help="retry: {times, until, do, else}：do 最多做 times 次，最后一步结果是 until 就停；都不行做 else"),
    "repeat": dict(keys=["repeat", "note"], label="重复",
                   help="repeat: {times, do}：do 依次做 times 次（“停止”随时有效）"),
    "alarm": dict(keys=["alarm", "then", "summary", "note"], label="报警",
                  help="发需要人确认的报警；then：skip_block 跳过这一块 / stop 整轮停 / continue 继续"),
    "record": dict(keys=["record", "reason", "note"], label="记一笔",
                   help="逐块记录：sorted 已分拣 / missed 夹空 / skipped 跳过（只在 for_each 里）"),
    "next_block": dict(keys=["next_block", "note"], label="下一块", help="结束这一块，处理下一块（只在 for_each 里）"),
    "stop": dict(keys=["stop", "note"], label="停止流程", help="整个流程到此为止，值是原因"),
}
TOP_KEYS = ["name", "title", "description", "version", "speed", "stop_after_alarms", "steps"]
RECORDS = ["sorted", "missed", "skipped"]
THEN = ["skip_block", "stop", "continue"]
ORDERS = ["reach", "scene"]


def catalog() -> dict:
    """The JSON a page needs to show, check and build scripts."""
    return dict(version=2, groups=GROUPS, statuses=list(STATUSES), capabilities=CAPABILITIES, control=CONTROL,
                top_keys=TOP_KEYS, records=RECORDS, then=THEN, orders=ORDERS)
