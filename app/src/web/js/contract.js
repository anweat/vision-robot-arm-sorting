// 日志契约 v1 的前端镜像。唯一来源是 src/tools/log_contract.py 的事件表，
// 文档见 docs/atlas-200i-dk-a2/18-日志契约.md。改事件或字段先改那边的事件表。
// 契约外的事件要跳过，契约外的字段要忽略。

export const CONTRACT_VERSION = 1;

export const CATEGORIES = ["recyclable", "kitchen", "hazardous", "other"];
export const MODES = ["manual", "preset", "auto"];
export const SOURCES = ["web", "mobile", "voice", "gesture", "timer", "button"];
export const RESULTS = ["sorted", "missed", "skipped", "failed", "cleared"];

export const CATEGORY_NAMES = {
  recyclable: "可回收物",
  kitchen: "厨余垃圾",
  hazardous: "有害垃圾",
  other: "其他垃圾",
};

export const MODE_NAMES = { manual: "手动", preset: "预设动作", auto: "视觉自主" };
export const RESULT_NAMES = {
  sorted: "已分拣", missed: "夹空", skipped: "跳过", failed: "失败", cleared: "挪开",
};
export const SOURCE_NAMES = {
  web: "网页", mobile: "手机", voice: "语音", gesture: "手势", timer: "定时", button: "按键",
};

// 目标位编号 -> 类别（config/sorting.json；实体容器到位后会变成容器号，不要写死）
export const BIN_CATEGORY = { 40: "recyclable", 41: "kitchen", 43: "hazardous", 44: "other" };

// 六轴软限位（config/arm_limits.json，2026-09-24）
export const JOINT_LIMITS = [
  { name: "J1 底座", min: 20, max: 160 },
  { name: "J2 大臂", min: 5, max: 175 },
  { name: "J3 小臂", min: 5, max: 175 },
  { name: "J4 腕俯仰", min: 5, max: 175 },
  { name: "J5 腕旋转", min: 95, max: 180 },
  { name: "J6 夹爪", min: 5, max: 175 },
];

// 夹爪（config/arm_limits.json）：角度越大越合拢
export const GRIPPER = { open: 30, openWide: 11, grip: 133, holdingBelow: 131, holdingAbove: 100 };

export const NAMED_POSES = {
  upright: [90, 90, 90, 90, null, null],
  observe: [90, 125, 5, 5, null, null],
};

// 事件表：与 log_contract.EVENTS 对应。record 非空的事件各产生一条逐块记录。
export const EVENTS = {
  pipeline_start: { status: "stable", meaning: "一轮分拣开始", record: null },
  pipeline_sorted: { status: "stable", meaning: "一块已放到它的目标位", record: "sorted" },
  pipeline_missed: { status: "stable", meaning: "夹空或搬运途中滑脱", record: "missed" },
  pipeline_skip: { status: "stable", meaning: "没有去抓", record: "skipped" },
  pipeline_approach_failed: { status: "stable", meaning: "接近途中遇阻，退回观察位", record: "failed" },
  pipeline_put_failed: { status: "stable", meaning: "放不到目标位，已就地放下", record: "failed" },
  pipeline_cleared: { status: "stable", meaning: "挪开挡路的块", record: "cleared" },
  pipeline_stuck: { status: "stable", meaning: "剩下的块互相挡着", record: null },
  pipeline_end: { status: "stable", meaning: "一轮结束及计数", record: null },
  scan_view: { status: "stable", meaning: "扫描到第几个视角", record: null },
  gripped: { status: "stable", meaning: "合爪后的读回判定", record: null },
  put: { status: "stable", meaning: "已在目标位松手", record: null },
  gripper_not_closed: { status: "stable", meaning: "夹爪没有响应合爪指令", record: null },
  gripper_awake: { status: "stable", meaning: "唤醒后夹爪恢复响应", record: null },
  gripper_still_asleep: { status: "stable", meaning: "唤醒后夹爪仍不响应", record: null },
  gripper_wake_refused: { status: "stable", meaning: "唤醒动作被拒绝", record: null },
  stopped: { status: "stable", meaning: "任务中止", record: null },
  table_z_disagrees: { status: "stable", meaning: "桌面高度与标定值相差过大", record: null },
  bins_located: { status: "stable", meaning: "实体垃圾桶按贴纸定位的结果", record: null },
  mode_changed: { status: "stable", meaning: "作业模式切换", record: null },
  unknown_object: { status: "planned", meaning: "发现不属于四类的物体", record: null },
  bin_full: { status: "stable", meaning: "容器已满，停止投放", record: null },
  stall: { status: "stable", meaning: "舵机堵转/遇阻，已停止后续指令", record: null },
  interaction: { status: "stable", meaning: "手势、表情或语音触发且已受理执行的指令", record: null },
  step: { status: "stable", meaning: "分步流程的一步开始或结束", record: null },
  alarm: { status: "stable", meaning: "需要人确认的报警", record: null },
  alarm_ack: { status: "stable", meaning: "报警已确认", record: null },
  speed: { status: "stable", meaning: "控制台速度改动", record: null },
  script_changed: { status: "stable", meaning: "流程脚本在某一端被保存或删除", record: null },
  pipeline_retry: { status: "stable", meaning: "空爪后退回重扫、找回再抓", record: null },
  pipeline_retry_failed: { status: "stable", meaning: "空爪重试没能进行", record: null },
  pipeline_order: { status: "stable", meaning: "本轮抓取顺序（由近及远）", record: null },
  gesture_gate: { status: "stable", meaning: "扫描后等操作员比 OK 手势", record: null },
};

// 分步流程（文档 32 §2）：步骤名、显示名、要不要选块、会不会动臂
export const FLOW_STEPS = [
  { key: "scan", label: "扫描识别", block: false, moves: true },
  { key: "plan", label: "解算验证", block: true, moves: false },
  { key: "grasp", label: "抓取", block: true, moves: true },
  { key: "check", label: "抓后检查", block: true, moves: false },
  { key: "retry", label: "空爪重试", block: true, moves: true },
  { key: "place", label: "放置", block: true, moves: true },
  { key: "verify", label: "核对", block: false, moves: false },
];
export const FLOW_EXTRA_STEPS = [
  { key: "back_off", label: "张爪退回", block: true, moves: true },
  { key: "home", label: "回观察位", block: false, moves: true },
];
export const ALARM_NAMES = {
  grip_failed: "重试用尽仍空爪", jammed: "夹爪卡在块上", grasp_failed: "抓取中途停止",
  place_failed: "放置中途停止", stall: "堵转", wait_gesture_timeout: "等手势超时",
};
export const alarmName = (kind) => ALARM_NAMES[kind] ?? kind;

// 告警提示（作业台「提示」栏与告警页）：日志里出问题的事件按严重程度归类，人点"知道了"才清。
// 和 alarm（流程脚本发的、要确认的报警，红条）分开：这些是旧流水线与执行侧直接写的故障事件。
// 确认走同一条 alarm_ack（kind="notices"），写进日志，所有端同步消失。
const where = (e) => (Array.isArray(e.cube) ? `（${e.cube[0]?.toFixed?.(1)}, ${e.cube[1]?.toFixed?.(1)}）cm` : "");
export const NOTICES = {
  pipeline_missed: { level: "warn", title: "抓空", text: (e) => `${categoryName(e.category)}${where(e)} ${e.reason ?? ""}` },
  pipeline_approach_failed: { level: "warn", title: "接近遇阻", text: (e) => `${where(e)} ${e.reason ?? ""}` },
  pipeline_put_failed: { level: "bad", title: "放置失败", text: (e) => `→ ${e.bin ?? "?"} ${e.reason ?? ""}` },
  pipeline_stuck: { level: "warn", title: "物块互相挡住", text: (e) => `目标位 ${e.bin ?? "?"} 被挡` },
  gripper_not_closed: { level: "warn", title: "夹爪没响应合爪", text: (e) => `第 ${e.attempt ?? "?"} 次，回读 ${e.readback ?? "—"}°` },
  gripper_still_asleep: { level: "bad", title: "夹爪唤醒失败", text: (e) => `回读 ${e.readback ?? "—"}°` },
  gripper_wake_refused: { level: "bad", title: "夹爪唤醒被拒", text: (e) => e.reason ?? "" },
  table_z_disagrees: { level: "warn", title: "桌面高度与标定不符", text: (e) => `扫描 ${e.scan_cm} cm / 标定 ${e.calibrated_cm} cm，需重新标定` },
  bin_full: { level: "warn", title: "容器已满", text: (e) => `${e.bin} 已有 ${e.count} 块，不再投放` },
  unknown_object: { level: "warn", title: "发现未知物体", text: (e) => `${where(e)} 不抓取，图片已存档` },
  stall: { level: "bad", title: "舵机遇阻", text: (e) => `J${e.joint} 期望 ${e.expected}° 回读 ${e.readback}°${e.where ? `（${e.where}）` : ""}` },
  pipeline_retry: { level: "warn", title: "空爪重试", text: (e) => `${categoryName(e.category)}${where(e)} ${e.reason ?? ""}` },
  pipeline_retry_failed: { level: "bad", title: "重试没能进行", text: (e) => `${where(e)} ${e.reason ?? ""}` },
  stopped: { level: "info", title: "任务中止", text: (e) => e.reason ?? "" },
};

// 异常日志页关心的事件：出问题的都在这里，其余进实时状态页的事件流
export const FAULT_EVENTS = [
  "pipeline_missed", "pipeline_skip", "pipeline_approach_failed", "pipeline_put_failed",
  "pipeline_stuck", "gripper_not_closed", "gripper_still_asleep", "gripper_wake_refused",
  "stopped", "table_z_disagrees", "stall", "bin_full", "unknown_object", "alarm",
  "pipeline_retry", "pipeline_retry_failed",
];

export const isFault = (event) => FAULT_EVENTS.includes(event);
export const categoryName = (c) => CATEGORY_NAMES[c] ?? c ?? "—";
export const resultName = (r) => RESULT_NAMES[r] ?? r ?? "—";
export const sourceName = (s) => SOURCE_NAMES[s] ?? s ?? "—";
