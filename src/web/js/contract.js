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
  mode_changed: { status: "planned", meaning: "作业模式切换", record: null },
  unknown_object: { status: "planned", meaning: "发现不属于四类的物体", record: null },
  bin_full: { status: "planned", meaning: "容器已满，停止投放", record: null },
  stall: { status: "planned", meaning: "舵机堵转，已停止后续指令", record: null },
  interaction: { status: "planned", meaning: "手势、表情或语音触发的指令", record: null },
};

// 异常日志页关心的事件：出问题的都在这里，其余进实时状态页的事件流
export const FAULT_EVENTS = [
  "pipeline_missed", "pipeline_skip", "pipeline_approach_failed", "pipeline_put_failed",
  "pipeline_stuck", "gripper_not_closed", "gripper_still_asleep", "gripper_wake_refused",
  "stopped", "table_z_disagrees", "stall", "bin_full", "unknown_object",
];

export const isFault = (event) => FAULT_EVENTS.includes(event);
export const categoryName = (c) => CATEGORY_NAMES[c] ?? c ?? "—";
export const resultName = (r) => RESULT_NAMES[r] ?? r ?? "—";
export const sourceName = (s) => SOURCE_NAMES[s] ?? s ?? "—";
