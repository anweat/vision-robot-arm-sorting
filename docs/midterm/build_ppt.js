// 生成 中期汇报.pptx（设计展示版，约 20 页）。配图在 figures/，详细说明见 中期汇报.md。
// 用法：NODE_PATH=<装了 pptxgenjs 的 node_modules> node build_ppt.js [apply_theme.js 路径]
// 预览时可设 PPT_FONT（本机有的中文字体）与 PPT_OUT（输出到别处），正式版默认微软雅黑。
const path = require("path");
const fs = require("fs");
const pptxgen = require("pptxgenjs");

const FIG = path.join(__dirname, "figures");
const OUT = process.env.PPT_OUT || path.join(__dirname, "中期汇报.pptx");
const FONT = process.env.PPT_FONT || "Microsoft YaHei";

const THEME = {
  name: "Sorting Green",
  headFontFace: FONT,
  bodyFontFace: FONT,
  colors: {
    dk1: "1B2A24", lt1: "FFFFFF", dk2: "1F4E3D", lt2: "EEF4F0",
    accent1: "2E7D5B", accent2: "F2A93B", accent3: "C0504D",
    accent4: "7FB59A", accent5: "5B6B65", accent6: "D9E7DF",
    hlink: "2E7D5B", folHlink: "5B6B65",
  },
};
const HEX = { done: "2E7D5B", part: "F2A93B", todo: "C0504D", ink: "1B2A24", mute: "5B6B65", dark: "1F4E3D", tint: "EEF4F0", line: "D9E7DF", soft: "7FB59A", pale: "CFE3D8", blue: "3B6EA5" };
const STATUS = { 实机: HEX.done, 仿真: HEX.part, 离线: HEX.blue, 设计: HEX.mute };

const pres = new pptxgen();
pres.layout = "LAYOUT_WIDE"; // 13.33 x 7.5
pres.theme = { headFontFace: FONT, bodyFontFace: FONT };
pres.title = "机械臂垃圾分拣系统 中期汇报";
const C = pres.SchemeColor;

pres.defineSlideMaster({
  title: "TITLE_DARK",
  background: { color: C.text2 },
  objects: [{ placeholder: { options: { name: "title", type: "title", x: 0.8, y: 1.9, w: 11.7, h: 1.2, fontSize: 40, bold: true, color: C.background1, valign: "middle", align: "left" }, text: "" } }],
});
pres.defineSlideMaster({
  title: "CONTENT",
  background: { color: C.background1 },
  objects: [{ placeholder: { options: { name: "title", type: "title", x: 0.6, y: 0.3, w: 10.2, h: 0.75, fontSize: 26, bold: true, color: C.text2, valign: "middle", align: "left", margin: 0 }, text: "" } }],
  slideNumber: { x: 12.4, y: 7.0, w: 0.6, h: 0.3, fontSize: 10, color: "5B6B65" },
});

// ── 小工具 ──────────────────────────────────────────────
const T = (s, text, o) => s.addText(text, Object.assign({ isTextBox: true, fontSize: 14, color: C.text1, valign: "top", margin: 0.05 }, o));
const card = (s, x, y, w, h, fill) => s.addShape(pres.shapes.ROUNDED_RECTANGLE, { x, y, w, h, rectRadius: 0.08, fill: { color: fill || HEX.tint }, line: { color: fill || HEX.tint } });
const circ = (s, x, y, d, color, label, fs) => {
  s.addShape(pres.shapes.OVAL, { x, y, w: d, h: d, fill: { color }, line: { color } });
  if (label) T(s, label, { x, y, w: d, h: d, fontSize: fs || 16, bold: true, color: "FFFFFF", align: "center", valign: "middle", margin: 0 });
};
const chev = (s, x, y) => s.addShape(pres.shapes.CHEVRON, { x, y, w: 0.22, h: 0.36, fill: { color: HEX.soft }, line: { color: HEX.soft } });
const bullets = (items) => items.map((t, j) => ({ text: t, options: { bullet: true, breakLine: j < items.length - 1 } }));
// 标题右侧的状态标签，如 tags(s, ["实机", "仿真"])
const tags = (s, list) => {
  let x = 12.75;
  [...list].reverse().forEach((t) => {
    const w = 0.85;
    x -= w + 0.1;
    s.addShape(pres.shapes.ROUNDED_RECTANGLE, { x, y: 0.5, w, h: 0.36, rectRadius: 0.18, fill: { color: STATUS[t] }, line: { color: STATUS[t] } });
    T(s, t, { x, y: 0.5, w, h: 0.36, fontSize: 12, bold: true, color: "FFFFFF", align: "center", valign: "middle", margin: 0 });
  });
};
// 读 PNG 尺寸，把图等比放进 (x, y, w, h) 框里，居中
const pngSize = (f) => { const b = fs.readFileSync(f); return [b.readUInt32BE(16), b.readUInt32BE(20)]; };
const fig = (s, name, x, y, w, h, alt, align) => {
  const f = path.join(FIG, name);
  const [iw, ih] = pngSize(f);
  const k = Math.min(w / iw, h / ih), dw = iw * k, dh = ih * k;
  const dx = align === "left" ? x : x + (w - dw) / 2;
  s.addImage({ path: f, x: dx, y: y + (h - dh) / 2, w: dw, h: dh, altText: alt || name });
  return { x: dx, y: y + (h - dh) / 2, w: dw, h: dh };
};
const cap = (s, text, x, y, w) => T(s, text, { x, y, w, h: 0.3, fontSize: 11, color: HEX.mute });
// 右侧要点栏
const points = (s, x, y, w, h, head, items, fs) => {
  card(s, x, y, w, h);
  T(s, head, { x: x + 0.2, y: y + 0.15, w: w - 0.4, h: 0.4, fontSize: 15, bold: true, color: HEX.done, margin: 0 });
  T(s, bullets(items), { x: x + 0.15, y: y + 0.65, w: w - 0.3, h: h - 0.8, fontSize: fs || 14, paraSpaceAfter: 9 });
};
const stat = (s, x, y, w, v, l, color) => {
  T(s, v, { x, y, w, h: 0.6, fontSize: 26, bold: true, color: color || HEX.done, margin: 0, valign: "middle" });
  T(s, l, { x, y: y + 0.6, w, h: 0.5, fontSize: 11, color: HEX.mute, margin: 0 });
};
const content = (sec, title, tagList) => {
  const s = pres.addSlide({ masterName: "CONTENT", sectionTitle: sec });
  s.addText(title, { placeholder: "title" });
  if (tagList) tags(s, tagList);
  return s;
};
const divider = (sec, num, title, sub) => {
  pres.addSection({ title: sec });
  const s = pres.addSlide({ masterName: "TITLE_DARK", sectionTitle: sec });
  T(s, num, { x: 0.8, y: 1.3, w: 3, h: 0.7, fontSize: 28, bold: true, color: HEX.soft });
  s.addText(title, { placeholder: "title" });
  T(s, sub, { x: 0.8, y: 3.2, w: 11.5, h: 0.9, fontSize: 18, color: HEX.pale });
  return s;
};

// ── 1 封面 ──────────────────────────────────────────────
pres.addSection({ title: "开场" });
let s = pres.addSlide({ masterName: "TITLE_DARK", sectionTitle: "开场" });
s.addText("智能机械臂垃圾分拣与管理系统", { placeholder: "title" });
T(s, "中期汇报 · 设计与实现 · 2026-11-20", { x: 0.8, y: 3.15, w: 10, h: 0.5, fontSize: 20, color: HEX.pale });
T(s, "Atlas 200I DK A2（昇腾 310B NPU）+ Dofbot 六舵机机械臂 + 腕部单目相机", { x: 0.8, y: 3.8, w: 11, h: 0.4, fontSize: 15, color: HEX.pale });
T(s, "赵杰雄 · 张浩然 · 刘子怡 · 王敏毓", { x: 0.8, y: 5.3, w: 11, h: 0.5, fontSize: 18, bold: true, color: "FFFFFF" });
T(s, "北京交通大学软件学院《软件开发与测试综合实训》2026 秋", { x: 0.8, y: 5.85, w: 11, h: 0.4, fontSize: 13, color: HEX.pale });
s.addNotes("本次汇报按设计展开：整体架构，场地与垃圾桶，视觉，仿真与路径解算，三端设计，最后是后续计划。每页标了状态：实机、仿真、离线、设计值。");

// ── 2 分工 ──────────────────────────────────────────────
s = content("开场", "人员分工：硬件组 2 人 + 前端组 2 人，各占 25%");
const team = [
  ["赵杰雄", "H1 感知与执行", "检测模型与板上推理、多视角定位与棱边解算、路径解算、仿真后端、流程编排、日志契约、集成"],
  ["张浩然", "H2 外设与交互", "亚克力垃圾桶与标定、声光、堵转、溢满检测、手势交互与识别桥、预设动作、上板调参"],
  ["刘子怡", "F1 电脑端前端", "板载网关对外服务、作业台与功能页、控制面板与三维视图、流程脚本编辑、定时、告警"],
  ["王敏毓", "F2 手机端与数据", "Android App（跳板、双摇杆、任务页）、记录与统计、Excel/CSV 导出、前端测试与回放"],
];
team.forEach(([name, role, work], i) => {
  const x = 0.6 + (i % 2) * 6.15, y = 1.35 + Math.floor(i / 2) * 2.35;
  card(s, x, y, 5.95, 2.15);
  circ(s, x + 0.25, y + 0.25, 0.75, i < 2 ? HEX.done : HEX.part, name.slice(0, 1), 22);
  T(s, name, { x: x + 1.2, y: y + 0.22, w: 2.0, h: 0.45, fontSize: 20, bold: true, color: C.text2, margin: 0 });
  T(s, role, { x: x + 1.2, y: y + 0.66, w: 3.2, h: 0.35, fontSize: 13, color: HEX.mute, margin: 0 });
  T(s, "25%", { x: x + 4.45, y: y + 0.18, w: 1.3, h: 0.55, fontSize: 26, bold: true, color: HEX.done, align: "right", margin: 0 });
  T(s, "工作占比", { x: x + 4.45, y: y + 0.72, w: 1.3, h: 0.3, fontSize: 11, color: HEX.mute, align: "right", margin: 0 });
  T(s, work, { x: x + 0.25, y: y + 1.15, w: 5.45, h: 0.9, fontSize: 14 });
});
card(s, 0.6, 6.15, 12.1, 0.75, "E3EEE8");
T(s, [
  { text: "完成情况：", options: { bold: true, color: HEX.done } },
  { text: "基础四项与整体板上流程已跑通；扩展功能除" },
  { text: "麦克风收音（语音控制）", options: { bold: true, color: "9A5B00" } },
  { text: "与" },
  { text: "堆叠抓取", options: { bold: true, color: "9A5B00" } },
  { text: "外均已完成" },
], { x: 0.8, y: 6.15, w: 11.8, h: 0.75, fontSize: 15, valign: "middle" });
s.addNotes("四人分工沿用需求审查方案，工作量各占 25%。功能上只剩语音的麦克风收音和堆叠抓取。");

// ── 3 整体架构 ──────────────────────────────────────────
s = content("开场", "整体架构：执行与安全在板上，客户端只发指令", ["实机", "仿真"]);
fig(s, "arch_overview.png", 0.5, 1.2, 8.6, 5.6, "整体架构图");
points(s, 9.35, 1.3, 3.4, 5.4, "设计原则", [
  "限位、禁入区、回读核对只在控制台实现一次",
  "对外只开网关 :8090，控制台只听本机",
  "手机跳板只做 TCP 转发，不跑业务",
  "仿真与实机答同一套 /api/*，前端无 if sim",
], 13);
s.addNotes("三端：板卡端负责执行和安全，网页和 App 只发指令看状态，中间一部安卓手机做网络跳板。四条原则如右。");

// ── 4 地图迭代 ──────────────────────────────────────────
divider("场地", "01", "场地设计：地图与垃圾桶", "桌面地图三版迭代 · 亚克力插板式垃圾桶 · 窄桌双桶 · 实机进桶测试");
s = content("场地", "地图迭代：v2 满格 → v3 让位四桶 → v4 窄桌双桶", ["实机", "设计"]);
fig(s, "map_iteration_v2_v3_v4.png", 0.6, 1.2, 12.1, 3.4, "三版地图对比");
const maps = [
  ["v2 · 09-24", "20 枚 15 mm 码，5×4 网格", "触点标定残差 2.8 mm；每视角可见 2–4 枚码", "实机长期使用"],
  ["v3 · 10-11", "14 枚，去掉桶下 6 枚", "印四桶落位线；桶间净距仅 1.9 mm", "只有设计"],
  ["v4 · 10-30", "19 枚：保留 8 枚 + 补 11 枚", "桌宽 29.7 cm，改双桶；注册可退到 2 枚码", "11-06 起实机使用"],
];
maps.forEach(([h, a, b, st], i) => {
  const x = 0.6 + i * 4.1;
  card(s, x, 4.85, 3.9, 1.95);
  T(s, h, { x: x + 0.2, y: 4.95, w: 2.2, h: 0.4, fontSize: 17, bold: true, color: C.text2, margin: 0 });
  T(s, st, { x: x + 2.2, y: 4.98, w: 1.55, h: 0.35, fontSize: 11, bold: true, color: i === 1 ? HEX.mute : HEX.done, align: "right", margin: 0 });
  T(s, [{ text: a, options: { breakLine: true } }, { text: b, options: { color: HEX.mute } }], { x: x + 0.2, y: 5.45, w: 3.55, h: 1.25, fontSize: 13, paraSpaceAfter: 4, margin: 0 });
});
s.addNotes("地图是 A4 横放的 ArUco 码纸，给出桌面坐标系、训练数据的真值位置和投放目标。v2 加码保证每个视角都能看到码，触点标定残差 2.8 毫米；v3 为四桶方案去掉桶下的码；v4 因桌宽只有 29.7 厘米改为双桶，去掉 12 枚、补 11 枚。");

// ── 5 垃圾桶建模与图纸 ──────────────────────────────────
s = content("场地", "垃圾桶建模：插板式亚克力方桶，一套参数出全部图纸", ["设计"]);
fig(s, "bin_drawing_p1.png", 0.6, 1.2, 4.6, 3.25, "图纸封面与轴测图");
fig(s, "bin_drawing_p3.png", 0.6, 4.55, 4.6, 2.5, "装配图");
fig(s, "bin_cut_sheet.png", 5.35, 1.2, 3.6, 3.25, "开料排版");
cap(s, "开料排版 297×388 mm，利用率 85%", 5.35, 4.45, 3.6);
const binSpec = [["内口 / 外廓", "80×80 / 84×84 mm"], ["壁高 / 总高", "40 / 42 mm"], ["板厚", "2 mm（主耦合尺寸）"], ["侧板", "82×42，每桶 4 块相同"], ["底板", "94×121，8 个插槽"], ["单桶重", "约 58 g"]];
card(s, 9.2, 1.2, 3.55, 5.85);
T(s, "关键尺寸", { x: 9.4, y: 1.35, w: 3, h: 0.4, fontSize: 15, bold: true, color: HEX.done, margin: 0 });
binSpec.forEach(([k, v], i) => {
  T(s, k, { x: 9.4, y: 1.9 + i * 0.62, w: 3.2, h: 0.28, fontSize: 11, color: HEX.mute, margin: 0 });
  T(s, v, { x: 9.4, y: 2.15 + i * 0.62, w: 3.2, h: 0.32, fontSize: 14, bold: true, margin: 0 });
});
T(s, "make_bins.py 一次生成：配置 JSON、6 页 A3 图纸、DXF 切割文件、贴纸", { x: 9.4, y: 5.75, w: 3.2, h: 1.2, fontSize: 12, color: HEX.ink, margin: 0 });
cap(s, "插舌对槽竖插、溶剂胶固化；板厚变了用 --thickness-mm 全部重算", 5.35, 4.9, 3.7);
s.addNotes("桶是插板式亚克力方桶：内口 80、壁高 40、板厚 2 毫米。所有尺寸由板厚耦合，一个脚本同时出配置、图纸、DXF 和贴纸；板厚实测变了就整体重算，不手改单个尺寸。");

// ── 6 双桶布局 ──────────────────────────────────────────
s = content("场地", "从四桶到窄桌双桶：落位、投放槽位与禁入区", ["设计", "仿真"]);
fig(s, "bin_layout_topview.png", 0.6, 1.2, 12.1, 4.3, "四桶与双桶俯视布局");
const binPts = [
  ["桌宽 29.7 cm", "四桶两侧悬空，先做厨余 + 有害两类"],
  ["桶口投放", "桶口上方约 5.5 cm 松手，每桶 4 个槽位"],
  ["禁入区", "桶壁进碰撞模型，扫描与路径自动避让"],
  ["仿真结果", "8/8 块投入对应的桶"],
];
binPts.forEach(([h, d], i) => {
  const x = 0.6 + i * 3.08;
  card(s, x, 5.7, 2.9, 1.2);
  T(s, h, { x: x + 0.15, y: 5.78, w: 2.6, h: 0.4, fontSize: 15, bold: true, color: C.text2, margin: 0 });
  T(s, d, { x: x + 0.15, y: 6.2, w: 2.65, h: 0.65, fontSize: 12, color: HEX.mute, margin: 0 });
});
s.addNotes("左边是原四桶方案配 v3 地图，右边是窄桌双桶配 v4。桶壁写进禁入区，扫描视角和路径都会避开；仿真里 8 块全部投入对应的桶，这只说明几何能对上。");

// ── 7 垃圾桶实机测试 ────────────────────────────────────
s = content("场地", "垃圾桶实机测试：3 块进对桶，0 次投错", ["实机"]);
fig(s, "bin_real_20261106.png", 0.6, 1.2, 5.4, 4.6, "11-06 实机腕部相机照片");
cap(s, "2026-11-06 腕部相机：双桶 + v4 地图", 0.6, 5.85, 5.4);
const runs = [
  ["15:49", "绿块 · 厨余", "进 2 号桶", HEX.done],
  ["15:53", "有害", "进 3 号桶", HEX.done],
  ["16:01", "有害", "夹空，张开抬起", HEX.part],
  ["16:03", "有害", "进 3 号桶", HEX.done],
  ["11-19", "演示：拨开 2.2 cm", "重扫找回，进 2 号桶", HEX.done],
];
T(s, "11-06 下午逐轮记录", { x: 6.3, y: 1.2, w: 6, h: 0.4, fontSize: 15, bold: true, color: HEX.done, margin: 0 });
runs.forEach(([t, b, r, c], i) => {
  const y = 1.7 + i * 0.62;
  card(s, 6.3, y, 6.4, 0.52);
  T(s, t, { x: 6.45, y, w: 0.9, h: 0.52, fontSize: 13, bold: true, valign: "middle", margin: 0 });
  T(s, b, { x: 7.4, y, w: 2.4, h: 0.52, fontSize: 13, valign: "middle", margin: 0 });
  T(s, r, { x: 9.8, y, w: 2.8, h: 0.52, fontSize: 13, bold: true, color: c, valign: "middle", margin: 0 });
});
points(s, 6.3, 4.9, 6.4, 1.95, "发现与改进", [
  "空爪核对：两桶槽位都到位，2 号桶指尖低 0.4 cm",
  "松手偏高会弹，偏向桶边 1–2 cm → 改为下探更低再松手",
], 13);
s.addNotes("11 月 6 日下午实机进桶：3 块进对桶，1 次夹空，0 次投错；11 月 19 日演示时故意拨开块，系统重扫找回后再抓，进 2 号桶。发现松手偏高会弹，下一步改成下探更低再松手。只有第一块写进了正式文档，其余在本地日志里。");

// ── 8 YOLO 数据 ─────────────────────────────────────────
divider("视觉", "02", "视觉：检测、定位与手势", "YOLO 训练与上板 · 检测框到物块中心与朝向 · 手势识别模型");
s = content("视觉", "YOLO 数据：标签来自摆放，真实 + 合成混合", ["实机", "离线"]);
fig(s, "yolo_four_class_blocks.png", 0.6, 1.2, 2.3, 3.1, "四类贴纸物块");
cap(s, "四类贴纸物块", 0.6, 4.3, 2.3);
fig(s, "yolo_autolabel_capture.png", 3.05, 1.2, 4.1, 3.1, "自动标注采集");
cap(s, "按定位码摆放，标签自动生成", 3.05, 4.3, 4.1);
fig(s, "yolo_synth_samples.png", 7.3, 1.2, 5.45, 3.1, "合成训练样本");
cap(s, "贴面合成：随机位姿、光照、遮挡", 7.3, 4.3, 5.4);
stat(s, 0.8, 4.95, 2.8, "160 张", "真实采集（21 轮，281 个框）");
stat(s, 3.9, 4.95, 2.8, "800 帧", "合成数据（3 138 个框）");
stat(s, 7.0, 4.95, 2.8, "按整轮留出", "同一摆放的 8 个视角不拆开");
stat(s, 10.1, 4.95, 2.8, "+2.0 点", "合成数据带来的 mAP@0.5 提升");
s.addNotes("物块按地图码摆放，所以标签由摆放位置自动生成，不依赖模型自己识别。真实 160 张加合成 800 帧；验证集按整轮留出，避免同一摆放的多视角让指标虚高。");

// ── 9 YOLO 训练与上板 ───────────────────────────────────
s = content("视觉", "YOLO 训练与上板：mAP 0.952，NPU 12.9 ms/帧", ["离线", "实机"]);
fig(s, "yolo_training_curve.png", 0.6, 1.2, 7.4, 2.65, "训练曲线与版本对比");
fig(s, "yolo_om_vs_onnx.png", 0.6, 4.05, 7.4, 2.85, "OM 与 ONNX 逐锚点对照");
const yv = [["0.952", "mAP@0.5（v3，YOLOv8n，60 轮）"], ["12.9 ms", "板上 NPU 单帧推理"], ["60 / 60", "全流程找到且类别正确"], ["一致", "OM 与 ONNX 逐锚点对照"]];
yv.forEach(([v, l], i) => {
  const y = 1.3 + i * 1.4;
  card(s, 8.3, y, 4.45, 1.25);
  stat(s, 8.5, y + 0.08, 4.1, v, l);
});
s.addNotes("YOLOv8n 在 Mac 上混合训练 60 轮，v3 的 mAP@0.5 是 0.952。转成昇腾 OM 后在板上 NPU 推理 12.9 毫秒每帧，与 ONNX 逐锚点对照一致。");

// ── 10 检测到中心 ───────────────────────────────────────
s = content("视觉", "从检测框到物块中心与朝向：多视角融合 + 棱边解算", ["实机", "离线"]);
fig(s, "roi_edge_solve_real.png", 0.6, 1.2, 7.6, 4.4, "ROI 棱边解算");
cap(s, "11-06 真实扫描上重跑：检测框 → 梯度轮廓收敛 → 顶面四角与朝向", 0.6, 5.6, 7.6);
const roiSteps = [
  "框中心射线 ∩ Z=1.5 cm 平面，投到地图",
  "多视角聚类投票（2.5 cm），类别累加",
  "顶面正射图 + 30 mm 方框模板定中心与 yaw",
  "棱边解算：轮廓法向梯度拟合 (x, y, yaw)",
  "3.2 cm 最小间距去重，视角 < 3 判幻影",
];
card(s, 8.45, 1.2, 4.3, 4.75);
T(s, "计算步骤", { x: 8.65, y: 1.32, w: 3.9, h: 0.4, fontSize: 15, bold: true, color: HEX.done, margin: 0 });
roiSteps.forEach((t, i) => {
  const y = 1.85 + i * 0.8;
  circ(s, 8.65, y + 0.05, 0.4, HEX.dark, String(i + 1), 13);
  T(s, t, { x: 9.2, y, w: 3.45, h: 0.7, fontSize: 13, margin: 0 });
});
stat(s, 8.6, 6.0, 2.0, "1.04 cm", "中心误差 p90");
stat(s, 10.7, 6.0, 2.0, "60/60", "15 轮全部找到");
s.addNotes("先把检测框中心的射线与半个块高的平面求交，多视角聚类投票；再在顶面正射图上用方框模板定中心和朝向；棱边解算用轮廓法向梯度拟合 x、y、yaw，有质量门，不合格就保留旧结果。高度只信触碰标定，不信相机。");

// ── 11 手势 ─────────────────────────────────────────────
s = content("视觉", "手势识别：HRNet 四分类上板 9 ms/帧，规则防误触", ["实机", "仿真"]);
fig(s, "gesture_confusion_val_vs_live.png", 0.6, 1.2, 7.4, 3.25, "训练域与现场混淆矩阵");
fig(s, "gesture_npu_latency.png", 0.6, 4.6, 7.4, 2.3, "NPU 推理耗时");
points(s, 8.3, 1.2, 4.45, 5.7, "要点", [
  "沿用课程 HRNet：like / ok / palm / fist",
  "训练域 97.9%，现场 71.2%，fist 易判成 palm",
  "裁剪实验：背景是主因，下一步加手部检测",
  "规则：阈值 0.6、连续 3 帧、冷却 2 s、默认未武装",
  "流程里作为 wait_gesture 步骤：比 OK 开始分拣",
], 13);
s.addNotes("手势用课程的 HRNet 四分类，板上 9 毫秒每帧，三个模型可以并发。现场准确率比训练域低，主因是背景；所以规则上要求阈值、连续帧和冷却，并默认不武装，只映射固定指令。");

// ── 12 仿真模型 ─────────────────────────────────────────
divider("算法", "03", "仿真与路径解算", "与规划共用一套模型的仿真后端 · 抓取姿态解算 · A* 绕障 · 执行与回读");
s = content("算法", "新仿真模型：课程 STL 整臂 + 四连杆夹爪 + 底座方台", ["仿真"]);
fig(s, "sim_web3d.png", 0.6, 1.2, 6.2, 3.55, "网页仿真三维");
cap(s, "网页三维（深度缓冲），课程 STL 网格", 0.6, 4.75, 6.2);
fig(s, "sim_gripper_fourbar.png", 7.0, 1.2, 5.75, 2.3, "四连杆夹爪");
cap(s, "四连杆：合爪时指垫沿轴前移 26 mm", 7.0, 3.5, 5.75);
fig(s, "sim_camera_detect.png", 7.0, 3.9, 5.75, 2.55, "合成相机与检测对照");
cap(s, "合成相机画面直接喂检测模型做对照", 7.0, 6.45, 5.75);
card(s, 0.6, 5.2, 6.2, 1.7, "E3EEE8");
T(s, bullets(["规划与仿真共用整臂网格、四连杆与方台判据", "抓取判据由实机回放定，一致率 14/16", "仿真只证几何与接口自洽，不代表实机成功率"]), { x: 0.75, y: 5.3, w: 5.95, h: 1.5, fontSize: 13, paraSpaceAfter: 6 });
s.addNotes("仿真后端和控制台答同一套接口，前端不用区分。模型用课程 STL 整臂、从 STL 拆出的平行四连杆夹爪和底座方台，和规划器是同一套。抓取判据用 16 次实机合爪回放定下来，一致 14 次。");

// ── 13 路径解算 ─────────────────────────────────────────
s = content("算法", "路径解算：先解抓取姿态，再用 A* 绕障", ["离线"]);
fig(s, "plan_astar_wall.png", 0.6, 1.2, 6.6, 4.15, "A* 绕墙示例");
cap(s, "离线复跑：墙高 6 / 10 / 14 cm，A* 绕行 0.1–0.5 s", 0.6, 5.35, 6.6);
const algo = [
  ["场景建模", "2.5D 高度栅格 2.5 mm；误差平方和根合成余量"],
  ["抓取姿态", "俯仰 80–200° 逐度逆解；J5 对齐棱边；四连杆补偿"],
  ["路径搜索", "关节直线优先；否则指尖格点 1.5 cm 上跑 A*"],
  ["校验兜底", "捷径化后全点复检；三档 full → touching → pipeline"],
];
algo.forEach(([h, d], i) => {
  const y = 1.2 + i * 1.08;
  card(s, 7.45, y, 5.3, 0.95);
  circ(s, 7.6, y + 0.22, 0.5, HEX.dark, String(i + 1), 15);
  T(s, h, { x: 8.25, y: y + 0.08, w: 4.4, h: 0.35, fontSize: 15, bold: true, color: C.text2, margin: 0 });
  T(s, d, { x: 8.25, y: y + 0.45, w: 4.45, h: 0.45, fontSize: 12, color: HEX.mute, margin: 0 });
});
stat(s, 7.5, 5.65, 2.1, "152/163", "28 次实机扫描回放可抓");
stat(s, 9.85, 5.65, 1.2, "7/7", "实机夹住过的块");
stat(s, 11.0, 5.65, 1.75, "12–31", "可抓半径（cm）");
s.addNotes("四步：场景写成 2.5D 高度栅格，误差合成余量；抓取姿态对每个目标逐度求逆解得到可行俯仰区间；路径先试关节直线，不通在指尖格点上跑 A*；最后全点复检，约束分三档放宽。离线回放 28 次实机扫描，够得着的 152 块全部能规划。");

// ── 14 解算到回传 ───────────────────────────────────────
s = content("算法", "从解算到回传：执行分段、回读判空、核对与日志", ["实机", "离线", "仿真"]);
fig(s, "plan_pipeline_flow.png", 0.5, 1.15, 8.4, 4.45, "扫描到回传的流程");
[["35 → 139", "扫描回放：抓+放可规划 /163"], ["2/7 → 7/7", "实机夹住过的块，改进前后"], ["8 → 14", "抓取判据与实机回放一致（/16）"]].forEach(([v, l], i) => {
  const x = 0.6 + i * 2.8;
  card(s, x, 5.75, 2.65, 1.15, "E3EEE8");
  stat(s, x + 0.15, 5.8, 2.4, v, l);
});
points(s, 9.05, 1.2, 3.7, 5.75, "执行与回传", [
  "分段下发 8°/400 ms，偏差 > 4° 即停",
  "合爪按回读判 holding / empty",
  "触地标定：细步下探，夹取偏高 0.8 cm 已修",
  "核对扫描比对起始场景",
  "每步写日志契约事件，SSE 推到网页与 App",
], 13);
s.addNotes("规划结果交给控制台分段执行，每段核对回读，偏差超过 4 度就停在回读位。合爪后按回读判断夹住还是空爪；触地标定查出过夹取偏高 0.8 厘米。最后核对扫描，所有步骤写日志事件推到网页和 App。");

// ── 15 板卡端 ───────────────────────────────────────────
divider("三端", "04", "三端设计：板卡 · 网页 · 手机 App", "进程划分与 I2C 独占 · YAML 流程管线与能力表 · 接口与日志契约 · 网页作业台 · Android 跳板与双摇杆");
s = content("三端", "板卡端：三个常驻进程 + 按需子进程，控制台独占 I2C", ["实机"]);
const procs = [
  ["相机服务", ":8080", "/dev/video0 唯一持有者，带时间戳和序号的帧"],
  ["控制台", "127.0.0.1:8765", "唯一安全边界：限位、禁入区、租约、分段回读、声光"],
  ["板载网关", ":8090", "对外唯一入口：口令、SSE 推送、日志、脚本库、定时"],
  ["任务执行器", "子进程", "旧流水线或分步编排器，同一时刻只跑一个"],
  ["识别桥", "子进程", "手势 / 语音推理，只把结果报给网关"],
  ["NPU 推理", "ais_bench", "四类检测 OM；一个模型一个进程"],
];
procs.forEach(([n, p, d], i) => {
  const x = 0.6 + (i % 3) * 4.08, y = 1.25 + Math.floor(i / 3) * 1.9;
  card(s, x, y, 3.9, 1.7, i < 3 ? HEX.tint : "F4F6F5");
  T(s, n, { x: x + 0.2, y: y + 0.15, w: 2.2, h: 0.4, fontSize: 17, bold: true, color: C.text2, margin: 0 });
  T(s, p, { x: x + 2.0, y: y + 0.18, w: 1.75, h: 0.35, fontSize: 11, color: HEX.done, bold: true, align: "right", margin: 0 });
  T(s, d, { x: x + 0.2, y: y + 0.65, w: 3.5, h: 0.95, fontSize: 13, margin: 0 });
});
card(s, 0.6, 5.2, 12.1, 1.7, "E3EEE8");
T(s, bullets([
  "I2C 独占：舵机、灯、蜂鸣器只由控制台写，第二个进程加锁失败直接退出",
  "实机 1–5 号单舵机写入无效，所有运动用六舵机批量写；“停止”不当急停用",
  "/root/arm-app 与仓库 app/ 布局相同，board_up.sh 一键起停；11-18 网关与跳板上板",
]), { x: 0.8, y: 5.3, w: 11.8, h: 1.5, fontSize: 13, paraSpaceAfter: 6 });
s.addNotes("板上三个常驻进程：相机服务、控制台、网关；任务执行器和识别桥由网关按需拉起。控制台是唯一安全边界，也独占 I2C，因为 11-19 实测流水线进程自己开 I2C 会报错。");

// ── 16 YAML 流程 ────────────────────────────────────────
s = content("三端", "流程管线：YAML 脚本 + 能力表，网页 / 手机 / 板上共用", ["仿真"]);
fig(s, "flow_gesture_sort.png", 0.6, 1.2, 6.4, 4.4, "gesture_sort 流程图");
cap(s, "gesture_sort.yaml：扫描 → 等 OK 手势 → 逐块抓放 → 核对", 0.6, 5.6, 6.4);
const yaml = [
  "name: gesture_sort",
  "speed: 20",
  "steps:",
  "  - step: scan",
  "    with: {bins: bins_acrylic_v2_pair}",
  "    on_result:",
  "      empty: [{stop: 桌上没有找到物块}]",
  "  - step: wait_gesture",
  "    with: {label: ok, timeout_s: 180}",
  "  - for_each: blocks",
  "    order: reach",
  "    do:",
  "      - step: plan",
  "      - step: grasp",
  "      - step: check   # 空爪 → retry",
].join("\n");
s.addShape(pres.shapes.ROUNDED_RECTANGLE, { x: 7.2, y: 1.2, w: 5.55, h: 3.9, rectRadius: 0.08, fill: { color: "1F2A26" }, line: { color: "1F2A26" } });
s.addText(yaml, { isTextBox: true, x: 7.35, y: 1.3, w: 5.3, h: 3.7, fontSize: 12, fontFace: "Courier New", color: "D9E7DF", valign: "top", margin: 0.05 });
const caps = [["ready", "今天就能执行", HEX.done], ["partial", "接口在，输入端未接", HEX.part], ["unavailable", "动臂前整份拒绝", HEX.todo]];
caps.forEach(([k, d, c], i) => {
  const x = 7.2 + i * 1.88;
  card(s, x, 5.3, 1.78, 1.6);
  T(s, k, { x: x + 0.1, y: 5.4, w: 1.6, h: 0.4, fontSize: 14, bold: true, color: c, margin: 0 });
  T(s, d, { x: x + 0.1, y: 5.85, w: 1.6, h: 0.95, fontSize: 12, margin: 0 });
});
s.addNotes("流程用 YAML 写：每步是能力表里的一项能力，有参数和按结果分支；支持逐块、重试、重复、报警。能力表是唯一来源，校验错误带行路径；用到不可用能力的脚本在动臂前整份拒绝，不会停在桌面中间。20 项能力里只有等语音和播报不可用。");

// ── 17 接口设计 ─────────────────────────────────────────
s = content("三端", "接口设计：指令统一走 /api/cmd，状态与日志 SSE 推送", ["实机", "仿真"]);
fig(s, "api_run_script_sequence.png", 0.6, 1.2, 7.0, 4.6, "run_script 接口时序");
cap(s, "一次 run_script：下发 → 校验 → 子进程执行 → 控制台 → 日志 SSE", 0.6, 5.8, 7.0);
const apis = [
  ["POST /api/cmd", "start · run_script · flow_step · stop · goto · drive …"],
  ["GET /api/status/stream", "SSE 2.5 Hz：六轴回读、骨架、夹爪、灯"],
  ["GET /api/log/stream", "日志契约事件，断线按 Last-Event-ID 续传"],
  ["/api/scripts*", "列表、能力表、校验、保存（rev 乐观并发）"],
  ["/api/interact/*", "手势 / 语音识别结果上报与判定"],
];
apis.forEach(([p, d], i) => {
  const y = 1.2 + i * 0.95;
  card(s, 7.85, y, 4.9, 0.82);
  T(s, p, { x: 8.0, y: y + 0.06, w: 4.6, h: 0.35, fontSize: 13, bold: true, fontFace: "Courier New", color: HEX.done, margin: 0 });
  T(s, d, { x: 8.0, y: y + 0.42, w: 4.65, h: 0.35, fontSize: 12, color: HEX.ink, margin: 0 });
});
T(s, "日志契约：事件表唯一来源 log_contract.py，schema 由它生成；source 区分 web / mobile / voice / gesture / timer", { x: 7.85, y: 6.0, w: 4.9, h: 0.9, fontSize: 12, color: HEX.mute, margin: 0 });
s.addNotes("网关对外统一：所有指令走 /api/cmd，带 source 区分来源；状态和日志用 SSE 推送，断线可续传；脚本保存带版本号，多端同时改不会互相覆盖。日志事件表是唯一来源，前端只依赖契约。");

// ── 18 网页前端 ─────────────────────────────────────────
s = content("三端", "网页前端：作业台首页 + 六个功能页，能力表驱动", ["仿真", "实机"]);
const pages = [
  ["作业台", "首页：现场三维 / 回读骨架 + 相机 + 提示栏 + 操作"],
  ["流程脚本", "脚本库 + 可视化编辑 + 运行时逐行点亮"],
  ["手动操作", "速度、夹爪、关节滑块、坐标移动、声光、识别开关"],
  ["告警", "报警确认、异常筛选、未知物体存档图"],
  ["记录与日志", "统计、逐块记录、原始事件流、导出"],
  ["任务计划", "定时启动"],
  ["设置", "连接、偏好、“板卡端待接入”清单"],
];
pages.forEach(([n, d], i) => {
  const x = 0.6 + (i % 2) * 3.6, y = 1.2 + Math.floor(i / 2) * 1.35;
  card(s, x, y, 3.45, 1.2, i === 0 ? "E3EEE8" : HEX.tint);
  T(s, n, { x: x + 0.18, y: y + 0.1, w: 3.1, h: 0.4, fontSize: 16, bold: true, color: C.text2, margin: 0 });
  T(s, d, { x: x + 0.18, y: y + 0.52, w: 3.15, h: 0.65, fontSize: 12, color: HEX.mute, margin: 0 });
});
fig(s, "sim_web3d.png", 7.95, 1.2, 4.8, 2.75, "作业台三维视图");
cap(s, "作业台接仿真时的三维视图", 7.95, 3.95, 4.8);
points(s, 7.95, 4.4, 4.8, 2.5, "设计要点", [
  "无构建原生 ES Module，网关同源托管",
  "编辑器步骤、参数、分支都由能力表生成",
  "提示栏三级：报警 / 告警 / 状态；页头常驻停止",
], 12);
s.addNotes("网页由网关托管，首页是作业台，另有六个功能页。脚本编辑器的可选步骤和参数表单都由能力表生成，板卡端加了能力网页自动出现。");

// ── 19 手机 App ─────────────────────────────────────────
s = content("三端", "手机 App：同时是网络跳板和遥控端", ["实机", "仿真"]);
fig(s, "app_mobile_pages_and_links.png", 0.6, 1.2, 7.8, 4.4, "App 页签与连接方式");
points(s, 8.65, 1.2, 4.1, 4.4, "页面", [
  "控制：速度、停止、相机、双摇杆 10 Hz",
  "任务：开始 / 停止、报警确认、单步执行",
  "网页：WebView 复用网关页面",
  "连接：网关地址、口令、跳板开关",
], 13);
stat(s, 0.8, 5.85, 2.6, "56 ms", "摇杆往返中位（仿真）");
stat(s, 3.6, 5.85, 2.6, "0.4–0.6 s", "Mac→Tailscale→跳板→板卡 SSH");
stat(s, 6.4, 5.85, 2.6, "11-18", "跳板上板，实机试控");
card(s, 8.65, 5.85, 4.1, 1.05, "E3EEE8");
T(s, "手机插板卡 USB-A 开 RNDIS 共享，转发 :8090 与 :2222；UDP 37020 广播自动找板卡", { x: 8.8, y: 5.9, w: 3.85, h: 0.95, fontSize: 12, valign: "middle", margin: 0 });
s.addNotes("App 一身两用：插在板卡上做跳板，经 Tailscale 让远端访问板卡网关和 SSH；同时是遥控端，双摇杆 10 赫兹驱动。11 月 18 日跳板上板，用户在 App 上试控过实机。");

// ── 20 后续 ─────────────────────────────────────────────
pres.addSection({ title: "后续" });
s = pres.addSlide({ masterName: "TITLE_DARK", sectionTitle: "后续" });
T(s, "后续开发方向", { x: 0.8, y: 0.5, w: 11.7, h: 0.8, fontSize: 30, bold: true, color: "FFFFFF" });
const next = [
  ["收尾剩余功能", ["语音接入麦克风收音（模型已上板）", "堆叠优先抓取、搭积木塔", "记录导出补齐前端界面"]],
  ["新版上板与验证", ["分步编排器、YAML 脚本、新规划器上实机", "垃圾桶尺寸验收、v4 触点标定", "下探更低再松手，验证每桶 4 块"]],
  ["验收与第二阶段", ["连续 10 轮成功率、失败处理量化", "12-18 交叉验收，12-25 答辩", "四类桶、非方块物体几何重建"]],
];
next.forEach(([h, items], i) => {
  const x = 0.8 + i * 4.05;
  T(s, h, { x, y: 1.6, w: 3.8, h: 0.5, fontSize: 20, bold: true, color: i === 0 ? "F2A93B" : HEX.soft });
  T(s, bullets(items), { x, y: 2.2, w: 3.8, h: 3.2, fontSize: 15, color: "FFFFFF", paraSpaceAfter: 12 });
});
T(s, "谢谢！欢迎提问", { x: 0.8, y: 6.1, w: 11.7, h: 0.6, fontSize: 20, bold: true, color: HEX.pale });

(async () => {
  await pres.writeFile({ fileName: OUT });
  const at = process.argv[2];
  if (at) await require(at).applyTheme(OUT, THEME);
  // 主题里东亚字体默认为空，中文会落到“等线”；改成与正文同一字体
  const JSZip = require("jszip");
  const zip = await JSZip.loadAsync(fs.readFileSync(OUT), { createFolders: false });
  const f = "ppt/theme/theme1.xml";
  let xml = await zip.file(f).async("string");
  xml = xml.replace(/<a:ea typeface=""\/>/g, `<a:ea typeface="${FONT}"/>`).replace(/(<a:font script="Hans" typeface=")[^"]*"/g, `$1${FONT}"`);
  zip.file(f, xml);
  Object.keys(zip.files).filter((k) => zip.files[k].dir).forEach((k) => delete zip.files[k]);
  fs.writeFileSync(OUT, await zip.generateAsync({ type: "nodebuffer", compression: "DEFLATE" }));
  console.log("wrote", OUT);
})();
