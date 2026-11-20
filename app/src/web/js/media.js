// 画面组件：腕部相机 与 三维视图。作业台和手动操作页共用。
//
// 相机：只在自己可见时拉；上一帧没回来不发下一帧；失败指数退避（0.6 s → 10 s）。
// 三维：接仿真控制台时内嵌它自己的工作台（/sim/?embed=scene，桌面、物块、桶都是仿真场景）；
// 接实机时没有那一页，改画按回读的骨架（俯视 + 侧视），点位取自服务端正运动学（arm.chain_cm），
// 页面不自己再算一遍运动学。

import { authUrl, fetchHealth } from "./api.js";

export function createCamera(img, hint, visible) {
  const BASE = 600, MAX = 10000;
  let busy = false, fail = 0, timer = null;
  const schedule = () => {
    clearTimeout(timer);
    timer = setTimeout(tick, Math.min(BASE * 2 ** fail, MAX));
  };
  const tick = () => {
    if (!visible()) { schedule(); return; }
    if (busy) return;
    busy = true;
    img.src = authUrl(`/api/frame?t=${Date.now()}`);
  };
  img.addEventListener("load", () => {
    busy = false; fail = 0;
    if (hint) hint.textContent = `腕部相机 ${img.naturalWidth}×${img.naturalHeight} · 约 1.6 帧/秒`;
    img.classList.remove("off");
    schedule();
  });
  img.addEventListener("error", () => {
    busy = false; fail += 1;
    if (hint) hint.textContent = `没有相机画面：未接控制台或相机还没出图（${Math.min(BASE * 2 ** fail, MAX) / 1000} s 后重试）`;
    img.classList.add("off");
    schedule();
  });
  tick();
  return { kick: () => { fail = 0; tick(); } };
}

/** 是不是仿真控制台：问一次 /api/health，探到之后记住 */
let healthPromise = null;
export function health(refresh = false) {
  if (!healthPromise || refresh) healthPromise = fetchHealth().catch(() => ({}));
  return healthPromise;
}

export function createScene(box, hint) {
  box.innerHTML = `
    <iframe class="sim-frame" title="仿真工作台" hidden></iframe>
    <canvas class="skeleton" hidden></canvas>`;
  const frame = box.querySelector("iframe");
  const canvas = box.querySelector("canvas");
  let simUrl = null, mode = "none", shown = false;
  const embed = (base) => base + (base.includes("?") ? "&" : "?") + "embed=scene";

  const apply = () => {
    const sim = mode === "sim" && shown;
    frame.hidden = mode !== "sim";
    canvas.hidden = mode !== "real";
    if (sim && frame.getAttribute("src") !== embed(simUrl)) frame.src = embed(simUrl);
    if (!sim && frame.getAttribute("src") && frame.getAttribute("src") !== "about:blank") frame.src = "about:blank";
    if (hint) hint.textContent = mode === "sim" ? "仿真工作台：桌面、物块、桶都是仿真场景；拖动旋转、滚轮缩放"
      : mode === "real" ? "实机骨架：按六轴回读的正运动学画（俯视 + 侧视），厘米"
      : "未接控制台：没有三维视图";
  };

  const probe = async (refresh) => {
    const h = await health(refresh);
    const next = h.console && h.console_sim ? "sim" : h.console ? "real" : "none";
    if (next === "sim") simUrl = h.console.replace(/\/+$/, "") + "/sim/";
    if (next !== mode) { mode = next; apply(); }
  };
  probe(false);
  // 控制台可能比页面晚连上，或中途换了：隔一会儿再问一次
  setInterval(() => { if (mode !== "sim") probe(true); }, 8000);

  return {
    show() { shown = true; apply(); },
    hide() { shown = false; apply(); },
    update(store) {
      if (mode !== "real" || canvas.hidden) return;
      drawSkeleton(canvas, store.status.arm ?? {});
    },
  };
}

const ORDER = ["base", "shoulder", "elbow", "wrist", "servo5", "tip"];

function drawSkeleton(canvas, arm) {
  const w = canvas.clientWidth, h = canvas.clientHeight;
  if (!w || !h) return;
  const dpr = window.devicePixelRatio || 1;
  if (canvas.width !== Math.round(w * dpr)) { canvas.width = Math.round(w * dpr); canvas.height = Math.round(h * dpr); }
  const g = canvas.getContext("2d");
  g.setTransform(dpr, 0, 0, dpr, 0, 0);
  const css = getComputedStyle(document.documentElement);
  const col = (n) => css.getPropertyValue(n).trim() || "#888";
  g.clearRect(0, 0, w, h);
  const chain = arm.chain_cm;
  const pts = chain ? ORDER.filter((k) => chain[k]).map((k) => chain[k]) : [];
  const half = w / 2;
  const views = [
    { x0: 0, title: "俯视（x, y）", map: (p) => [p[0], p[1]], range: [[-30, 30], [-5, 40]] },
    { x0: half, title: "侧视（臂展, z）", map: (p) => [Math.hypot(p[0], p[1]) * (p[1] < 0 ? -1 : 1), p[2]], range: [[-5, 40], [-5, 45]] },
  ];
  for (const v of views) {
    const pad = 22, vw = half - pad * 2, vh = h - pad * 2;
    const [[xa, xb], [ya, yb]] = v.range;
    const s = Math.min(vw / (xb - xa), vh / (yb - ya));
    const ox = v.x0 + pad + (vw - s * (xb - xa)) / 2, oy = pad + (vh + s * (yb - ya)) / 2;
    const P = (q) => [ox + (q[0] - xa) * s, oy - (q[1] - ya) * s];
    g.strokeStyle = col("--line"); g.lineWidth = 1;
    for (let gx = Math.ceil(xa / 10) * 10; gx <= xb; gx += 10) {
      const [a] = P([gx, ya]); g.beginPath(); g.moveTo(a, P([gx, ya])[1]); g.lineTo(a, P([gx, yb])[1]); g.stroke();
    }
    for (let gy = Math.ceil(ya / 10) * 10; gy <= yb; gy += 10) {
      const [, b] = P([xa, gy]); g.beginPath(); g.moveTo(P([xa, gy])[0], b); g.lineTo(P([xb, gy])[0], b); g.stroke();
    }
    g.fillStyle = col("--dim"); g.font = "12px sans-serif";
    g.fillText(v.title, v.x0 + pad, 14);
    if (v.x0 > 0) {                                 // 侧视的桌面线
      g.strokeStyle = col("--warn"); g.beginPath(); g.moveTo(...P([xa, 0])); g.lineTo(...P([xb, 0])); g.stroke();
    }
    if (!pts.length) { g.fillText("等回读…", v.x0 + pad, h / 2); continue; }
    const q = pts.map((p) => P(v.map(p)));
    g.strokeStyle = col("--accent"); g.lineWidth = 4; g.lineCap = "round"; g.lineJoin = "round";
    g.beginPath(); q.forEach(([x, y], i) => (i ? g.lineTo(x, y) : g.moveTo(x, y))); g.stroke();
    g.fillStyle = col("--text");
    q.forEach(([x, y]) => { g.beginPath(); g.arc(x, y, 3, 0, Math.PI * 2); g.fill(); });
    const [tx, ty] = q.at(-1);
    g.fillStyle = arm.holding ? col("--good") : col("--warn");
    g.beginPath(); g.arc(tx, ty, 5, 0, Math.PI * 2); g.fill();
  }
}
