// 画面组件：腕部相机 与 三维视图。作业台和手动操作页共用。
//
// 相机：只在自己可见时拉；上一帧没回来不发下一帧；失败指数退避（0.6 s → 10 s）。
// 三维：接仿真控制台时内嵌它自己的工作台（/sim/?embed=scene，桌面、物块、桶都是仿真场景）；
// 接实机时没有那一页，用 WebGL 画整臂网格（T12）：网格取自 /api/arm/meshes（课程 STL 抽稀），
// 每段 4x4 姿态取自 status.arm.frames（无臂时 /api/arm/pose?joints= 纯运动学算），
// 再叠加 /api/blocks 的扫描物块与平台禁入圆。位姿与仿真同源，页面不自己算运动学。
// 浏览器没有 WebGL 或网格不可用时，退回原来的俯视/侧视线框骨架。
//
// 开发/验收辅助：URL 带 ?mockscene=1 时强制进入实机模式，用 /api/arm/pose 算一份固定姿态
// 加合成物块，不接机械臂也能核对整臂网格渲染。生产地址不带这个参数。

import { authUrl, fetchHealth } from "./api.js";

const MOCK_SCENE = new URLSearchParams(location.search).has("mockscene");
const MOCK_JOINTS = [90, 90, 90, 90, 90, 90];

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

async function getJson(path) {
  const r = await fetch(authUrl(path), { cache: "no-store" });
  if (!r.ok) throw new Error(`${path} -> ${r.status}`);
  return r.json();
}

/** ?mockscene=1 的合成物块：与 /api/blocks 同结构，无臂时也能看方块 + 禁入圆。 */
function mockScene() {
  return {
    block_side_cm: 3,
    keep_out: { platform_radius_cm: 8 },
    blocks: [
      { centre_cm: [-9, 13, 1.5], category: "kitchen" },
      { centre_cm: [7, 16, 1.5], category: "recyclable" },
      { centre_cm: [13, 8, 1.5], category: "hazardous" },
      { centre_cm: [2, 21, 1.5], category: "other" },
      { centre_cm: [-4, 7, 1.5], category: "kitchen" },
    ],
  };
}

export function createScene(box, hint) {
  box.innerHTML = `
    <iframe class="sim-frame" title="仿真工作台" hidden></iframe>
    <canvas class="skeleton" hidden></canvas>`;
  const frame = box.querySelector("iframe");
  const canvas = box.querySelector("canvas");
  let simUrl = null, mode = "none", shown = false;
  let blocksData = null;              // /api/blocks
  let mockBlocks = MOCK_SCENE ? mockScene() : null;
  let mockFrames = null;              // ?mockscene=1：/api/arm/pose 算的一次姿态
  let glView = null, glFailed = false;

  const embed = (base) => base + (base.includes("?") ? "&" : "?") + "embed=scene";

  const ensureGL = () => {
    if (glView || glFailed) return glView;
    try { glView = createGLView(canvas); } catch { glFailed = true; }
    return glView;
  };

  const loadMesh = async () => {
    try { (ensureGL())?.setMesh(await getJson("/api/arm/meshes")); } catch { /* 缺网格：退回骨架 */ }
  };
  const loadMockFrames = async () => {
    try { mockFrames = (await getJson(`/api/arm/pose?joints=${MOCK_JOINTS.join(",")}`)).frames; }
    catch { mockFrames = null; }
  };

  const apply = () => {
    const sim = mode === "sim" && shown;
    frame.hidden = mode !== "sim";
    canvas.hidden = mode !== "real";
    if (sim && frame.getAttribute("src") !== embed(simUrl)) frame.src = embed(simUrl);
    if (!sim && frame.getAttribute("src") && frame.getAttribute("src") !== "about:blank") frame.src = "about:blank";
    if (mode === "real") {
      ensureGL(); loadMesh();
      if (MOCK_SCENE && !mockFrames) loadMockFrames();
    }
    if (hint) hint.textContent = mode === "sim" ? "仿真工作台：桌面、物块、桶都是仿真场景；拖动旋转、滚轮缩放"
      : mode === "real" ? "整臂网格：按回读姿态画（拖动旋转、滚轮缩放）；方块来自 /api/blocks"
      : "未接控制台：没有三维视图";
  };

  const probe = async (refresh) => {
    if (MOCK_SCENE) { if (mode !== "real") { mode = "real"; apply(); } return; }
    const h = await health(refresh);
    const next = h.console && h.console_sim ? "sim" : h.console ? "real" : "none";
    if (next === "sim") simUrl = h.console.replace(/\/+$/, "") + "/sim/";
    if (next !== mode) { mode = next; apply(); }
  };
  probe(false);
  setInterval(() => { if (mode !== "sim") probe(true); }, 8000);
  // 实机模式下周期性刷新桌上物块（方块位置会随分拣变化）
  setInterval(async () => {
    if (mode !== "real" || !shown || MOCK_SCENE) return;
    try { blocksData = await getJson("/api/blocks"); } catch { /* 保持旧值 */ }
  }, 2500);

  // 旋转/缩放交互
  let yaw = -0.6, pitch = 0.42, dist = 105;
  let drag = null;
  canvas.addEventListener("mousedown", (e) => { drag = { x: e.clientX, y: e.clientY }; });
  window.addEventListener("mouseup", () => { drag = null; });
  window.addEventListener("mousemove", (e) => {
    if (!drag) return;
    yaw -= (e.clientX - drag.x) * 0.01;
    pitch = Math.max(-0.2, Math.min(1.35, pitch + (e.clientY - drag.y) * 0.01));
    drag = { x: e.clientX, y: e.clientY };
  });
  canvas.addEventListener("wheel", (e) => {
    e.preventDefault();
    dist = Math.max(30, Math.min(260, dist * (e.deltaY > 0 ? 1.1 : 0.9)));
  }, { passive: false });

  const draw = () => {
    if (mode !== "real" || canvas.hidden) return;
    const scene = MOCK_SCENE ? mockBlocks : blocksData;
    const v = ensureGL();
    if (!v || !v.ready()) return;
    if (MOCK_SCENE && mockFrames) v.setFrames(mockFrames);
    v.render({ scene, yaw, pitch, dist });
  };
  let raf = 0;
  const loop = () => { raf = requestAnimationFrame(loop); draw(); };
  loop();

  return {
    show() {
      shown = true; apply();
      if (mode === "real") loadMesh();
      if (mode === "real" && !MOCK_SCENE) getJson("/api/blocks").then((b) => { blocksData = b; }).catch(() => {});
      if (MOCK_SCENE && !mockFrames) loadMockFrames();
    },
    hide() { shown = false; apply(); },
    update(store) {
      if (mode !== "real" || canvas.hidden) return;
      if (MOCK_SCENE) return;
      // 接实机时每帧姿态来自 status.arm.frames；WebGL 不可用时退回线框骨架
      const frames = store?.status?.arm?.frames;
      if (frames && glView && glView.ready()) { glView.setFrames(frames); return; }
      if (glFailed || !glView) drawSkeleton(canvas, store?.status?.arm ?? {});
    },
  };
}

// ---------- 旧线框骨架（WebGL 不可用/无网格时的兜底） ----------
const ORDER = ["base", "shoulder", "elbow", "wrist", "servo5", "tip"];

function drawSkeleton(canvas, arm) {
  const w = canvas.clientWidth, h = canvas.clientHeight;
  if (!w || !h) return;
  const dpr = window.devicePixelRatio || 1;
  if (canvas.width !== Math.round(w * dpr)) { canvas.width = Math.round(w * dpr); canvas.height = Math.round(h * dpr); }
  const g = canvas.getContext("2d");
  if (!g) return;
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
    if (v.x0 > 0) {
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

// ---------- WebGL 整臂网格渲染 ----------
const VS = `
attribute vec3 a_pos; attribute vec3 a_nrm; attribute vec3 a_col;
uniform mat4 u_mvp; uniform mat3 u_nmodel;
varying vec3 v_n; varying vec3 v_c;
void main(){ v_n = u_nmodel * a_nrm; v_c = a_col; gl_Position = u_mvp * vec4(a_pos, 1.0); }`;
const FS = `
precision mediump float;
varying vec3 v_n; varying vec3 v_c;
void main(){
  vec3 n = normalize(v_n);
  float d = max(dot(n, normalize(vec3(0.45, -0.55, 0.8))), 0.0);
  gl_FragColor = vec4(v_c * (0.4 + 0.7 * d), 1.0);
}`;

const CAT_COLOUR = {
  recyclable: [0.31, 0.70, 0.85], kitchen: [0.37, 0.77, 0.54],
  hazardous: [0.88, 0.42, 0.35], other: [0.88, 0.66, 0.29],
};
const ARM_COLOUR = [0.68, 0.74, 0.80];

function createGLView(canvas) {
  const gl = canvas.getContext("webgl", { antialias: true, alpha: true }) || canvas.getContext("experimental-webgl");
  if (!gl) throw new Error("no webgl");

  const compile = (type, src) => {
    const s = gl.createShader(type);
    gl.shaderSource(s, src); gl.compileShader(s);
    if (!gl.getShaderParameter(s, gl.COMPILE_STATUS)) throw new Error(gl.getShaderInfoLog(s) || "shader");
    return s;
  };
  const prog = gl.createProgram();
  gl.attachShader(prog, compile(gl.VERTEX_SHADER, VS));
  gl.attachShader(prog, compile(gl.FRAGMENT_SHADER, FS));
  gl.linkProgram(prog);
  if (!gl.getProgramParameter(prog, gl.LINK_STATUS)) throw new Error("link");
  gl.useProgram(prog);
  const loc = { pos: gl.getAttribLocation(prog, "a_pos"), nrm: gl.getAttribLocation(prog, "a_nrm"),
    col: gl.getAttribLocation(prog, "a_col"), mvp: gl.getUniformLocation(prog, "u_mvp"),
    nmodel: gl.getUniformLocation(prog, "u_nmodel") };

  gl.enable(gl.DEPTH_TEST);
  gl.clearColor(0, 0, 0, 0);

  const parts = [];               // {buf, count, frame}
  let mesh = null;
  let frames = null;
  let scene = null;
  let dynTri = null, dynLine = null;

  const buf = (data) => {
    const b = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, b);
    gl.bufferData(gl.ARRAY_BUFFER, data, gl.DYNAMIC_DRAW);
    return b;
  };
  const bindAttribs = (b) => {
    gl.bindBuffer(gl.ARRAY_BUFFER, b);
    const stride = 9 * 4;
    gl.enableVertexAttribArray(loc.pos); gl.vertexAttribPointer(loc.pos, 3, gl.FLOAT, false, stride, 0);
    gl.enableVertexAttribArray(loc.nrm); gl.vertexAttribPointer(loc.nrm, 3, gl.FLOAT, false, stride, 12);
    gl.enableVertexAttribArray(loc.col); gl.vertexAttribPointer(loc.col, 3, gl.FLOAT, false, stride, 24);
  };

  // 把 part 的三角形展开成 (pos, nrm, col) 扁平数组，单位 0.1 mm -> cm
  const buildPart = (part) => {
    const v = part.v, f = part.f, n = f.length;
    const out = new Float32Array(n * 9);
    const c = ARM_COLOUR;
    for (let i = 0; i < n; i += 3) {
      const ia = f[i] * 3, ib = f[i + 1] * 3, ic = f[i + 2] * 3;
      const ax = v[ia] * 0.01, ay = v[ia + 1] * 0.01, az = v[ia + 2] * 0.01;
      const bx = v[ib] * 0.01, by = v[ib + 1] * 0.01, bz = v[ib + 2] * 0.01;
      const cx = v[ic] * 0.01, cy = v[ic + 1] * 0.01, cz = v[ic + 2] * 0.01;
      let nx = (by - ay) * (cz - az) - (bz - az) * (cy - ay);
      let ny = (bz - az) * (cx - ax) - (bx - ax) * (cz - az);
      let nz = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
      const len = Math.hypot(nx, ny, nz) || 1; nx /= len; ny /= len; nz /= len;
      const o = i * 9;
      out.set([ax, ay, az, nx, ny, nz, c[0], c[1], c[2]], o);
      out.set([bx, by, bz, nx, ny, nz, c[0], c[1], c[2]], o + 9);
      out.set([cx, cy, cz, nx, ny, nz, c[0], c[1], c[2]], o + 18);
    }
    return { buf: buf(out), count: n };
  };

  const setMesh = (data) => {
    if (mesh) return;
    mesh = data;
    for (const [name, part] of Object.entries(data.parts)) {
      const g = buildPart(part);
      parts.push({ ...g, frame: part.frame, name });
    }
  };
  const setFrames = (fr) => { frames = fr; };

  // ---- 矩阵工具（列主序）----
  const mul = (a, b) => {
    const o = new Float32Array(16);
    for (let c = 0; c < 4; c++) for (let r = 0; r < 4; r++) {
      let s = 0; for (let k = 0; k < 4; k++) s += a[k * 4 + r] * b[c * 4 + k]; o[c * 4 + r] = s;
    }
    return o;
  };
  const perspective = (fovy, aspect, near, far) => {
    const f = 1 / Math.tan(fovy / 2), nf = 1 / (near - far);
    return new Float32Array([f / aspect, 0, 0, 0, 0, f, 0, 0, 0, 0, (far + near) * nf, -1, 0, 0, 2 * far * near * nf, 0]);
  };
  const lookAt = (eye, ctr, up) => {
    let zx = eye[0] - ctr[0], zy = eye[1] - ctr[1], zz = eye[2] - ctr[2];
    let l = Math.hypot(zx, zy, zz) || 1; zx /= l; zy /= l; zz /= l;
    let xx = up[1] * zz - up[2] * zy, xy = up[2] * zx - up[0] * zz, xz = up[0] * zy - up[1] * zx;
    l = Math.hypot(xx, xy, xz) || 1; xx /= l; xy /= l; xz /= l;
    const yx = zy * xz - zz * xy, yy = zz * xx - zx * xz, yz = zx * xy - zy * xx;
    return new Float32Array([
      xx, yx, zx, 0, xy, yy, zy, 0, xz, yz, zz, 0,
      -(xx * eye[0] + xy * eye[1] + xz * eye[2]),
      -(yx * eye[0] + yy * eye[1] + yz * eye[2]),
      -(zx * eye[0] + zy * eye[1] + zz * eye[2]), 1]);
  };
  // frames 是行主序 [row][col] -> 列主序 4x4 与 3x3 法线
  const frameCols = (f) => {
    const m = new Float32Array(16);
    for (let r = 0; r < 4; r++) for (let c = 0; c < 4; c++) m[c * 4 + r] = f[r][c];
    const nrm = new Float32Array(9);
    for (let r = 0; r < 3; r++) for (let c = 0; c < 3; c++) nrm[c * 3 + r] = f[r][c];
    return { m, nrm };
  };
  const IDENTITY = new Float32Array([1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]);
  const IDENTITY3 = new Float32Array([1, 0, 0, 0, 1, 0, 0, 0, 1]);

  // 动态几何：物块(三角) + 禁入圆/地面(线)
  const rebuildDyn = (sc) => {
    const tris = [];
    const lines = [];
    const side = (sc?.block_side_cm ?? 3) / 2;
    for (const b of sc?.blocks || []) {
      const c = b.centre_cm || [0, 0, 0];
      const col = CAT_COLOUR[b.category] || CAT_COLOUR.other;
      pushBox(tris, c[0], c[1], side, col);
    }
    // 禁入圆（虚线，z=0 平面）
    const r = sc?.keep_out?.platform_radius_cm;
    if (r) {
      const col = [0.88, 0.66, 0.29];
      const N = 96;
      for (let i = 0; i < N; i++) {
        if (i % 3 === 2) continue;                     // 每三段留空一段，形成虚线
        const a0 = (i / N) * Math.PI * 2, a1 = ((i + 1) / N) * Math.PI * 2;
        lines.push([Math.cos(a0) * r, Math.sin(a0) * r, 0, 0, 0, 1, ...col],
                   [Math.cos(a1) * r, Math.sin(a1) * r, 0, 0, 0, 1, ...col]);
      }
    }
    // 地面网格（10 cm 一格，范围 ±40）
    const gc = [0.16, 0.24, 0.30];
    for (let gx = -40; gx <= 40; gx += 10) {
      lines.push([gx, -40, 0, 0, 0, 1, ...gc], [gx, 40, 0, 0, 0, 1, ...gc]);
      lines.push([-40, gx, 0, 0, 0, 1, ...gc], [40, gx, 0, 0, 0, 1, ...gc]);
    }
    const flat = (rows) => new Float32Array(rows.flat());
    if (dynTri) gl.deleteBuffer(dynTri.buf);
    if (dynLine) gl.deleteBuffer(dynLine.buf);
    dynTri = tris.length ? { buf: buf(flat(tris)), count: tris.length } : null;
    dynLine = lines.length ? { buf: buf(flat(lines)), count: lines.length } : null;
    scene = sc;
  };

  function pushBox(out, cx, cy, half, col) {
    const z0 = 0, z1 = half * 2;
    const x0 = cx - half, x1 = cx + half, y0 = cy - half, y1 = cy + half;
    const faces = [
      [[x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y0, z1], [x1, y1, z1], [x0, y1, z1], [0, 0, 1]],
      [[x0, y0, z0], [x1, y1, z0], [x1, y0, z0], [x0, y0, z0], [x0, y1, z0], [x1, y1, z0], [0, 0, -1]],
      [[x0, y0, z0], [x1, y0, z0], [x1, y0, z1], [x0, y0, z0], [x1, y0, z1], [x0, y0, z1], [0, -1, 0]],
      [[x0, y1, z0], [x0, y1, z1], [x1, y1, z1], [x0, y1, z0], [x1, y1, z1], [x1, y1, z0], [0, 1, 0]],
      [[x0, y0, z0], [x0, y0, z1], [x0, y1, z1], [x0, y0, z0], [x0, y1, z1], [x0, y1, z0], [-1, 0, 0]],
      [[x1, y0, z0], [x1, y1, z0], [x1, y1, z1], [x1, y0, z0], [x1, y1, z1], [x1, y0, z1], [1, 0, 0]],
    ];
    for (const f of faces) {
      const n = [f[6][0], f[6][1], f[6][2]];
      for (let k = 0; k < 6; k++) out.push([...f[k], ...n, ...col]);
    }
  }

  const ready = () => !!mesh;

  const render = ({ scene: sc, yaw, pitch, dist }) => {
    const w = canvas.clientWidth, h = canvas.clientHeight;
    if (!w || !h) return;
    const dpr = window.devicePixelRatio || 1;
    if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) {
      canvas.width = Math.round(w * dpr); canvas.height = Math.round(h * dpr);
    }
    gl.viewport(0, 0, canvas.width, canvas.height);
    gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);

    if (sc !== scene) rebuildDyn(sc);

    const target = [0, 10, 16];
    const cp = Math.cos(pitch), sp = Math.sin(pitch);
    const eye = [target[0] + dist * Math.cos(yaw) * cp, target[1] + dist * Math.sin(yaw) * cp, target[2] + dist * sp];
    const view = lookAt(eye, target, [0, 0, 1]);
    const proj = perspective(0.9, w / h, 1, 1000);
    const base = mul(proj, view);

    gl.useProgram(prog);

    // 地面与禁入圆（线）
    if (dynLine) {
      gl.uniformMatrix4fv(loc.mvp, false, base);
      gl.uniformMatrix3fv(loc.nmodel, false, IDENTITY3);
      bindAttribs(dynLine.buf);
      gl.drawArrays(gl.LINES, 0, dynLine.count);
    }
    // 物块（三角）
    if (dynTri) {
      gl.uniformMatrix4fv(loc.mvp, false, base);
      gl.uniformMatrix3fv(loc.nmodel, false, IDENTITY3);
      bindAttribs(dynTri.buf);
      gl.drawArrays(gl.TRIANGLES, 0, dynTri.count);
    }
    // 整臂网格
    if (mesh && frames) {
      for (const p of parts) {
        const f = frames[p.frame];
        if (!f) continue;
        const { m, nrm } = frameCols(f);
        gl.uniformMatrix4fv(loc.mvp, false, mul(base, m));
        gl.uniformMatrix3fv(loc.nmodel, false, nrm);
        bindAttribs(p.buf);
        gl.drawArrays(gl.TRIANGLES, 0, p.count);
      }
    }
  };

  return { setMesh, setFrames, render, ready };
}
