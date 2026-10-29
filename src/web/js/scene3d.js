// 三维状态视图（F1-3）：骨架点来自服务端算好的正运动学 status.arm.chain_cm（厘米），
// 桌上物块来自 GET /api/blocks，禁入区尺寸由服务端从 config/arm_limits.json 读好并转成厘米。
// 这里只负责旋转、投影和画。刻意不在前端重算运动学——图上看到的位置，和控制台拿去做
// 禁入区检查的必须是同一份数。世界坐标沿用渗出坐标系：x 右（臂的右侧）、y 远离臂、z 上。

import { GRIPPER } from "./contract.js";

const CHAIN = ["base", "shoulder", "elbow", "wrist", "servo5", "tip"];
const LABELS = { tip: "末端", wrist: "腕", elbow: "肘", servo5: "腕旋" };
const DIST = 40;            // 相机到原点的距离，厘米
const rad = (d) => (d * Math.PI) / 180;

// 物块类别色，与 style.css 的 --c-* 保持一致
const BLOCK_COLOUR = {
  recyclable: "#4fb3d9",
  kitchen: "#5fc48a",
  hazardous: "#e06a5a",
  other: "#e0a84a",
};

const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const scale = (v, s) => [v[0] * s, v[1] * s, v[2] * s];
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const norm = (v) => { const m = Math.hypot(v[0], v[1], v[2]) || 1; return [v[0] / m, v[1] / m, v[2] / m]; };

export function createSceneView(canvas) {
  const ctx = canvas.getContext("2d");
  let yaw = 38, pitch = 20, zoom = 1;
  let drag = null;
  let chain = null;
  let scene = null;         // {blocks, block_side_cm, keep_out}，来自 /api/blocks

  const rotate = ([x, y, z]) => {
    const cy = rad(yaw), cp = rad(pitch);
    const rx = x * Math.cos(cy) - y * Math.sin(cy);
    const ry = x * Math.sin(cy) + y * Math.cos(cy);
    return [rx, ry * Math.cos(cp) - z * Math.sin(cp), ry * Math.sin(cp) + z * Math.cos(cp)];
  };

  // 相机空间 (右, 深, 上) -> 画布坐标；深度为第二分量
  const project = (p) => {
    const [rx, depth, up] = rotate(p);
    const w = canvas.clientWidth || 360, h = canvas.clientHeight || 260;
    const f = Math.min(w, h) * 1.4 * zoom;
    const d = Math.max(6, DIST + depth);
    return [w / 2 + (rx * f) / d, h / 2 + 34 - (up * f) / d, d];
  };

  const line = (from, to, width, colour) => {
    ctx.strokeStyle = colour; ctx.lineWidth = width; ctx.lineCap = "round";
    ctx.beginPath(); ctx.moveTo(...from); ctx.lineTo(...to); ctx.stroke();
  };

  const poly = (pts, fill, stroke, width = 1) => {
    ctx.beginPath(); ctx.moveTo(pts[0][0], pts[0][1]);
    for (const p of pts.slice(1)) ctx.lineTo(p[0], p[1]);
    ctx.closePath();
    if (fill) { ctx.fillStyle = fill; ctx.fill(); }
    if (stroke) { ctx.strokeStyle = stroke; ctx.lineWidth = width; ctx.stroke(); }
  };

  const disc = (r, z) => {
    const pts = [];
    for (let i = 0; i < 48; i++) {
      const a = (i / 48) * Math.PI * 2;
      pts.push(project([r * Math.cos(a), r * Math.sin(a), z]));
    }
    return pts;
  };

  // 平台与禁入区。禁入区是平台外扩一个余量后的圆柱，虚线画出——演示时能一眼看出
  // 臂有没有踩线（status.arm.clearance_cm 变负就是踩线了）
  function drawPlatform(keep) {
    if (!keep) return;
    const r = keep.platform_radius_cm ?? 8;
    const h = keep.platform_min_z_cm ?? 7.6;
    const m = keep.platform_margin_cm ?? 0.4;
    const top = disc(r, h), bottom = disc(r, 0);
    poly(bottom, "#16232e", null);
    for (let i = 0; i < 48; i += 6) line(bottom[i].slice(0, 2), top[i].slice(0, 2), 1, "#2a3d4c");
    poly(top, "#22384a", "#3d5f78");
    ctx.setLineDash([4, 3]);
    poly(disc(r + m, h + m), null, "rgba(224,106,90,.5)");
    ctx.setLineDash([]);
  }

  // 物块：30 mm 立方体。base_cm 是底面中心，所以往 z 上长一个边长
  function drawBlock(block, side) {
    const [cx, cy, cz] = block.centre_cm;
    const h = side ?? 3;
    const corners = [];
    for (const dx of [-h / 2, h / 2])
      for (const dy of [-h / 2, h / 2])
        for (const dz of [0, h]) corners.push(project([cx + dx, cy + dy, cz + dz]));
    // 生成顺序：每个 (dx,dy) 先 z=0 后 z=h，所以奇数索引是顶面
    const top = [corners[1], corners[3], corners[7], corners[5]];
    const bottom = [corners[0], corners[2], corners[6], corners[4]];
    const colour = BLOCK_COLOUR[block.category] ?? BLOCK_COLOUR.other;
    ctx.globalAlpha = block.held ? 0.95 : 0.8;
    poly(top, colour, colour);
    ctx.globalAlpha = 1;
    for (let i = 0; i < 4; i++) line(top[i].slice(0, 2), bottom[i].slice(0, 2), 1, colour);
    ctx.setLineDash([2, 2]);
    poly(bottom, null, colour);
    ctx.setLineDash([]);
  }

  // 夹爪：从末端沿"servo5 -> tip"方向伸出两片，间距按回读角度张合。
  // 回读越大越合拢（config：张约 11–30，完全合拢 133），null 表示还没下发过夹爪指令
  function drawGripper(pts, grip) {
    const tip = pts.tip, pre = pts.servo5 ?? pts.wrist;
    if (!tip || !pre) return;
    const dir = norm(sub(tip, pre));
    let perp = cross(dir, [0, 0, 1]);
    if (Math.hypot(perp[0], perp[1], perp[2]) < 1e-3) perp = cross(dir, [1, 0, 0]);
    perp = norm(perp);
    const ratio = grip == null ? 0.6
      : Math.max(0, Math.min(1, (GRIPPER.grip - grip) / (GRIPPER.grip - GRIPPER.openWide)));
    const gap = 0.35 + 1.55 * ratio;              // 半开间距，厘米
    const end = add(tip, scale(dir, 1.6));
    for (const s of [1, -1]) {
      const a = add(tip, scale(perp, s * gap * 0.35));
      const b = add(end, scale(perp, s * gap));
      line(project(a).slice(0, 2), project(b).slice(0, 2), 2.4, "#8fa5b4");
    }
  }

  function draw() {
    const dpr = window.devicePixelRatio || 1;
    const w = canvas.clientWidth || 360, h = canvas.clientHeight || 260;
    if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) {
      canvas.width = Math.round(w * dpr); canvas.height = Math.round(h * dpr);
    }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);
    ctx.fillStyle = "#0b141b"; ctx.fillRect(0, 0, w, h);

    // 桌面网格：±20 cm，每 5 cm 一条，给姿态一个参照
    ctx.lineWidth = 1;
    for (let v = -20; v <= 20; v += 5) {
      const a = project([v, -20, 0]), b = project([v, 20, 0]);
      const c = project([-20, v, 0]), d = project([20, v, 0]);
      ctx.strokeStyle = v === 0 ? "#2f4859" : "#1c2b36";
      ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(c[0], c[1]); ctx.lineTo(d[0], d[1]); ctx.stroke();
    }

    // 画家算法：平台、物块、臂各按中心深度排序，远的先画
    const items = [];
    const keep = scene?.keep_out;
    if (keep) items.push({ d: project([0, 0, (keep.platform_min_z_cm ?? 7.6) / 2])[2], fn: () => drawPlatform(keep) });
    const side = scene?.block_side_cm ?? 3;
    for (const b of scene?.blocks ?? []) {
      if (b?.centre_cm?.length !== 3) continue;
      items.push({ d: project([b.centre_cm[0], b.centre_cm[1], b.centre_cm[2] + side / 2])[2], fn: () => drawBlock(b, side) });
    }
    if (chain) {
      const anchor = chain.tip ?? chain.base ?? [0, 0, 0];
      items.push({ d: project(anchor)[2], fn: () => drawArm() });
    }
    items.sort((a, b) => b.d - a.d).forEach((it) => it.fn());

    ctx.fillStyle = "#5a7180"; ctx.font = "11px sans-serif"; ctx.textAlign = "right";
    ctx.fillText("拖动旋转 · 滚轮缩放", w - 6, h - 6);
  }

  function drawArm() {
    const pts = {};
    for (const key of CHAIN) if (chain[key]) pts[key] = project(chain[key]);
    if (!pts.shoulder) return;

    line(pts.base.slice(0, 2), pts.shoulder.slice(0, 2), 7, "#33586b");   // 底座立柱
    const seg = (a, b) => pts[a] && pts[b] && line(pts[a].slice(0, 2), pts[b].slice(0, 2), 6, "#4fb3d9");
    seg("shoulder", "elbow");
    seg("elbow", "wrist");
    seg("wrist", "servo5");
    line(pts.servo5.slice(0, 2), pts.tip.slice(0, 2), 4, "#8fa5b4");      // 到末端的连杆
    drawGripper(pts, chain.grip_readback ?? null);

    const order = CHAIN.filter((k) => pts[k]).sort((a, b) => pts[b][2] - pts[a][2]);
    for (const key of order) {
      const [sx, sy] = pts[key];
      const tip = key === "tip";
      ctx.beginPath(); ctx.arc(sx, sy, tip ? 4.5 : 3.5, 0, 7);
      ctx.fillStyle = tip ? "#5fc48a" : "#e3edf3"; ctx.fill();
      if (LABELS[key]) {
        ctx.fillStyle = "#8fa5b4"; ctx.font = "11px sans-serif"; ctx.textAlign = "left";
        ctx.fillText(LABELS[key], sx + 7, sy + 3);
      }
    }
  }

  canvas.addEventListener("pointerdown", (e) => {
    drag = { x: e.clientX, y: e.clientY, yaw, pitch };
    canvas.setPointerCapture(e.pointerId);
  });
  canvas.addEventListener("pointermove", (e) => {
    if (!drag) return;
    yaw = drag.yaw + (e.clientX - drag.x) * 0.6;
    pitch = Math.max(-80, Math.min(80, drag.pitch + (e.clientY - drag.y) * 0.4));
    draw();
  });
  canvas.addEventListener("pointerup", () => { drag = null; });
  canvas.addEventListener("wheel", (e) => {
    e.preventDefault();
    zoom = Math.max(0.4, Math.min(3.5, zoom * (1 - e.deltaY * 0.0012)));
    draw();
  }, { passive: false });
  window.addEventListener("resize", draw);

  return {
    /**
     * @param {{base,shoulder,elbow,wrist,servo5,tip}|null} value 服务端给的骨架点，厘米
     * @param {{blocks:Array, block_side_cm:number, keep_out:object}|null} world /api/blocks 给的物块与禁入区
     */
    set(value, world = null) {
      const next = value && Object.keys(value).length ? value : null;
      const nextWorld = world && typeof world === "object" ? world : null;
      const changed = next !== chain || nextWorld !== scene;
      chain = next; scene = nextWorld;
      if (changed) draw();
    },
    draw,
  };
}
