// Shell for the simulation back end (dev/sim/sim_server.py). The arm is drawn from the course STL meshes (the
// parts the Blender scene is built from, thinned by dev/sim/web_meshes.py and served at /api/sim/meshes), each
// placed with the frames /api/sim/body gives: the URDF link chain and the four-bar gripper of planning.gripper
// at its present opening, so the fingers open and close as the grasp rule and the collision checks assume.
// Without the meshes (they come from 资料/, not the repository) the planner's point samples are drawn
// instead. The camera frustum still comes from the old console's kinematics function (same hand-eye).
import {geometry} from '/scene.js';

const $ = id => document.getElementById(id);
const owner = crypto.randomUUID();
const NAMES = {recyclable: '可回收物', kitchen: '厨余垃圾', hazardous: '有害垃圾', other: '其他垃圾'};
const COLOURS = {recyclable: '#3f7fd0', kitchen: '#4aa35a', hazardous: '#d0483f', other: '#d9b53a'};
const JOINTS = ['底座旋转', '肩部俯仰', '肘部俯仰', '腕部俯仰', '末端旋腕', '夹爪开合'];
const RESULT = {correct: ['正确', 'ok'], wrong_class: ['类别错', 'bad'], missed: ['漏检', 'bad'], ignored: ['不计（露出太少）', 'mid']};

let token = '', scene = null, state = null, world = null, body = null, meshes = null, beat = null, view = 'raw', worldRevision = -1;
fetch('/api/sim/meshes').then(r => r.ok ? r.json() : null).then(m => {
  if (!m) return;
  meshes = Object.entries(m.parts).map(([name, p]) => ({name, frame: p.frame, v: Float64Array.from(p.v, x => x / 1e4), f: Uint32Array.from(p.f)}));
  draw();
}).catch(() => {});
const MESH_COLOURS = {base_link: [70, 92, 108], link1: [205, 170, 70], link2: [214, 220, 226], link3: [214, 220, 226],
                      link4: [205, 170, 70], gripper_static: [60, 64, 70], gripper_finger_right: [232, 160, 60],
                      gripper_finger_left: [232, 160, 60]};
const PART_COLOURS = {link1: '#9db8c9', link2: '#7fc4e6', link3: '#5fa3cf', link4: '#a6dcf2',
                      gripper_static: '#c9a25a', gripper_links: '#d8b26a', gripper_fingers: '#f0b850'};
// 嵌入模式（?embed=scene）：被手动操作页借用 3D 视图时，不再拉相机流、日志流和分拣轮询，
// 那些面板在父页面已经有一份。控件本身照常初始化——setState 要更新关节读回，缺了会报 null。
const EMBED = new URLSearchParams(location.search).get('embed') === 'scene';

// ---------------------------------------------------------------- calls

const log = [];
async function call(method, path, body) {
  const started = performance.now();
  const init = method === 'GET' ? {} : {method, headers: {'Content-Type': 'application/json', 'X-Console-Token': token},
                                        body: JSON.stringify({owner, ...body})};
  const r = await fetch(path, init);
  const data = await r.json();
  log.unshift(`${r.status} ${method} ${path} ${Math.round(performance.now() - started)} ms`);
  log.length = Math.min(log.length, 40);
  $('apiLog').replaceChildren(...log.map(t => Object.assign(document.createElement('li'), {textContent: t})));
  if (!r.ok) throw Object.assign(Error(data.error || r.statusText), {data});
  return data;
}
const post = (path, body = {}) => call('POST', path, body);
const get = path => call('GET', path);

function fail(e) { $('error').textContent = e.message; $('error').hidden = false; }
// run a control action; a 409 leaves the server's revision unknown, so the status is fetched again
const act = fn => async () => {
  $('error').hidden = true;
  try { const out = await fn(); if (out && out.revision !== undefined) setState(out); }
  catch (e) { fail(e); try { setState(await get('/api/status')); } catch (_) { /* shown by the poll */ } }
};
const moving = body => body;   // revision is no longer required (doc 32 §1.2)

// ---------------------------------------------------------------- state

function setState(s) {
  state = s;
  const armed = s.armed;
  $('armed').textContent = armed ? '已使能' : '未使能'; $('armed').classList.toggle('on', armed);
  $('busy').textContent = s.busy ? '动作中' : '静止'; $('busy').classList.toggle('on', s.busy);
  $('holding').textContent = s.sim.holding ? `夹着 ${s.sim.holding}` : '夹爪空'; $('holding').classList.toggle('on', !!s.sim.holding);
  $('rev').textContent = `revision ${s.revision}`;
  if (s.error) fail(Error(s.error));
  s.pose.forEach((q, i) => { $(`q${i}`).textContent = q == null ? '—' : `${q}°`; });
  const cm = p => p.map(v => (v * 100).toFixed(1)).join(' / ');
  $('readout').innerHTML = s.fk ? `<span>夹爪尖 cm <b>${cm(s.fk.tip)}</b></span><span>俯角 <b>${s.fk.tip_pitch_deg[0]}°</b></span>
    <span>禁入区余量 <b>${s.clearance_cm.toFixed(1)} cm</b></span><span>速度 <b>${s.speed ? Math.round(s.speed.value) + '°/s' : '—'}</b></span>
    <span>夹爪读回 <b>${s.pose[5]}°${s.grip && s.grip.result ? ' · ' + s.grip.result : ''}</b></span>
    <span>画面 <b>#${s.camera.seq} · ${s.camera.age_s == null ? '—' : s.camera.age_s.toFixed(2) + ' s'}</b></span>` : '';
  if (s.sim.revision !== worldRevision) refreshWorld();
  draw();
}

async function refreshWorld() {
  [world, body] = await Promise.all([fetch('/api/sim/world').then(r => r.json()),
                                     fetch('/api/sim/body').then(r => r.json()).catch(() => null)]);
  worldRevision = world.revision;
  if (scene && world.map.name !== scene.map.name) scene = (await (await fetch('/api/bootstrap')).json()).scene;
  $('mapSelect').value = world.map.name ?? 'none';
  $('props').tBodies[0].replaceChildren(...world.props.map(p => {
    const tr = document.createElement('tr');
    tr.innerHTML = `<td>${p.id}</td><td><span style="color:${p.colour}">■</span> ${p.name || (p.kind === 'bin' ? '垃圾桶' : '盒体')}</td>
      <td>${p.category ? NAMES[p.category] : '—'}</td><td>${p.map_cm[0].toFixed(1)}, ${p.map_cm[1].toFixed(1)}</td><td>${p.yaw_deg}°</td>
      <td>${p.kind === 'bin' ? (p.holds.join(' ') || '空') : '—'}</td><td></td>`;
    const remove = Object.assign(document.createElement('button'), {textContent: '移除'});
    remove.onclick = act(() => post('/api/sim/prop/remove', {id: p.id}).then(refreshWorld));
    tr.lastElementChild.append(remove);
    return tr;
  }));
  $('blocks').tBodies[0].replaceChildren(...world.blocks.map(b => {
    const tr = document.createElement('tr');
    tr.innerHTML = `<td>${b.id}${b.held ? ' ✊' : ''}</td><td style="color:${COLOURS[b.category]}">${NAMES[b.category]}</td><td>${b.up}</td>
      <td>${b.map_cm[0].toFixed(1)}, ${b.map_cm[1].toFixed(1)}${b.inside ? ' · 在 ' + b.inside + ' 里' : b.on_map ? '' : ' · 图外'}</td><td>${b.base_cm[2].toFixed(1)}</td><td>${b.yaw_deg}°</td><td></td>`;
    const remove = Object.assign(document.createElement('button'), {textContent: '移除'});
    remove.onclick = act(() => post('/api/sim/remove', {id: b.id}).then(refreshWorld));
    tr.lastElementChild.append(remove);
    return tr;
  }));
  if (!$('newCategory').options.length) {
    $('newCategory').replaceChildren(...Object.keys(NAMES).map(c => new Option(NAMES[c], c)));
    $('newCategory').onchange = () => $('newUp').replaceChildren(...world.faces[$('newCategory').value].map(f => new Option('朝上：' + f, f)));
    $('newCategory').onchange();
  }
  draw();
}

// ---------------------------------------------------------------- 3D view (software z-buffer on a 2D canvas)
//
// The view used to sort whole polygons by their centroid depth (painter's algorithm). That cannot get big faces
// right -- the platform lid is one polygon, a block next to it is six small ones -- so blocks showed through the
// platform and the arm. Now every primitive is rasterised into a depth buffer: opaque faces write depth,
// outlines are tested against it (drawn after, a hair toward the viewer), translucent faces and the camera
// frustum are blended far-to-near without writing depth, and only the pad-centre marker sits on top.
// The projection is orthographic, so depth is linear in screen space and interpolates exactly.

const COLOURS_RGBA = new Map();
function rgba(css) {
  let c = COLOURS_RGBA.get(css);
  if (c) return c;
  let m;
  if (css[0] === '#') {
    const h = css.slice(1), two = h.length >= 6;
    const at = i => two ? parseInt(h.slice(2 * i, 2 * i + 2), 16) : parseInt(h[i] + h[i], 16);
    c = [at(0), at(1), at(2), (two ? h.length === 8 : h.length === 4) ? at(3) / 255 : 1];
  } else if ((m = css.match(/rgba?\(([^)]+)\)/))) {
    const v = m[1].split(',').map(Number);
    c = [v[0], v[1], v[2], v[3] ?? 1];
  } else {
    c = [255, 0, 255, 1];
  }
  COLOURS_RGBA.set(css, c);
  return c;
}

class ZBuffer {
  constructor(W, H, bg) {
    this.W = W; this.H = H;
    this.z = new Float32Array(W * H).fill(-Infinity);
    this.img = new ImageData(W, H);
    const px = this.img.data, [r, g, b] = rgba(bg);
    for (let i = 0; i < px.length; i += 4) { px[i] = r; px[i + 1] = g; px[i + 2] = b; px[i + 3] = 255; }
  }
  // depth: larger = nearer the viewer (P's third coordinate)
  tri(ax, ay, az, bx, by, bz, cx, cy, cz, col, write) {
    const area = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
    if (Math.abs(area) < 1e-6) return;
    const W = this.W, H = this.H, zb = this.z, px = this.img.data;
    const x0 = Math.max(0, Math.floor(Math.min(ax, bx, cx))), x1 = Math.min(W - 1, Math.ceil(Math.max(ax, bx, cx)));
    const y0 = Math.max(0, Math.floor(Math.min(ay, by, cy))), y1 = Math.min(H - 1, Math.ceil(Math.max(ay, by, cy)));
    const [r, g, b, a] = col, inv = 1 / area, eps = -1e-7, opaque = a >= 0.999;
    for (let y = y0; y <= y1; y++) {
      const py = y + 0.5;
      for (let x = x0; x <= x1; x++) {
        const qx = x + 0.5;
        const w0 = ((bx - qx) * (cy - py) - (by - py) * (cx - qx)) * inv;
        if (w0 < eps) continue;
        const w1 = ((cx - qx) * (ay - py) - (cy - py) * (ax - qx)) * inv;
        if (w1 < eps) continue;
        const w2 = 1 - w0 - w1;
        if (w2 < eps) continue;
        const z = w0 * az + w1 * bz + w2 * cz, i = y * W + x;
        if (z <= zb[i]) continue;
        const o = 4 * i;
        if (opaque) { px[o] = r; px[o + 1] = g; px[o + 2] = b; }
        else { px[o] += (r - px[o]) * a; px[o + 1] += (g - px[o + 1]) * a; px[o + 2] += (b - px[o + 2]) * a; }
        if (write) zb[i] = z;
      }
    }
  }
  poly(q, col, write) {               // convex polygon, fan from the first vertex
    for (let i = 1; i + 1 < q.length; i++)
      this.tri(q[0][0], q[0][1], q[0][2], q[i][0], q[i][1], q[i][2], q[i + 1][0], q[i + 1][1], q[i + 1][2], col, write);
  }
  line(p, q, width, col, write) {      // a flat strip facing the viewer, depth along it
    const dx = q[0] - p[0], dy = q[1] - p[1], L = Math.hypot(dx, dy) || 1, hw = Math.max(0.5, width / 2);
    const nx = -dy / L * hw, ny = dx / L * hw, ex = dx / L * hw * 0.5, ey = dy / L * hw * 0.5;
    const a = [p[0] + nx - ex, p[1] + ny - ey, p[2]], b = [p[0] - nx - ex, p[1] - ny - ey, p[2]];
    const c = [q[0] - nx + ex, q[1] - ny + ey, q[2]], d = [q[0] + nx + ex, q[1] + ny + ey, q[2]];
    this.poly([a, b, c, d], col, write);
  }
}

const cam = {yaw: -0.7, pitch: 0.5, scale: 1500};
function draw() {
  if (!scene || !state) return;
  const canvas = $('scene'), ratio = devicePixelRatio || 1, w = canvas.clientWidth, h = canvas.clientHeight;
  canvas.width = w * ratio; canvas.height = h * ratio;
  const ctx = canvas.getContext('2d'); ctx.scale(ratio, ratio);
  const cy = Math.cos(cam.yaw), sy = Math.sin(cam.yaw), cp = Math.cos(cam.pitch), sp = Math.sin(cam.pitch);
  const k = cam.scale * Math.min(1, w / 620);
  const P = p => {                     // -> [screen x, screen y, nearness]
    const x = p[0], y = p[1] - 0.15, z = p[2] - 0.05, xr = cy * x - sy * y, yr = sy * x + cy * y;
    return [w / 2 + xr * k, h * 0.55 - (z * cp + yr * sp) * k, z * sp - yr * cp];
  };
  // primitives, rasterised at the end. bias only separates coplanar layers (table < page < markers): it is
  // scaled to a tenth of a millimetre per unit, far below any real gap between surfaces
  const BIAS = 1e-4, EDGE = 1.5e-3;   // outlines sit 1.5 mm toward the viewer so they win over their own face
  const solid = [], edges = [], glass = [], overlay = [];
  const poly = (pts, fill, stroke, bias = 0) => {
    const q = pts.map(P).map(v => [v[0], v[1], v[2] + bias * BIAS]);
    if (fill) (rgba(fill)[3] < 0.999 ? glass : solid).push({q, col: rgba(fill)});
    if (stroke) q.forEach((v, i) => edges.push({p: v, q: q[(i + 1) % q.length], width: 1, col: rgba(stroke)}));
  };
  const line = (a, b, colour, width, bias = 0) => {
    const p = P(a), q = P(b), col = rgba(colour);
    p[2] += bias * BIAS; q[2] += bias * BIAS;
    (col[3] < 0.999 ? glass : solid).push({p, q, width, col});
  };

  const map = scene.map, off = map.placement.base_from_map, z0 = off.dz;
  const onTable = (x, y) => [x + off.dx, y + off.dy, z0];
  poly([[-0.3, -0.12, z0], [0.3, -0.12, z0], [0.3, 0.42, z0], [-0.3, 0.42, z0]], '#141f28', '#22323f', -10);
  if (map.name) {
    const [pw, ph] = (map.page_cm ?? [29.7, 21]).map(v => v / 100);
    poly([onTable(-pw / 2, 0), onTable(pw / 2, 0), onTable(pw / 2, ph), onTable(-pw / 2, ph)], '#c9d0d4', null, -9);
    for (const corners of Object.values(map.layout_m)) poly(corners.map(c => onTable(c[0], c[1])), '#20262b', null, -8);
  }
  // platform top: the lid model (platform_min_z_m is -1 since the platform keep-out only warns, doc 32 §6)
  const ko = state.keep_out, r = ko.platform_radius_m, top = ko.platform_square?.top_z_m ?? (ko.platform_min_z_m > 0 ? ko.platform_min_z_m : 0.076), ring = [...Array(24)].map((_, i) => i * Math.PI / 12);
  // the platform: the rounded-square lid of app/config/platform_model.json (as in the Blender scene), else a disc
  const sq = ko.platform_square;
  if (sq && sq.outline) {
    const o = sq.outline, cr = o.corner_radius_m, hw = o.half_width_m, corners = [[hw - cr, o.front_y_m - cr, 0], [-hw + cr, o.front_y_m - cr, 90],
                                                                             [-hw + cr, o.back_y_m + cr, 180], [hw - cr, o.back_y_m + cr, 270]];
    const rim = corners.flatMap(([cx, cy, a0]) => [...Array(7)].map((_, i) => { const a = (a0 + i * 15) * Math.PI / 180; return [cx + cr * Math.cos(a), cy + cr * Math.sin(a)]; }));
    rim.forEach((p, i) => { const q = rim[(i + 1) % rim.length];
      poly([[p[0], p[1], z0], [q[0], q[1], z0], [q[0], q[1], top], [p[0], p[1], top]], '#d9dde0', '#b9bfc4', -0.6); });
    poly(rim.map(p => [p[0], p[1], top]), '#23272b', '#3a4046', -0.5);   // drawn before the arm, which stands on it
  } else {
    poly(ring.map(a => [r * Math.cos(a), r * Math.sin(a), z0]), '#1d2c38', '#3b5466', -7);
  }

  const CORNERS = [[[1, -1, -1], [1, 1, -1], [1, 1, 1], [1, -1, 1]], [[-1, -1, -1], [-1, 1, -1], [-1, 1, 1], [-1, -1, 1]],
                   [[-1, 1, -1], [1, 1, -1], [1, 1, 1], [-1, 1, 1]], [[-1, -1, -1], [1, -1, -1], [1, -1, 1], [-1, -1, 1]],
                   [[-1, -1, 1], [1, -1, 1], [1, 1, 1], [-1, 1, 1]], [[-1, -1, -1], [1, -1, -1], [1, 1, -1], [-1, 1, -1]]];
  for (const p of world ? world.props : []) {
    const T = p.base_T_prop;
    for (const part of p.parts) {
      const at = c => [0, 1, 2].map(i => T[i][3] + [0, 1, 2].reduce((s, j) => s + T[i][j] * (part.centre_cm[j] + c[j] * part.size_cm[j] / 2) / 100, 0));
      CORNERS.forEach(f => poly(f.map(at), p.colour + 'b0', p.colour));
    }
  }
  for (const b of world ? world.blocks : []) {
    const T = b.base_T_block, hs = world.block_side_cm / 200;
    const at = (x, y, z) => [0, 1, 2].map(i => T[i][0] * x * hs + T[i][1] * y * hs + T[i][2] * z * hs + T[i][3]);
    const faces = [[[1, -1, -1], [1, 1, -1], [1, 1, 1], [1, -1, 1]], [[-1, -1, -1], [-1, 1, -1], [-1, 1, 1], [-1, -1, 1]],
                   [[-1, 1, -1], [1, 1, -1], [1, 1, 1], [-1, 1, 1]], [[-1, -1, -1], [1, -1, -1], [1, -1, 1], [-1, -1, 1]],
                   [[-1, -1, 1], [1, -1, 1], [1, 1, 1], [-1, 1, 1]], [[-1, -1, -1], [1, -1, -1], [1, 1, -1], [-1, 1, -1]]];
    // the face lying upward in the block's own frame carries its colour; the stickered faces are pale
    faces.forEach((f, i) => poly(f.map(c => at(...c)), i >= 4 ? COLOURS[b.category] : '#e9e4d8', b.held ? '#ffffff' : '#30404c'));
  }

  const g = geometry(state.pose.map((v, i) => v ?? [90, 105, 5, 5, 95, 30][i]), scene.hand_eye.cam2gripper);
  if (!(meshes && ko.platform_square)) line([0, 0, z0], [0, 0, top], '#506a7d', 26);
  if (meshes && body && body.frames) {
    // shaded triangles, back faces dropped; toward-viewer direction of the projection below
    const toward = [-cp * sy, -cp * cy, sp], L = [0.35, -0.45, 0.82], ln = Math.hypot(...L);
    for (const part of meshes) {
      const T = body.frames[part.frame];
      if (!T) continue;
      const v = part.v, n = v.length / 3, wv = new Float64Array(v.length), scr = new Array(n);
      for (let i = 0; i < n; i++) {
        const x = v[3 * i], y = v[3 * i + 1], z = v[3 * i + 2];
        for (let r = 0; r < 3; r++) wv[3 * i + r] = T[r][0] * x + T[r][1] * y + T[r][2] * z + T[r][3];
        scr[i] = P([wv[3 * i], wv[3 * i + 1], wv[3 * i + 2]]);
      }
      const base = MESH_COLOURS[part.name] ?? [150, 160, 170], f = part.f;
      for (let t = 0; t < f.length; t += 3) {
        const a = f[t] * 3, b = f[t + 1] * 3, c = f[t + 2] * 3;
        const u = [wv[b] - wv[a], wv[b + 1] - wv[a + 1], wv[b + 2] - wv[a + 2]];
        const w2 = [wv[c] - wv[a], wv[c + 1] - wv[a + 1], wv[c + 2] - wv[a + 2]];
        const nx = u[1] * w2[2] - u[2] * w2[1], ny = u[2] * w2[0] - u[0] * w2[2], nz = u[0] * w2[1] - u[1] * w2[0];
        const nl = Math.hypot(nx, ny, nz) || 1;
        if (nx * toward[0] + ny * toward[1] + nz * toward[2] < 0) continue;
        const lit = 0.38 + 0.62 * Math.max(0, (nx * L[0] + ny * L[1] + nz * L[2]) / (nl * ln));
        const fill = `rgb(${base.map(x => Math.round(x * lit)).join(',')})`;
        solid.push({q: [scr[f[t]], scr[f[t + 1]], scr[f[t + 2]]], col: rgba(fill)});
      }
    }
    const q = P(body.pad_centre);
    overlay.push(() => { ctx.beginPath(); ctx.arc(q[0], q[1], 3, 0, 7); ctx.fillStyle = '#ff6b6b'; ctx.fill(); });
  } else if (body) {
    // model samples as dots, each depth-sorted with the blocks and bins; size follows the zoom
    const dot = Math.max(1.6, k / 900);
    for (const [part, pts] of Object.entries(body.parts)) {
      const colour = PART_COLOURS[part] ?? '#9fb3bf';
      for (const p of pts) {
        const q = P(p), h = dot / 2;
        solid.push({q: [[q[0] - h, q[1] - h, q[2]], [q[0] + h, q[1] - h, q[2]], [q[0] + h, q[1] + h, q[2]], [q[0] - h, q[1] + h, q[2]]], col: rgba(colour)});
      }
    }
    // pad centre and the jaw axis through it (the grasp rule's reference)
    const pc = body.pad_centre, ax = body.jaw_axis.map(v => v * body.gap_mm / 2000);
    line(pc.map((v, i) => v - ax[i]), pc.map((v, i) => v + ax[i]), '#ff6b6b', 2, 0.05);
    const q = P(pc);
    overlay.push(() => { ctx.beginPath(); ctx.arc(q[0], q[1], 3.5, 0, 7); ctx.fillStyle = '#ff6b6b'; ctx.fill(); });
  } else {
    // no body model from this back end: the old stick figure
    line([0, 0, top], g.points[1], '#7fc4e6', 12, 0.02);
    g.points.slice(1).forEach((p, i) => { if (i) line(g.points[i], p, i === 4 ? '#e0a84a' : '#7fc4e6', i === 4 ? 5 : 9, 0.02); });
  }
  // camera frustum down to the table
  const K = scene.intrinsics.camera_matrix, f = K[0][0];
  const hit = [[0, 0], [640, 0], [640, 480], [0, 480]].map(([u, v]) => {
    const c = [(u - K[0][2]) / f, (v - K[1][2]) / f, 1], d = [0, 1, 2].map(i => g.camAxes[0][i] * c[0] + g.camAxes[1][i] * c[1] + g.camAxes[2][i] * c[2]);
    const t = d[2] < -1e-3 ? Math.min((z0 - g.cam[2]) / d[2], 0.6) : 0.25;
    return g.cam.map((p, i) => p + d[i] * t);
  });
  hit.forEach((p, i) => { line(g.cam, p, 'rgba(95,196,138,.55)', 1, 0.05); line(p, hit[(i + 1) % 4], 'rgba(95,196,138,.8)', 1.5, 0.05); });

  // rasterise at 2x (or the screen's ratio, if higher) and let drawImage smooth it down: cheap anti-aliasing
  const ss = Math.max(2, ratio), W = Math.max(1, Math.round(w * ss)), H = Math.max(1, Math.round(h * ss));
  const zb = new ZBuffer(W, H, '#0b141b');
  const S = v => [v[0] * ss, v[1] * ss, v[2]];
  // a line item has p (and its end in q); a face has only q, its corners
  for (const it of solid) it.p ? zb.line(S(it.p), S(it.q), it.width * ss, it.col, true) : zb.poly(it.q.map(S), it.col, true);
  for (const e of edges) zb.line(S([e.p[0], e.p[1], e.p[2] + EDGE]), S([e.q[0], e.q[1], e.q[2] + EDGE]), e.width * ss, e.col, false);
  const mid = it => it.p ? (it.p[2] + it.q[2]) / 2 : it.q.reduce((s, v) => s + v[2], 0) / it.q.length;
  glass.sort((a, b) => mid(a) - mid(b));
  for (const it of glass) it.p ? zb.line(S(it.p), S(it.q), it.width * ss, it.col, false) : zb.poly(it.q.map(S), it.col, false);
  const buf = document.createElement('canvas');
  buf.width = W; buf.height = H;
  buf.getContext('2d').putImageData(zb.img, 0, 0);
  ctx.imageSmoothingEnabled = true; ctx.imageSmoothingQuality = 'high';
  ctx.drawImage(buf, 0, 0, w, h);
  overlay.forEach(f => f());
  ctx.fillStyle = '#8aa0ad'; ctx.font = '12px system-ui, sans-serif';
  ctx.fillText(meshes && body ? `课程 STL 网格（抽稀）· 夹爪四连杆开口 ${body.gap_mm.toFixed(0)} mm · 红点：指垫中心`
               : body ? `规划用的采样点（没有网格：缺 dev/sim/assets/web_meshes.json）· 夹爪开口 ${body.gap_mm.toFixed(0)} mm · 红点：指垫中心与爪轴`
               : '旧骨架（后端没有 /api/sim/body）', 10, h - 10);
}
{
  const canvas = $('scene'); let drag = null;
  canvas.onpointerdown = e => { drag = [e.clientX, e.clientY]; canvas.setPointerCapture(e.pointerId); };
  canvas.onpointermove = e => { if (!drag) return; cam.yaw += (e.clientX - drag[0]) * 0.008; cam.pitch = Math.max(0.05, Math.min(1.5, cam.pitch + (e.clientY - drag[1]) * 0.006)); drag = [e.clientX, e.clientY]; draw(); };
  canvas.onpointerup = canvas.onpointercancel = () => { drag = null; };
  canvas.onwheel = e => { e.preventDefault(); cam.scale = Math.max(500, Math.min(4000, cam.scale * Math.exp(-e.deltaY * 0.001))); draw(); };
  new ResizeObserver(draw).observe(canvas);
}

// ---------------------------------------------------------------- camera panel

const HINTS = {
  raw: '/stream.mjpg：按当前关节角和桌面物块合成的 640×480 画面，带镜头畸变、阴影和噪声。',
  detect: '/api/sim/annotated：细白框是真值，绿框检测类别正确，橙框类别错，红框是误检。',
  locate: '/api/annotated/board：旧控制台的地图定位（地图码 + 颜色块）跑在合成画面上，与实机同一段代码。',
};
async function refreshView() {
  if (view === 'raw') return;
  const bust = `?t=${Date.now()}`;
  if (view === 'locate') { $('frame').src = '/api/annotated/board' + bust; return; }
  try {
    const d = await (await fetch('/api/sim/detect')).json();
    if (d.error) throw Error(d.error);
    $('frame').src = '/api/sim/annotated' + bust;
    $('detectTable').tBodies[0].replaceChildren(...d.blocks.map(b => {
      const tr = document.createElement('tr'), [text, cls] = RESULT[b.result];
      tr.innerHTML = `<td>${b.id}</td><td>${NAMES[b.category]}</td><td>${b.detection ? NAMES[b.detection.category] : '—'}</td>
        <td>${b.detection ? b.detection.conf.toFixed(2) : '—'}</td><td>${b.iou ?? '—'}</td><td class="${cls}">${text}</td>`;
      return tr;
    }), ...d.false_positives.map(f => {
      const tr = document.createElement('tr');
      tr.innerHTML = `<td>—</td><td>—</td><td>${NAMES[f.category]}</td><td>${f.conf.toFixed(2)}</td><td>—</td><td class="bad">误检</td>`;
      return tr;
    }));
    $('viewHint').textContent = `${HINTS.detect} 模型 ${d.model}，阈值 ${d.conf}，IoU ≥ ${d.match_iou} 算匹配。`;
  } catch (e) { $('viewHint').textContent = '检测不可用：' + e.message; }
}
function setView(name) {
  view = name;
  document.querySelectorAll('#viewTabs button').forEach(b => b.classList.toggle('on', b.dataset.view === name));
  $('detectTable').hidden = name !== 'detect';
  $('viewHint').textContent = HINTS[name];
  if (name === 'raw') $('frame').src = '/stream.mjpg'; else refreshView();
}
document.querySelectorAll('#viewTabs button').forEach(b => { b.onclick = () => setView(b.dataset.view); });

// ---------------------------------------------------------------- controls

// no enable step (doc 32 §1.2): the first move takes control; heartbeats keep it while this page is visible
if (!beat) beat = setInterval(() => { if (document.visibilityState === 'visible' && state?.armed) post('/api/heartbeat').catch(() => {}); }, 600);
$('speed').oninput = () => { $('speedVal').textContent = `${$('speed').value}°/s`; };
$('speed').onchange = act(() => post('/api/speed', {speed: Number($('speed').value)}));
$('stop').onclick = act(() => post('/api/stop'));
$('home').onclick = act(() => post('/api/goto', {named: 'observe_v1'}));
document.querySelectorAll('[data-grip]').forEach(b => { b.onclick = act(() => post('/api/gripper', moving({action: b.dataset.grip}))); });
document.querySelectorAll('[data-cart]').forEach(b => { b.onclick = act(() => post('/api/cartesian', moving({axis: b.dataset.cart, amount: Number(b.dataset.amount)}))); });
$('reset').onclick = act(() => post('/api/sim/reset', {count: Number($('resetCount').value)}).then(refreshWorld));
$('clear').onclick = act(() => post('/api/sim/reset', {count: 0}).then(refreshWorld));
$('mapSelect').onchange = act(() => post('/api/sim/map', {name: $('mapSelect').value}).then(refreshWorld));
$('loadBins').onclick = act(() => post('/api/sim/bins', {name: $('binsSelect').value}).then(refreshWorld));
$('addBin').onclick = act(() => post('/api/sim/prop', {kind: 'bin', category: $('binCategory').value, map_cm: [Number($('binX').value), Number($('binY').value)],
                                                        yaw_deg: Number($('binYaw').value), inner_cm: [Number($('binSize').value), Number($('binSize').value)],
                                                        height_cm: Number($('binHeight').value)}).then(refreshWorld));
$('addBlock').onclick = act(() => post('/api/sim/block', {category: $('newCategory').value, map_cm: [Number($('newX').value), Number($('newY').value)],
                                                          yaw_deg: Number($('newYaw').value), up: $('newUp').value}).then(refreshWorld));
function buildJoints(limits) {
  $('joints').replaceChildren(...JOINTS.map((name, i) => {
    const row = Object.assign(document.createElement('div'), {className: 'joint'});
    row.innerHTML = `<div>J${i + 1} · ${name}<small>${limits[i + 1].join(' – ')}°</small></div><strong id="q${i}">—</strong><div class="row"></div>`;
    for (const delta of [-10, -5, -1, 1, 5, 10]) {
      const b = Object.assign(document.createElement('button'), {textContent: (delta > 0 ? '+' : '') + delta});
      b.onclick = act(() => post('/api/jog', moving({joint: i + 1, delta})));
      row.lastElementChild.append(b);
    }
    return row;
  }));
}

// ---------------------------------------------------------------- every endpoint, callable

const R = '"revision": $REV';   // replaced by the current revision when the entry is picked
const API = [
  ['状态与画面'],
  ['GET', '/api/bootstrap', '', 'token、静态场景（手眼、内参、禁入区、地图）、首个 status'],
  ['GET', '/api/status', '', '轮询用；本服务多一个 sim 字段，localization.board 多一个 blocks4 字段'],
  ['GET', '/api/export', '', '本次会话的采样、标定、测量'],
  ['IMG', '/api/frame', '', '最新一帧 JPEG'],
  ['IMG', '/snapshot.jpg', '', '同上，带 X-Frame-Time / X-Frame-Seq 头（仿板载相机服务）'],
  ['IMG', '/stream.mjpg', '', 'MJPEG 流，<img src> 可直接显示'],
  ['GET', '/health', '', '仿板载相机服务的健康检查'],
  ['IMG', '/api/annotated/board', '', '地图定位标注图；没有定位结果时 503'],
  ['操作权'],
  ['POST', '/api/takeover', '{}', '先停止当前持有者再取得操作权；平时不需要：第一条运动指令即取得'],
  ['POST', '/api/heartbeat', '{}', '2 秒租约续期；本页持有操作权时每 600 ms 自动发'],
  ['POST', '/api/stop', '{}', '撤销操作权（任何端都可发）'],
  ['POST', '/api/speed', '{"speed": 20}', '速度 °/s（3–40），运动中改从下一段生效'],
  ['运动'],
  ['POST', '/api/jog', '{"joint": 1, "delta": 5}', '关节点动，±1–30° 整数'],
  ['POST', '/api/cartesian', '{"axis": "z", "amount": 1}', '夹爪尖沿基座轴 0–5 cm，或 pitch 0–15°'],
  ['POST', '/api/gripper', '{"action": "grip"}', 'wide / open / grip；夹爪尖在物块上方且够低时会夹起它'],
  ['POST', '/api/goto', '{"named": "observe_v1", "dry": true}', '到命名位或 "pose": [J1..J6]（null 不动）；dry=true 只返回分段与余量'],
  ['POST', '/api/goto', '{"pose": [90, 80, 30, 20, 95, null]}', '整条路径先检查再动'],
  ['POST', '/api/drive', '{"mode": "cartesian", "v": [0, 0, 0.5, 0], "wrist": 0}', '摇杆：每 100 ms 刷新一次，300 ms 不刷新即停；v=[径向, J1, 上下, 俯仰] 或 joint 模式 [J1..J4]'],
  ['仿真场景与检测对照'],
  ['GET', '/api/sim/world', '', '桌面真值：每块的类别、朝上的面、位置、是否被夹着、base_T_block；contacts 碰桶、block_contacts 碰块、platform_contacts 压进底座盖板'],
  ['GET', '/api/sim/body', '', '整臂模型（规划用的同一套）：各部件位姿 frames、各连杆点云、四连杆夹爪、指垫中心、爪轴、开口 mm'],
  ['GET', '/api/sim/meshes', '', '课程 STL 抽稀后的网格（dev/sim/web_meshes.py 生成，不入库）；配合 frames 画整臂'],
  ['POST', '/api/sim/reset', '{"count": 4, "seed": 1}', '清空后随机放 count 块；seed 可省'],
  ['POST', '/api/sim/block', '{"category": "hazardous", "map_cm": [0, 10], "yaw_deg": 15, "up": "syringe"}', '放一块；带 "id" 则移动已有的块'],
  ['POST', '/api/sim/remove', '{"id": "b1"}', '拿走一块'],
  ['GET', '/api/sim/options', '', '可选的地图、桶设计文件、类别和各类物块的面'],
  ['POST', '/api/sim/map', '{"name": "map_a4_v3"}', '换桌面地图：app/config/ 下的 map_*.json 名，或 "none" 不铺地图'],
  ['POST', '/api/sim/prop', '{"kind": "bin", "category": "kitchen", "map_cm": [-12, 16], "yaw_deg": 0, "inner_cm": [8, 8], "height_cm": 4, "wall_cm": 0.2}', '放一个垃圾桶；或 {"kind": "box", "size_cm": [5, 5, 3], "colour": "#888888", ...} 放实心盒'],
  ['POST', '/api/sim/prop/remove', '{"id": "p1"}', '拿走一个物体'],
  ['POST', '/api/sim/bins', '{"name": "bins_acrylic_v1"}', '按桶的设计文件，在规划位置立起全部桶'],
  ['GET', '/api/sim/detect', '', '四类检测模型在当前画面上的结果与真值逐块对照；可加 ?conf=0.4'],
  ['IMG', '/api/sim/annotated', '', '对照图'],
  ['分拣日志与任务'],
  ['GET', '/api/log?after=0&limit=5', '', '日志事件，每条带 seq（行号）；用返回的 next 续读'],
  ['GET', '/api/log/records', '', '逐块记录：时间、类别、来源、目标位、耗时、结果'],
  ['GET', '/api/log/summary', '', '统计：各结果计数、各类别计数、轮数、耗时'],
  ['GET', '/api/scene', '', '最近一次扫描的 scene.json'],
  ['IMG', '/api/scene/image', '', '最近一次扫描的重建图'],
  ['GET', '/api/sim/sort', '', '分拣任务状态'],
  ['POST', '/api/sim/sort', '{"rounds": 2, "blocks": 4}', '用真实分拣流水线在仿真里跑若干轮；需先 /api/stop 释放操作权'],
  ['POST', '/api/sim/sort/stop', '{}', '终止分拣任务'],
  ['POST', '/api/sim/faults', '{"grip_miss": 0.3}', '注入故障：合爪夹空的比例'],
  ['语音与手势指令（识别结果 -> 指令，见 docs 25）'],
  ['GET', '/api/interact', '', '武装状态、指令词表、手势映射与阈值、最近的识别记录（含被忽略/拒绝的，带原因）'],
  ['POST', '/api/interact/arm', '{"armed": true}', '武装/解除；未武装时只有“停止”会执行'],
  ['POST', '/api/interact/voice', '{"text": "启动"}', '一句识别文本；命中指令词才执行。停止/启动/跳舞（跳舞返回 rejected，预设动作未接入）'],
  ['POST', '/api/interact/gesture', '{"label": "palm", "confidence": 0.9}', '一帧手势分类；连续 3 帧 ≥0.6 才触发，palm 绑定停止，其余手势暂未绑定'],
  ['记录与标定'],
  ['POST', '/api/sample', '{"kind": "note", "label": "", "note": ""}', '采样：状态 + 画面存入会话目录'],
  ['POST', '/api/record', '{"on": true, "label": "trial", "note": ""}', '开始/结束录制'],
  ['POST', '/api/verify', '{"kind": "grasped", "note": ""}', '人工确认记录：grasped / released / boundary_clear / platform_contact / table_contact'],
  ['POST', '/api/freeze', '{}', '冻结当前画面用于标定；返回 frozen.id'],
  ['IMG', '/api/frozen', '', '最近一次冻结的画面'],
  ['POST', '/api/calibrate', '{"frame_id": "", "plane": "table", "points": [[100, 100, -5, 15], [540, 100, 5, 15], [540, 380, 5, 8], [100, 380, -5, 8]]}', '冻结帧上的平面单应标定：[u, v, x_cm, y_cm] 至少 4 点；frame_id 填冻结返回的 id'],
  ['POST', '/api/measure', '{"frame_id": "", "pixels": [[320, 240]]}', '冻结帧上像素 → 平面坐标，或两点距离'],
  ['POST', '/api/localization/save', '{}', '保存当前定位结果'],
];
let picked = null;
function pick(entry, button) {
  picked = entry;
  document.querySelectorAll('#apiList button').forEach(b => b.classList.toggle('on', b === button));
  const [method, path, body, note] = entry;
  $('apiLine').textContent = `${method === 'IMG' ? 'GET' : method} ${path}`;
  $('apiBody').value = body.replace('$REV', state ? state.revision : 0);
  $('apiBody').hidden = method !== 'POST';
  $('apiNote').textContent = note;
}
$('apiList').replaceChildren(...API.map(entry => {
  if (entry.length === 1) return Object.assign(document.createElement('h3'), {textContent: entry[0]});
  const b = document.createElement('button');
  b.innerHTML = `<span class="m">${entry[0] === 'IMG' ? 'GET' : entry[0]}</span>${entry[1]}`;
  b.onclick = () => pick(entry, b);
  return b;
}));
$('apiSend').onclick = async () => {
  if (!picked) return;
  const [method, path] = picked, out = $('apiResult');
  try {
    if (method === 'IMG') {
      if (path === '/stream.mjpg') { out.textContent = '这是连续流，见右上“原始画面”。'; return; }
      const r = await fetch(path);
      if (!r.ok) throw Error(`${r.status} ${(await r.json()).error}`);
      const head = [...r.headers].filter(([k]) => k.startsWith('x-frame') || k === 'content-type').map(([k, v]) => `${k}: ${v}`).join('\n');
      out.replaceChildren(head + '\n', Object.assign(new Image(), {src: URL.createObjectURL(await r.blob()), style: 'max-width:100%;margin-top:6px'}));
      return;
    }
    const data = method === 'GET' ? await get(path) : await post(path, JSON.parse($('apiBody').value || '{}'));
    out.textContent = JSON.stringify(data, null, 1);
    if (data.revision !== undefined && data.pose) setState(data); else setState(await get('/api/status'));
    if (path.startsWith('/api/sim/')) refreshWorld();
  } catch (e) {
    out.textContent = '错误：' + e.message;
    try { setState(await get('/api/status')); } catch (_) { /* shown by the poll */ }
  }
};

// ---------------------------------------------------------------- sorting log (docs/current/18-日志契约.md)

const RECORD = {pipeline_sorted: ['已分拣', 'ok'], pipeline_missed: ['夹空/滑脱', 'bad'], pipeline_skip: ['跳过', 'mid'],
                pipeline_approach_failed: ['接近失败', 'bad'], pipeline_put_failed: ['投放失败', 'bad'], pipeline_cleared: ['挪开', 'mid']};
let round = 0, lines = 0;
function onLogEvent(e) {
  lines = e.seq;
  $('logCount').textContent = `${lines} 行`;
  if (e.event === 'pipeline_start') round += 1;
  const li = document.createElement('li');
  li.value = e.seq;
  li.textContent = `${new Date(e.t * 1000).toLocaleTimeString()} ${e.event}` + (e.reason ? ` · ${e.reason}` : '');
  if (RECORD[e.event]) li.className = 'rec';
  $('eventFeed').prepend(li);
  while ($('eventFeed').children.length > 80) $('eventFeed').lastElementChild.remove();
  if (RECORD[e.event]) {
    const [text, cls] = RECORD[e.event], tr = document.createElement('tr');
    tr.innerHTML = `<td>${e.seq}</td><td>${new Date(e.t * 1000).toLocaleTimeString()}</td><td>${round}</td><td>${NAMES[e.category] ?? '未知'}</td>
      <td>${e.cube[0].toFixed(1)}, ${e.cube[1].toFixed(1)}</td><td>${e.bin ?? '—'}</td><td>${e.seconds ?? '—'}</td><td class="${cls}">${text}</td><td></td>`;
    tr.lastElementChild.textContent = e.reason ?? '';
    $('records').tBodies[0].prepend(tr);
    while ($('records').tBodies[0].children.length > 14) $('records').tBodies[0].lastElementChild.remove();
  }
  if (e.event === 'pipeline_end' || e.event === 'pipeline_start') refreshSummary();
}
async function refreshSummary() {
  const s = await (await fetch('/api/log/summary')).json(), r = s.by_result;
  $('logSummary').textContent = `${s.rounds_finished} 轮 · 分拣 ${r.sorted} · 夹空 ${r.missed} · 跳过 ${r.skipped} · 失败 ${r.failed}`;
  $('sceneImage').src = `/api/scene/image?t=${Date.now()}`;
}
async function refreshSort() {
  const s = await (await fetch('/api/sim/sort')).json();
  $('sortState').textContent = s.running ? '分拣中' : s.exit_code == null ? '未运行' : s.exit_code === 0 ? '已完成' : `已退出（${s.exit_code}）：${s.output.at(-1) ?? ''}`;
  $('sortState').classList.toggle('on', s.running);
}
$('sortStart').onclick = act(async () => {
  if (state.armed) await post('/api/stop');
  await post('/api/sim/faults', {grip_miss: Number($('gripMiss').value)});
  await post('/api/sim/sort', {rounds: Number($('sortRounds').value)});
  refreshSort();
});
$('sortStop').onclick = act(() => post('/api/sim/sort/stop').then(refreshSort));

// ---------------------------------------------------------------- start

async function poll() {
  try {
    const s = await (await fetch('/api/status')).json();
    if (!state || s.revision !== state.revision || s.sim.revision !== worldRevision || s.busy !== state.busy || s.armed !== state.armed) setState(s);
    else { state = s; }
  } catch (e) { fail(Error('仿真服务不可达')); }
  setTimeout(poll, 400);
}
(async () => {
  try {
    const boot = await get('/api/bootstrap');
    token = boot.token; scene = boot.scene;
    const options = await (await fetch('/api/sim/options')).json();
    $('mapSelect').replaceChildren(...options.maps.map(m => new Option(m, m)), new Option('不铺地图', 'none'));
    $('binsSelect').replaceChildren(...options.bins.map(m => new Option(m, m)));
    $('binCategory').replaceChildren(...options.categories.map(c => new Option(NAMES[c] + '桶', c)));
    buildJoints(boot.status.limits);
    await refreshWorld();
    setState(boot.status);
    if (!EMBED) {
      setView('raw');
      pick(API[1], $('apiList').querySelector('button'));
      setInterval(refreshView, 1200);
      setInterval(refreshSort, 1500);
      refreshSummary();
      new EventSource('/api/log/stream').onmessage = m => onLogEvent(JSON.parse(m.data));
    }
    poll();
  } catch (e) { fail(e); }
})();
addEventListener('pagehide', () => { if (beat) navigator.sendBeacon && post('/api/stop').catch(() => {}); });
