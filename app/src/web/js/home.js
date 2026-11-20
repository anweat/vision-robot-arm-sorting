// 作业台（首页）：画面在中间——仿真三维 / 实机骨架 与 腕部相机；右侧是提示栏、操作面板与设备状态；
// 下方是正在跑的流程（脚本结构树里点亮当前一行）和本轮计数。日志、配置、脚本编辑都在别的页。

import { MODES, MODE_NAMES, GRIPPER, alarmName } from "./contract.js";
import { sendCmd, fetchFlow, fetchScripts, fetchScript } from "./api.js";
import { toast, fmtRel } from "./format.js";
import { createCamera, createScene } from "./media.js";
import { renderNotices, bindAcks } from "./notices.js";
import { renderTree } from "./scripttree.js";

const esc = (s) => String(s ?? "").replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" })[c]);
const LAYOUT_KEY = "armHomeLayout";
const LED_TEXT = { off: "灭", red: "红", green: "绿", blue: "蓝", yellow: "黄", white: "白" };

async function cmd(payload, ok) {
  try {
    const r = await sendCmd(payload.cmd, payload);
    if (ok) toast(ok, "good");
    return r;
  } catch (err) {
    toast(err.message, "bad");
    return null;
  }
}

export function createHomeView(root, { catalog }) {
  root.innerHTML = `
    <div class="home">
      <div class="home-main">
        <section class="panel stage">
          <div class="stage-head">
            <h2>现场画面</h2>
            <div class="seg" id="hLayout">
              <button data-layout="both">并排</button><button data-layout="scene">三维</button><button data-layout="cam">相机</button>
            </div>
          </div>
          <div class="stage-body" id="hStage">
            <div class="stage-scene"><div class="scene-box" id="hScene"></div><p class="hint" id="hSceneHint"></p></div>
            <div class="stage-cam"><img id="hCam" class="cam" alt="腕部相机画面"><p class="hint" id="hCamHint">等待画面…</p></div>
          </div>
        </section>
        <section class="panel">
          <div class="row spread">
            <h2 style="margin:0">流程进度 <small id="hFlowTitle">没有在跑的流程</small></h2>
            <span class="chip" id="hFlowChip">空闲</span>
          </div>
          <div class="cards compact" id="hCounts"></div>
          <div class="now-step" id="hNow"></div>
          <details id="hTreeBox" class="tree-box" open>
            <summary>脚本结构 <small id="hTreeHint"></small></summary>
            <div id="hTree" class="tree compact"></div>
          </details>
          <h3>最近步骤</h3>
          <ol class="timeline" id="hTimeline"></ol>
        </section>
      </div>
      <aside class="home-side">
        <section class="panel">
          <h2>操作</h2>
          <div class="ops-main">
            <button id="hStart" class="primary big">开始分拣</button>
            <button id="hStop" class="danger big">停止</button>
          </div>
          <p class="hint" id="hStartHint">「开始」跑已在实机验证的旧流水线（扫描 → 定位 → 抓放）。</p>
          <div class="row">
            <select id="hScript" style="flex:1;min-width:0"></select>
            <button id="hRun">运行脚本</button>
          </div>
          <p class="hint" id="hScriptHint"></p>
          <div class="row">
            <span class="dim">速度</span>
            <input type="range" id="hSpeed" min="3" max="40" step="1" value="20" style="flex:1">
            <strong id="hSpeedV">—</strong>
          </div>
          <div class="row">
            <span class="dim">夹爪</span>
            <button data-grip="open">张开</button><button data-grip="grip">合拢</button><button data-grip="wide">张到最大</button>
          </div>
          <div class="row">
            <span class="dim">位姿</span>
            <button data-goto="observe_v1">观察位</button><button data-goto="upright">直立</button>
          </div>
          <div class="row" id="hModes"><span class="dim">模式</span></div>
        </section>
        <section class="panel">
          <div class="row spread"><h2 style="margin:0">提示 <small id="hNoticeCount"></small></h2>
            <span><button id="hAckNotices">告警知道了</button> <button id="hAckAll">全部知道了</button></span></div>
          <ul class="notices" id="hNotices"></ul>
        </section>
        <section class="panel">
          <h2>设备状态 <small>回读来自设备</small></h2>
          <div class="kv" id="hState"></div>
          <div class="joints mini" id="hJoints"></div>
        </section>
      </aside>
    </div>`;

  const $ = (id) => root.querySelector("#" + id);

  // ---- 画面 --------------------------------------------------------------------------------------------
  const stage = $("hStage");
  let layout = "both";
  try { layout = localStorage.getItem(LAYOUT_KEY) || "both"; } catch { /* 无痕模式 */ }
  const scene = createScene($("hScene"), $("hSceneHint"));
  const applyLayout = () => {
    stage.dataset.layout = layout;
    $("hLayout").querySelectorAll("button").forEach((b) => b.classList.toggle("on", b.dataset.layout === layout));
    if (layout === "cam") scene.hide(); else if (!root.hidden) scene.show();
    try { localStorage.setItem(LAYOUT_KEY, layout); } catch { /* 无痕模式 */ }
  };
  $("hLayout").addEventListener("click", (ev) => {
    const b = ev.target.closest("button[data-layout]");
    if (b) { layout = b.dataset.layout; applyLayout(); }
  });
  createCamera($("hCam"), $("hCamHint"), () => !root.hidden && !document.hidden && layout !== "scene");

  // ---- 操作 --------------------------------------------------------------------------------------------
  $("hStart").addEventListener("click", () => {
    if (!confirm("开始一整轮分拣会动机械臂。设备周围安全、有人在场？")) return;
    cmd({ cmd: "start" }, "已启动一整轮分拣");
  });
  $("hStop").addEventListener("click", () => cmd({ cmd: "stop" }, "已停止"));
  $("hRun").addEventListener("click", async () => {
    const name = $("hScript").value;
    if (!name) return toast("先选一份脚本", "bad");
    const s = scripts.find((x) => x.name === name);
    if (s?.moves && !confirm(`「${s.title}」会动机械臂。设备周围安全、有人在场？`)) return;
    if (await cmd({ cmd: "run_script", name }, `脚本「${s?.title ?? name}」已启动`)) setTimeout(refreshFlow, 400);
  });
  root.querySelectorAll("button[data-grip]").forEach((b) =>
    b.addEventListener("click", () => cmd({ cmd: "gripper", action: b.dataset.grip }, `夹爪：${b.textContent}`)));
  root.querySelectorAll("button[data-goto]").forEach((b) =>
    b.addEventListener("click", () => cmd({ cmd: "goto", named: b.dataset.goto }, `去${b.textContent}`)));
  const speed = $("hSpeed");
  let speedDrag = false;
  speed.addEventListener("input", () => { speedDrag = true; $("hSpeedV").textContent = `${speed.value}°/s`; });
  speed.addEventListener("change", async () => { await cmd({ cmd: "speed", speed: +speed.value }, `速度 ${speed.value}°/s`); speedDrag = false; });
  $("hModes").insertAdjacentHTML("beforeend", MODES.map((m) => `<button data-mode="${m}">${MODE_NAMES[m]}</button>`).join(""));
  $("hModes").addEventListener("click", (ev) => {
    const b = ev.target.closest("button[data-mode]");
    if (b) cmd({ cmd: "set_mode", mode: b.dataset.mode }, `已切到${MODE_NAMES[b.dataset.mode]}`);
  });
  bindAcks(root);
  $("hAckNotices").addEventListener("click", () => import("./notices.js").then((m) => m.ack("notices")));
  $("hAckAll").addEventListener("click", () => import("./notices.js").then((m) => m.ack("*")));

  // ---- 数据：流程进程、脚本库、正在跑的脚本内容 -------------------------------------------------------------
  let flow = null, scripts = [], scriptRev = -1, running = null, runningScript = null;
  async function refreshFlow() {
    try { flow = await fetchFlow(); } catch { flow = null; }
    const name = flow?.process?.script ?? null;
    if (name && name !== running) {
      running = name;
      try { runningScript = (await fetchScript(name)).check?.script ?? null; } catch { runningScript = null; }
    } else if (!name) {
      running = null; runningScript = null;
    }
  }
  async function refreshScripts() {
    try { scripts = (await fetchScripts()).scripts ?? []; } catch { scripts = []; }
    const sel = $("hScript"), keep = sel.value;
    sel.innerHTML = `<option value="">— 选一份流程脚本 —</option>` + scripts.map((s) =>
      `<option value="${esc(s.name)}"${s.runnable ? "" : " disabled"}>${esc(s.title)}${s.builtin ? "" : "（已保存）"}${s.runnable ? "" : " · 不可运行"}</option>`).join("");
    if (scripts.some((s) => s.name === keep)) sel.value = keep;
  }
  $("hScript").addEventListener("change", () => {
    const s = scripts.find((x) => x.name === $("hScript").value);
    $("hScriptHint").textContent = !s ? "" : `${s.description ?? ""}${s.partial?.length ? `；待接入：${s.partial.join("、")}` : ""}${s.moves ? "；会动机械臂" : "；不动机械臂"}`;
  });
  setInterval(() => { if (!root.hidden) refreshFlow(); }, 2000);
  refreshFlow();

  // ---- 每次刷新 --------------------------------------------------------------------------------------------
  return {
    show() { if (layout !== "cam") scene.show(); refreshFlow(); },
    hide() { scene.hide(); },
    update(store) {
      if (store.scriptRev !== scriptRev) { scriptRev = store.scriptRev; refreshScripts(); }
      scene.update(store);
      const arm = store.status.arm ?? {};
      const p = flow?.process ?? store.status.flow ?? {};
      const busy = !!p.running;
      $("hStart").disabled = busy;
      $("hRun").disabled = busy;
      $("hStartHint").textContent = p.task === "flow"
        ? "「开始」按 sort.yaml 跑分步编排器（试验）。"
        : "「开始」跑已在实机验证的旧流水线（扫描 → 定位 → 抓放）；其他流程用下面的脚本。";

      if (arm.speed && !speedDrag) {
        speed.min = arm.speed.min; speed.max = arm.speed.max; speed.value = arm.speed.value;
        $("hSpeedV").textContent = `${Math.round(arm.speed.value)}°/s`;
      }
      $("hModes").querySelectorAll("button").forEach((b) => b.classList.toggle("on", b.dataset.mode === store.status.mode));

      // 提示
      const { count } = renderNotices($("hNotices"), store, p);
      $("hNoticeCount").textContent = count ? `${count} 条待确认` : "";

      // 设备状态
      const kv = [
        ["控制台", !arm.attached ? "未接" : arm.error ? "出错" : arm.age_s > 2 ? "回读过期" : "在线", !arm.attached ? "mid" : arm.error || arm.age_s > 2 ? "bad" : "good"],
        ["操作权", !arm.attached ? "—" : arm.armed ? (arm.owner ?? "有人持有") : "空闲", ""],
        ["运动", !arm.attached ? "—" : arm.busy ? "运动中" : "静止", arm.busy ? "mid" : ""],
        ["夹爪", arm.grip_readback == null ? "—" : `${arm.grip_action ?? ""} ${arm.grip_readback}°`, ""],
        ["抓到", arm.holding == null ? "—" : arm.holding ? "夹住" : "空", arm.holding ? "good" : ""],
        ["末端 cm", arm.tip_cm ? arm.tip_cm.map((v) => v.toFixed(1)).join(", ") : "—", ""],
        ["安全余量", arm.clearance_cm == null ? "—" : `${arm.clearance_cm.toFixed(1)} cm`, arm.clearance_cm < 0 ? "bad" : ""],
        ["速度", arm.speed ? `${Math.round(arm.speed.value)}°/s` : "—", ""],
        ["灯", arm.signal?.led ? LED_TEXT[arm.signal.led.colour] ?? `rgb(${arm.signal.led.rgb.join(",")})` : "—", ""],
        ["识别端", store.status.listen?.running ? "监听中" : "未开", store.status.listen?.running ? "good" : ""],
      ];
      const kvHtml = kv.map(([k, v, c]) => `<span>${k}</span><b class="${c}">${esc(v)}</b>`).join("");
      if ($("hState").dataset.sig !== kvHtml) { $("hState").dataset.sig = kvHtml; $("hState").innerHTML = kvHtml; }
      const pose = store.pose;
      const jHtml = (pose ?? Array(6).fill(null)).map((v, i) =>
        `<span>J${i + 1}</span><b>${v == null ? "—" : Math.round(v) + "°"}</b>`).join("")
        + `<span class="dim" style="grid-column:1/-1">J6 是夹爪：合拢约 ${GRIPPER.grip}°，空载闭合读数约 132°</span>`;
      if ($("hJoints").dataset.sig !== jHtml) { $("hJoints").dataset.sig = jHtml; $("hJoints").innerHTML = jHtml; }

      // 流程进度
      const f = store.flow;
      const title = runningScript?.title ?? f.title ?? (p.task === "pipeline" && busy ? "旧流水线（扫描 → 定位 → 抓放）" : null);
      $("hFlowTitle").textContent = title ? `${title}${running ? `（${running}）` : ""}` : "没有在跑的流程";
      $("hFlowChip").textContent = busy ? "运行中" : f.state === "stopped" ? "上次停止" : f.state === "failed" ? "上次失败"
        : f.state === "done" ? "上次完成" : p.exit_code ? `退出码 ${p.exit_code}` : "空闲";
      $("hFlowChip").className = "chip " + (busy ? "on" : f.state === "failed" || p.exit_code ? "warn" : "");
      const c = store.current ?? store.rounds.at(-1) ?? { sorted: 0, missed: 0, skipped: 0, failed: 0 };
      const cards = [["已分拣", c.sorted, "good"], ["夹空", c.missed, c.missed ? "bad" : ""], ["跳过", c.skipped, c.skipped ? "mid" : ""],
        ["失败", c.failed ?? 0, c.failed ? "bad" : ""], ["报警", store.alarms.length, store.alarms.length ? "bad" : ""]];
      const cHtml = cards.map(([k, v, cl]) => `<div class="card"><span>${k}</span><b class="${cl}">${v ?? 0}</b></div>`).join("");
      if ($("hCounts").dataset.sig !== cHtml) { $("hCounts").dataset.sig = cHtml; $("hCounts").innerHTML = cHtml; }

      const cur = f.current;
      $("hNow").innerHTML = busy && cur
        ? `<b>正在：${esc(catalog()?.capabilities?.[cur.step]?.label ?? cur.step)}</b>${cur.block ? ` · ${esc(cur.block)}` : ""}${cur.summary ? ` · <span class="dim">${esc(cur.summary)}</span>` : ""}`
        : f.result ? `<span class="dim">上次结果：${esc(f.result)}</span>` : "";

      $("hTreeBox").hidden = !runningScript;
      $("hTreeHint").textContent = runningScript ? "绿色做完、蓝色进行中、黄色结果不顺；点击去脚本页看全部" : "";
      if (runningScript) {
        const html = renderTree(runningScript, { catalog: catalog(), live: f.paths, current: cur?.path, compact: true });
        if ($("hTree").dataset.sig !== html) { $("hTree").dataset.sig = html; $("hTree").innerHTML = html; }
      }

      const base = f.log[0]?.t;
      const tl = f.log.slice(-10).reverse().map((e) => {
        if (e.event === "alarm") return `<li class="bad"><em>${fmtRel(e.t, base)}</em> 报警 ${esc(alarmName(e.kind))} · ${esc(e.summary)}</li>`;
        if (e.event === "step") {
          const label = catalog()?.capabilities?.[e.step]?.label ?? (e.step === "retry" ? "重试" : e.step === "repeat" ? "重复" : e.step);
          const cl = e.state === "done" ? "good" : e.state === "started" ? "dim" : "mid";
          return `<li class="${cl}"><em>${fmtRel(e.t, base)}</em> ${esc(label)}${e.block ? ` · ${esc(e.block)}` : ""} · ${esc(e.state === "started" ? (e.summary || "开始") : (e.result ?? e.state))}${e.state !== "started" && e.summary ? ` · ${esc(e.summary)}` : ""}</li>`;
        }
        return `<li><em>${fmtRel(e.t, base)}</em> ${esc(e.event)}</li>`;
      }).join("") || `<li class="dim">还没有步骤记录（旧流水线的进度见下方计数与「记录」页）</li>`;
      if ($("hTimeline").dataset.sig !== tl) { $("hTimeline").dataset.sig = tl; $("hTimeline").innerHTML = tl; }
    },
  };
}
