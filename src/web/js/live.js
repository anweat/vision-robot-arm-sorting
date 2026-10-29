// F1-2 实时状态页：一轮分拣正在发生什么，以及六轴角度的实时回读。
// 画面与机器人状态来自 --console 指向的执行侧（仿真或实机）；没有控制台时这两块显示“未接”。

import { JOINT_LIMITS, EVENTS, categoryName, resultName, GRIPPER } from "./contract.js";
import { fmtXY, cls, fmtRel } from "./format.js";
import { authUrl } from "./api.js";

const KEY_CARDS = [
  ["连接", "conn"], ["模式", "mode"], ["轮次", "round"],
  ["已分拣", "sorted"], ["夹空", "missed"], ["跳过", "skipped"], ["延迟", "latency"],
];

// 执行侧回读。tip_cm 是末端坐标（厘米），clearance_cm 是离禁入区的余量，负数是踩线。
const ARM_CARDS = [
  ["使能", "armed"], ["运动", "busy"], ["夹爪", "grip"], ["抓到", "holding"],
  ["末端 X", "tipX"], ["末端 Y", "tipY"], ["末端 Z", "tipZ"],
  ["安全余量", "clearance"], ["桌上物块", "blocks"], ["回读档位", "profile"],
];

export function createLiveView(root) {
  root.innerHTML = `
    <div class="cards" id="liveCards"></div>
    <div class="grid-2">
      <section class="panel">
        <h2>腕部相机 <small>仿真为按标定 ray-cast 的合成画面，实机为真相机，路径同名同义</small></h2>
        <img id="camFrame" class="cam" alt="腕部相机画面">
        <p class="hint" id="camHint">等待画面…</p>
      </section>
      <section class="panel">
        <h2>执行侧回读 <small>数据来自设备</small></h2>
        <div class="cards" id="armCards"></div>
        <p class="hint" id="armHint"></p>
      </section>
    </div>
    <div class="grid-2">
      <section class="panel">
        <h2>六轴角度 <small>实时回读，超出软限位标红</small></h2>
        <div id="liveJoints" class="joints"></div>
      </section>
      <section class="panel">
        <h2>当前动作 <small>最近一条事件</small></h2>
        <div id="liveNow" class="now">—</div>
        <h3>夹爪</h3>
        <div id="liveGrip" class="grip">—</div>
      </section>
    </div>
    <section class="panel">
      <h2>本轮逐块记录 <small>时间 / 类别 / 来源 / 目标 / 耗时 / 结果</small></h2>
      <table id="liveRecords"><thead><tr>
        <th>#</th><th>时间</th><th>类别</th><th>来源 cm</th><th>目标位</th><th>耗时 s</th><th>结果</th>
      </tr></thead><tbody></tbody></table>
    </section>
    <section class="panel">
      <h2>事件流 <small>只列事件表内的事件，最新的在上</small></h2>
      <ol id="liveFeed" class="feed" reversed></ol>
    </section>`;

  const $ = (id) => root.querySelector("#" + id);
  const cards = $(`liveCards`);
  cards.innerHTML = KEY_CARDS.map(([label, key]) =>
    `<div class="card"><span>${label}</span><b data-key="${key}">—</b></div>`).join("");
  const joints = $("liveJoints");
  joints.innerHTML = JOINT_LIMITS.map((j, i) => `
    <div class="joint" data-i="${i}">
      <span>${j.name}</span>
      <strong>—</strong>
      <div class="bar"><i></i><u data-min="${j.min}" data-max="${j.max}"></u></div>
    </div>`).join("");

  const setCard = (key, value, cls = "") => {
    const el = cards.querySelector(`[data-key="${key}"]`);
    if (el && el.textContent !== String(value)) el.textContent = value;
    if (el) el.className = cls;
  };

  const armCards = $(`armCards`);
  armCards.innerHTML = ARM_CARDS.map(([label, key]) =>
    `<div class="card"><span>${label}</span><b data-arm="${key}">—</b></div>`).join("");
  const armHint = $("armHint");
  const setArm = (key, value, cls = "") => {
    const el = armCards.querySelector(`[data-arm="${key}"]`);
    if (el && el.textContent !== String(value)) el.textContent = value;
    if (el) el.className = cls;
  };

  // 相机只在自己这一页可见时拉：切到别的页签就停，不去占 8770 的渲染线程
  const cam = $("camFrame");
  const camHint = $("camHint");
  const tickCam = () => {
    if (root.hidden) return;
    cam.src = authUrl(`/api/frame?t=${Date.now()}`);
  };
  cam.addEventListener("load", () => {
    camHint.textContent = `${cam.naturalWidth}×${cam.naturalHeight} · 每 0.6 s 一帧`;
  });
  cam.addEventListener("error", () => {
    camHint.textContent = "没有画面：未接控制台（--console）或相机还没出图";
  });
  setInterval(tickCam, 600);
  tickCam();

  let lastEventSeq = -1;

  return {
    update(store) {
      const mode = store.status.mode ?? "manual";
      setCard("conn", { connecting: "连接中", live: "实时", polling: "轮询", error: "重连中" }[store.connection] ?? "—",
              store.connection === "live" ? "good" : store.connection === "error" ? "bad" : "mid");
      setCard("mode", { manual: "手动", preset: "预设", auto: "视觉自主" }[mode] ?? mode);
      // 轮次口径：一次 start = 一轮。中断的轮标"未结束"不算完成；无任务的 pipeline 只算日志历史
      setCard("round", store.current
        ? `第 ${store.current.no} 轮 · ${store.currentHandled}/${store.current.cubes || "?"}` +
          (store.current.unfinished ? " · 未结束" : "")
        : store.rounds.length ? `已完成 ${store.rounds.length} 轮`
        : store.historyRounds ? `日志历史 ${store.historyRounds} 轮（只读）`
        : "未启动任务");
      const c = store.current ?? { sorted: 0, missed: 0, skipped: 0 };
      setCard("sorted", c.sorted, "good");
      setCard("missed", c.missed, c.missed ? "bad" : "");
      setCard("skipped", c.skipped, c.skipped ? "mid" : "");
      setCard("latency", store.latency === null ? "—" : `${store.latency.toFixed(2)} s`,
              store.latency !== null && store.latency <= 1 ? "good" : "bad");

      const arm = store.status.arm ?? {};
      setArm("armed", !arm.attached ? "未接" : arm.armed ? "已使能" : "未使能", arm.armed ? "good" : "mid");
      setArm("busy", !arm.attached ? "—" : arm.busy ? "运动中" : "静止", arm.busy ? "mid" : "");
      setArm("grip", arm.grip_action ? `${arm.grip_action} · ${arm.grip_readback ?? "—"}°` : "—");
      setArm("holding", arm.holding == null ? "—" : arm.holding ? "夹住" : "夹空",
             arm.holding ? "good" : "");
      ["tipX", "tipY", "tipZ"].forEach((k, i) => setArm(k, arm.tip_cm?.[i] ?? "—"));
      setArm("clearance", arm.clearance_cm == null ? "—" : `${arm.clearance_cm.toFixed(1)} cm`,
             arm.clearance_cm == null ? "" : arm.clearance_cm < 0 ? "bad" : "good");
      setArm("blocks", arm.blocks ?? "—");
      setArm("profile", arm.profile ?? "—");
      armHint.textContent = !arm.attached
        ? "未接控制台：运动类指令按 501 返回，只有日志与模式可用。"
        : arm.error ? `控制台错误：${arm.error}`
        : `revision ${arm.revision ?? "—"} · ${arm.error_text || "无异常"}`;

      const pose = store.pose;
      joints.querySelectorAll(".joint").forEach((el) => {
        const i = +el.dataset.i, lim = JOINT_LIMITS[i];
        const value = pose?.[i];
        const out = value !== null && value !== undefined && (value < lim.min || value > lim.max);
        el.querySelector("strong").textContent = value == null ? "—" : `${Math.round(value)}°`;
        el.querySelector("strong").className = out ? "bad" : "";
        const ratio = value == null ? 0 : (value - lim.min) / (lim.max - lim.min);
        el.querySelector(".bar i").style.width = `${Math.max(0, Math.min(1, ratio)) * 100}%`;
      });

      const last = store.lastEvent;
      if (last && last.seq !== lastEventSeq) {
        lastEventSeq = last.seq;
        const spec = EVENTS[last.event];
        $("liveNow").innerHTML = `<b>${last.event}</b><span>${spec?.meaning ?? ""}</span>`;
        const grip = last.grip ?? (last.event === "gripped" ? last.grip : null);
        const gripEl = $("liveGrip");
        if (grip) {
          const held = grip.result === "holding";
          gripEl.innerHTML = `指令 ${grip.command ?? "—"}° · 回读 ${grip.readback ?? "—"}° · ` +
            `<b class="${held ? "good" : "bad"}">${held ? "夹住" : grip.result === "empty" ? "夹空" : "没收拢"}</b>`;
        } else if (pose) {
          gripEl.textContent = `J6 ${pose[5] == null ? "—" : Math.round(pose[5])}°（合拢 ${GRIPPER.grip}°）`;
        }
      }

      // 本轮 = 一次 start 之后的逐块记录；没有任务时回看日志里最后一轮（只读）
      const base = store.events[0]?.t ?? store.records[0]?.t;
      const cur = store.current ?? store.rounds.at(-1) ?? null;
      const rows = (cur
        ? store.records.filter((r) => r.t >= cur.t && (!cur.endedAt || r.t <= cur.endedAt))
        : store.records.filter((r) => r.round === (store.records.at(-1)?.round ?? 0))
      ).slice(-40);
      $("liveRecords").querySelector("tbody").innerHTML = rows.map((r) => `
        <tr><td>${r.seq}</td><td>${fmtRel(r.t, base)}</td><td>${categoryName(r.category)}</td>
        <td>${fmtXY(r.cube)}</td><td>${r.bin ?? "—"}</td>
        <td>${r.seconds == null ? "—" : r.seconds.toFixed(1)}</td>
        <td class="${cls(r.result)}">${resultName(r.result)}</td></tr>`).join("")
        || `<tr><td colspan="7" class="dim">本轮还没有逐块记录</td></tr>`;

      const feed = $("liveFeed");
      const recent = store.events.slice(-60).reverse();
      feed.innerHTML = recent.map((e) => {
        const spec = EVENTS[e.event];
        return `<li><span class="dim">${e.seq}</span> <b>${e.event}</b>
          <span class="dim">${spec?.meaning ?? ""}</span>
          ${e.category ? `<span class="tag c-${e.category}">${categoryName(e.category)}</span>` : ""}
          ${e.bin ? `<span class="tag">→ ${e.bin}</span>` : ""}</li>`;
      }).join("");
    },
  };
}


