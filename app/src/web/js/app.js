// 装配：页面路由（#home 等）、事件流、状态推送、全局横幅与一键停止。
// 首页「作业台」放画面 + 提示 + 操作；流程脚本、手动操作、告警、记录、任务计划、设置各一页。
// 页面只依赖日志契约与网关的 /api/*，实机与仿真共用同一套路径。

import { Store } from "./store.js";
import { LogFeed, StatusFeed, fetchLog, sendTimer, releaseControl, sendCmd, fetchCapabilities } from "./api.js";
import { createHomeView } from "./home.js";
import { createScriptsView } from "./scripts.js";
import { createControlView } from "./control.js";
import { createAlarmsView } from "./alarms.js";
import { createLogsView } from "./logs.js";
import { createTimersView } from "./timers.js";
import { createSettingsView } from "./settings.js";
import { MODE_NAMES, alarmName } from "./contract.js";
import { toast } from "./format.js";
import { health } from "./media.js";

const PAGES = [
  ["home", "作业台"], ["scripts", "流程脚本"], ["manual", "手动操作"], ["alarms", "告警"],
  ["logs", "记录与日志"], ["timers", "任务计划"], ["settings", "设置"],
];
const BACKFILL = 2000;                  // 首屏最多回放的历史行数

const store = new Store();
const root = document.getElementById("views");
const nav = document.getElementById("tabs");

// ---- 偏好（只存本浏览器） ------------------------------------------------------------------------------
const pref = (k, d) => { try { return localStorage.getItem(k) ?? d; } catch { return d; } };
const setPref = (k, v) => { try { localStorage.setItem(k, v); } catch { /* 无痕模式 */ } };
let debug = new URLSearchParams(location.search).get("debug") === "1" || pref("armDebug", "0") === "1";
let beepOn = pref("armBeep", "1") === "1";
const prefs = {
  debug: () => debug, beep: () => beepOn,
  setDebug(v) { debug = v; applyDebug(); },
  setBeep(v) { beepOn = v; setPref("armBeep", v ? "1" : "0"); },
};
const chip = (id) => document.getElementById(id);
function applyDebug() {
  document.body.classList.toggle("debug", debug);
  setPref("armDebug", debug ? "1" : "0");
  chip("chipDebug").hidden = !debug;
}
document.getElementById("gear").addEventListener("click", () => prefs.setDebug(!debug));
applyDebug();

// ---- 能力表：脚本页、作业台都要；拉一次，失败隔一会儿再拉 ----------------------------------------------------
let catalogData = null;
const catalog = () => catalogData;
(async function loadCatalog() {
  try { catalogData = await fetchCapabilities(); render(); } catch { setTimeout(loadCatalog, 5000); }
})();

// ---- 页面 ----------------------------------------------------------------------------------------------
const views = {};
for (const [key, label] of PAGES) {
  const section = document.createElement("section");
  section.className = "view";
  section.hidden = true;
  section.dataset.page = key;
  root.appendChild(section);
  views[key] = { section, label, view: null };
}
views.home.view = createHomeView(views.home.section, { catalog });
views.scripts.view = createScriptsView(views.scripts.section, { catalog });
views.manual.view = createControlView(views.manual.section);
views.alarms.view = createAlarmsView(views.alarms.section);
views.logs.view = createLogsView(views.logs.section);
views.timers.view = createTimersView(views.timers.section);
views.settings.view = createSettingsView(views.settings.section, { catalog, prefs });

nav.innerHTML = PAGES.map(([key, label]) => `<a href="#${key}" data-tab="${key}">${label}<i class="nav-badge" hidden></i></a>`).join("");
let active = null;
function route() {
  const key = (location.hash || "#home").slice(1).split("/")[0];
  const next = views[key] ? key : "home";
  if (next === active) return;
  if (active) views[active].view?.hide?.();
  active = next;
  nav.querySelectorAll("a").forEach((a) => a.classList.toggle("on", a.dataset.tab === active));
  for (const [k, v] of Object.entries(views)) v.section.hidden = k !== active;
  views[active].view?.show?.();
  render();
}
window.addEventListener("hashchange", route);

// ---- 一键停止：任何页面都在页头 ---------------------------------------------------------------------------
document.getElementById("globalStop").addEventListener("click", async () => {
  try { await sendCmd("stop"); toast("已停止：流程与机械臂都停了，不会自动恢复", "good"); }
  catch (err) { toast(err.message, "bad"); }
});

// ---- 心跳：页面可见时每 0.8 s 一次；网关只在收到心跳时替本页续控制台租约，关页/切后台即停 -----------------------
setInterval(() => {
  if (document.visibilityState === "visible") sendCmd("heartbeat", {}).catch(() => {});
}, 800);

// ---- 定时任务待发横幅：任何页面都要看得到（到点前几秒还能取消） ----------------------------------------------------
const banner = document.getElementById("timerBanner");
function renderBanner() {
  const t = store.status.timer ?? {};
  if (!t.pending || !t.at) { banner.hidden = true; banner.innerHTML = ""; return; }
  banner.hidden = false;
  banner.innerHTML = `
    <b>${Math.max(0, Math.round(t.countdown_s ?? 0))} s 后自动启动一整轮分拣</b>
    <span>${new Date(t.at * 1000).toLocaleString("zh-CN", { hour12: false })} · 现在取消还来得及</span>
    <button id="panicBtn" class="danger">取消这次</button>`;
  banner.querySelector("#panicBtn").addEventListener("click", async () => {
    if (!t.id) return toast("找不到这个任务，刷新一下", "bad");
    try { await sendTimer({ cancel: t.id }); toast("已取消，这次不会启动", "good"); }
    catch (err) { toast(err.message, "bad"); }
  });
}

// ---- 报警红条：流程脚本发的、要确认的报警，任何页面顶上都有；新报警响一声、标题加 ⚠ ------------------------------------
const alarmBanner = document.getElementById("alarmBanner");
let alarmHeard = 0, noticeHeard = 0, alarmSig = "";
const TITLE = document.title;
function beep(kind = "alarm") {
  if (!beepOn) return;
  try {
    const ctx = new (window.AudioContext || window.webkitAudioContext)();
    const tones = kind === "alarm" ? [[0, 880], [0.25, 660], [0.5, 880]] : [[0, 660]];
    for (const [at, f] of tones) {
      const o = ctx.createOscillator(), g = ctx.createGain();
      o.frequency.value = f; g.gain.value = 0.08;
      o.connect(g).connect(ctx.destination);
      o.start(ctx.currentTime + at); o.stop(ctx.currentTime + at + 0.18);
    }
    setTimeout(() => ctx.close(), 1200);
  } catch { /* 浏览器没允许出声：只靠红条 */ }
}
alarmBanner.addEventListener("click", (ev) => {
  const btn = ev.target.closest("button[data-ack]");
  if (btn) import("./notices.js").then((m) => m.ack(btn.dataset.ack));
});
function renderAlarms() {
  const list = store.alarms;
  const total = list.length + store.notices.length;
  document.title = total ? `⚠ ${total} · ${TITLE}` : TITLE;
  if (store.alarmSeq > alarmHeard) { if (alarmHeard && list.length) beep("alarm"); alarmHeard = store.alarmSeq; }
  if (store.noticeSeq > noticeHeard) { if (noticeHeard && store.notices.length) beep("notice"); noticeHeard = store.noticeSeq; }
  const badge = nav.querySelector('a[data-tab="alarms"] .nav-badge');
  badge.hidden = !total;
  badge.textContent = total;
  badge.className = "nav-badge" + (list.length ? " bad" : "");
  const sig = list.map((a) => a.seq).join(",");
  if (sig === alarmSig) return;                 // 没变就不重画，免得按钮在点击间被换掉
  alarmSig = sig;
  alarmBanner.hidden = !list.length;
  const esc = (s) => String(s ?? "").replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" })[c]);
  const kinds = [...new Set(list.map((a) => a.kind))];
  alarmBanner.innerHTML = `<b>报警 ${list.length}</b>` + list.slice(-4).map((a) => `
      <span class="alarm-item"><b>${esc(alarmName(a.kind))}</b>${a.block ? ` · ${esc(a.block)}` : ""} · ${esc(a.summary)}</span>`).join("")
    + kinds.map((k) => `<button data-ack="${esc(k)}">知道了：${esc(alarmName(k))}</button>`).join("")
    + (kinds.length > 1 ? `<button data-ack="*">全部知道了</button>` : "");
}

// ---- 流程脚本发的提示（notify）：任何页面弹一下 --------------------------------------------------------------------
let notifySeen = null;                  // 回填完成前为 null：历史里的提示不弹
function popNotify() {
  if (notifySeen === null) return;
  for (const e of store.events.slice(-20)) {
    if (e.event === "step" && e.step === "notify" && e.state === "done" && (e.seq ?? 0) > notifySeen) {
      toast(`流程提示：${e.summary ?? ""}`, "good");
      notifySeen = e.seq;
    }
  }
}

function render() {
  if (!active) return;
  views[active].view?.update?.(store);
  // 连接分两层：浏览器↔网关（store.connection）和网关↔控制台（arm.age_s/error）。后者更危险：
  // 控制台掉线时 age_s 用网关墙钟算会一直涨，这里优先显示"回读过期"，不让页头假显示"实时"。
  const arm = store.status.arm;
  const stale = arm?.attached && (arm.error || (arm.age_s != null && arm.age_s > 2));
  if (stale) {
    chip("chipConn").textContent = "回读过期";
    chip("chipConn").className = "chip warn";
  } else {
    chip("chipConn").textContent = { connecting: "连接中", live: "实时", polling: "轮询", error: "重连中" }[store.connection] ?? "—";
    chip("chipConn").className = "chip " + (store.connection === "live" ? "on" : store.connection === "error" ? "warn" : "");
  }
  chip("chipMode").textContent = `模式 ${MODE_NAMES[store.status.mode] ?? store.status.mode ?? "—"}`;
  const p = store.status.flow ?? {};
  chip("chipTask").textContent = p.running ? `运行中${p.script ? ` · ${p.script}` : p.task === "pipeline" ? " · 旧流水线" : ""}` : "空闲";
  chip("chipTask").className = "chip " + (p.running ? "on" : "");
  chip("chipLatency").textContent = store.status.replay ? "回放"
    : (store.latency === null ? "距上次事件 —" : `距上次事件 ${store.latency.toFixed(1)} s`);
  renderBanner();
  renderAlarms();
  popNotify();
}

store.subscribe(render);
route();

// 首屏把已有日志回填一遍，实时流从末尾接着走；两边都按 seq 去重，条数与 /api/log total 一致
(async () => {
  let lastSeq = 0, lastT = null;
  try {
    const { events, total } = await fetchLog(0, BACKFILL);
    store.total = total ?? events.length;
    events.forEach((e) => store.push(e));
    lastSeq = events.at(-1)?.seq ?? 0;
    lastT = events.at(-1)?.t ?? null;
  } catch { /* 服务未起时页面仍可用 */ }
  notifySeen = lastSeq;                         // 回填的旧提示不弹
  await store.refresh();
  render();

  const feed = new LogFeed({
    onEvent: (e) => {
      store.push(e);
      store.latency = feed.latency;
      if (store.events.length % 20 === 0) store.refresh();
    },
    onState: (s) => { store.connection = s; render(); },
  });
  feed.start(lastSeq, { t: lastT, wall: performance.now() });
  const status = new StatusFeed({ onStatus: (s) => store.setStatus(s) });
  status.start();

  setInterval(() => store.refresh(), 5000);
  setInterval(() => { store.latency = feed.latency; render(); }, 1000);

  // 切到后台：释放操作权，并断开两条长连接（浏览器同域只给 6 条连接，开几个标签页就会被占满）。
  // 回来时按 seq 续上，不重复收事件。
  document.addEventListener("visibilitychange", () => {
    if (document.hidden) {
      releaseControl();
      feed.pause();
      status.pause();
      views[active]?.view?.hide?.();
    } else {
      feed.resume();
      status.resume();
      views[active]?.view?.show?.();
    }
  });
})();

window.addEventListener("pagehide", releaseControl);

health().then((h) => {
  const c = chip("chipArm");
  c.textContent = h.console ? `控制台 ${h.console_sim ? "仿真" : "实机"} ${h.console.replace(/^https?:\/\//, "")}` : "控制台 未接";
  c.className = "chip " + (h.console_ready ? "on" : "warn");
});
window.addEventListener("resize", render);
window.addEventListener("error", (e) => toast(e.message, "bad"));
