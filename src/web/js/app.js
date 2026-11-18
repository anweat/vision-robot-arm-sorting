// 装配：页签、事件流、状态推送。页面只依赖日志契约，实机与仿真共用同一套路径。

import { Store } from "./store.js";
import { LogFeed, StatusFeed, fetchLog, sendTimer, releaseControl, authUrl } from "./api.js";
import { createLiveView } from "./live.js";
import { createFaultsView } from "./faults.js";
import { createControlView } from "./control.js";
import { createTimersView } from "./timers.js";
import { createUnknownView } from "./unknown.js";
import { MODE_NAMES } from "./contract.js";
import { toast } from "./format.js";

const TABS = [
  ["live", "概览"], ["control", "手动操作"], ["faults", "告警"],
  ["unknown", "告警（异常物体）"], ["timers", "任务计划"],
];
const BACKFILL = 2000;                  // 首屏最多回放的历史行数

const store = new Store();
const root = document.getElementById("views");
const nav = document.getElementById("tabs");

const views = {};
for (const [key, label] of TABS) {
  const section = document.createElement("section");
  section.className = "view";
  section.hidden = true;
  root.appendChild(section);
  views[key] = { section, label, view: null };
}
views.live.view = createLiveView(views.live.section);
views.faults.view = createFaultsView(views.faults.section);
views.control.view = createControlView(views.control.section);
views.timers.view = createTimersView(views.timers.section);
views.unknown.view = createUnknownView(views.unknown.section);

let active = "live";
nav.innerHTML = TABS.map(([key, label]) =>
  `<button data-tab="${key}"${key === active ? ' class="on"' : ""}>${label}</button>`).join("");
nav.addEventListener("click", (ev) => {
  const btn = ev.target.closest("button[data-tab]");
  if (!btn) return;
  active = btn.dataset.tab;
  nav.querySelectorAll("button").forEach((b) => b.classList.toggle("on", b === btn));
  render();
  views[active]?.view?.show?.();          // 新页签按需装载（首次进手动操作才加载仿真工作台）
});

const chip = (id) => document.getElementById(id);

// 扩展与调试：默认收起预设动作等附加能力；?debug=1 或点 ⚙ 打开，状态持久化
const DEBUG_KEY = "armDebug";
let debug = new URLSearchParams(location.search).get("debug") === "1"
  || localStorage.getItem(DEBUG_KEY) === "1";
const applyDebug = () => {
  document.body.classList.toggle("debug", debug);
  localStorage.setItem(DEBUG_KEY, debug ? "1" : "0");
  chip("chipDebug").hidden = !debug;
};
document.getElementById("gear").addEventListener("click", () => {
  debug = !debug;
  applyDebug();
});
applyDebug();

// 定时任务的待发提示任何页签都要能看到（到点前几秒还能取消的那几秒，人不能正好不在那一页）
const banner = document.getElementById("timerBanner");
function renderBanner() {
  const t = store.status.timer ?? {};
  if (!t.pending || !t.at) {
    banner.hidden = true;
    banner.innerHTML = "";
    return;
  }
  banner.hidden = false;
  banner.innerHTML = `
    <b>${Math.max(0, Math.round(t.countdown_s ?? 0))} s 后自动启动一整轮分拣</b>
    <span>${new Date(t.at * 1000).toLocaleString("zh-CN", { hour12: false })} · 现在取消还来得及</span>
    <button id="panicBtn" class="danger">取消这次</button>`;
  banner.querySelector("#panicBtn").addEventListener("click", async () => {
    if (!t.id) return toast("找不到这个任务，刷新一下", "bad");
    try {
      await sendTimer({ cancel: t.id });
      toast("已取消，这次不会启动", "good");
    } catch (err) {
      toast(err.message, "bad");
    }
  });
}

function render() {
  for (const [key, v] of Object.entries(views)) v.section.hidden = key !== active;
  // 有些视图没有 update（例如任务计划自己定时刷新），必须可选调用，
  // 否则 render 每秒抛一次 TypeError，整页都不再更新
  views[active].view?.update?.(store);

  // 连接状态分两层：浏览器↔服务器（store.connection）和服务器↔控制台（arm.age_s/error）。
  // 后者更危险——控制台掉线、轮询线程被异常杀死时，服务端 age_s 用本地墙钟算会持续涨，
  // 这里优先显示"回读过期"，不能让页头假显示"实时"盖住一条已失去监控的臂。
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
  // 这个数量的是"距上次事件多久"（空闲时会 1:1 往上涨），不是"事件到展示"的延迟；
  // 真正量延迟的是 tests 里的 latency 脚本，改名以免验收时拿错指标。
  // 回放时日志的 t 是写入时刻，这个数恒为 0——那不叫实时，直接标"回放"。
  chip("chipLatency").textContent = store.status.replay
    ? "距上次事件 回放"
    : (store.latency === null ? "距上次事件 —" : `距上次事件 ${store.latency.toFixed(2)} s`);
  chip("chipTotal").textContent = `${store.total} 事件`;
  renderBanner();
}

store.subscribe(render);

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
  // 从回填末尾接续，延迟基线 = 最近一条回填事件的 t + 当时墙钟
  feed.start(lastSeq, { t: lastT, wall: performance.now() });

  const status = new StatusFeed({
    onStatus: (s) => store.setStatus(s),
  });
  status.start();

  setInterval(() => store.refresh(), 5000);
  setInterval(() => { store.latency = feed.latency; render(); }, 1000);

  // 切到后台：释放操作权，并断开两条长连接。浏览器同域只给 6 条连接，
  // 开三个标签页就会被 SSE 占满，再开页面连 HTML 都拿不到，一直转圈。
  // 回来时按 seq 续上，不重复收事件。
  document.addEventListener("visibilitychange", () => {
    if (document.hidden) {
      releaseControl();
      feed.pause();
      status.pause();
      views.control?.view?.hide?.();      // 整个页面在后台才卸掉内嵌的仿真台
    } else {
      feed.resume();
      status.resume();
      views.control?.view?.show?.();
    }
  });
})();

// 页面关闭时释放操作权，机械臂不停在一个没人看的页面名下
window.addEventListener("pagehide", releaseControl);

// 控制台：运动指令转发给它（仿真或实机）。它自己的 3D 工作台已经内嵌在"手动操作"页，
// 不再另开一个跳转链接。
fetch(authUrl("/api/health")).then((r) => r.json()).then((h) => {
  const chip = document.getElementById("chipArm");
  chip.textContent = h.console ? `控制台 ${h.console.replace(/^https?:\/\//, "")}` : "控制台 未接";
  chip.className = "chip " + (h.console_ready ? "on" : "warn");
}).catch(() => { });
window.addEventListener("resize", render);
window.addEventListener("error", (e) => toast(e.message, "bad"));
