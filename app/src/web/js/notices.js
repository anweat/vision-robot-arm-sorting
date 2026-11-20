// 提示栏：三类东西放在一起，按严重程度排——
//   报警   流程脚本发的 alarm（needs_ack），红色，要人点“知道了”；没确认时语音/手势/定时不能启动
//   告警   日志里的故障事件（抓空、夹爪不响应、放置失败、堵转、桶满、未知物体…），黄/红，点“知道了”清
//   状态   从回读与进程状态算出来的（控制台未接、回读过期、操作权在别人手里、流程异常退出、回放中），
//          条件消失就自己消失，不需要确认
// 确认都写进日志（alarm_ack），所有端同步消失。

import { NOTICES, alarmName } from "./contract.js";
import { sendCmd } from "./api.js";
import { toast, fmtRel } from "./format.js";

const esc = (s) => String(s ?? "").replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" })[c]);

export function conditions(store, flowProcess) {
  const out = [];
  const arm = store.status.arm ?? {};
  if (store.connection === "error") out.push({ level: "bad", title: "和网关断开", text: "正在重连；这期间页面上的状态不是最新的" });
  if (!arm.attached) out.push({ level: "info", title: "未接控制台", text: "只能看日志；运动、夹爪、流程都不可用（网关启动时加 --console）" });
  else if (arm.error || (arm.age_s != null && arm.age_s > 2))
    out.push({ level: "bad", title: "回读过期", text: arm.error ? `控制台错误：${arm.error}` : `${arm.age_s.toFixed(1)} s 没有新回读，机械臂状态未知` });
  if (arm.armed && arm.owner && !["web", "gateway"].includes(arm.owner) && !String(arm.owner).startsWith("flow"))
    out.push({ level: "info", title: "操作权在别处", text: `${arm.owner} 持有操作权；要接管去「手动操作」` });
  if (arm.error_text) out.push({ level: "warn", title: "控制台提示", text: arm.error_text });
  if (arm.clearance_cm != null && arm.clearance_cm < 0) out.push({ level: "bad", title: "进入禁入区", text: `余量 ${arm.clearance_cm.toFixed(1)} cm` });
  if (store.status.replay) out.push({ level: "info", title: "回放中", text: "显示的是样例日志回放，不是实时数据" });
  const p = flowProcess ?? store.status.flow;
  if (p && !p.running && p.exit_code) out.push({ level: "warn", title: "流程异常退出", text: `退出码 ${p.exit_code}${(p.output ?? []).length ? "：" + p.output.at(-1) : ""}` });
  const t = store.status.timer;
  if (t?.pending) out.push({ level: "warn", title: "定时任务待发", text: `${Math.round(t.countdown_s ?? 0)} s 后自动启动；页顶横幅可取消` });
  return out;
}

export function renderNotices(el, store, flowProcess, { limit = 8 } = {}) {
  const base = store.events[0]?.t;
  const alarms = store.alarms.map((a) => ({ level: "bad", title: `报警：${alarmName(a.kind)}`, text: `${a.block ? a.block + " · " : ""}${a.summary ?? ""}`,
    t: a.t, ack: a.kind }));
  const notices = store.notices.slice().reverse().map((e) => {
    const n = NOTICES[e.event];
    return { level: n.level, title: n.title, text: n.text(e), t: e.t, seq: e.seq };
  });
  const conds = conditions(store, flowProcess);
  const shown = [...alarms, ...notices.slice(0, limit)];
  const more = notices.length - Math.min(notices.length, limit);
  const row = (n) => `<li class="notice n-${n.level}">
      <div><b>${esc(n.title)}</b><span>${esc(n.text)}</span></div>
      ${n.t != null ? `<em>${fmtRel(n.t, base)}</em>` : ""}
      ${n.ack ? `<button data-ack="${esc(n.ack)}">知道了</button>` : ""}</li>`;
  const html = (conds.length ? conds.map(row).join("") : "")
    + shown.map(row).join("")
    + (more > 0 ? `<li class="notice n-info"><span>还有 ${more} 条更早的告警，见「告警」页</span></li>` : "");
  const sig = html;
  if (el.dataset.sig === sig) return { count: alarms.length + notices.length };
  el.dataset.sig = sig;
  el.innerHTML = html || `<li class="notice n-ok"><b>一切正常</b><span>没有报警、告警，设备连接正常</span></li>`;
  return { count: alarms.length + notices.length };
}

export async function ack(kind) {
  try {
    await sendCmd("alarm_ack", { kind });
    toast(kind === "*" ? "报警与告警已全部确认" : kind === "notices" ? "告警已确认" : "报警已确认", "good");
  } catch (err) {
    toast(err.message, "bad");
  }
}

export function bindAcks(root) {
  root.addEventListener("click", (ev) => {
    const btn = ev.target.closest("button[data-ack]");
    if (btn) ack(btn.dataset.ack);
  });
}
