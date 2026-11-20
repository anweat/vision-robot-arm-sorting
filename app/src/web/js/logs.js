// 记录与日志：逐块记录（六项）、统计、原始事件流。数据都按日志契约读。

import { EVENTS, categoryName, resultName } from "./contract.js";
import { fmtXY, cls, fmtRel } from "./format.js";

const esc = (s) => String(s ?? "").replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" })[c]);

export function createLogsView(root) {
  root.innerHTML = `
    <section class="panel">
      <h2>统计 <small>块成功率 = 已分拣 /（已分拣 + 夹空 + 跳过 + 失败）；挪开不计入</small></h2>
      <div class="cards" id="lSum"></div>
    </section>
    <section class="panel">
      <div class="row spread"><h2 style="margin:0">逐块记录 <small>时间 / 类别 / 来源 / 目标 / 耗时 / 结果</small></h2>
        <label>只看 <select id="lRound"><option value="cur">本轮</option><option value="all">全部</option></select></label></div>
      <table><thead><tr><th>#</th><th>时间</th><th>轮</th><th>类别</th><th>来源 cm</th><th>目标位</th><th>耗时 s</th><th>结果</th><th>原因</th></tr></thead>
      <tbody id="lRecords"></tbody></table>
    </section>
    <section class="panel">
      <div class="row spread"><h2 style="margin:0">事件流 <small>只列事件表内的事件，最新的在上</small></h2>
        <input id="lFilter" type="text" placeholder="按事件名筛选，如 step / alarm / pipeline" style="width:260px"></div>
      <ol id="lFeed" class="feed tall" reversed></ol>
    </section>`;
  const $ = (id) => root.querySelector("#" + id);
  let filter = "";
  $("lFilter").addEventListener("input", () => { filter = $("lFilter").value.trim(); });
  return {
    update(store) {
      const s = store.summary ?? {};
      const blocks = s.by_result ?? {};
      const n = (k) => blocks?.[k] ?? 0;
      const denom = n("sorted") + n("missed") + n("skipped") + n("failed");
      const cards = [["轮次", s.rounds_finished ?? store.rounds.length], ["已分拣", n("sorted")], ["夹空", n("missed")],
        ["跳过", n("skipped")], ["失败", n("failed")], ["挪开", n("cleared")],
        ["块成功率", denom ? `${Math.round((n("sorted") / denom) * 100)}%` : "—"], ["事件", store.total]];
      const sumHtml = cards.map(([k, v]) => `<div class="card"><span>${k}</span><b>${esc(v)}</b></div>`).join("");
      if ($("lSum").dataset.sig !== sumHtml) { $("lSum").dataset.sig = sumHtml; $("lSum").innerHTML = sumHtml; }

      const base = store.events[0]?.t ?? store.records[0]?.t;
      const cur = store.current ?? store.rounds.at(-1) ?? null;
      const rows = ($("lRound").value === "all" ? store.records
        : cur ? store.records.filter((r) => r.t >= cur.t && (!cur.endedAt || r.t <= cur.endedAt))
        : store.records.filter((r) => r.round === (store.records.at(-1)?.round ?? 0))).slice(-200).reverse();
      const rHtml = rows.map((r) => `<tr><td>${r.seq}</td><td>${fmtRel(r.t, base)}</td><td>${r.round ?? "—"}</td>
        <td>${categoryName(r.category)}</td><td>${fmtXY(r.cube)}</td><td>${esc(r.bin ?? "—")}</td>
        <td>${r.seconds == null ? "—" : r.seconds.toFixed(1)}</td><td class="${cls(r.result)}">${resultName(r.result)}</td>
        <td class="dim">${esc(r.reason ?? "")}</td></tr>`).join("") || `<tr><td colspan="9" class="dim">还没有逐块记录</td></tr>`;
      if ($("lRecords").dataset.sig !== rHtml) { $("lRecords").dataset.sig = rHtml; $("lRecords").innerHTML = rHtml; }

      const recent = store.events.filter((e) => !filter || e.event.includes(filter)).slice(-150).reverse();
      const fHtml = recent.map((e) => `<li><span class="dim">${e.seq}</span> <b>${esc(e.event)}</b>
          <span class="dim">${esc(EVENTS[e.event]?.meaning ?? "")}</span>
          ${e.step ? `<span class="tag">${esc(e.step)} ${esc(e.state ?? "")}${e.result ? " → " + esc(e.result) : ""}</span>` : ""}
          ${e.category ? `<span class="tag c-${e.category}">${categoryName(e.category)}</span>` : ""}
          ${e.bin ? `<span class="tag">→ ${esc(e.bin)}</span>` : ""}
          ${e.summary || e.reason ? `<span class="dim">${esc(e.summary ?? e.reason)}</span>` : ""}</li>`).join("");
      if ($("lFeed").dataset.sig !== fHtml) { $("lFeed").dataset.sig = fHtml; $("lFeed").innerHTML = fHtml; }
    },
  };
}
