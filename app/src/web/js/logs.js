// 记录与日志：逐块记录（六项）、统计、原始事件流。数据都按日志契约读。

import { EVENTS, categoryName, resultName } from "./contract.js";
import { fmtXY, cls, fmtRel, toast } from "./format.js";
import { exportCsv, exportXls } from "./export.js";

const esc = (s) => String(s ?? "").replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" })[c]);

export function createLogsView(root) {
  root.innerHTML = `
    <section class="panel">
      <h2>统计 <small>块成功率 = 已分拣 /（已分拣 + 夹空 + 跳过 + 失败）；挪开不计入</small></h2>
      <div class="cards" id="lSum"></div>
    </section>
    <section class="panel">
      <div class="row spread"><h2 style="margin:0">逐块记录 <small>时间 / 类别 / 来源 / 目标 / 耗时 / 结果</small></h2>
        <span class="row" style="margin:0">
          <label>只看 <select id="lRound"><option value="cur">本轮</option><option value="all">全部</option></select></label>
          <input id="lSession" type="text" placeholder="按会话/run 筛选（卡号筛选用：会话名带卡号即可）" style="width:280px">
          <button id="lExportCsv" type="button">导出 CSV</button>
          <button id="lExportXls" type="button">导出 Excel</button>
        </span></div>
      <table><thead><tr><th>#</th><th>时间</th><th>轮</th><th>会话</th><th>类别</th><th>来源 cm</th><th>目标位</th><th>耗时 s</th><th>结果</th><th>原因</th></tr></thead>
      <tbody id="lRecords"></tbody></table>
    </section>
    <section class="panel">
      <div class="row spread"><h2 style="margin:0">事件流 <small>只列事件表内的事件，最新的在上</small></h2>
        <input id="lFilter" type="text" placeholder="按事件名筛选，如 step / alarm / pipeline" style="width:260px"></div>
      <ol id="lFeed" class="feed tall" reversed></ol>
    </section>`;
  const $ = (id) => root.querySelector("#" + id);
  let filter = "";
  let sessionFilter = "";
  let latest = null;
  $("lFilter").addEventListener("input", () => { filter = $("lFilter").value.trim(); });
  // 会话/run 子串筛选（不区分大小写）：记录按 run 筛，事件按 run 或 session 筛。
  // 这是「按卡号筛选」的底层机制——H1 给会话标上卡号（R00…/D1…）后，同一个框就能筛出该卡的会话。
  const runLabel = (r) => String(r?.run ?? "").split(/[\\/]/).pop() || "—";
  const sessMatch = (e) => !sessionFilter ||
    ((e.run ?? "") + " " + (e.session ?? "")).toLowerCase().includes(sessionFilter);
  $("lSession").addEventListener("input", () => { sessionFilter = $("lSession").value.trim().toLowerCase(); });
  // 导出与表格共用同一套筛选（本轮/全部 + 会话）；导出不截断 200 条，导出行数 = 筛选后记录数。
  const filteredRecords = () => {
    const store = latest;
    if (!store) return [];
    const cur = store.current ?? store.rounds.at(-1) ?? null;
    const roundSel = $("lRound").value === "all" ? store.records
      : cur ? store.records.filter((r) => r.t >= cur.t && (!cur.endedAt || r.t <= cur.endedAt))
      : store.records.filter((r) => r.round === (store.records.at(-1)?.round ?? 0));
    return roundSel.filter(sessMatch);
  };
  $("lExportCsv").addEventListener("click", () => {
    const recs = filteredRecords();
    exportCsv(recs, "records.csv");
    toast(`已导出 ${recs.length} 条记录（CSV）`);
  });
  $("lExportXls").addEventListener("click", () => {
    const recs = filteredRecords();
    exportXls(recs, "records.xls");
    toast(`已导出 ${recs.length} 条记录（Excel）`);
  });
  return {
    update(store) {
      latest = store;
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
      const rows = filteredRecords().slice(-200).reverse();
      const rHtml = rows.map((r) => `<tr><td>${r.seq}</td><td>${fmtRel(r.t, base)}</td><td>${r.round ?? "—"}</td>
        <td class="dim">${esc(runLabel(r))}</td>
        <td>${categoryName(r.category)}</td><td>${fmtXY(r.cube)}</td><td>${esc(r.bin ?? "—")}</td>
        <td>${r.seconds == null ? "—" : r.seconds.toFixed(1)}</td><td class="${cls(r.result)}">${resultName(r.result)}</td>
        <td class="dim">${esc(r.reason ?? "")}</td></tr>`).join("") || `<tr><td colspan="10" class="dim">还没有逐块记录</td></tr>`;
      if ($("lRecords").dataset.sig !== rHtml) { $("lRecords").dataset.sig = rHtml; $("lRecords").innerHTML = rHtml; }

      const recent = store.events.filter((e) =>
        (!filter || e.event.includes(filter)) && sessMatch(e)).slice(-150).reverse();
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
