// F1-2 异常日志页：出问题的事件单独成页，便于验收“识别、抓取、投放、路径四类失败都有处理”。
// 只列契约 §3 里 FAULT_EVENTS 的事件；未知物体、堵转、溢满是拟定事件，有数据就显示。

import { EVENTS, FAULT_EVENTS, categoryName } from "./contract.js";
import { fmtXY, fmtRel, fmtSeconds } from "./format.js";

export function createFaultsView(root) {
  root.innerHTML = `
    <section class="panel">
      <h2>异常事件 <small>按事件类型筛选；不在事件表内的事件不计入</small></h2>
      <div class="row" id="faultFilter"></div>
      <table id="faultTable"><thead><tr>
        <th>#</th><th>时间</th><th>事件</th><th>含义</th><th>类别</th>
        <th>位置 cm</th><th>目标位</th><th>耗时</th><th>原因</th>
      </tr></thead><tbody></tbody></table>
    </section>
    <section class="panel">
      <h2>分布 <small>各异常类型的条数</small></h2>
      <div id="faultCounts" class="bars"></div>
    </section>`;

  const filterEl = root.querySelector("#faultFilter");
  const tbody = root.querySelector("#faultTable tbody");
  const countsEl = root.querySelector("#faultCounts");
  let active = "all";

  filterEl.innerHTML = [["all", "全部"], ...FAULT_EVENTS.map((e) => [e, e])]
    .map(([value, label]) => `<button data-f="${value}"${value === active ? ' class="on"' : ""}>${label}</button>`)
    .join("");
  filterEl.addEventListener("click", (ev) => {
    const btn = ev.target.closest("button");
    if (!btn) return;
    active = btn.dataset.f;
    filterEl.querySelectorAll("button").forEach((b) => b.classList.toggle("on", b === btn));
    render();
  });

  let cache = [];

  function render() {
    const base = cache[0]?.t;
    const rows = cache.filter((e) => active === "all" || e.event === active);
    tbody.innerHTML = rows.map((e) => `
      <tr><td>${e.seq}</td><td>${fmtRel(e.t, base)}</td>
      <td><b>${e.event}</b></td><td class="dim">${EVENTS[e.event]?.meaning ?? ""}</td>
      <td>${e.category ? categoryName(e.category) : "—"}</td>
      <td>${fmtXY(e.cube)}</td><td>${e.bin ?? "—"}</td>
      <td>${fmtSeconds(e.seconds)}</td>
      <td class="dim">${e.reason ?? ""}</td></tr>`).join("")
      || `<tr><td colspan="9" class="dim">没有这类异常</td></tr>`;

    const groups = new Map();
    for (const e of cache) groups.set(e.event, (groups.get(e.event) ?? 0) + 1);
    const max = Math.max(1, ...groups.values());
    countsEl.innerHTML = [...groups.entries()].sort((a, b) => b[1] - a[1]).map(([name, n]) => `
      <div class="barrow"><span>${name}</span>
        <div class="bar"><i style="width:${(n / max) * 100}%"></i></div><b>${n}</b></div>`).join("")
      || `<p class="dim">还没有异常事件</p>`;
  }

  return {
    update(store) {
      cache = store.events.filter((e) => FAULT_EVENTS.includes(e.event));
      render();
    },
  };
}
