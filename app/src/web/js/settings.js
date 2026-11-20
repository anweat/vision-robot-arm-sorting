// 设置与连接：网关/控制台状态、页面偏好（扩展与调试、报警提示音）、语音与手势规则（只读）、
// 板卡端还缺的接口（能力表里 status 不是 ready 的）。

import { fetchHealth, fetchInteract } from "./api.js";
import { STATUS_TEXT } from "./scripttree.js";

const esc = (s) => String(s ?? "").replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" })[c]);

export function createSettingsView(root, { catalog, prefs }) {
  root.innerHTML = `
    <div class="grid-2">
      <section class="panel">
        <h2>连接 <small>网关（本页的服务）与控制台（执行侧）</small></h2>
        <div class="kv" id="cConn"></div>
        <button id="cRefresh">刷新</button>
      </section>
      <section class="panel">
        <h2>页面偏好 <small>只存在这个浏览器里</small></h2>
        <label class="field row"><input type="checkbox" id="cDebug"> 扩展与调试（预设动作、语音/手势调试输入）；也可用 <code>?debug=1</code></label>
        <label class="field row"><input type="checkbox" id="cBeep"> 新报警时响一声</label>
        <p class="hint">作业台画面布局（并排 / 三维 / 相机）在作业台右上角切换，同样记在本浏览器。</p>
      </section>
    </div>
    <section class="panel">
      <h2>语音与手势规则 <small>控制台 app/config/interaction.json；改规则不用改代码</small></h2>
      <div id="cInteract" class="dim">读取中…</div>
    </section>
    <section class="panel">
      <h2>板卡端待接入 <small>能力表里还不能直接用的项；脚本可以写，运行时按状态拒绝或提示</small></h2>
      <table class="table"><thead><tr><th>能力</th><th>状态</th><th>缺什么</th></tr></thead><tbody id="cTodo"></tbody></table>
    </section>`;
  const $ = (id) => root.querySelector("#" + id);
  $("cDebug").addEventListener("change", () => prefs.setDebug($("cDebug").checked));
  $("cBeep").addEventListener("change", () => prefs.setBeep($("cBeep").checked));

  async function load() {
    let h = {};
    try { h = await fetchHealth(); } catch { /* 网关不通 */ }
    const rows = [["网关", h.ok ? "在线" : "连不上"], ["日志契约", h.contract_version ?? "—"], ["模式", h.mode ?? "—"],
      ["控制台", h.console ?? "未接"], ["控制台类型", !h.console ? "—" : h.console_sim ? "仿真" : "实机"],
      ["控制台回读", h.console_ready ? "正常" : h.console_error ?? "—"], ["页面口令", new URLSearchParams(location.search).get("token") || sessionStorage.getItem("armToken") ? "已带" : "未用"]];
    $("cConn").innerHTML = rows.map(([k, v]) => `<span>${k}</span><b>${esc(v)}</b>`).join("");
    try {
      const it = await fetchInteract();
      $("cInteract").innerHTML = `<div class="kv">
        <span>武装</span><b>${it.armed ? "已武装" : "未武装（只有“停止”直接执行）"}</b>
        <span>语音词表</span><b>${Object.entries(it.voice?.words ?? {}).map(([k, w]) => `${esc(k)}：${esc(w.join("/"))}`).join("<br>")}</b>
        <span>手势映射</span><b>${Object.entries(it.gesture?.map ?? {}).map(([k, v]) => `${esc(k)} → ${esc(v)}`).join("，")}</b>
        <span>手势判定</span><b>置信度 ≥ ${it.gesture?.threshold}，连续 ${it.gesture?.frames} 帧，冷却 ${it.gesture?.cooldown_s} s</b></div>
        <p class="hint">脚本里的「等手势」读的就是这里的识别结果；识别端武装时，手势还会执行上面映射的指令。</p>`;
    } catch {
      $("cInteract").textContent = "读不到：未接控制台，或控制台没有 /api/interact";
    }
  }
  $("cRefresh").addEventListener("click", load);
  load();

  return {
    show() { load(); },
    update() {
      $("cDebug").checked = prefs.debug();
      $("cBeep").checked = prefs.beep();
      const cat = catalog();
      const html = cat ? Object.entries(cat.capabilities).filter(([, c]) => c.status !== "ready").map(([n, c]) =>
        `<tr><td><b>${esc(c.label)}</b> <code>${esc(n)}</code></td><td><span class="badge ${c.status === "partial" ? "warn" : "bad"}">${STATUS_TEXT[c.status]}</span></td><td>${esc(c.needs)}</td></tr>`).join("")
        : `<tr><td colspan="3" class="dim">能力表还没拉到</td></tr>`;
      if ($("cTodo").dataset.sig !== html) { $("cTodo").dataset.sig = html; $("cTodo").innerHTML = html; }
    },
  };
}
