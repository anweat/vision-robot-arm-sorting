// 任务页：完整流程的启停、S1–S7 步骤条（每步可单独执行）、本次流程记录（文档 32 §2–§3）。
// 步骤的状态来自日志里的 step 事件（store.flow），块与桶、进程状态来自 GET /api/flow。

import { FLOW_STEPS, FLOW_EXTRA_STEPS, ALARM_NAMES, categoryName, resultName } from "./contract.js";
import { sendCmd, fetchFlow } from "./api.js";
import { toast, fmtRel } from "./format.js";

const esc = (s) => String(s ?? "").replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" })[c]);

// 结果词的中文：与 src/flow/steps.py 的 result 一一对应
const RESULT_TEXT = {
  done: "完成", empty: "没有物块", found: "找到", gone: "找不到", ok: "通过", rejected: "解算不通过",
  no_destination: "没有对应的桶", failed: "失败", holding: "夹住", slipped: "滑脱", jammed: "卡住",
  mismatch: "不一致",
};
// 同一个词在不同步骤里意思不同：检查的 empty 是空爪，扫描的 empty 是桌上没块
const STEP_RESULT_TEXT = { check: { empty: "空爪" } };
const resultText = (step, result) => STEP_RESULT_TEXT[step]?.[result] ?? RESULT_TEXT[result] ?? result ?? "";

export function createFlowView(root) {
  root.innerHTML = `
    <section class="panel">
      <h2>完整流程 <small><span id="flowScriptBy">按 </span><code id="flowScript">config/flows/sort.yaml</code><span id="flowScriptHint"> 依次执行；改顺序、重试次数只改脚本</span></small></h2>
      <div class="row">
        <button id="flowStart" class="primary">开始分拣</button>
        <button id="flowStop" class="danger">停止</button>
        <span class="chip" id="flowChip">空闲</span>
        <span class="dim" id="flowSession"></span>
      </div>
      <pre class="flow-out" id="flowOut" hidden></pre>
    </section>
    <section class="panel">
      <h2>步骤 <small>每一步都能单独执行，从本会话上一步的产物接着做；带 ● 的会动机械臂</small></h2>
      <p class="hint" id="flowTrial" hidden>试验功能：这些单步用的是新规划器，它对实机场景还太保守（多数块规划不出），优化前「开始」不走这里。</p>
      <div class="row">
        <label>物块 <select id="flowBlock"></select></label>
        <span class="dim" id="flowScene"></span>
      </div>
      <div class="steps" id="flowSteps"></div>
      <div class="row">
        <span class="dim">其他</span>
        ${FLOW_EXTRA_STEPS.map((s) => `<button data-step="${s.key}">${s.moves ? "● " : ""}${s.label}</button>`).join("")}
      </div>
      <p class="hint">单步执行会先放开本页的操作权（流程用自己的身份取得）；同一时刻只跑一个流程或一步。“停止”随时有效，停在原地、不自动恢复。</p>
    </section>
    <section class="panel">
      <h2>本次记录 <small>步骤结果、逐块记录与报警，最新在下</small></h2>
      <table class="table"><thead><tr><th>时间</th><th>步骤</th><th>物块</th><th>结果</th><th>说明</th></tr></thead>
      <tbody id="flowLog"></tbody></table>
    </section>`;

  const $ = (id) => root.querySelector("#" + id);
  const stepsEl = $("flowSteps");
  stepsEl.innerHTML = FLOW_STEPS.map((s, i) => `
    <div class="step" data-key="${s.key}">
      <div class="step-head"><span class="step-no">S${i + 1}</span><b>${s.label}</b>${s.moves ? '<span class="dim" title="会动机械臂">●</span>' : ""}</div>
      <div class="step-state">未执行</div>
      <div class="step-sum dim"></div>
      <button data-step="${s.key}">单独执行</button>
    </div>`).join("");

  let flow = null;          // GET /api/flow 的最近一次结果
  let busy = false;

  async function refresh() {
    try {
      flow = await fetchFlow();
    } catch {
      flow = null;
    }
  }

  async function run(payload, ok) {
    if (busy) return;
    busy = true;
    try {
      await sendCmd(payload.cmd, payload);
      toast(ok, "good");
    } catch (err) {
      toast(err.message, "bad");
    } finally {
      busy = false;
      setTimeout(refresh, 400);
    }
  }

  $("flowStart").addEventListener("click", () => run({ cmd: "start" }, "完整流程已启动"));
  $("flowStop").addEventListener("click", () => run({ cmd: "stop" }, "已停止"));
  root.addEventListener("click", (ev) => {
    const btn = ev.target.closest("button[data-step]");
    if (!btn) return;
    const key = btn.dataset.step;
    const spec = [...FLOW_STEPS, ...FLOW_EXTRA_STEPS].find((s) => s.key === key);
    const block = $("flowBlock").value || null;
    if (spec.block && !block) return toast("先扫描，再选一块", "bad");
    run({ cmd: "flow_step", step: key, block: spec.block ? block : null, session: null },
        `${spec.label}已开始${spec.block ? `（${block}）` : ""}`);
  });

  setInterval(() => { if (!root.hidden) refresh(); }, 2000);
  refresh();

  return {
    update(store) {
      const p = flow?.process ?? {};
      const f = store.flow;
      const running = !!p.running;
      $("flowChip").textContent = running ? "运行中" : p.exit_code == null ? "空闲" : p.exit_code === 0 ? "上次正常结束" : `上次退出码 ${p.exit_code}`;
      $("flowChip").className = "chip " + (running ? "on" : p.exit_code ? "warn" : "");
      $("flowStart").disabled = running;
      $("flowSession").textContent = f.session || flow?.session ? `会话 ${f.session ?? flow.session}` : "";
      // 「开始」跑哪一个（网关 --task）：默认已验证的旧流水线；分步编排器的新规划器还在优化（文档 34 §4.2）
      if (p.task === "pipeline") $("flowScript").textContent = "旧流水线（扫描→定位→抓放，已在实机验证）";
      else if (flow?.script) $("flowScript").textContent = flow.script;
      $("flowTrial").hidden = p.task !== "pipeline";
      $("flowScriptBy").hidden = $("flowScriptHint").hidden = p.task === "pipeline";
      // 起不来时把子进程最后几行给出来（例如解释器缺依赖），不只是"没动静"
      const out = $("flowOut");
      out.hidden = running || !p.exit_code || !(p.output ?? []).length;
      out.textContent = (p.output ?? []).join("\n");

      // 块选择：保留当前选项；桶里的块已分拣，不给选
      const sel = $("flowBlock");
      const keep = sel.value;
      const blocks = flow?.blocks ?? [];
      const sig = JSON.stringify(blocks);
      if (sel.dataset.sig !== sig) {
        sel.dataset.sig = sig;
        sel.innerHTML = `<option value="">—</option>` + blocks.map((b) =>
          `<option value="${esc(b.name)}"${b.in_bin ? " disabled" : ""}>${esc(b.name)} · ${esc(categoryName(b.category))} · (${b.centre_cm.join(", ")}) cm${b.in_bin ? ` · 已在 ${esc(b.in_bin)}` : ""}</option>`).join("");
        if (blocks.some((b) => b.name === keep)) sel.value = keep;
      }
      $("flowScene").textContent = flow?.scan
        ? `${flow.scan}：桌上 ${blocks.filter((b) => !b.in_bin).length} 块，桶 ${(flow.bins ?? []).map((b) => `${b.name}（${categoryName(b.category)}）`).join("、") || "无"}`
        : "还没有扫描结果";

      // 步骤条：每步最后一次事件
      for (const el of stepsEl.querySelectorAll(".step")) {
        const e = f.steps[el.dataset.key];
        const state = !e ? "idle" : e.state === "started" ? "run" : e.state === "done" ? "ok" : "bad";
        el.className = "step " + state;
        el.querySelector(".step-state").textContent = !e ? "未执行"
          : e.state === "started" ? (e.summary || "进行中…")
          : `${resultText(e.step, e.result)}${e.block ? ` · ${e.block}` : ""}`;
        el.querySelector(".step-sum").textContent = e && e.state !== "started" ? (e.summary ?? "") : "";
        el.querySelector(".step-sum").title = e?.summary ?? "";
      }
      root.querySelectorAll("button[data-step]").forEach((b) => { b.disabled = running || busy; });

      // 记录：时间相对本次第一条（板卡时钟不可信，见 format.js）
      const base = f.log[0]?.t;
      $("flowLog").innerHTML = f.log.slice().reverse().slice(0, 40).reverse().map((e) => {
        if (e.event === "alarm") {
          return `<tr class="bad"><td>${fmtRel(e.t, base)}</td><td>报警</td><td>${esc(e.block ?? "")}</td>
            <td>${esc(ALARM_NAMES[e.kind] ?? e.kind)}</td><td>${esc(e.summary)}</td></tr>`;
        }
        if (e.event === "step") {
          const name = [...FLOW_STEPS, ...FLOW_EXTRA_STEPS].find((s) => s.key === e.step)?.label ?? e.step;
          const cls = e.state === "done" ? "" : e.state === "started" ? "dim" : "mid";
          return `<tr class="${cls}"><td>${fmtRel(e.t, base)}</td><td>${esc(name)}</td><td>${esc(e.block ?? "")}</td>
            <td>${esc(e.state === "started" ? "开始" : resultText(e.step, e.result))}</td><td>${esc(e.summary ?? "")}</td></tr>`;
        }
        const what = { pipeline_sorted: "sorted", pipeline_missed: "missed", pipeline_skip: "skipped" }[e.event];
        return `<tr class="${what === "sorted" ? "good" : "mid"}"><td>${fmtRel(e.t, base)}</td><td>记录</td>
          <td>${esc(categoryName(e.category))}</td><td>${esc(resultName(what))}</td>
          <td>${esc(e.bin ? `→ ${e.bin}` : e.reason ?? "")}</td></tr>`;
      }).join("") || `<tr><td colspan="5" class="dim">还没有流程记录</td></tr>`;
    },
    show() { refresh(); },
  };
}
