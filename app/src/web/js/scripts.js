// 流程脚本页：脚本库（网关上的同一份，各端共用）、结构视图与可视化编辑、YAML 原文、能力表、单步调试。
//
// 解析与校验只在服务端（POST /api/scripts/check，src/flow/script.py）：页面把文本发过去，拿回解析后的结构和
// 逐路径的错误/警告来画，所以页面、手机 App、板上编排器看到的是同一套规则。结构编辑在页面里改对象，
// 再用 yaml.js 写回文本（会丢注释，编辑前提示一次）。保存带 rev 做乐观并发：别的端在这之间存过就 409。

import { FLOW_STEPS, FLOW_EXTRA_STEPS, categoryName } from "./contract.js";
import { fetchScripts, fetchScript, checkScript, saveScript, deleteScript, sendCmd, fetchFlow } from "./api.js";
import { toast } from "./format.js";
import { dump } from "./yaml.js";
import { renderTree, describe, KIND_OF, STATUS_TEXT, getAt, parentOf, nodePathOf } from "./scripttree.js";

const esc = (s) => String(s ?? "").replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" })[c]);
const clone = (o) => JSON.parse(JSON.stringify(o ?? null));

const CONTROL_TEMPLATES = {
  for_each: () => ({ for_each: "blocks", order: "reach", do: [] }),
  retry: () => ({ retry: { times: 2, until: "holding", do: [], else: [] } }),
  repeat: () => ({ repeat: { times: 2, do: [] } }),
  alarm: () => ({ alarm: "custom", then: "continue", summary: "需要人看一下" }),
  record: () => ({ record: "missed", reason: "" }),
  next_block: () => ({ next_block: true }),
  stop: () => ({ stop: "到此为止" }),
};
const NEW_SCRIPT = { title: "新流程", description: "", steps: [{ step: "notify", with: { text: "开始" } }] };

export function createScriptsView(root, { catalog }) {
  root.innerHTML = `
    <div class="scripts">
      <aside class="panel script-lib">
        <div class="row spread"><h2 style="margin:0">脚本库 <small>各端共用</small></h2><button id="sNew">新建</button></div>
        <ul class="lib" id="sLib"></ul>
        <p class="hint">内置脚本随应用发布（<code>config/flows/</code>），只读，可另存后改；保存的在 <code>data/flows/</code>，任一端存了别的端会自动刷新。</p>
      </aside>
      <section class="script-main">
        <div class="panel">
          <div class="row spread">
            <div class="script-title"><b id="sTitle">—</b> <code id="sName"></code> <span id="sBadges"></span></div>
            <div class="row" style="margin:0">
              <button id="sRun" class="primary">运行</button>
              <button id="sStop" class="danger">停止</button>
              <span class="sep"></span>
              <button id="sSave">保存</button><button id="sSaveAs">另存为</button>
              <button id="sRevert">还原</button><button id="sDelete" class="danger">删除</button>
            </div>
          </div>
          <p class="hint" id="sDesc"></p>
          <div class="banner warn" id="sConflict" hidden></div>
          <nav class="subtabs" id="sTabs">
            <button data-tab="tree" class="on">结构</button><button data-tab="yaml">YAML</button>
            <button data-tab="caps">能力表</button><button data-tab="steps">单步调试</button>
          </nav>
          <div data-pane="tree">
            <div class="tree-layout">
              <div class="tree" id="sTree"></div>
              <div class="tree-edit" id="sEdit"></div>
            </div>
          </div>
          <div data-pane="yaml" hidden>
            <textarea id="sText" class="yaml" spellcheck="false"></textarea>
            <p class="hint">语法：<code>- step: 能力名</code> + <code>with: {参数}</code> + <code>on_result: {结果: [动作]}</code>；控制动作 for_each / retry / repeat / alarm / record / next_block / stop。改完停顿一下就会在服务端重新校验。</p>
          </div>
          <div data-pane="caps" hidden><div id="sCaps"></div></div>
          <div data-pane="steps" hidden><div id="sSteps"></div></div>
        </div>
        <section class="panel">
          <h2>校验 <small id="sCheckHint">服务端解析，与板上编排器同一套规则</small></h2>
          <ul class="issues" id="sIssues"></ul>
        </section>
      </section>
    </div>`;

  const $ = (id) => root.querySelector("#" + id);
  let lib = [], doc = null, tab = "tree", selected = null, addTo = null, scriptRev = -1;
  let running = null, checkTimer = null, commentWarned = false, lastStore = null;

  // ---- 脚本库 ----------------------------------------------------------------------------------------
  async function refreshLib() {
    try { const r = await fetchScripts(); lib = r.scripts ?? []; running = r.running ?? null; } catch { lib = []; }
    paintLib();
  }
  function paintLib() {
    $("sLib").innerHTML = lib.map((s) => `
      <li class="${doc?.name === s.name ? "on" : ""}" data-name="${esc(s.name)}">
        <b>${esc(s.title)}</b>
        <span class="dim">${esc(s.name)}${s.builtin ? " · 内置" : " · 已保存"}</span>
        <span>${s.moves ? '<span class="badge" title="会动机械臂">动臂</span>' : ""}${!s.ok ? '<span class="badge bad">有错误</span>'
          : !s.runnable ? '<span class="badge bad">不可运行</span>' : s.partial?.length ? '<span class="badge warn">待接入</span>' : ""}${running === s.name ? '<span class="badge good">运行中</span>' : ""}</span>
      </li>`).join("") || `<li class="dim">网关上没有脚本（或连不上网关）</li>`;
  }
  $("sLib").addEventListener("click", (ev) => {
    const li = ev.target.closest("li[data-name]");
    if (li) open(li.dataset.name);
  });

  async function open(name) {
    if (doc?.dirty && !confirm("当前脚本有没保存的改动，放弃吗？")) return;
    try {
      const got = await fetchScript(name);
      doc = { name: got.name, builtin: got.builtin, rev: got.rev, text: got.text, check: got.check,
              script: got.check?.script ?? null, dirty: false, isNew: false };
      selected = null; addTo = null; commentWarned = false;
      $("sConflict").hidden = true;
      paint();
    } catch (err) {
      toast(`打不开 ${name}：${err.message}`, "bad");
    }
  }
  $("sNew").addEventListener("click", () => {
    if (doc?.dirty && !confirm("当前脚本有没保存的改动，放弃吗？")) return;
    const script = clone(NEW_SCRIPT);
    doc = { name: null, builtin: false, rev: null, text: dump(script), script, check: null, dirty: true, isNew: true };
    selected = null;
    recheck(true);
    paint();
  });

  // ---- 校验 --------------------------------------------------------------------------------------------
  async function recheck(now = false) {
    clearTimeout(checkTimer);
    const run = async () => {
      if (!doc) return;
      const text = doc.text;
      try {
        const c = await checkScript(text);
        if (doc?.text !== text) return;            // 期间又改了，等下一次
        doc.check = c;
        if (c.script) doc.script = c.script;
      } catch (err) {
        $("sCheckHint").textContent = `校验失败：${err.message}`;
      }
      paint();
    };
    if (now) run(); else checkTimer = setTimeout(run, 450);
  }

  function edit(mutator) {
    if (!doc?.script) return toast("先修好 YAML 的解析错误，再用结构编辑", "bad");
    if (!commentWarned && !doc.isNew && /#/.test(doc.text)) {
      commentWarned = true;
      toast("结构编辑会重写 YAML，原文里的注释不会保留");
    }
    const script = clone(doc.script);
    mutator(script);
    doc.script = script;
    doc.text = dump(script);
    doc.dirty = true;
    recheck();
    paint();
  }

  // ---- 结构编辑：增删移、选中改参数 ----------------------------------------------------------------------
  $("sTree").addEventListener("click", (ev) => {
    const btn = ev.target.closest("[data-act]");
    if (!btn || !doc) return;
    const { act, path } = btn.dataset;
    if (act === "select") { selected = path; addTo = null; return paint(); }
    if (act === "add") { addTo = path; selected = null; return paint(); }
    ev.stopPropagation();
    if (act === "up" || act === "down") {
      edit((s) => {
        const [arr, i] = parentOf(s, path);
        const j = act === "up" ? i - 1 : i + 1;
        if (!Array.isArray(arr) || j < 0 || j >= arr.length) return;
        [arr[i], arr[j]] = [arr[j], arr[i]];
        selected = path.replace(/\[\d+\]$/, `[${j}]`);
      });
    } else if (act === "del") {
      const d = describe(getAt(doc.script, path), catalog());
      if (!confirm(`删除「${d.title}」${KIND_OF(getAt(doc.script, path)) !== "step" ? "及其下面的动作" : ""}？`)) return;
      edit((s) => { const [arr, i] = parentOf(s, path); arr.splice(i, 1); selected = null; });
    } else if (act === "del-branch") {
      edit((s) => {
        const m = path.match(/^(.*)\.on_result\.([a-z_]+)$/);
        const step = getAt(s, m[1]);
        delete step.on_result[m[2]];
        if (!Object.keys(step.on_result).length) delete step.on_result;
      });
    }
  });

  function insert(action) {
    const target = addTo;
    edit((s) => {
      const arr = getAt(s, target);
      if (!Array.isArray(arr)) return;
      arr.push(action);
      selected = `${target}[${arr.length - 1}]`;
    });
    addTo = null;
    paint();
  }

  function stepTemplate(name) {
    const cap = catalog()?.capabilities?.[name];
    const w = {};
    for (const [k, p] of Object.entries(cap?.params ?? {})) if (p.required) w[k] = p.default ?? "";
    return Object.keys(w).length ? { step: name, with: w } : { step: name };
  }

  function paintPalette() {
    const cat = catalog();
    if (!cat) return `<p class="dim">能力表还没拉到</p>`;
    const inBlock = /\.do(\[|$)/.test(addTo) && /for_each|\.do/.test(addTo) && String(addTo).includes(".do");
    const groups = Object.entries(cat.groups).map(([g, label]) => {
      const caps = Object.entries(cat.capabilities).filter(([, c]) => c.group === g);
      if (!caps.length) return "";
      return `<h3>${esc(label)}</h3><div class="palette">${caps.map(([n, c]) => `
        <button data-add-step="${esc(n)}" class="cap-${c.status}" title="${esc(c.help)}${c.needs ? "\n" + esc(c.needs) : ""}">
          ${c.moves ? "● " : ""}${esc(c.label)}<small>${esc(n)}${c.status !== "ready" ? " · " + STATUS_TEXT[c.status] : ""}</small></button>`).join("")}</div>`;
    }).join("");
    const ctl = Object.entries(cat.control).filter(([k]) => k !== "step").map(([k, c]) => `
      <button data-add-ctl="${esc(k)}" title="${esc(c.help ?? "")}">${esc(c.label)}<small>${esc(k)}</small></button>`).join("");
    return `<div class="row spread"><b>添加到 <code>${esc(addTo)}</code></b><button data-add-cancel>取消</button></div>
      ${groups}<h3>流程控制</h3><div class="palette">${ctl}</div>
      <p class="hint">● 会动机械臂；黄色“待接入”可以写进脚本，运行时要有输入端；红色“不可用”写进去后脚本会被拒绝运行（板卡端还没接口）。${inBlock ? "" : "record / next_block / skip_block 只能在「对每一块」里用。"}</p>`;
  }

  function field(key, spec, value) {
    const id = `f_${key}`;
    const v = value === undefined ? "" : value;
    let input;
    if (spec.type === "enum") {
      input = `<select data-field="${esc(key)}" data-type="enum">${(value === undefined && !spec.required ? [`<option value="__unset">（默认 ${esc(String(spec.default))}）</option>`] : [])
        .concat(spec.enum.map((o) => `<option value="${esc(JSON.stringify(o))}"${JSON.stringify(o) === JSON.stringify(value) ? " selected" : ""}>${esc(String(o))}</option>`)).join("")}</select>`;
    } else if (spec.type === "num" || spec.type === "int") {
      input = `<input type="number" data-field="${esc(key)}" data-type="${spec.type}" value="${esc(v)}" placeholder="${esc(spec.default ?? "")}"
        ${spec.min != null ? `min="${spec.min}"` : ""} ${spec.max != null ? `max="${spec.max}"` : ""} step="${spec.type === "int" ? 1 : "any"}">`;
    } else if (spec.type === "pose") {
      const txt = Array.isArray(value) ? value.map((x) => (x === null ? "null" : x)).join(", ") : "";
      input = `<input type="text" data-field="${esc(key)}" data-type="pose" value="${esc(txt)}" placeholder="90, 105, 5, 5, null, null">`;
    } else if (spec.type === "bool") {
      input = `<input type="checkbox" data-field="${esc(key)}" data-type="bool"${value ? " checked" : ""}>`;
    } else {
      input = `<input type="text" data-field="${esc(key)}" data-type="str" value="${esc(v)}" placeholder="${esc(spec.default ?? "")}">`;
    }
    return `<label class="field" for="${id}"><span>${esc(spec.label ?? key)}${spec.unit ? ` <small>${esc(spec.unit)}</small>` : ""}${spec.required ? " *" : ""}</span>${input}
      ${spec.help ? `<small class="dim">${esc(spec.help)}</small>` : ""}</label>`;
  }

  // 控制动作的可编辑字段：[键路径, 规格]
  function controlFields(a) {
    const cat = catalog();
    const k = KIND_OF(a);
    if (k === "for_each") return [["order", { type: "enum", label: "顺序", enum: cat.orders, default: "reach" }]];
    if (k === "retry") return [["retry.times", { type: "int", label: "次数", min: 1, max: 20, required: true }],
                               ["retry.until", { type: "str", label: "直到结果", help: "最后一步给出这个结果就算成功，如 holding" }]];
    if (k === "repeat") return [["repeat.times", { type: "int", label: "遍数", min: 1, max: 1000, required: true }]];
    if (k === "alarm") return [["alarm", { type: "str", label: "报警类型", required: true }],
                               ["then", { type: "enum", label: "之后", enum: cat.then, default: "continue" }],
                               ["summary", { type: "str", label: "说明" }]];
    if (k === "record") return [["record", { type: "enum", label: "记为", enum: cat.records, required: true }],
                                ["reason", { type: "str", label: "原因" }]];
    if (k === "stop") return [["stop", { type: "str", label: "原因", required: true }]];
    return [];
  }

  function paintEditor() {
    const el = $("sEdit");
    if (addTo) { el.innerHTML = paintPalette(); return; }
    const a = selected ? getAt(doc?.script, selected) : null;
    if (!a) {
      el.innerHTML = `<p class="dim">点左边一行改它的参数；点“＋ 添加”插入能力或流程控制。</p>`;
      return;
    }
    const d = describe(a, catalog());
    let body = "";
    if (d.kind === "step") {
      const cap = d.cap;
      body = cap ? Object.entries(cap.params).map(([k, spec]) => field(`with.${k}`, spec, a.with?.[k])).join("") : `<p class="bad">没有这项能力</p>`;
      if (cap) {
        const have = Object.keys(a.on_result ?? {});
        const free = [...cap.results, "default"].filter((r) => !have.includes(r));
        body += `<div class="field"><span>结果</span><small class="dim">会给出：${cap.results.map((r) => `<code>${esc(r)}</code>${cap.success.includes(r) ? "（顺利）" : ""}`).join(" ")}。
          没写分支的不顺利结果：在“重试”里结束这次尝试，否则报警（块里跳过这一块，块外整轮停）。</small>
          ${free.length ? `<div class="row"><select id="sBranch">${free.map((r) => `<option>${esc(r)}</option>`).join("")}</select><button id="sAddBranch">加结果分支</button></div>` : ""}</div>
          <p class="hint">${esc(cap.help)}${cap.needs ? `<br><b class="${cap.status === "unavailable" ? "bad" : "mid"}">${esc(STATUS_TEXT[cap.status])}：${esc(cap.needs)}</b>` : ""}</p>`;
      }
    } else {
      body = controlFields(a).map(([k, spec]) => field(k, spec, getAt(a, k))).join("") ||
        `<p class="dim">${esc(catalog()?.control?.[d.kind]?.help ?? "没有参数")}</p>`;
    }
    body += field("note", { type: "str", label: "备注", help: "只给人看，运行时忽略" }, a.note);
    el.innerHTML = `<div class="row spread"><b>${d.icon} ${esc(d.title)}</b><code>${esc(selected)}</code></div>${body}`;
  }

  $("sEdit").addEventListener("click", (ev) => {
    const s = ev.target.closest("[data-add-step]"), c = ev.target.closest("[data-add-ctl]");
    if (s) return insert(stepTemplate(s.dataset.addStep));
    if (c) return insert(CONTROL_TEMPLATES[c.dataset.addCtl]());
    if (ev.target.closest("[data-add-cancel]")) { addTo = null; return paint(); }
    if (ev.target.id === "sAddBranch") {
      const r = $("sBranch").value, path = selected;
      edit((sc) => { const a = getAt(sc, path); a.on_result = { ...(a.on_result ?? {}), [r]: [] }; });
    }
  });
  $("sEdit").addEventListener("change", (ev) => {
    const input = ev.target.closest("[data-field]");
    if (!input || !selected) return;
    const key = input.dataset.field, type = input.dataset.type, path = selected;
    let value;
    if (type === "bool") value = input.checked;
    else if (type === "enum") value = input.value === "__unset" ? undefined : JSON.parse(input.value);
    else if (type === "num" || type === "int") value = input.value === "" ? undefined : Number(input.value);
    else if (type === "pose") {
      const parts = input.value.split(/[,，\s]+/).filter(Boolean);
      value = parts.length ? parts.map((x) => (/^(null|-|—)$/i.test(x) ? null : Number(x))) : undefined;
      if (value && value.some((x) => x !== null && !Number.isFinite(x))) return toast("关节角要写数字或 null", "bad");
    }
    else value = input.value === "" ? undefined : input.value;
    edit((sc) => {
      const a = getAt(sc, path);
      const parts = key.split(".");
      let obj = a;
      for (const p of parts.slice(0, -1)) obj = obj[p] ?? (obj[p] = {});
      const last = parts.at(-1);
      if (value === undefined) delete obj[last]; else obj[last] = value;
      if (a.with && !Object.keys(a.with).length) delete a.with;
    });
  });

  // ---- YAML ------------------------------------------------------------------------------------------
  $("sText").addEventListener("input", () => {
    if (!doc) return;
    doc.text = $("sText").value;
    doc.dirty = true;
    recheck();
    paintHead();
  });

  // ---- 保存 / 删除 / 运行 ------------------------------------------------------------------------------
  async function doSave(name, rev) {
    try {
      const got = await saveScript(name, doc.text, rev);
      doc = { name: got.name, builtin: false, rev: got.rev, text: got.text, check: got.check,
              script: got.check?.script ?? doc.script, dirty: false, isNew: false };
      $("sConflict").hidden = true;
      toast(`已保存 ${name}，其他端会自动看到`, "good");
      refreshLib();
      paint();
    } catch (err) {
      if (err.status === 409 && err.body?.current) return showConflict(err.body.current);
      toast(err.message, "bad");
    }
  }
  function showConflict(current) {
    const b = $("sConflict");
    b.hidden = false;
    b.innerHTML = `<b>另一端刚改过「${esc(current.name)}」</b><span>先看看对方的版本再决定</span>
      <button data-conf="theirs">载入对方版本</button><button data-conf="mine" class="danger">用我的覆盖</button>`;
    b.onclick = (ev) => {
      const which = ev.target.closest("[data-conf]")?.dataset.conf;
      if (which === "theirs") { doc.dirty = false; open(current.name); }
      if (which === "mine") doSave(current.name, current.rev);
    };
  }
  const askName = (suggest) => {
    const name = prompt("脚本名（小写字母、数字、_、-）", suggest ?? "");
    return name ? name.trim() : null;
  };
  $("sSave").addEventListener("click", () => {
    if (!doc) return;
    if (doc.builtin || doc.isNew) return $("sSaveAs").click();
    doSave(doc.name, doc.rev);
  });
  $("sSaveAs").addEventListener("click", () => {
    if (!doc) return;
    const name = askName(doc.isNew ? doc.script?.name : `${doc.name}_copy`);
    if (!name) return;
    if (lib.some((s) => s.name === name) && !confirm(`${name} 已存在，覆盖？`)) return;
    doSave(name, null);
  });
  $("sRevert").addEventListener("click", () => {
    if (!doc || doc.isNew) return;
    doc.dirty = false;
    open(doc.name);
  });
  $("sDelete").addEventListener("click", async () => {
    if (!doc || doc.builtin || doc.isNew) return;
    if (!confirm(`删除「${doc.name}」？所有端都会看不到它。`)) return;
    try {
      await deleteScript(doc.name, doc.rev);
      toast("已删除", "good");
      doc = null;
      refreshLib();
      paint();
    } catch (err) {
      if (err.status === 409 && err.body?.current) return showConflict(err.body.current);
      toast(err.message, "bad");
    }
  });
  $("sRun").addEventListener("click", async () => {
    if (!doc || doc.isNew) return toast("先保存，再运行", "bad");
    if (doc.dirty) return toast("有没保存的改动：先保存（运行的是网关上存的那份）", "bad");
    if (doc.check?.moves && !confirm(`「${doc.script?.title ?? doc.name}」会动机械臂。设备周围安全、有人在场？`)) return;
    try {
      await sendCmd("run_script", { name: doc.name });
      toast("已启动；进度在作业台和这一页的结构里", "good");
      setTimeout(refreshLib, 500);
    } catch (err) { toast(err.message, "bad"); }
  });
  $("sStop").addEventListener("click", () => sendCmd("stop").then(() => toast("已停止", "good"), (e) => toast(e.message, "bad")));

  // ---- 子页签 ------------------------------------------------------------------------------------------
  $("sTabs").addEventListener("click", (ev) => {
    const b = ev.target.closest("button[data-tab]");
    if (!b) return;
    tab = b.dataset.tab;
    paint();
  });

  // ---- 校验结果：点一条跳到那一行 --------------------------------------------------------------------------
  $("sIssues").addEventListener("click", (ev) => {
    const li = ev.target.closest("li[data-path]");
    if (!li || !li.dataset.path) return;
    selected = nodePathOf(li.dataset.path) || null;
    addTo = null;
    tab = "tree";
    paint();
    root.querySelector(`.tree-node[data-path="${CSS.escape(selected ?? "")}"]`)?.scrollIntoView({ block: "center", behavior: "smooth" });
  });

  // ---- 能力表 ------------------------------------------------------------------------------------------
  function paintCaps() {
    const cat = catalog();
    if (!cat) { $("sCaps").innerHTML = `<p class="dim">能力表还没拉到</p>`; return; }
    $("sCaps").innerHTML = `
      <p class="hint">板上现有能力，唯一来源 <code>app/src/flow/capabilities.py</code>。“待接入”= 控制台一侧有接口、输入端没接；“不可用”= 板卡端还没接口。</p>
      <table class="table"><thead><tr><th>能力</th><th>分组</th><th>状态</th><th>动臂</th><th>参数</th><th>结果（顺利）</th></tr></thead><tbody>
      ${Object.entries(cat.capabilities).map(([n, c]) => `<tr>
        <td><b>${esc(c.label)}</b><br><code>${esc(n)}</code></td><td>${esc(cat.groups[c.group])}</td>
        <td><span class="badge ${c.status === "ready" ? "good" : c.status === "partial" ? "warn" : "bad"}">${STATUS_TEXT[c.status]}</span>${c.needs ? `<br><small class="dim">${esc(c.needs)}</small>` : ""}</td>
        <td>${c.moves ? "●" : ""}${c.block ? "<br><small class='dim'>要当前块</small>" : ""}</td>
        <td>${Object.entries(c.params).map(([k, p]) => `<code>${esc(k)}</code> ${esc(p.label)}${p.enum ? `：${p.enum.map((x) => esc(String(x))).join(" / ")}` : ""}${p.unit ? ` ${esc(p.unit)}` : ""}`).join("<br>") || "—"}</td>
        <td>${c.results.map((r) => c.success.includes(r) ? `<b>${esc(r)}</b>` : esc(r)).join(" / ")}</td></tr>`).join("")}
      </tbody></table>
      <h3>流程控制</h3>
      <table class="table"><tbody>${Object.entries(cat.control).map(([k, c]) => `<tr><td><code>${esc(k)}</code></td><td>${esc(c.label)}</td><td class="dim">${esc(c.help ?? "")}</td></tr>`).join("")}</tbody></table>`;
  }

  // ---- 画 ----------------------------------------------------------------------------------------------
  function paintHead() {
    const c = doc?.check;
    $("sTitle").textContent = doc ? (doc.script?.title ?? doc.script?.name ?? doc.name ?? "新脚本") : "选一份脚本，或新建";
    $("sName").textContent = doc ? (doc.name ?? "（未保存）") + (doc.dirty ? " · 有改动" : "") : "";
    $("sDesc").textContent = doc?.script?.description ?? "";
    $("sBadges").innerHTML = !doc ? "" : [
      doc.builtin ? '<span class="badge">内置只读</span>' : "",
      !c ? "" : !c.ok ? `<span class="badge bad">${c.errors.length} 个错误</span>` : !c.runnable ? '<span class="badge bad">不可运行</span>' : '<span class="badge good">可运行</span>',
      c?.warnings?.length ? `<span class="badge warn">${c.warnings.length} 个提示</span>` : "",
      c?.moves ? '<span class="badge">动臂</span>' : "",
      running && running === doc.name ? '<span class="badge good">运行中</span>' : "",
    ].join("");
    const busy = !!lastStore?.status?.flow?.running;
    $("sRun").disabled = !doc || doc.isNew || !c?.runnable || busy;
    $("sSave").disabled = !doc || !doc.dirty || (c && !c.ok);
    $("sSaveAs").disabled = !doc || (c && !c.ok);
    $("sRevert").disabled = !doc || doc.isNew || !doc.dirty;
    $("sDelete").disabled = !doc || doc.builtin || doc.isNew;
  }

  function paint() {
    paintLib();
    paintHead();
    root.querySelectorAll("#sTabs button").forEach((b) => b.classList.toggle("on", b.dataset.tab === tab));
    root.querySelectorAll("[data-pane]").forEach((p) => { p.hidden = p.dataset.pane !== tab; });
    const c = doc?.check;
    const issues = {};
    for (const w of c?.warnings ?? []) issues[nodePathOf(w.path)] = "warn";
    for (const e of c?.errors ?? []) issues[nodePathOf(e.path)] = "error";
    if (tab === "tree") {
      const live = running && doc?.name === running ? lastStore?.flow?.paths : null;
      const current = running && doc?.name === running ? lastStore?.flow?.current?.path : null;
      $("sTree").innerHTML = !doc ? `<p class="dim">左边选一份脚本</p>`
        : !doc.script ? `<p class="bad">YAML 解析失败，去「YAML」页签修</p>`
        : renderTree(doc.script, { catalog: catalog(), live, current, issues, selected, editable: true });
      paintEditor();
    }
    if (tab === "yaml" && $("sText").value !== (doc?.text ?? "")) $("sText").value = doc?.text ?? "";
    $("sText").disabled = !doc;
    if (tab === "caps") paintCaps();
    $("sIssues").innerHTML = !doc ? "" : !c ? `<li class="dim">校验中…</li>` : [
      ...c.errors.map((e) => `<li class="bad" data-path="${esc(e.path)}"><b>错误</b> ${esc(e.message)}</li>`),
      ...c.warnings.map((w) => `<li class="mid" data-path="${esc(w.path)}"><b>提示</b> ${esc(w.message)}</li>`),
    ].join("") || `<li class="good">没有问题：用到 ${c.uses.map((u) => esc(catalog()?.capabilities?.[u]?.label ?? u)).join("、") || "—"}</li>`;
  }

  // ---- 单步调试（文档 32 §2）：S1–S7 每步单独执行，从本会话上一步的产物接着做 -----------------------------
  const stepsEl = $("sSteps");
  stepsEl.innerHTML = `
    <p class="hint">试验功能：单步用的是新规划器，它对实机场景还偏保守。单步会先放开本页的操作权；同一时刻只跑一个流程或一步；“停止”随时有效。</p>
    <div class="row"><label>物块 <select id="stBlock"></select></label><span class="dim" id="stScene"></span></div>
    <div class="steps" id="stSteps">${FLOW_STEPS.map((s, i) => `
      <div class="step" data-key="${s.key}">
        <div class="step-head"><span class="step-no">S${i + 1}</span><b>${s.label}</b>${s.moves ? '<span class="dim" title="会动机械臂">●</span>' : ""}</div>
        <div class="step-state">未执行</div><div class="step-sum dim"></div>
        <button data-step="${s.key}">单独执行</button></div>`).join("")}</div>
    <div class="row"><span class="dim">其他</span>${FLOW_EXTRA_STEPS.map((s) => `<button data-step="${s.key}">${s.moves ? "● " : ""}${s.label}</button>`).join("")}</div>`;
  let flowInfo = null;
  const refreshFlowInfo = async () => { try { flowInfo = await fetchFlow(); } catch { flowInfo = null; } };
  stepsEl.addEventListener("click", async (ev) => {
    const btn = ev.target.closest("button[data-step]");
    if (!btn) return;
    const spec = [...FLOW_STEPS, ...FLOW_EXTRA_STEPS].find((s) => s.key === btn.dataset.step);
    const block = $("stBlock").value || null;
    if (spec.block && !block) return toast("先扫描，再选一块", "bad");
    try {
      await sendCmd("flow_step", { step: spec.key, block: spec.block ? block : null, session: null });
      toast(`${spec.label}已开始`, "good");
    } catch (err) { toast(err.message, "bad"); }
    setTimeout(refreshFlowInfo, 400);
  });
  setInterval(() => { if (!root.hidden && tab === "steps") refreshFlowInfo(); }, 2000);

  function paintSteps(store) {
    const blocks = flowInfo?.blocks ?? [];
    const sel = $("stBlock"), sig = JSON.stringify(blocks);
    if (sel.dataset.sig !== sig) {
      const keep = sel.value;
      sel.dataset.sig = sig;
      sel.innerHTML = `<option value="">—</option>` + blocks.map((b) =>
        `<option value="${esc(b.name)}"${b.in_bin ? " disabled" : ""}>${esc(b.name)} · ${esc(categoryName(b.category))} · (${b.centre_cm.join(", ")}) cm${b.in_bin ? ` · 已在 ${esc(b.in_bin)}` : ""}</option>`).join("");
      if (blocks.some((b) => b.name === keep)) sel.value = keep;
    }
    $("stScene").textContent = flowInfo?.scan ? `${flowInfo.scan}：桌上 ${blocks.filter((b) => !b.in_bin).length} 块` : "还没有扫描结果";
    const busy = !!flowInfo?.process?.running;
    for (const el of stepsEl.querySelectorAll(".step")) {
      const e = store.flow.steps[el.dataset.key];
      el.className = "step " + (!e ? "idle" : e.state === "started" ? "run" : e.state === "done" ? "ok" : "bad");
      el.querySelector(".step-state").textContent = !e ? "未执行" : e.state === "started" ? (e.summary || "进行中…") : `${e.result ?? ""}${e.block ? ` · ${e.block}` : ""}`;
      el.querySelector(".step-sum").textContent = e && e.state !== "started" ? (e.summary ?? "") : "";
    }
    stepsEl.querySelectorAll("button[data-step]").forEach((b) => { b.disabled = busy; });
  }

  refreshLib();
  paint();

  return {
    show() { refreshLib(); if (tab === "steps") refreshFlowInfo(); },
    /** 别处（作业台）要打开某份脚本 */
    open,
    update(store) {
      lastStore = store;
      if (store.scriptRev !== scriptRev) {
        const first = scriptRev === -1;
        scriptRev = store.scriptRev;
        if (!first) {
          refreshLib().then(() => {
            const remote = lib.find((s) => s.name === doc?.name);
            if (doc && !doc.isNew && remote && remote.rev !== doc.rev) {
              if (!doc.dirty) open(doc.name);
              else showConflict({ name: doc.name, rev: remote.rev });
            }
          });
        }
      }
      const wasRunning = running;
      running = store.status.flow?.running ? store.status.flow.script ?? running : null;
      if (wasRunning !== running) paintLib();
      if (tab === "tree" && running && doc?.name === running) {
        // 只重画树（编辑区不动，免得正在填的表单被冲掉）
        const html = renderTree(doc.script, { catalog: catalog(), live: store.flow.paths, current: store.flow.current?.path,
          selected, editable: true });
        if ($("sTree").dataset.sig !== html) { $("sTree").dataset.sig = html; $("sTree").innerHTML = html; }
      }
      if (tab === "steps") paintSteps(store);
      paintHead();
    },
  };
}
