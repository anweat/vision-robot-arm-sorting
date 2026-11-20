// 流程脚本的结构视图：把解析后的脚本（服务端 /api/scripts/check 给的 script）画成一棵卡片树。
// 每张卡片的 data-path 与编排器 step 事件的 path、校验问题的 path 是同一种写法
// （steps[1].do[2].on_result.empty[0]），所以"正在跑哪一行""哪一行有错"都能直接对上。
// 作业台（只读、紧凑）和流程脚本页（可编辑）共用这一份。

const esc = (s) => String(s ?? "").replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" })[c]);

export const KIND_OF = (a) => ["step", "for_each", "retry", "repeat", "alarm", "record", "next_block", "stop"].find((k) => a && k in a) ?? "?";

export const STATUS_TEXT = { ready: "可用", partial: "待接入", unavailable: "不可用" };
const RECORD_TEXT = { sorted: "已分拣", missed: "夹空", skipped: "跳过" };
const THEN_TEXT = { skip_block: "跳过这一块", stop: "整轮停", continue: "继续" };

function paramText(cap, params) {
  const p = params ?? {};
  const keys = Object.keys(p);
  if (!keys.length) return "";
  return keys.map((k) => {
    const spec = cap?.params?.[k];
    const v = p[k];
    const shown = typeof v === "boolean" ? (v ? "是" : "否") : v === null ? "—" : String(v);
    return `${esc(spec?.label ?? k)} ${esc(shown)}${esc(spec?.unit ?? "")}`;
  }).join(" · ");
}

/** 一行动作的标题与副标题 */
export function describe(a, catalog) {
  const kind = KIND_OF(a);
  const caps = catalog?.capabilities ?? {};
  if (kind === "step") {
    const cap = caps[a.step];
    return { kind, icon: cap?.moves ? "●" : "○", title: cap?.label ?? a.step, code: a.step, sub: paramText(cap, a.with),
             status: cap?.status ?? "unknown", cap };
  }
  if (kind === "for_each") return { kind, icon: "⟳", title: "对每一块", code: "for_each", sub: a.order === "scene" ? "按扫描顺序" : "由近及远" };
  if (kind === "retry") return { kind, icon: "↻", title: "重试", code: "retry", sub: `最多 ${a.retry?.times ?? "?"} 次${a.retry?.until ? `，直到 ${esc(a.retry.until)}` : ""}` };
  if (kind === "repeat") return { kind, icon: "⟲", title: "重复", code: "repeat", sub: `${a.repeat?.times ?? "?"} 遍` };
  if (kind === "alarm") return { kind, icon: "⚠", title: "报警", code: "alarm", sub: `${esc(a.alarm)} → ${THEN_TEXT[a.then ?? "continue"] ?? esc(a.then)}${a.summary ? ` · ${esc(a.summary)}` : ""}` };
  if (kind === "record") return { kind, icon: "✎", title: `记为${RECORD_TEXT[a.record] ?? a.record}`, code: "record", sub: esc(a.reason ?? "") };
  if (kind === "next_block") return { kind, icon: "⇥", title: "下一块", code: "next_block", sub: "" };
  if (kind === "stop") return { kind, icon: "■", title: "停止流程", code: "stop", sub: esc(a.stop) };
  return { kind, icon: "?", title: "不认识的动作", code: "?", sub: esc(JSON.stringify(a)) };
}

/** 子列表：[(标签, 路径, 动作数组, 删除这个分支的键)] */
export function children(a, path) {
  const kind = KIND_OF(a);
  if (kind === "for_each") return [["每一块", `${path}.do`, a.do ?? [], null]];
  if (kind === "retry") return [["每次尝试", `${path}.retry.do`, a.retry?.do ?? [], null],
                               ["仍不行", `${path}.retry.else`, a.retry?.else ?? [], null]];
  if (kind === "repeat") return [["每一遍", `${path}.repeat.do`, a.repeat?.do ?? [], null]];
  if (kind === "step" && a.on_result) {
    return Object.entries(a.on_result).map(([r, list]) => [r === "default" ? "其他结果" : `结果 ${r}`,
      `${path}.on_result.${r}`, list ?? [], r]);
  }
  return [];
}

/**
 * 画树。opts:
 *   catalog      能力表（/api/scripts/capabilities）
 *   live         {path: step 事件}：最近状态（started / done / failed）
 *   current      正在跑的那一行的 path
 *   issues       {path: "error" | "warn"}
 *   selected     选中的 path
 *   editable     画增删移按钮
 *   compact      作业台用：不画按钮、分支默认收起只留标题
 */
export function renderTree(script, opts = {}) {
  const steps = script?.steps;
  if (!Array.isArray(steps)) return `<p class="dim">没有可显示的步骤</p>`;
  return list(steps, "steps", opts, 0);
}

function list(actions, path, opts, depth) {
  const items = actions.map((a, i) => node(a, `${path}[${i}]`, opts, depth, i, actions.length)).join("");
  const add = opts.editable ? `<button class="tree-add" data-act="add" data-path="${esc(path)}">＋ 添加</button>` : "";
  if (!actions.length && !opts.editable) return `<div class="tree-list empty dim">（空：什么也不做）</div>`;
  return `<div class="tree-list" data-list="${esc(path)}">${items}${add}</div>`;
}

function node(a, path, opts, depth, index, count) {
  const d = describe(a, opts.catalog);
  const ev = opts.live?.[path];
  const state = opts.current === path ? "run" : ev ? (ev.state === "started" ? "run" : ev.state === "done" ? "ok" : "bad") : "";
  const issue = opts.issues?.[path];
  const cls = ["tree-node", `k-${d.kind}`, state && `s-${state}`, issue && `i-${issue}`, opts.selected === path && "sel",
    d.status && d.status !== "ready" && d.status !== "unknown" && `cap-${d.status}`].filter(Boolean).join(" ");
  const badge = d.status === "unavailable" ? `<span class="badge bad" title="${esc(d.cap?.needs)}">不可用</span>`
    : d.status === "partial" ? `<span class="badge warn" title="${esc(d.cap?.needs)}">待接入</span>`
    : d.status === "unknown" ? `<span class="badge bad">没有这项能力</span>` : "";
  const result = ev && ev.state !== "started" && ev.result ? `<span class="tree-result">${esc(ev.result)}${ev.summary ? ` · ${esc(ev.summary)}` : ""}</span>` : "";
  const tools = opts.editable ? `<span class="tree-tools">
      <button data-act="up" data-path="${esc(path)}" ${index === 0 ? "disabled" : ""} title="上移">↑</button>
      <button data-act="down" data-path="${esc(path)}" ${index === count - 1 ? "disabled" : ""} title="下移">↓</button>
      <button data-act="del" data-path="${esc(path)}" title="删除">✕</button></span>` : "";
  const kids = children(a, path);
  const kidsHtml = kids.length && !(opts.compact && depth >= 1) ? `<div class="tree-kids">${kids.map(([label, p, acts, branch]) => `
      <div class="tree-branch">
        <div class="tree-branch-head">${esc(label)}${opts.editable && branch ? ` <button data-act="del-branch" data-path="${esc(p)}" title="删除这个分支">✕</button>` : ""}</div>
        ${list(acts, p, opts, depth + 1)}
      </div>`).join("")}</div>` : "";
  return `<div class="${cls}" data-path="${esc(path)}">
      <div class="tree-row" data-act="select" data-path="${esc(path)}">
        <span class="tree-icon">${d.icon}</span>
        <b>${esc(d.title)}</b><code>${esc(d.code === "step" ? a.step : d.code)}</code>${badge}
        <span class="tree-sub">${d.sub}</span>${a.note ? `<span class="tree-note">${esc(a.note)}</span>` : ""}
        ${result}${tools}
      </div>${kidsHtml}</div>`;
}

// ---- 路径工具：编辑时按 path 找到/改动脚本对象 ----------------------------------------------------
const TOKEN = /\.?([a-z_]+)|\[(\d+)\]/g;
export function parsePath(path) {
  const out = [];
  for (const m of String(path).matchAll(TOKEN)) out.push(m[2] !== undefined ? +m[2] : m[1]);
  return out;
}

/** 取到 path 所指的值；中间的 on_result/retry/repeat 键按路径原样走 */
export function getAt(root, path) {
  let cur = root;
  for (const k of parsePath(path)) {
    if (cur == null) return undefined;
    cur = cur[k];
  }
  return cur;
}

/** path 指向一个动作：返回 [所在数组, 下标] */
export function parentOf(root, path) {
  const keys = parsePath(path);
  const idx = keys.pop();
  let cur = root;
  for (const k of keys) cur = cur?.[k];
  return [cur, idx];
}

/** 一个校验问题落在哪个动作上：去掉 .with.x、.on_result.r 这类尾巴，取最近的 [n] */
export function nodePathOf(issuePath) {
  const m = String(issuePath ?? "").match(/^(.*\[\d+\])/);
  return m ? m[1] : "";
}
