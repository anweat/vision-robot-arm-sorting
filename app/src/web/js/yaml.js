// 流程脚本 → YAML 文本（结构编辑后生成）。解析只在服务端做（src/flow/script.py + miniyaml），
// 这里只负责写：输出必须落在 miniyaml 认得的子集里（块映射/块序列、一行内的 [..] {..}、
// 单引号或双引号字符串，不用转义），这样板上没有 PyYAML 也读得回来。注释不会保留。

const TOP_ORDER = ["name", "title", "description", "version", "speed", "stop_after_alarms", "steps"];
const KIND_KEYS = ["step", "for_each", "retry", "repeat", "alarm", "record", "next_block", "stop"];
const AFTER_KIND = ["with", "order", "then", "summary", "reason", "until", "times", "on_result", "do", "else", "note"];

const isScalar = (v) => v === null || v === undefined || ["string", "number", "boolean"].includes(typeof v);
const isPlainObj = (v) => v && typeof v === "object" && !Array.isArray(v);

export function scalar(v) {
  if (v === null || v === undefined) return "null";
  if (typeof v === "boolean") return v ? "true" : "false";
  if (typeof v === "number") return Number.isFinite(v) ? String(v) : "null";
  const s = String(v);
  const risky = s === "" || /^\s|\s$/.test(s) || /[,\[\]{}#:'"&*!|>%@`]/.test(s) || /^-(\s|$)/.test(s)
    || /^(true|false|null|~|yes|no|on|off)$/i.test(s) || /^[-+]?(\d+\.?\d*|\.\d+)([eE][-+]?\d+)?$/.test(s);
  if (!risky) return s;
  if (!s.includes("'")) return `'${s}'`;
  if (!s.includes('"') && !s.includes("\\")) return `"${s}"`;
  return `'${s.replace(/'/g, "’")}'`;                // miniyaml 不认转义：换成全角撇号，不丢字
}

function ordered(obj) {
  const keys = Object.keys(obj);
  const kind = KIND_KEYS.find((k) => k in obj);
  const head = kind ? [kind, ...AFTER_KIND] : TOP_ORDER.some((k) => k in obj) ? TOP_ORDER : AFTER_KIND;
  return [...head.filter((k) => keys.includes(k)), ...keys.filter((k) => !head.includes(k))];
}

// 一行写得下的：标量组成的小映射 / 小列表，或"每项都是这种小映射"的短列表
function flow(v, depth = 0) {
  if (isScalar(v)) return scalar(v);
  if (depth > 2) return null;
  if (Array.isArray(v)) {
    if (!v.length) return "[]";
    const parts = v.map((x) => flow(x, depth + 1));
    if (parts.some((p) => p === null)) return null;
    return `[${parts.join(", ")}]`;
  }
  const keys = ordered(v);
  if (!keys.length) return "{}";
  const parts = keys.map((k) => {
    const inner = isScalar(v[k]) ? scalar(v[k]) : depth < 1 ? flow(v[k], depth + 1) : null;
    return inner === null ? null : `${k}: ${inner}`;
  });
  if (parts.some((p) => p === null)) return null;
  return `{${parts.join(", ")}}`;
}

const pad = (n) => " ".repeat(n);

function inline(key, v) {
  // with: 这种参数表、on_result 里只有一两个简单动作的分支，写成一行更好读
  if (isScalar(v)) return scalar(v);
  const f = flow(v);
  if (f === null || f.length > 72) return null;
  if (Array.isArray(v) && v.some((x) => isPlainObj(x) && Object.values(x).some((y) => !isScalar(y)))) return null;
  if (key === "steps" || key === "do" || key === "else") return Array.isArray(v) && !v.length ? "[]" : null;
  return f;
}

function mapLines(obj, indent, out) {
  for (const k of ordered(obj)) {
    const v = obj[k];
    if (v === undefined) continue;
    const one = inline(k, v);
    if (one !== null) { out.push(`${pad(indent)}${k}: ${one}`); continue; }
    out.push(`${pad(indent)}${k}:`);
    if (Array.isArray(v)) seqLines(v, indent + 2, out);
    else mapLines(v, indent + 2, out);
  }
}

function seqLines(arr, indent, out) {
  for (const item of arr) {
    if (isScalar(item)) { out.push(`${pad(indent)}- ${scalar(item)}`); continue; }
    if (Array.isArray(item)) {
      const f = flow(item);
      if (f !== null) { out.push(`${pad(indent)}- ${f}`); continue; }
      out.push(`${pad(indent)}-`);
      seqLines(item, indent + 2, out);
      continue;
    }
    const sub = [];
    mapLines(item, indent + 2, sub);
    if (!sub.length) { out.push(`${pad(indent)}- {}`); continue; }
    out.push(`${pad(indent)}- ${sub[0].slice(indent + 2)}`);
    out.push(...sub.slice(1));
  }
}

export function dump(script) {
  const out = [];
  mapLines(script ?? {}, 0, out);
  return out.join("\n") + "\n";
}
