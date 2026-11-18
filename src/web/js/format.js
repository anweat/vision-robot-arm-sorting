// 显示格式。注意契约 §2：板卡时钟比开发机快 8 小时，t 只能和同一份日志里的其他 t 相减，
// 所以这里不把 t 转成墙上时间，只显示相对起点或相对上一行的秒数。

export const fmtXY = (xy) =>
  Array.isArray(xy) ? `${xy[0]?.toFixed(1)}, ${xy[1]?.toFixed(1)}` : "—";

export const cls = (result) =>
  ({ sorted: "good", missed: "bad", failed: "bad", skipped: "mid", cleared: "mid" }[result] ?? "");

export const fmtRel = (t, base) =>
  typeof t !== "number" || base == null ? "—" : `${t - base >= 0 ? "+" : ""}${(t - base).toFixed(1)} s`;

export const fmtSeconds = (s) => (s == null ? "—" : `${s.toFixed(1)} s`);

/** 轻提示，用于指令回执与错误 */
let host = null;
export function toast(message, kind = "info") {
  if (!host) {
    host = document.createElement("div");
    host.className = "toasts";
    document.body.appendChild(host);
  }
  const el = document.createElement("div");
  el.className = `toast ${kind}`;
  el.textContent = message;
  host.appendChild(el);
  setTimeout(() => el.classList.add("out"), 3200);
  setTimeout(() => el.remove(), 3800);
}
