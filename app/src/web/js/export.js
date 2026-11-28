// 逐块记录导出：CSV（UTF-8 BOM，Excel 可直接打开）与 Excel（.xls 表格）。
// 纯前端生成，不改服务端、不引入新依赖；导出行数 = 传入记录数。
// 注意契约 §2：板卡时钟比开发机快 8 小时，t 只在同一份日志内相减，
// 所以导出的“时间”列用相对第一条记录的秒数，和页面显示一致，不冒充墙上时间。

import { categoryName, resultName } from "./contract.js";
import { fmtXY } from "./format.js";

const HEADERS = ["序号", "时间(s)", "轮", "会话/run", "类别", "来源 cm", "目标位", "耗时 s", "结果", "原因"];

function recordRows(records) {
  const base = records[0]?.t ?? null;
  return records.map((r) => {
    const t = typeof r.t === "number" && base != null ? (r.t - base).toFixed(1) : (r.t ?? "");
    const run = String(r.run ?? "").split(/[\\/]/).pop();
    return [
      r.seq ?? "",
      t,
      r.round ?? "",
      run,
      categoryName(r.category),
      fmtXY(r.cube),
      r.bin ?? "",
      r.seconds == null ? "" : r.seconds.toFixed(1),
      resultName(r.result),
      r.reason ?? "",
    ];
  });
}

function csvCell(v) {
  const s = String(v ?? "");
  return /[",\r\n]/.test(s) ? `"${s.replace(/"/g, '""')}"` : s;
}

export function exportCsv(records, filename = "records.csv") {
  const rows = [HEADERS, ...recordRows(records)];
  const csv = "\uFEFF" + rows.map((r) => r.map(csvCell).join(",")).join("\r\n");
  download(filename, "text/csv;charset=utf-8", csv);
}

// .xls 用 HTML 表格，Excel 直接打开；无需 xlsx 库，保持前端零依赖。
export function exportXls(records, filename = "records.xls") {
  const esc = (s) => String(s ?? "").replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" })[c]);
  const rows = [HEADERS, ...recordRows(records)];
  const body = rows.map((r) => `<tr>${r.map((c) => `<td>${esc(c)}</td>`).join("")}</tr>`).join("");
  const html = `<html xmlns:o="urn:schemas-microsoft-com:office:office" xmlns:x="urn:schemas-microsoft-com:office:excel">
<head><meta charset="UTF-8">
<!--[if gte mso 9]><xml><x:ExcelWorkbook><x:ExcelWorksheets><x:ExcelWorksheet>
<x:Name>记录</x:Name><x:WorksheetOptions><x:DisplayGridlines/></x:WorksheetOptions>
</x:ExcelWorksheet></x:ExcelWorksheets></x:ExcelWorkbook></xml><![endif]--></head>
<body><table border="1">${body}</table></body></html>`;
  download(filename, "application/vnd.ms-excel;charset=utf-8", html);
}

function download(filename, mime, content) {
  const blob = new Blob([content], { type: mime });
  const url = URL.createObjectURL(blob);
  const a = document.createElement("a");
  a.href = url;
  a.download = filename;
  document.body.appendChild(a);
  a.click();
  a.remove();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
}
