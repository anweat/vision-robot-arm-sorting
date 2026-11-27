import { categoryName, resultName } from "./contract.js";

const HEADERS = ["序号", "时间", "轮次", "类别", "来源X cm", "来源Y cm", "目标位", "耗时 s", "结果", "原因"];
const safeText = (v) => typeof v === "string" && /^[=+\-@\t\r]/.test(v) ? "'" + v : v;
const values = (r) => [r.seq, new Date(r.t * 1000).toISOString(), r.round ?? "", categoryName(r.category),
  r.cube?.[0] ?? "", r.cube?.[1] ?? "", r.bin ?? "", r.seconds ?? "", resultName(r.result), r.reason ?? ""];
const xml = (v) => String(v ?? "").replace(/[&<>"']/g, (c) => ({"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;","'":"&apos;"})[c]);

export function recordsCsv(records) {
  return "\ufeff" + [HEADERS, ...records.map(values)].map((row) => row.map((v) =>
    '"' + String(safeText(v) ?? "").replace(/"/g, '""') + '"').join(",")).join("\r\n") + "\r\n";
}

export function recordsExcel(records) {
  return '<?xml version="1.0" encoding="UTF-8"?><?mso-application progid="Excel.Sheet"?>' +
    '<Workbook xmlns="urn:schemas-microsoft-com:office:spreadsheet" xmlns:ss="urn:schemas-microsoft-com:office:spreadsheet">' +
    '<Worksheet ss:Name="分拣记录"><Table>' + [HEADERS, ...records.map(values)].map((row) => '<Row>' + row.map((v) =>
      `<Cell><Data ss:Type="${typeof v === "number" ? "Number" : "String"}">${xml(v)}</Data></Cell>`).join("") + '</Row>').join("") +
    '</Table></Worksheet></Workbook>';
}

export function downloadRecords(records, excel = false) {
  const blob = new Blob([excel ? recordsExcel(records) : recordsCsv(records)],
    {type: excel ? "application/vnd.ms-excel;charset=utf-8" : "text/csv;charset=utf-8"});
  const url = URL.createObjectURL(blob), a = document.createElement("a");
  a.href = url; a.download = `分拣记录-${records.length}.${excel ? "xml" : "csv"}`;
  a.click(); setTimeout(() => URL.revokeObjectURL(url), 1000);
}
