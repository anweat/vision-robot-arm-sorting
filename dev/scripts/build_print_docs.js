#!/usr/bin/env node
// 把 docs/ 里的 Markdown 生成可打印的 Word (.docx) 与 PDF，输出到 docs/defense/requirement-review/print/。
//
// 依赖（不入库）：npm install docx marked@12 image-size@1，然后
//   NODE_PATH=<node_modules 所在目录> node dev/scripts/build_print_docs.js
// PDF 由本机 Google Chrome 无界面打印生成；找不到 Chrome 时只生成 .docx 和 .html。

const fs = require("fs");
const path = require("path");
const { execFileSync } = require("child_process");
const { marked } = require("marked");
const sizeOf = require("image-size");
const D = require("docx");

const ROOT = path.resolve(__dirname, "..", "..");
const OUT = path.join(ROOT, "docs", "defense", "requirement-review", "print");
const SRC = path.join(ROOT, "docs");
const CHROME = "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome";

// name: 输出文件名；src: 源文件；pick: 只取这些二级标题的小节（缺省取全文）；title: 覆盖一级标题
const JOBS = [
  { name: "需求规格", src: "current/10-需求规格.md" },
  { name: "需求审查两页纸", src: "defense/requirement-review/14-需求审查两页纸.md", compact: true },
  { name: "需求审查讲稿", src: "defense/requirement-review/13-需求审查讲稿.md", pick: ["二、5 分钟讲稿", "三、问答预案（3 分钟）", "四、讲前检查"], title: "需求审查讲稿（10.9）" },
  { name: "探索记录", src: "archive/stage1-research/11-探索记录.md" },
];

function select(md, job) {
  if (!job.pick) return md;
  const parts = md.split(/^## /m);
  const keep = parts.slice(1).filter((p) => job.pick.includes(p.split("\n")[0].trim()));
  let body = keep.map((p) => "## " + p).join("\n");
  if (job.pick.length === 1) body = body.replace(/^## .*\n/, ""); // 单节时标题由 title 承担
  return `# ${job.title}\n\n${body}`.replace(/^---\s*$/gm, "");
}

// ---------- Word ----------

const FONT = { ascii: "Arial", hAnsi: "Arial", eastAsia: "PingFang SC", cs: "Arial" };
const MONO = { ascii: "Menlo", hAnsi: "Menlo", eastAsia: "PingFang SC", cs: "Menlo" };
const PAGE_W = 11906, MARGIN = 1134, BODY_W = PAGE_W - 2 * MARGIN; // A4，页边距 2 cm

function runs(tokens, base, dir, st) {
  const out = [];
  for (const t of tokens || []) {
    if (t.type === "strong") out.push(...runs(t.tokens, { ...base, bold: true }, dir, st));
    else if (t.type === "em") out.push(...runs(t.tokens, { ...base, italics: true }, dir, st));
    else if (t.type === "del") out.push(...runs(t.tokens, { ...base, strike: true }, dir, st));
    else if (t.type === "codespan") out.push(new D.TextRun({ ...base, text: unescape(t.text), font: MONO, size: (base.size || st.size) - 1 }));
    else if (t.type === "br") out.push(new D.TextRun({ ...base, break: 1 }));
    else if (t.type === "link") {
      const kids = runs(t.tokens, /^https?:/.test(t.href) ? { ...base, color: "1F5FA8", underline: {} } : base, dir, st);
      out.push(/^https?:/.test(t.href) ? new D.ExternalHyperlink({ link: t.href, children: kids }) : kids);
    } else if (t.type === "image") out.push(image(t, dir, st.rowH));
    else if (t.tokens && t.tokens.length) out.push(...runs(t.tokens, base, dir, st));
    else out.push(new D.TextRun({ ...base, text: unescape(t.text ?? t.raw ?? "") }));
  }
  return out.flat();
}

function unescape(s) {
  return s.replace(/&lt;/g, "<").replace(/&gt;/g, ">").replace(/&amp;/g, "&").replace(/&quot;/g, '"').replace(/&#39;/g, "'");
}

function image(t, dir, rowH) {
  const file = path.resolve(dir, decodeURI(t.href));
  const { width, height } = sizeOf(file);
  const maxW = 640, maxH = 330; // 像素，96 dpi
  const k = rowH ? rowH / height : Math.min(maxW / width, maxH / height, 1);
  return new D.ImageRun({
    type: path.extname(file).slice(1).toLowerCase().replace("jpeg", "jpg"),
    data: fs.readFileSync(file),
    transformation: { width: Math.round(width * k), height: Math.round(height * k) },
  });
}

function visualLen(s) {
  let n = 0;
  for (const ch of s) n += ch.charCodeAt(0) > 0x2e80 ? 2 : 1;
  return n;
}

function table(t, dir, st) {
  const cols = t.header.length;
  const weight = Array.from({ length: cols }, (_, c) => {
    const lens = [t.header[c], ...t.rows.map((r) => r[c])].map((cell) => visualLen(cell.text));
    return Math.min(Math.max(Math.max(...lens), 9), 46);
  });
  const sum = weight.reduce((a, b) => a + b, 0);
  const widths = weight.map((w) => Math.floor((w / sum) * BODY_W));
  widths[cols - 1] += BODY_W - widths.reduce((a, b) => a + b, 0);
  const border = { style: D.BorderStyle.SINGLE, size: 4, color: "B5B5B5" };
  const cell = (c, i, head) =>
    new D.TableCell({
      width: { size: widths[i], type: D.WidthType.DXA },
      margins: { top: 50, bottom: 50, left: 90, right: 90 },
      shading: head ? { type: D.ShadingType.CLEAR, fill: "EDEFF2", color: "auto" } : undefined,
      borders: { top: border, bottom: border, left: border, right: border },
      children: [new D.Paragraph({ spacing: { after: 0, line: 264 }, children: runs(c.tokens, { size: st.table, bold: head || undefined }, dir, st) })],
    });
  return new D.Table({
    width: { size: BODY_W, type: D.WidthType.DXA },
    columnWidths: widths,
    rows: [
      new D.TableRow({ tableHeader: true, cantSplit: true, children: t.header.map((c, i) => cell(c, i, true)) }),
      ...t.rows.map((r) => new D.TableRow({ cantSplit: true, children: r.map((c, i) => cell(c, i, false)) })),
    ],
  });
}

function blocks(tokens, dir, st, ctx = { level: 0 }) {
  const out = [];
  for (const t of tokens) {
    if (t.type === "heading") {
      const lv = [D.HeadingLevel.TITLE, D.HeadingLevel.HEADING_1, D.HeadingLevel.HEADING_2, D.HeadingLevel.HEADING_3][Math.min(t.depth, 4) - 1];
      out.push(new D.Paragraph({ heading: lv, keepNext: true, children: runs(t.tokens, {}, dir, st) }));
    } else if (t.type === "paragraph") {
      const imgs = t.tokens.filter((x) => x.type === "image");
      const onlyImage = imgs.length > 0 && t.tokens.every((x) => x.type === "image" || !x.raw.trim());
      if (imgs.length > 1) {
        // 一行多图：高度统一，总宽不超过版心
        const ratio = imgs.reduce((a, x) => { const d = sizeOf(path.resolve(dir, decodeURI(x.href))); return a + d.width / d.height; }, 0);
        st = { ...st, rowH: Math.floor((640 - 8 * imgs.length) / ratio) };
      }
      const caption = t.tokens.length === 1 && t.tokens[0].type === "em";
      out.push(new D.Paragraph({
        alignment: onlyImage || caption ? D.AlignmentType.CENTER : undefined,
        keepNext: onlyImage || undefined,
        spacing: { after: st.after, line: st.line },
        children: runs(onlyImage ? imgs.flatMap((x, i) => (i ? [{ type: "text", text: " " }, x] : [x])) : t.tokens, caption ? { size: st.size - 2, color: "555555" } : {}, dir, st),
      }));
    } else if (t.type === "table") {
      out.push(table(t, dir, st), new D.Paragraph({ spacing: { after: st.after / 2 }, children: [] }));
    } else if (t.type === "list") {
      const ref = t.ordered ? `ol${ctx.next()}` : "ul";
      t.items.forEach((item) => {
        const first = item.tokens.find((x) => x.type === "text" || x.type === "paragraph");
        const box = item.task ? [new D.TextRun({ text: item.checked ? "☑ " : "☐ " })] : [];
        out.push(new D.Paragraph({
          numbering: item.task ? undefined : { reference: ref, level: ctx.level },
          indent: item.task ? { left: 360, hanging: 360 } : undefined,
          spacing: { after: 60, line: st.line },
          children: [...box, ...runs(first ? first.tokens : [], {}, dir, st)],
        }));
        const rest = item.tokens.filter((x) => x !== first);
        if (rest.length) out.push(...blocks(rest, dir, st, { ...ctx, level: ctx.level + 1 }));
      });
      out.push(new D.Paragraph({ spacing: { after: st.after / 2 }, children: [] }));
    } else if (t.type === "code") {
      t.text.split("\n").forEach((line) =>
        out.push(new D.Paragraph({
          shading: { type: D.ShadingType.CLEAR, fill: "F4F5F7", color: "auto" },
          spacing: { after: 0, line: 252 },
          children: [new D.TextRun({ text: line || " ", font: MONO, size: 15 })],
        })));
      out.push(new D.Paragraph({ spacing: { after: st.after / 2 }, children: [] }));
    } else if (t.type === "blockquote") {
      for (const p of t.tokens.filter((x) => x.type === "paragraph"))
        out.push(new D.Paragraph({
          indent: { left: 300 },
          border: { left: { style: D.BorderStyle.SINGLE, size: 12, color: "9AA3AE", space: 8 } },
          spacing: { after: st.after, line: st.line },
          children: runs(p.tokens, { color: "444444" }, dir, st),
        }));
    }
  }
  return out;
}

async function docx(md, job, dir) {
  const st = job.compact ? { size: 19, table: 17, after: 70, line: 264 } : { size: 21, table: 18, after: 140, line: 320 };
  let olCount = 0;
  const ctx = { level: 0, next: () => olCount++ };
  // 每个有序列表一个独立编号实例，避免跨列表续号
  const tokens = marked.lexer(md);
  const orderedLists = JSON.stringify(tokens).split('"ordered":true').length;
  const body = blocks(tokens, dir, st, ctx);
  const level = (fmt, text) => [0, 1, 2].map((l) => ({
    level: l, format: fmt, text: fmt === D.LevelFormat.BULLET ? ["•", "–", "·"][l] : `%${l + 1}.`,
    alignment: D.AlignmentType.LEFT, style: { paragraph: { indent: { left: 420 * (l + 1), hanging: 320 } } },
  }));
  const head = (size, before, after, color) => ({ run: { size, bold: true, font: FONT, color }, paragraph: { spacing: { before, after }, keepNext: true } });
  const doc = new D.Document({
    creator: "Softwrae_mechanical_arm",
    title: job.title || job.name,
    styles: {
      default: {
        document: { run: { font: FONT, size: st.size } },
        title: head(job.compact ? 30 : 36, 0, 160, "111111"),
        heading1: head(job.compact ? 22 : 28, job.compact ? 140 : 320, 110, "1B3A5C"),
        heading2: head(job.compact ? 20 : 24, 220, 90, "1B3A5C"),
        heading3: head(st.size, 160, 70, "333333"),
      },
    },
    numbering: {
      config: [
        { reference: "ul", levels: level(D.LevelFormat.BULLET) },
        ...Array.from({ length: orderedLists + 1 }, (_, i) => ({ reference: `ol${i}`, levels: level(D.LevelFormat.DECIMAL) })),
      ],
    },
    sections: [{
      properties: { page: { size: { width: PAGE_W, height: 16838 }, margin: { top: job.compact ? 850 : MARGIN, bottom: job.compact ? 850 : MARGIN, left: MARGIN, right: MARGIN } } },
      footers: job.compact ? undefined : {
        default: new D.Footer({ children: [new D.Paragraph({ alignment: D.AlignmentType.CENTER, children: [new D.TextRun({ children: [D.PageNumber.CURRENT, " / ", D.PageNumber.TOTAL_PAGES], size: 16, color: "777777" })] })] }),
      },
      children: body,
    }],
  });
  fs.writeFileSync(path.join(OUT, job.name + ".docx"), await D.Packer.toBuffer(doc));
}

// ---------- HTML / PDF ----------

function html(md, job, dir) {
  const body = marked.parse(md)
    .replace(/src="([^"]+)"/g, (_, src) => `src="file://${encodeURI(path.resolve(dir, decodeURI(src)))}"`)
    // 短单元格不换行，避免“排/期”这样的断行把行高撑成两行
    .replace(/<(td|th)>((?:<strong>)?[^<]{1,12}(?:<\/strong>)?)<\/\1>/g, (m, tag, text) => (visualLen(text.replace(/<[^>]+>/g, "")) <= 22 ? `<${tag} style="white-space:nowrap">${text}</${tag}>` : m));
  const c = job.compact;
  return `<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>${job.title || job.name}</title><style>
@page { size: A4; margin: ${c ? "9mm 12mm" : "18mm 18mm 20mm"}; }
body { font: ${c ? "9pt/1.38" : "10.5pt/1.62"} "PingFang SC", "Microsoft YaHei", Arial, sans-serif; color: #1a1a1a; margin: 0; }
h1 { font-size: ${c ? "16pt" : "18pt"}; margin: 0 0 ${c ? "5pt" : "10pt"}; }
h2 { font-size: ${c ? "11pt" : "14pt"}; color: #1b3a5c; margin: ${c ? "6pt 0 3pt" : "20pt 0 8pt"}; border-bottom: 0.6pt solid #c9d1da; padding-bottom: 2pt; break-after: avoid; }
h3 { font-size: ${c ? "9.5pt" : "12pt"}; color: #1b3a5c; margin: 14pt 0 6pt; break-after: avoid; }
h4 { font-size: 10.5pt; margin: 10pt 0 4pt; break-after: avoid; }
p { margin: 0 0 ${c ? "3.5pt" : "7pt"}; }
table { border-collapse: collapse; width: 100%; margin: ${c ? "3pt 0 4pt" : "6pt 0 10pt"}; font-size: ${c ? "8.4pt" : "9pt"}; line-height: ${c ? 1.3 : 1.4}; }
th, td { border: 0.5pt solid #b5b5b5; padding: ${c ? "1.7pt 4pt" : "3.5pt 5pt"}; vertical-align: top; text-align: left; }
th { background: #edeff2; } tr { break-inside: avoid; } thead { display: table-header-group; }
code { font: 0.88em Menlo, monospace; background: #f1f2f4; padding: 0 2pt; border-radius: 2pt; }
pre { background: #f4f5f7; padding: 6pt 8pt; font-size: 8.5pt; line-height: 1.45; white-space: pre-wrap; break-inside: avoid; }
pre code { background: none; padding: 0; }
img { max-width: ${c ? "100%" : "78%"}; max-height: 82mm; display: block; margin: ${c ? "3pt auto 2pt" : "6pt auto 3pt"}; break-inside: avoid; }
p:has(> img + img) { display: flex; justify-content: space-between; align-items: stretch; gap: 5pt; height: 24mm; }
p:has(> img + img) img { height: 100%; width: auto; max-width: none; margin: 0; border: 0.5pt solid #c9d1da; }
p:has(> img:only-child) { break-after: avoid; } p > em:only-child { display: block; text-align: center; font-size: ${c ? "8pt" : "9pt"}; color: #555; font-style: normal; }
blockquote { margin: 6pt 0; padding-left: 8pt; border-left: 2pt solid #9aa3ae; color: #444; }
ul, ol { margin: 0 0 7pt; padding-left: 18pt; } li { margin-bottom: 2pt; } li:has(> input) { list-style: none; margin-left: -14pt; }
a { color: inherit; text-decoration: none; } a[href^="http"] { color: #1f5fa8; } hr { display: none; }
</style><body>${body}</body></html>`;
}

(async () => {
  fs.mkdirSync(OUT, { recursive: true });
  for (const job of JOBS) {
    const md = select(fs.readFileSync(path.join(SRC, job.src), "utf8"), job);
    await docx(md, job, SRC);
    const page = path.join(OUT, job.name + ".html");
    fs.writeFileSync(page, html(md, job, SRC));
    if (fs.existsSync(CHROME)) {
      execFileSync(CHROME, ["--headless", "--disable-gpu", "--no-pdf-header-footer", `--print-to-pdf=${path.join(OUT, job.name + ".pdf")}`, "file://" + encodeURI(page)], { stdio: "ignore" });
      fs.unlinkSync(page);
    }
    console.log("ok", job.name);
  }
})();
