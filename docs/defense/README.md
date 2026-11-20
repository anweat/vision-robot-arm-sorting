# 答辩与汇报材料

每次答辩一个子目录，交完就不再改（后续更正写进 `docs/current/`）。

| 场次 | 日期 | 目录 | 内容 |
|---|---|---|---|
| 需求审查 | 2026-10-09（第 5 周） | `requirement-review/` | [13 讲稿与问答预案](requirement-review/13-需求审查讲稿.md)、[14 两页纸说明](requirement-review/14-需求审查两页纸.md)；`print/` 是打印件（docx/pdf，由 `dev/scripts/build_print_docs.js` 生成，需求规格正文在 [current/10](../current/10-需求规格.md)） |
| 中期汇报 | 2026-11-20 | `midterm/` | [中期汇报（设计详述）](midterm/中期汇报.md)、`build_ppt.js`（生成 `中期汇报.pptx`，pptx 按 `.gitignore` 不入库）、`figures/` 配图 |

重新生成：

```bash
node docs/defense/midterm/build_ppt.js
```

```bash
node dev/scripts/build_print_docs.js
```

`build_ppt.js` 需要 npm 包 `pptxgenjs`；`build_print_docs.js` 用本机 Chrome 无头打印 PDF（见脚本开头）。
