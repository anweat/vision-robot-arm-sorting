# 文档索引

更新：2026-11-19。

| 想看 | 去哪 |
|---|---|
| 项目入口：目标、设备、全部专题文档 | [Atlas 200I DK A2 工作区总览](atlas-200i-dk-a2/README.md) |
| 板卡应用怎么部署、启动、测试，各部分实机/仿真状态 | [app/README](../app/README.md) |
| 仓库为什么分成 `app/`、`dev/`、`archive/`，什么搬到了哪 | [34 仓库结构与应用版本](atlas-200i-dk-a2/34-仓库结构与应用版本.md) |
| 硬件安全边界与全部实机实测（按日期倒序） | [05 板卡连接与硬件实测](05-板卡连接与硬件实测.md) |
| 接口：控制台、网关、日志契约 | [15 接口文档](atlas-200i-dk-a2/15-接口文档.md)、[18 日志契约](atlas-200i-dk-a2/18-日志契约.md)、[31 板载端分离](atlas-200i-dk-a2/31-板载端分离与中继通信.md)、[32 操作逻辑简化](atlas-200i-dk-a2/32-操作逻辑简化与分步任务.md) |
| 仿真后端 | [16 仿真模块](atlas-200i-dk-a2/16-仿真模块.md) |
| 设计决定 | [decisions/](decisions/0001-视觉空间与执行分层.md) |
| 离线实验 | [experiments/](experiments/多视角方块定位离线实验.md) |

## 目录

| 目录 | 内容 |
|---|---|
| `atlas-200i-dk-a2/` | 专题文档（01–34），新结论写在这里 |
| `05-板卡连接与硬件实测.md` | 硬件实测总记录，也是 `AGENTS.md` 引用的安全边界 |
| `bins/`、`calibration/` | 垃圾桶打版包（DXF/PDF）、可打印的地图与标定板 |
| `images/` | 文档配图；`images/blocks/` 是物块贴面照片，`images/sim/` 是 Blender 渲染 |
| `hardware-test-results/` | 板上冒烟与模型测试的原始 JSON |
| `inventory/` | `scripts/inventory_resources.py` 生成的资料盘点（2026-09 快照） |
| `sim/` | Blender 回放视频 |
| `archive/early/` | 2026-09-14 的课程包盘点与早期方案（原 `docs/01`–`04` 和旧总览），不再更新，结论以专题文档为准 |
| `archive/print/` | 需求审查的打印件（docx/pdf） |

日志契约的 JSON Schema 和回放样例属于应用，在 `app/contracts/`、`app/samples/`。
