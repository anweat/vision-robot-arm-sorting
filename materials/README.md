# 开发过程资料（打印、切割、照片）

更新：2026-11-20。这里放做实物要用的文件和实物照片，不是代码要读的配置：代码读的地图码布局、桶尺寸在 `app/config/`（`map_a4_v*.json`、`bins_acrylic_*.json`），和这里的 PDF/DXF 由同一个脚本一起生成。

| 目录 | 内容 | 生成脚本 | 状态 |
|---|---|---|---|
| `maps/` | 桌面地图 A4：v2（实机在用）、v3、v4（窄桌，`ARM_MAP=map_a4_v4`） | `dev/tools/make_map.py`（同时写 `app/config/map_a4_<v>.json`） | 必须 100% 比例打印；v3、v4 为设计值 |
| `calibration/` | ChArUco 15 mm 标定板、12 mm 棋盘格 | — | 相机内参与手眼标定用 |
| `bins/` | 亚克力垃圾桶打版包：DXF 切割图、零件图、图纸 PDF、桶贴纸（[说明](bins/README.md)） | `dev/tools/make_bins.py`、`bins_fab.py` | 设计值；桶位、槽位、投放高度未经实机验证（[24](../docs/current/24-垃圾桶建模与仿真联调.md)） |
| `photos/blocks/` | 四块物块各面贴纸的实物照片（2026-09-24） | — | 核对类别（[07](../docs/current/07-物块与贴纸类别.md)）用；仿真贴面在 `dev/sim/assets/block_faces/` |

换新一版地图或桶：生成新版本号的文件，旧版留着，不覆盖（已有实机记录引用旧版）。
