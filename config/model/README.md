# 从 PR #5 引入的建模/仿真数据

来源：`anweat/vision-robot-arm-sorting` PR #5（`ops-simplify-steps` → `main`），
提交 `21ce0e6`「仓库整理为 app/ + dev/ + archive/；仿真换用四连杆、整臂网格与底座方台模型」。

PR 把那套东西放在 `app/config/` 下，本仓库没有做目录重组，所以这里保留**原有布局**，
把新增的模型文件集中放在 `config/model/`，避免和运行期配置（`arm_limits.json`、
`map_a4_v4.json`、`bins_*.json`）混在一起。

| 文件 | 内容 | 本仓库是否运行期使用 |
|---|---|---|
| `gripper_geometry.json` | 夹爪四连杆的网格/体素点集（`points_m`、`sides`、`static_shells`） | 否，仅供建模/仿真与夹爪解算参考 |
| `arm_links_geometry.json` | 整臂各连杆的网格点集（碰撞检查用） | 否，运行期碰撞走 `src/spatial/workspace.py` 的几何 |
| `platform_model.json` | 底座方台：前 8 / 半宽 11 / 后 −22 cm，圆角 6 cm，顶面 z 7.6 cm | 参考；数值与 `arm_limits.json keep_out` 对齐 |
| `motion_planning.json` | `src/planning` 的规划常数（抓取高度、开口档位、余量） | 否，本仓库未启用 `src/planning` |
| `planning_tuning.json` | 规划的场景误差与实机微调 | 否 |
| `gripper_model.json` | 夹爪几何与开合模型（PR 版本） | 参考；本仓库运行期用 `config/gripper_model.json` |

## 两个需要说明的数

1. **桌面高度**：`motion_planning.json` 的 `table_z_model_m = -0.0087` 是“网格手指看到的桌面”
   （五次指尖触地拟合）；本仓库运行期用 `config/arm_limits.json` 的
   `keep_out.table_surface_z_m = -0.0142`，来自触点标定会话 `20260925_005756_78eb`，并且
   2026-11-19 的实机分拣（多轮、含重试）都在用它。**两者差 5.5 mm，运行期保持 −0.0142**，
   换数需要重做触点标定，不能只改文件。

2. **底座形状**：PR 把平台从 r 8 cm 的圆柱改成圆角方台（前 8 / 半宽 11 / 后 −22 / 圆角 6 cm）。
   本仓库的 `arm_limits.json keep_out` 仍是圆柱口径（`platform_radius_m = 0.08`），因为
   `src/spatial/workspace.py` 这个版本还没有方台分支；方台参数已抄进 `platform_model.json`，
   等合并 `src/planning` 时再一次性切换。

## 复现对比

```python
import json
pm = json.load(open("config/model/platform_model.json"))
al = json.load(open("config/arm_limits.json"))
print(pm["outline"], pm["table_z_m"], al["keep_out"]["table_surface_z_m"])
```
