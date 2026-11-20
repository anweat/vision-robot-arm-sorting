# 项目工作约定

- 开发目标与资料状态从 [工作区总览](docs/atlas-200i-dk-a2/README.md) 开始读取；`docs/archive/early/`（原 `docs/01`–`04`）是先前本地课程包盘点，`docs/05` 是板卡实测记录，都不覆盖官方仓库后续发现。
- `资料/` 是原始和第三方参考资料。不要直接修改官方/课程源文件，也不要整本执行含顶层机械臂运动的 Notebook。板上运行的应用代码、网页前端、手机 App、上板脚本和设备配置进入 `app/`（布局与板上 `/root/arm-app` 相同，`app/` 内不引用仓库其他目录）；仿真、离线工具和实验进入 `dev/`；被替代的实现移到 `archive/`，不再维护（[34](docs/atlas-200i-dk-a2/34-仓库结构与应用版本.md)）。
- 区分“文件存在”“官方示例描述可运行”“本机实测通过”。新增性能与硬件能力结论须附日期、设备、版本、条件和结果证据。
- 四贴纸垃圾图案是当前任务；旧四色模型的 Blue/Green/Red/Yellow 标签不可直接当作垃圾类别。
- 当前实测 1–5 号单舵机写入不生效，六舵机批量小步动作和回读可用。任何动作先查 [硬件安全边界](docs/05-板卡连接与硬件实测.md)，执行前人工确认设备周围安全。不要把 `Arm_Action_Mode(0)` 未经验证地命名为急停。
- 优先用项目 CodeGraph 查符号、调用关系和影响；纯文字、课程内容和非代码资料用文本搜索。若索引不可用，按工具提示处理，不自动重建。
- `dev/sim/` 是独立的仿真后端（[说明](docs/atlas-200i-dk-a2/16-仿真模块.md)）：不导入 `Arm_Lib`，复用 `app/src/teleop` 的 `Controller`，夹爪、整臂与底座用规划的同一套模型（`planning.gripper`/`planning.body`/`platform_model.json`）。仿真结果只说明几何与接口自洽，不写成实机结论。
- 前端依赖的日志事件以 [日志契约](docs/atlas-200i-dk-a2/18-日志契约.md)为准，唯一来源是 `app/src/tools/log_contract.py` 的事件表；增改事件或字段先改事件表，再运行 `python3 app/src/tools/log_contract.py schema > app/contracts/log_event.schema.json`。
- `data/` 与 `资料/` 不入库；模型与物块贴面在 Release（`assets-v3`/`assets-v4`），换模型发新的 Release，不覆盖旧包。例外（2026-11-20）：检测模型 v3 的 onnx/om 入库在 `app/assets/models/blocks4/v3/`，物块贴面与仿真网格在 `dev/sim/assets/`，克隆即能跑仿真；代码先读 `data/` 再读这些副本，换模型时一并更新。
- 官方 `ros2_robot_arm` 固定源码已归档；[导入审计](docs/atlas-200i-dk-a2/06-官方源码导入审计.md)记录提交 SHA、NOTICE、逐文件校验、模型缺项和关键接口。上游快照保持原样，不把其配置改写成已在当前板验证的应用。
- 桌面只有 A4 长边宽，实体桶先做厨余+有害两类（`app/config/bins_acrylic_v2_pair.json`，地图 `map_a4_v4`，用 `ARM_MAP=map_a4_v4` 读）；方案、仿真结果与实机联调清单见 [24](docs/atlas-200i-dk-a2/24-垃圾桶建模与仿真联调.md)，其中的数字都是仿真，桶位、槽位、投放高度未经实机验证；`run.py --bins` 带着空夹爪先用 `app/src/tasks/bin_checkout.py` 核对再抓块。
- 流程脚本能做什么以 `app/src/flow/capabilities.py` 为唯一来源（[35](docs/atlas-200i-dk-a2/35-流程脚本与前端整理.md)）：加能力先改能力表，再在 `flow/steps.py` 或 `flow/actions.py` 实现同名步骤；板卡端还没有接口的能力标 `unavailable` 并写 `needs`，不假装可用。
