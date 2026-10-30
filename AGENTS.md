# 项目工作约定

- 开发目标与资料状态从 [工作区总览](docs/atlas-200i-dk-a2/README.md) 开始读取；旧 `docs/01`–`05` 是先前本地课程包盘点及板卡实测记录，不覆盖官方仓库后续发现。
- `资料/` 是原始和第三方参考资料。不要直接修改官方/课程源文件，也不要整本执行含顶层机械臂运动的 Notebook。新应用代码进入独立 `src/`，设备配置进入 `config/`。
- 区分“文件存在”“官方示例描述可运行”“本机实测通过”。新增性能与硬件能力结论须附日期、设备、版本、条件和结果证据。
- 四贴纸垃圾图案是当前任务；旧四色模型的 Blue/Green/Red/Yellow 标签不可直接当作垃圾类别。
- 当前实测 1–5 号单舵机写入不生效，六舵机批量小步动作和回读可用。任何动作先查 [硬件安全边界](docs/05-板卡连接与硬件实测.md)，执行前人工确认设备周围安全。不要把 `Arm_Action_Mode(0)` 未经验证地命名为急停。
- 优先用项目 CodeGraph 查符号、调用关系和影响；纯文字、课程内容和非代码资料用文本搜索。若索引不可用，按工具提示处理，不自动重建。
- `src/sim/` 是独立的仿真后端（[说明](docs/atlas-200i-dk-a2/16-仿真模块.md)）：不导入 `Arm_Lib`，复用 `src/teleop` 的 `Controller`。仿真结果只说明几何与接口自洽，不写成实机结论。
- 前端依赖的日志事件以 [日志契约](docs/atlas-200i-dk-a2/18-日志契约.md)为准，唯一来源是 `src/tools/log_contract.py` 的事件表；增改事件或字段先改事件表，再运行 `python3 src/tools/log_contract.py schema > docs/contracts/log_event.schema.json`。
- `data/` 与 `资料/` 不入库；模型与物块贴面在 Release `assets-v3`，换模型发新的 Release，不覆盖旧包。
- 官方 `ros2_robot_arm` 固定源码已归档；[导入审计](docs/atlas-200i-dk-a2/06-官方源码导入审计.md)记录提交 SHA、NOTICE、逐文件校验、模型缺项和关键接口。上游快照保持原样，不把其配置改写成已在当前板验证的应用。
- 桌面只有 A4 长边宽，实体桶先做厨余+有害两类（`config/bins_acrylic_v2_pair.json`，地图 `map_a4_v4`，用 `ARM_MAP=map_a4_v4` 读）；方案、仿真结果与实机联调清单见 [24](docs/atlas-200i-dk-a2/24-垃圾桶建模与仿真联调.md)，其中的数字都是仿真，桶位、槽位、投放高度未经实机验证；`run.py --bins` 带着空夹爪先用 `src/tasks/bin_checkout.py` 核对再抓块。
