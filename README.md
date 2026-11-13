# 基于计算机视觉的智能机械臂垃圾分拣与管理系统

只用**腕部单目相机**完成「看 → 算 → 抓 → 记」：扫描桌面、识别四类垃圾、算出位置与朝向、抓取并分类投放，每次动作落盘为事件记录。硬件为 Atlas 200I DK A2（昇腾 310B NPU）+ Dofbot 六舵机机械臂。

北京交通大学软件学院《软件开发与测试综合实训》课程项目，2026 秋季学期。

## 当前状态

| 功能 | 状态 | 实测 |
|---|---|---|
| 视觉识别 | 已跑通，板上 NPU | 自训四类模型 mAP@0.5 **0.952**，**12.9 ms/帧** |
| 任意位置分拣 | 已跑通 | 最好一轮 **4/4**、99 秒，每块中位 **22 秒** |
| 管理看板 | **正式前端尚未开始**；接口文档、仿真后端和壳子页面已就绪 | 仿真后端 2026-10-11 在 Mac 上跑通，未接实机 |
| 兜底机制 | 代码已实现 7 条失败路径 | 待按课程口径量化 |
| 实体容器 | 桶的图纸与地图 v3 已出，**未切割、未到货** | 设计值，未测量 |
| 工程基础 | — | **237 项测试**：开发机全部通过；全新克隆取回资产后 231 通过、6 跳过（2026-10-11）。约 1.2 万行 Python 应用代码 |

离线评分（15 轮手摆 60 块）：**100% 找到、100% 类别正确**，中心误差中位 0.0 cm、p90 1.04 cm。

> 本仓库区分三种状态：**文件存在**、**官方示例描述可运行**、**本机实测通过**。所有性能与硬件结论都附日期、设备、版本、条件和证据。

## 已有工作索引

按“想做什么”找入口。状态一栏区分**实机实测**、**仅离线/仿真验证**和**设计值**。

| 想做什么 | 代码 | 文档 | 状态 |
|---|---|---|---|
| 看懂整个项目做什么、不做什么、怎么验收 | — | [需求规格](docs/atlas-200i-dk-a2/10-需求规格.md) | 2026-10-09 需求审查用 |
| 更精确的物块中心/夹取角（不换模型） | `src/vision/roi_pose.py`（已接入定位）；`src/tasks/grip_solver.py`（离线，未接入） | [ROI 棱边解算与腕部角度审计](docs/atlas-200i-dk-a2/28-ROI棱边解算与腕部角度审计.md) | 合成/离线验证，实机未测 |
| 知道自己该做哪几项、多少工作量 | — | [分工与任务量](docs/atlas-200i-dk-a2/17-分工与任务量.md) | 人日为估算；姓名待填 |
| 开发前端或手机端 | `src/sim/`（仿真后端与壳子页面）；`src/teleop/web/`（旧页面，参考） | [接口文档](docs/atlas-200i-dk-a2/15-接口文档.md)、[仿真模块](docs/atlas-200i-dk-a2/16-仿真模块.md) | 仅仿真验证 |
| 做日志页、统计页、导出 | `src/tools/log_contract.py`；`docs/samples/`（实机与仿真日志样例） | [日志契约](docs/atlas-200i-dk-a2/18-日志契约.md) | 契约 v1；实机样例 0 处不符 |
| 遥控机械臂、采样、标定 | `src/teleop/`、`src/adapters/safe_arm.py` | [遥控与场景标定工作台](docs/atlas-200i-dk-a2/08-遥控与场景标定工作台.md) | 实机实测 |
| 自动分拣一轮 | `src/pipeline/run.py`、`src/pipeline/loop.py`、`src/tasks/` | [探索记录](docs/atlas-200i-dk-a2/11-探索记录.md) | 实机实测，累计 18/22 块 |
| 识别四类物块 | `src/vision/blocks_detector.py`、`train_blocks.py`、`synth_blocks.py` | [物块与贴纸类别](docs/atlas-200i-dk-a2/07-物块与贴纸类别.md)、[资产清单](docs/atlas-200i-dk-a2/09-资产清单.md) | 实机实测，板上 NPU |
| 正逆解、相机位姿、地图定位 | `src/spatial/` | [硬件实测](docs/05-板卡连接与硬件实测.md) 2026-09-25 触点标定与手眼 | 实机实测 |
| 限位、禁入区、夹爪判定 | `config/arm_limits.json`、`src/spatial/workspace.py` | [硬件实测](docs/05-板卡连接与硬件实测.md) | 实机实测 |
| 打印地图、做垃圾桶 | `src/tools/make_map.py`、`make_bins.py`；`docs/calibration/`、`docs/bins/` | [实体容器选型约束](docs/atlas-200i-dk-a2/12-实体容器选型约束.md) | 地图 v2 实机在用；v3、v4（窄桌）与桶为设计值；桶打版包见 `docs/bins/README.md`；窄桌双桶方案与联调清单见 [24](docs/atlas-200i-dk-a2/24-垃圾桶建模与仿真联调.md) |
| 复盘一次运行 | `src/tools/analyze_session.py`、`run_stats.py` | — | 离线 |
| 查官方资料与上游源码 | `资料/`（不入库） | [官方源码导入审计](docs/atlas-200i-dk-a2/06-官方源码导入审计.md)、[官方案例与调研](docs/atlas-200i-dk-a2/03-官方案例与调研.md) | 归档 |

### 不在仓库里的东西

`data/` 和 `资料/` 整个目录不入库（`.gitignore`），仓库不用 LFS。克隆下来的人**没有**下面这些；前两项挂在 Release 上，其余要从持有的机器另行拷贝：

| 缺什么 | 放在哪 | 从哪拿 | 没有它会怎样 |
|---|---|---|---|
| 四类检测模型 v3（`best.onnx`、`best.pt`、`blocks4_yolov8n.om`） | `data/models/blocks4/v3/` | Release `assets-v4` 的 `blocks4_v3_models.zip`（22 MB） | 自动分拣和仿真里的检测对照不可用；仿真的画面与控制不受影响 |
| 物块 20 张贴面 | `data/reference/block_faces/` | Release `assets-v4` 的 `block_faces.zip`（0.6 MB） | **仿真后端无法启动**，`tests/test_sim.py` 全部跳过 |
| 语音识别模型（WeNet 编码器，Atlas 200I DK A2） | `data/models/wenet_asr/` | Release `assets-v4` 的 `asr_wenet_offline_encoder.zip`（65 MB） | E7 语音控制不能用；其余功能不受影响 |
| 手势识别模型（HRNet 四分类，Atlas 200I DK A2） | `data/models/gesture_hrnet/` | Release `assets-v4` 的 `gesture_hrnet_4cls.zip`（103 MB） | E1 手势交互不能用；其余功能不受影响 |
| 数据集、扫描画面、采集画面 | `data/datasets/`、`data/runs/`、`data/captures/` | 另行拷贝 | 不能重训。事件日志的样例已入库（`docs/samples/`） |
| 官方三类模型、课程包、`Arm_Lib` 源码、上游 ROS 2 工程 | `data/models/official_arm/`、`资料/` | 约 2.7 GB | 第三方内容，按[资产清单](docs/atlas-200i-dk-a2/09-资产清单.md)里的链接自行下载 |

### 取回模型与贴面

压缩包里的路径都从仓库根目录算起，在仓库根目录解压即落到正确位置。[Release 页面](https://github.com/anweat/vision-robot-arm-sorting/releases/tag/assets-v4)上可以直接点下载，或者：

```sh
gh release download assets-v4 -R anweat/vision-robot-arm-sorting -D /tmp/arm-assets
```

```sh
cd /tmp/arm-assets && shasum -a 256 -c SHA256SUMS.txt
```

```sh
unzip -o /tmp/arm-assets/blocks4_v3_models.zip -d . && unzip -o /tmp/arm-assets/block_faces.zip -d .
unzip -o /tmp/arm-assets/asr_wenet_offline_encoder.zip -d . && unzip -o /tmp/arm-assets/gesture_hrnet_4cls.zip -d .
```

最后两条在仓库根目录执行。完成后 `tests/test_sim.py` 不再跳过，仿真后端可以启动。

两个新模型的**可用性已于 2026-10-16 上板验证**：语音 10 轮识别文本一致（推理中位 121.0 ms）、手势输出 `[1,4]` 且与 ONNX argmax 一致（中位 9.07 ms）。**识别率与端到端时延仍未测**；来源与芯片核对见[语音与手势模型调研](docs/atlas-200i-dk-a2/19-语音与手势模型调研.md)，实测条件与原始数据见[硬件实测](docs/05-板卡连接与硬件实测.md)。`assets-v3` 仍然保留，只含前两项资产。

- 只做前端联调：两个包都要（贴面必需；没有模型时“检测对照”一栏报不可用，其余正常）。
- 上板跑分拣：用包里的 `blocks4_yolov8n.om`（`Ascend310B1`）；`best.onnx` 用于 Mac 等其他机器。
- 模型是 v3：2026-09-26 训练，136 张真实图 + 800 张合成图，验证集 mAP@0.5 0.952；训练细节见[资产清单](docs/atlas-200i-dk-a2/09-资产清单.md)。换了模型要发新的 Release 并改这里的标签名，不要覆盖旧包。
- 权重由 Ultralytics YOLOv8n（AGPL-3.0）训练得到；贴面是本组实物物块的照片，贴纸图案来自课程套件。两者只为本课程项目使用。

## 文档入口

- **[需求规格](docs/atlas-200i-dk-a2/10-需求规格.md)** — 按课程需求审查五项：做什么 / 不做什么 / 验收指标 / 接口契约 / 已知风险
- **[探索记录](docs/atlas-200i-dk-a2/11-探索记录.md)** — 这段开发做了什么、为什么那样做、哪些判断被实测推翻（配图）；[讲解与展示用的在线版](https://claude.ai/code/artifact/76dbd50f-7f45-4ba0-a161-8ab46088fbe2)
- **[资产清单](docs/atlas-200i-dk-a2/09-资产清单.md)** — 代码、模型、数据集、标定、实测记录与公开来源
- **[硬件实测](docs/05-板卡连接与硬件实测.md)** — 全部实机结论，按日期倒序
- **[工作区总览](docs/atlas-200i-dk-a2/README.md)** — 设备、代码地图、官方来源与实施基线

## 快速开始

```sh
uv run --with 'opencv-python<5' --with numpy --with onnxruntime python -m unittest discover -s tests   # 237 项测试，不需要硬件
```

测试结果取决于机器上有什么（2026-10-11 逐种核对过）：

| 环境 | 结果 |
|---|---|
| 持有全部 `data/` 的开发机 | 237 项全部通过 |
| 全新克隆 + 取回 Release 资产（见下文） | 231 项通过，6 项跳过（需要 `data/captures/` 里的实机采集画面） |
| 全新克隆、没有取回资产 | 通过其余部分，29 项跳过（仿真和贴面相关） |
| 没有 numpy / OpenCV（直接 `python3 -m unittest`） | 6 个测试文件导入失败，属预期；请用上面的 `uv run` 命令 |

```sh
python3 src/pipeline/run.py --dry-run --scan <扫描目录>   # 用已有扫描重建场景，不动机械臂
```

实机运行需要连接开发板与机械臂，**任何动作前先阅读[硬件安全边界](docs/05-板卡连接与硬件实测.md)并人工确认设备周围安全**。

## 遥控与几何预演

新增本地工作台：`python3 src/teleop/server.py`，打开 <http://127.0.0.1:8765/>。默认离线模拟，支持关节点动、腕部相机视锥、场景物块、冻结帧平面标定与采样。

[使用说明、当前记录索引和实机边界](docs/atlas-200i-dk-a2/08-遥控与场景标定工作台.md)。本次只完成离线验证，未执行实机动作。

## 仿真后端（给前端联调）

```sh
uv run --with 'opencv-python<5' --with numpy --with onnxruntime src/sim/sim_server.py --map map_a4_v3 --bins bins_acrylic_v1
```

打开 <http://127.0.0.1:8770/sim/>。独立模块 `src/sim/`：与实机控制台同一套 `/api/*`（同一个 `Controller`），外加合成的腕部相机画面、可抓放的物块、可更换的地图和垃圾桶、检测模型与真值的对照，以及列出全部接口的壳子页面。允许跨域，`--lan` 可供手机和局域网访问。不导入 `Arm_Lib`，不可能驱动实机。

[接口、参数来源、未建模的部分和 2026-10-11 的评估结果](docs/atlas-200i-dk-a2/16-仿真模块.md)。仿真结果不是实机精度。

## 开发入口

- [开发资料总览](docs/README.md)
- [软硬件资源](docs/01-软硬件资源.md)
- [现有代码与接口](docs/02-代码与接口.md)
- [空间定位与组合任务方案](docs/03-空间定位与任务组合.md)
- [摄像头识别、机械臂模块与 ROS 入口审计](docs/04-摄像头识别与机械臂模块审计.md)
- [板卡连接与硬件实测](docs/05-板卡连接与硬件实测.md)
- [官方指南与机械臂依赖归档](资料/官方资料/Atlas-200I-DK-A2-机械臂/README.md)
- [官方 ROS 2 工程固定源码与导入审计](docs/atlas-200i-dk-a2/06-官方源码导入审计.md)
- [全部文件清单](docs/inventory/全部资源.csv)

## 原始资料

- [资料归档导航](README-资料导航.md)
- [第二阶段离线图文说明](资料/课程说明/第二阶段-机械臂基础控制/index.html)

原始资料保持原样。运行 `python scripts/inventory_resources.py` 可重新生成清单，不执行模型或机械臂动作。不要直接批量执行课程片段或 Notebook：其中包含顶层运动指令。实机限位按指定的第二阶段课程说明执行，测试状态与未通过项见硬件实测文档。
