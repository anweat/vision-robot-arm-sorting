# app — 板卡应用完整版本

更新：2026-11-19。这里是**部署到板上的全部东西**，加上网页前端和手机 App。目录布局和板上的 `/root/arm-app` 完全相同（`app/src/…` 在板上就是 `/root/arm-app/src/…`），整个目录单独拷走就能部署和运行，不依赖仓库里的其他目录。

仓库其他部分：`dev/` 是仿真后端、离线分析工具和实验脚本，会引用这里的模块；`docs/` 是文档；`archive/` 放已被替代的旧实现。结构和归档清单见 [34-仓库结构与应用版本](../docs/atlas-200i-dk-a2/34-仓库结构与应用版本.md)。

## 组成

| 部分 | 位置 | 在哪运行 | 作用 |
|---|---|---|---|
| 相机服务 | `src/adapters/board_camera_server.py` | 板上 :8080（仅本机） | `/dev/video0` 的唯一持有者，给出带采集时间的帧 |
| 控制台 | `src/teleop/server.py --hardware --execute` | 板上 127.0.0.1:8765 | 唯一的安全边界：限位、禁入区、隐式操作权与 2 s 租约、分段回读、速度、摇杆驱动、细步触地（[32](../docs/atlas-200i-dk-a2/32-操作逻辑简化与分步任务.md)、[33](../docs/atlas-200i-dk-a2/33-触地标定与精确回读.md)） |
| 板载网关 | `src/web/serve.py --lan --port 8090` | 板上 :8090 | 对外唯一入口：口令、多端、SSE 推送、日志与统计、定时、任务、报警。「开始」默认跑已验证的旧流水线（`--task pipeline` → `src/pipeline/task.py`）；`--task flow` 是分步编排器，试验中 |
| 网页前端 | `src/web/index.html`、`src/web/js/` | 由网关托管 | 作业/控制/任务/历史/定时页，无构建的 ES Module |
| 分拣流水线（默认） | `src/pipeline/run.py`、`src/pipeline/task.py`、`src/tasks/console_grasp.py` | 板上，由网关拉起 | 扫描→场景→抓放，实机跑通（最好一轮 4/4） |
| 分步流程（试验） | `src/flow/`（编排器、S1–S7 步骤）、`config/flows/*.yaml` | 板上，网关 `--task flow` 或单步 | 扫描→定位→建图→规划→执行→核对；规划器对实机场景太保守，优化前不当默认（[34](../docs/atlas-200i-dk-a2/34-仓库结构与应用版本.md) §4.2、§6） |
| 规划与几何 | `src/planning/`、`src/spatial/`、`src/vision/`、`src/pipeline/` | 板上 | 碰撞地图、A\*、四连杆夹爪、整臂网格碰撞、正逆解、地图定位、检测 |
| 操作员任务 | `src/tasks/`：`console_grasp`、`sort_blocks`、`scramble`、`bin_checkout`、`touch_survey`、`gripper_probe` | 板上，经控制台 | 抓放、分拣、打乱、桶核对、触地标定、夹爪慢合探测 |
| 语音/手势指令 | `src/interact/intents.py` | 仿真已接；实机未接 | 识别结果→指令的判定（[25](../docs/atlas-200i-dk-a2/25-语音手势接口.md)） |
| 手机 App | `mobile/android/` | 安卓手机 | 跳板（Tailscale 转发网关与 SSH）+ 双摇杆控制 + 任务页（[说明](mobile/android/README.md)） |
| 上板脚本 | `scripts/board_up.sh`、`scripts/board_beacon.py`、`scripts/board_check.py`、`scripts/deploy.sh` | 板上 / Mac | 起停三项服务、局域网广播板卡地址、板卡自检（只读）、从 Mac 部署；整条链路检查用仓库的 `dev/tools/check_links.py`（[36](../docs/atlas-200i-dk-a2/36-启动与链路检测指南.md)） |
| 配置 | `config/` | — | 限位、地图、桶、夹爪与连杆模型、标定、规划参数、流程脚本 |
| 接口契约与样例 | `contracts/log_event.schema.json`、`samples/` | — | 日志契约的 JSON Schema（由 `src/tools/log_contract.py` 生成）；回放与测试用的日志样例 |
| 测试 | `tests/` | Mac | 只测这里的代码，不需要硬件 |

`data/` 不入库：板上是真实目录（模型、运行记录、网关口令 `data/gateway_token`）；在 Mac 上 `app/data` 是指向仓库根 `data/` 的软链接。

## 部署与启动

```sh
# Mac：网线直连（root@192.168.137.100）
app/scripts/deploy.sh
# Mac：经手机跳板（31 §2.1），再重启三项服务
BOARD_HOST=root@<手机 Tailscale IP> BOARD_PORT=2222 app/scripts/deploy.sh --restart
```

```sh
# 板上（/root/arm-app）：起相机、控制台、网关，可重复执行；stop 停掉三项
sh scripts/board_up.sh
```

启动控制台**不会让机械臂动**：控制台只读舵机，直到某个客户端发出第一条运动指令时才取得操作权。任何动作前先看[硬件安全边界](../docs/05-板卡连接与硬件实测.md)，并人工确认设备周围安全。“停止”不是急停：`Arm_Action_Mode(0)` 未经验证。

## 测试

```sh
uv run --with 'opencv-python<5' --with numpy --with onnxruntime --with pyyaml python -m unittest discover -s app/tests
```

2026-11-19，Mac：344 项通过，其中 1 项跳过（需要 `ais_bench`，只在板上有）。仿真与离线工具的测试在仓库根的 `tests/`。

## 状态（区分仿真与实机）

| 部分 | 实机 | 仿真 |
|---|---|---|
| 控制台运动接口（一套限位、隐式操作权、速度、`goto`、`drive`） | 板上在用（2026-11-18 触地标定经它驱动） | 经网关整轮联调 |
| 细步触地、精确回读 | 2026-11-18 实测（[33](../docs/atlas-200i-dk-a2/33-触地标定与精确回读.md)） | 舵机模型对比 |
| 网关上板、手机跳板 | 2026-11-18 经跳板 SSH 上板；网关在板上起过 | App 经 Tailscale 对仿真联调 |
| 分步流程 S1–S7、网页任务页 | **未上实机**；2026-11-20 规划器按实机记录改进后，离线跑 28 次实机扫描：够得着的 152 块全部能规划抓取，抓 + 放 139/163，实机夹住过的 7 块全部规划得出（[34](../docs/atlas-200i-dk-a2/34-仓库结构与应用版本.md) §8）；「开始」仍默认旧流水线 | 三种布局整轮通过；带桶随机布局投对 8/20（旧流水线 9/20） |
| 手机双摇杆 | **未上实机**；速度上限 40°/s 未量 | 已联调 |

板上那份代码以本目录为准；2026-11-19 整理时跳板手机离线，没能和板上的副本逐文件比对。
