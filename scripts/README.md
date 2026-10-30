# scripts 脚本索引

更新：2026-10-23。本目录存放**资料盘点与受控硬件诊断脚本**（[工作区总览](../docs/atlas-200i-dk-a2/README.md)）。
本文只是索引：**不移动、不改动任何脚本**；脚本的行为以源码为准，与语音/手势相关的结论与数字见
[语音手势接入基线](../docs/atlas-200i-dk-a2/21-语音手势接入基线.md)。

## 运行位置与前置

| 位置 | 解释器 / 环境 | 说明 |
|---|---|---|
| **板卡**（Atlas 200I DK A2，`192.168.137.100`） | `/root/miniconda3/bin/python` 3.9.2；`numpy`、`cv2`、`torchaudio`、`ais_bench`、`onnxruntime 1.14.0` | 用 NPU 前先 `source /usr/local/Ascend/ascend-toolkit/set_env.sh`；调试目录常用 `/root/model-smoke/` |
| **本机（macOS）** | `python3` + `numpy` / `opencv-python` / `onnxruntime`（离线视觉）；Docker 镜像 `cann-atc:6.2rc1-py39` | 只有 `convert_om.sh` 需要容器；其余本机脚本是资料维护类 |
| **Windows（可选）** | PowerShell 5+、`ssh` | 仅 `connect_board.ps1` |

- `data/` 与 `资料/` **不入库**；脚本里出现的这些路径只在持有目录的机器上有效。模型与贴面从
  Release `assets-v3` / `assets-v4` 取回，见 [资产清单](../docs/atlas-200i-dk-a2/09-资产清单.md)。
- **会驱动硬件的脚本**（`hardware_smoke_test.py` 的 `--motion/--center-unreadable` **会动机械臂**，
  `--rgb/--buzzer` 会点亮/鸣响；`course_group_motion_test.py`、`course_pose_read_test.py` 也会动机械臂）
  执行前必须先查
  [硬件安全边界](../docs/05-板卡连接与硬件实测.md)，并人工确认设备周围安全。
- 语音/手势模型脚本（`model_smoke_test.py`、`gesture_*`、`npu_*`）**不 import `Arm_Lib`、不驱动舵机**，
  可以只做推理。

## 一、语音 / 手势 / NPU（板上）

| 脚本名 | 用途 | 怎么跑 | 依赖 | 是否需要板卡 | 输出 |
|---|---|---|---|---|---|
| [`model_smoke_test.py`](model_smoke_test.py) | OM 冒烟：加载、输入输出规格、各阶段耗时；手势可选与 ONNX 逐值对照 | `python3 scripts/model_smoke_test.py asr --dir <目录> [--wav x.wav] [--runs 3]`；`python3 scripts/model_smoke_test.py gesture --dir <目录> [--image hand.jpg] [--runs 20] [--seed 20261016]` | `ais_bench`、`numpy`（手势可选 `cv2`、`onnxruntime`） | **是** | 末尾 JSON；`--out` 另存干净副本；手势会写 `/tmp/gesture_smoke_input.npy` |
| [`gesture_capture.py`](gesture_capture.py) | 从板载相机服务抓帧、逐帧跑 `hrnet.om`、按 0.6 阈值统计 | `python3 scripts/gesture_capture.py --label fist --seconds 5`（`--label` 必填：`fist`/`like`/`ok`/`palm`；`--dir` 默认 `/root/model-smoke/gesture_hrnet`；`--out` 默认 `/root/model-smoke/captures`） | `ais_bench`、`numpy`、`cv2`、板载相机服务 `:8080/snapshot.jpg` | **是**（且相机服务在跑） | `<out>/<label>/` 帧图 + `<out>/results.json` + 屏显 JSON |
| [`gesture_val_eval.py`](gesture_val_eval.py) | 在课程验证集上算混淆矩阵，核对标签顺序与训练域表现 | `python3 scripts/gesture_val_eval.py --images val/gesture_val --dir gesture_hrnet [--write cm.json]` | `ais_bench`、`numpy`、`cv2` | **是** | 屏显混淆矩阵/准确率；`--write` 输出 JSON |
| [`npu_concurrency_test.py`](npu_concurrency_test.py) | 三模型（语音/手势/检测器）共存、线程并发、一模型一进程的延迟与吞吐 | `python3 scripts/npu_concurrency_test.py --dir /root/model-smoke --seconds 6` | `ais_bench`、`numpy`、`npu-smi` | **是** | 末尾 JSON（含 `npu-smi` 显存前后、`--worker` 内部模式） |
| [`npu_thread_variants.py`](npu_thread_variants.py) | 对照实验：session 跨线程共享是否必崩；线程内自建 session 是否可行 | `python3 scripts/npu_thread_variants.py --dir /root/model-smoke --seconds 5` | `ais_bench`、`numpy` | **是** | 末尾 JSON（变体 A/B/C） |

`model_smoke_test.py asr` 要求 `--dir` 里同时有 `offline_encoder.om`、`vocab.txt`、`model.py` 和
默认的 `sample.wav`（`assets-v4` 的 `asr_wenet_offline_encoder.zip` 齐全）；本机 `资料/展开资源/wenet/WeNet/`
那份只有 om/vocab/wav，**缺 `model.py`**。手势的 ONNX 对照要求同目录有 `hrnet.onnx`（可选）。

## 二、硬件诊断与机械臂（板上，可能产生运动）

| 脚本名 | 用途 | 怎么跑 | 依赖 | 是否需要板卡 | 输出 |
|---|---|---|---|---|---|
| [`hardware_smoke_test.py`](hardware_smoke_test.py) | 六舵机回读、RGB、蜂鸣器、相机、单舵机小步、居中不可读舵机 | `python3 scripts/hardware_smoke_test.py --read-only`（或 `--rgb` `--buzzer` `--camera` `--motion [--step 5]` `--center-unreadable`），六个模式互斥且必选其一 | `Arm_Lib`（`--camera` 只需 `cv2`） | **是** | 屏显 JSON（`MOTION_TEST`/`CAMERA_TEST=PASS` 等） |
| [`course_group_motion_test.py`](course_group_motion_test.py) | 验证课程 `Arm_serial_servo_write6` 六舵机批量路径：±5° 小步后复位 | `python3 scripts/course_group_motion_test.py` | `Arm_Lib` | **是** | 屏显 `COURSE_GROUP_MOTION_TEST` JSON |
| [`course_pose_read_test.py`](course_pose_read_test.py) | 跑课程“全 90°”复位姿态并核对回读，再按基线复位 | `python3 scripts/course_pose_read_test.py` | `Arm_Lib` | **是** | 屏显 JSON（前后回读） |

`hardware_smoke_test.py --camera` 直接 `cv2.VideoCapture(0)`，与板载相机服务抢 `/dev/video0`；
跑之前先停 `board_camera_server.py`。

## 三、部署、转换与运行（Mac / 板卡）

| 脚本名 | 用途 | 怎么跑 | 依赖 | 是否需要板卡 | 输出 |
|---|---|---|---|---|---|
| [`convert_om.sh`](convert_om.sh) | ONNX → `Ascend310B1` `.om`（Mac 容器内，不需要板卡） | `sh scripts/convert_om.sh [data/models/blocks4/v3]` | Docker 镜像 `cann-atc:6.2rc1-py39`、`~/Ascend/cann-6.2.RC1-aarch64/ascend-toolkit` | **否** | `<模型目录>/blocks4_yolov8n.om`（7.1 MB 量级） |
| [`deploy_to_board.sh`](deploy_to_board.sh) | `src/` + `config/` 同步到板上，并重启板载相机服务 | `sh scripts/deploy_to_board.sh` | `ssh`、`scp`/`tar`、`curl`；板卡可达 | **目标板卡** | 板上 `/root/arm-app/`；屏显相机健康检查 URL |
| [`start_console.sh`](start_console.sh) | 板卡重启后恢复：相机服务、手机反向隧道、控制台（`--hardware --execute`）、Mac→板卡端口转发 | `scripts/start_console.sh [phone-ip:port]` | `ssh`（需先开 ControlMaster）、板卡上的 `venv-vision` | **是** | 板上服务/隧道；屏显 `http://127.0.0.1:8765/` |
| [`connect_board.ps1`](connect_board.ps1) | Windows PowerShell 下一键探测 SSH 端口并连接 | `powershell -File scripts/connect_board.ps1 [-HostName 192.168.137.100] [-UserName root] [-Port 22]` | Windows PowerShell、`ssh`、`Test-NetConnection` | **是** | 交互式 SSH 会话 |

## 四、资料与文档维护（本机）

| 脚本名 | 用途 | 怎么跑 | 依赖 | 是否需要板卡 | 输出 |
|---|---|---|---|---|---|
| [`inventory_resources.py`](inventory_resources.py) | 只读盘点工作区：文件清单、大小、SHA-256、Python/Notebook 结构；不执行模型或机械臂代码 | `python3 scripts/inventory_resources.py` | Python 标准库 | **否** | `docs/inventory/全部资源.csv`、`代码结构.json`、`盘点统计.json` |
| [`organize_course_guides.py`](organize_course_guides.py) | 把浏览器导出的课程 JSON 整理成离线图文归档 | `python3 -I scripts/organize_course_guides.py <export.json> <输出目录> [旧导出.json]` | Python 标准库 | **否** | 输出目录里的 `README.md`、`index.html`、图文/代码片段 |
| [`organize_stage2.py`](organize_stage2.py) | 整理第二阶段课程导出：图片本地化、代码片段拆分、生成 HTML/TXT | `python3 scripts/organize_stage2.py`（先有 `资料/课程说明/第二阶段-机械臂基础控制/课程原始内容.json`） | `beautifulsoup4`、Python 标准库、可访问 `10.126.59.136` | **否** | `资料/课程说明/第二阶段-机械臂基础控制/` 下的 HTML/TXT/MD/图片 |
| [`build_print_docs.js`](build_print_docs.js) | 把 `docs/atlas-200i-dk-a2/` 里指定的几份 Markdown 生成为可打印 `.docx`/`.pdf` | `NODE_PATH=<node_modules> node scripts/build_print_docs.js` | Node.js；`docx`、`marked@12`、`image-size`；PDF 需要本机 Google Chrome | **否** | `docs/print/*.docx`，有 Chrome 时另有 `*.pdf` |

## 常见用法

```sh
# 板上：语音/手势 OM 冒烟（不 import Arm_Lib）
source /usr/local/Ascend/ascend-toolkit/set_env.sh
python3 scripts/model_smoke_test.py asr --dir /root/model-smoke/wenet_asr --runs 3 --out /tmp/asr.json
python3 scripts/model_smoke_test.py gesture --dir /root/model-smoke/gesture_hrnet --runs 20 --out /tmp/gesture.json

# 板上：现场手势抓拍（先把板载相机服务起在 :8080）
python3 scripts/gesture_capture.py --label fist --seconds 5

# 板上：训练域混淆矩阵
python3 scripts/gesture_val_eval.py --images val/gesture_val --dir /root/model-smoke/gesture_hrnet --write /tmp/cm.json

# 板上：三模型共存 / 线程归属
python3 scripts/npu_concurrency_test.py --dir /root/model-smoke --seconds 6
python3 scripts/npu_thread_variants.py --dir /root/model-smoke --seconds 5

# 本机：资料盘点 / 打印件
python3 scripts/inventory_resources.py
NODE_PATH="$HOME/node_modules" node scripts/build_print_docs.js
```

## 约束

- 本索引对应 2026-10-23 的 `scripts/` 现状（16 个文件：11 个 `.py`、3 个 `.sh`、1 个 `.js`、1 个 `.ps1`）；
  新增脚本时请同步更新本表，不要为“整理”移动或改名现有脚本。
- `gesture_capture.py` 的 `--dir`/`--out` 与 `npu_*` 的 `--dir` 默认值是**板上调试目录**
  （`/root/model-smoke/`），不是仓库内的 `data/models/`；`model_smoke_test.py` 的 `--dir` 是必填参数。
- 所有脚本的结论分级（文件存在 / 板卡实测 / 尚未测）以文档里的原始记录为准，不要只看脚本能否运行。
