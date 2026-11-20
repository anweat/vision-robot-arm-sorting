# 实测与实验的原始结果（入库）

更新：2026-11-20。这里只放**小而且要被文档引用**的原始结果；大体积的运行数据（扫描画面、会话、数据集）在本机 `data/`，不入库。

| 目录 | 内容 | 谁写的 | 引用它的文档 |
|---|---|---|---|
| `hardware-tests/` | 板上冒烟、模型、NPU 并发、语音/手势、声光与演示调参的原始 JSON，文件名以日期开头 | `dev/scripts/` 里的 `hardware_smoke_test.py`、`model_smoke_test.py`、`npu_concurrency_test.py`、`gesture_val_eval.py` 等（在板上跑，结果拷回） | [05 硬件实测](../docs/current/05-板卡连接与硬件实测.md) |
| `experiments/` | 不接板卡的 2/3 视角方块定位实验（说明 + 结果 JSON + 合成视角图） | `dev/spatial/multiview_demo.py` | [实验说明](experiments/多视角方块定位离线实验.md) |
| `inventory/` | 2026-09 的资料盘点快照（全部文件 CSV、代码结构、统计） | `dev/scripts/inventory_resources.py` | 归档的阶段 0 文档 |
| `sim/` | Blender 运动学回放视频 | `dev/scripts/blender_arm_playback.py` | [28a](../docs/current/28a-三维模型与Blender仿真.md) |

新增结果的约定（`AGENTS.md`）：文件名带日期；内容里写设备、版本、条件；文档里引用时注明这是实机、仿真还是离线。
