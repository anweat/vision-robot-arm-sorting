# src/web — 对外服务与电脑端网页（F1）

F1-1 的服务与 F1-2/3/4 的页面。只依赖标准库（服务端）与无构建的原生 ES Module（前端），
日志一律按[日志契约 v1](../../docs/atlas-200i-dk-a2/18-日志契约.md)读取，唯一来源是
`src/tools/log_contract.py`。

## 跑起来

```sh
# 没有实机：回放样例日志（20 倍速、循环），页面、事件名、字段与实机相同
python3 src/web/serve.py --replay docs/samples/log_sim_8rounds.jsonl --speed 20 --loop

# 接仿真后端产生的活日志
python3 src/web/serve.py --log data/sim/log.jsonl

# 局域网给手机看（阶段二的 E8 也会用这一层）
python3 src/web/serve.py --log data/log.jsonl --lan --token <口令>

# 接控制台：运动指令真的发出去，不再返回 501
python3 src/web/serve.py --console http://127.0.0.1:8770 --log data/sim/log.jsonl
```

## 接控制台（`--console`）

控制台是执行侧，可以是仿真 `src/sim/sim_server.py`（8770）或实机 `src/teleop/server.py`；
两边答同一套 `/api/*`（接口文档 §4），所以**换目标不用改这里一行代码**——但实机有一条
额外约束：使能（`enable`）要求服务端启动时显式加 `--onsite`，即启动服务的人必须在设备
现场；不加时网页点"使能"会被拒并写明原因。目前仿真已跑通，**实机尚未联调**。接上之后：

- `POST /api/cmd` 的 `enable` / `gripper` / `set_joints` / `move_to` / `stop` 转发过去真执行
- `GET /api/status` 多出 `arm` 段：六轴回读、使能、运动中、夹爪、末端坐标、安全余量、桌上物块
- `GET /api/frame` 转来腕部相机画面（仿真是合成渲染，实机是真相机）
- 页头显示控制台地址，并外链它自己的 3D 工作台（ `/sim/` ）

**安全边界不在这里重复实现**：控制台只接受小步（关节点动 ±1/2/5/10°、直角每步 ≤ 3 cm，且要带新鲜
`revision`），所以网页给的绝对目标会在服务端拆成一串小步交它逐个校验；限位、禁入区、堵转由它把关，
它拒绝时把中文原因原样回给页面（例如"保持当前俯角无法到达"），本服务不假装执行。

使能只有 2 s（`core.enable`），多步运动期间由后台线程每 1 s 续期；操作权被抢或页面切走就停。

不接控制台时，运动类指令仍按 501 返回，只有日志与模式可用——这是阶段一的默认状态。

## 定时启动 W4（`--timers`）

**默认关闭**：不传 `--timers` 时这个功能整块不存在，服务不排任何任务，也不会在没人在场
的时候自己启动机械臂。要启用必须显式给出任务表文件：

```sh
python3 src/web/serve.py --console http://127.0.0.1:8770 \
    --log data/sim/log.jsonl --timers data/web/timers.json
```

需求规格定的是「启动时刻偏差 ≤ 5 s」，兜底是「到点先提示 5 s，可在页面取消」：

- 检查周期 0.5 s，所以启动偏差在这个量级
- 到点前 5 s 进"待发"，`/api/status` 的 `timer.pending` 变 `true`，此时仍可 `POST /api/timers {"cancel": <id>}` 取消
- 到点才调 `state.start_task()`——和网页下发 `start` 走的是同一条路径
- 想每天固定时刻自动跑就用 `repeat: "daily"`；一次性任务跑完自动删除。任务存在文件里，服务重启不丢

```sh
curl -X POST localhost:8080/api/timers -d '{"at":"2026-10-23T09:00:00","repeat":"daily"}'
curl -X POST localhost:8080/api/timers -d '{"cancel":"t1761000000000"}'
```

任务 IDs 与 `data/web/timers.json` 是运行产物，按需自行 gitignore。

## 未知物体告警 E6（`--archive`）

判定与图片由 H1-6 的视觉侧给出：它把图片存进 `--archive` 目录（默认 `data/archive/`），
并写一条 `unknown_object` 事件（必填 `cube` 与 `image`）。本服务只负责安全地取出来给页面看，
**自己不做任何识别**。所以 H1-6 接进来之前，这一页会老实显示"还没有"，不拿别的事件凑数。

图片按名字取，而名字来自日志事件 = 外部输入，所以有目录边界校验（`serve.safe_under`）：
`../` 穿越、绝对路径、盘符、空字节一律拒绝。改动那条边界请同步改 `tests/test_archive.py`。

## 验收数据

`tests/test_w2_acceptance.py` 自带仿真控制台量下面三条，跑完把结论打印出来；
换 `--console` 指向实机即可在同一脚本上重跑，脚本不用改。

```sh
python3 tests/test_w2_acceptance.py --out w2.txt
```

截至 2026-10-23 的实测（仿真执行侧）：

| 指标 | 需求 | 实测 | |
|---|---|---|---|
| 角度刷新 | ≥ 2 Hz | 2.49 Hz | 通过 |
| 滑块响应 | ≤ 1 s | 401 ms | 通过 |
| 坐标到位误差 | ≤ 1 cm | 1.8 mm | 通过 |

读这三行要注意两点：

- **滑块响应 401 ms 是上界**。状态推送周期就是 0.4 s，网页看到的粒度同样如此，
  所以真实响应不会比这个数更大，但可能更小——要更精确的得绕开推送层直接看回读。
- **坐标到位 1.8 mm 是仿真值**。仿真是理想运动学，没有实机的齿轮间隙、负载形变和
  标定误差。实机的这一条要等 H1-3 接进来后重跑才算数，现在不能拿它当实机达标。

量不出来的指标一律报"无法测量"，不拿通过凑数。

打开 <http://127.0.0.1:8080/>。五个页签对应分工表里的任务：

| 页签 | 任务 | 验收指标 |
|---|---|---|
| 概览 | F1-2 | 事件发生到展示 ≤ 1 s（页头有延迟估计）；本轮逐块记录六项齐全 |
| 手动操作 | F1-3 + F1-4 | 六轴角度 ≥ 2 Hz 回读；滑块与坐标下发；模式切换写 `mode_changed` |
| 告警 | F1-2 | 识别/抓取/投放/路径四类失败可查；支持按事件筛选 |
| 告警（异常物体） | F1-6 | 列出 `unknown_object` 事件与存档图片；视觉侧接入前如实显示"还没有" |
| 任务计划 | F1-5 | 任务可排、可看、可取消；到点前 5 s 全站横幅倒计时，启动偏差 ≤ 5 s |

预设动作等附加能力默认收在"扩展与调试"里：顶栏 ⚙ 或 `?debug=1` 开启，状态存浏览器。

## 接口

与仿真后端（`src/sim/sim_server.py`）同名同义，页面从仿真切到实机不用改：

| 路径 | 返回 |
|---|---|
| `GET /api/log?after=&limit=` | `{contract_version, events, total, next}`，`seq` 为 1 起行号 |
| `GET /api/log/stream?after=` | SSE，`id` 是 `seq`，断线按 `Last-Event-ID` 续传 |
| `GET /api/log/records` | 逐块记录（时间/类别/来源/目标/耗时/结果 + 轮次） |
| `GET /api/log/summary` | 统计（复用 `log_contract.summary()`） |
| `GET /api/scene` `/api/scene/image` | 最近一次扫描；还没有时 404 |
| `GET /api/status` `/api/status/stream` | 模式、六轴角度与 `arm` 段（接了控制台才有）；SSE 2.5 Hz（≥ 2 Hz 达标） |
| `GET /api/frame` | 腕部相机 JPEG，转自控制台；未接控制台时 404 |
| `GET/POST /api/timers` | W4 定时任务列表 / 新增 / 取消（需 `--timers`） |
| `GET /api/blocks` | 桌上物块的基座坐标、类别、颜色，外加禁入区尺寸（厘米）；执行侧没有这一层时 `blocks` 为空数组 |
| `GET /api/archive` | E6 未知物体告警列表（从日志里筛 `unknown_object`） |
| `GET /api/archive/<name>` | 存档图片。名字来自事件的 `image` 字段，按目录边界校验后才读 |
| `POST /api/cmd` | 需求规格 ④：`start` `stop` `set_mode` `run_preset` `set_joints` `move_to` `unlock_bin`，另加 `enable` `gripper` |
| `GET /api/health` | 健康检查，含 `console` / `console_ready` / `console_error` |

`/api/cmd` 校验字段并写日志：`set_mode` 直接生效并写 `mode_changed`，`stop` 写 `stopped`；
运动类指令在控制台（H1-3）接入前一律 501，**不假装执行**。多步运动在服务端串行化：
上一段没走完时新的运动指令返回 409，页面提示等一等。

写入口径（B1）：不带 `Access-Control-Allow-Origin: *`；写请求要求同源（`Origin` 对得上
`Host`，对不上 403）且 `Content-Type: application/json`（否则 415）。`--token` 开启
`X-Token` 鉴权（页面经 `?token=` 或会话存储传递）；静态资源免鉴权，免得开了 token 白屏。
`--lan` 不给 `--token` 时会自动生成一个并打印到终端，不裸奔。

> 接口细节随后续迭代并入[接口文档](../../docs/atlas-200i-dk-a2/15-接口文档.md) §8。

## 阶段一验收口径（10.23）

- **条数一致**：`/api/log` 的 `total` = 日志行数；`records` 与 `log_contract.records()` 逐条相等。见 `tests/test_web_serve.py`。
- **延迟 ≤ 1 s**：契约禁止把 `t` 和开发机时钟相减（板卡快 8 小时），所以页面用
  「日志自身时间轴的推进 vs 页面墙钟的推进」之差估计延迟，页头实时显示。
- 测试：`python3 tests/test_web_serve.py`。

## 文件

```
serve.py          对外 HTTP 服务（F1-1）
index.html        页面骨架
style.css         深色主题，与 src/sim/web 一致
js/contract.js    契约常量的前端镜像（事件表改了这里要跟着改）
js/api.js         数据层：SSE + 轮询退化 + /api/cmd
js/store.js       事件流 → 轮次/计数/模式 的派生状态
js/live.js        实时状态页（F1-2）
js/faults.js      异常日志页（F1-2）
js/control.js     控制面板 + 模式切换（F1-3/F1-4）
js/scene3d.js     三维状态视图（骨架点用服务端正运动学回读，不在前端重算）
js/format.js      显示格式与轻提示
```

## 还没做的（按分工排期）

- F1-1 另一半：~~作为唯一客户端持有控制台 token 的代理~~ —— 已做，见上面"接控制台"（`Console` 取
  `/api/bootstrap` 的一次性凭证，所有请求带 `X-Console-Token`，控制台重启后自动重取）。
- ~~F1-5 定时启动~~、~~F1-6 未知物体告警页~~：已做，见上面两节。
- `start`（跑分拣）在仿真侧是 `POST /api/sim/sort`，实机侧是流水线；两端入口不同，等 E1 定下来再接。
- 预设动作（跳舞 / 叠积木 / 搬运）是一组动作，由硬件组提供，接进来之前页面禁用并写明原因。
- F1-7 接实机联调。
- 登录页（`--token` 先顶着）。
- 手机端页面（分拣记录、统计汇总、导出）与语音/手势接入页：F2 的范围，不在本目录。
