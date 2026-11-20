# Android 手机端：跳板 + 控制

同一个 App 有两个用途（[31 §2.1](../../../docs/atlas-200i-dk-a2/31-板载端分离与中继通信.md)、[32 §4](../../../docs/atlas-200i-dk-a2/32-操作逻辑简化与分步任务.md)）：

- **跳板**：插在板卡上的那部手机开 USB 网络共享，App 的前台服务把板卡的网关（本机 :8090）和 SSH（本机 :2222）转发给 Tailscale 上的其他设备。只放行本机、Tailscale 段（100.64.0.0/10）和本机 tun 网段；局域网可在「连接」页放行。
- **控制**：底部五个页签——控制（双摇杆、相机、速度、夹爪、停止）、任务（开始/停止、单步、报警确认）、网页（WebView 打开 `app/src/web` 的页面）、连接、旧版（原生状态与导出）。

所有指令经板载网关 `app/src/web/serve.py` 的 `POST /api/cmd`（`source: "mobile"`），限位、禁入、死人开关都在控制台；App 不做安全判断，网关拒绝什么就显示什么。

## 编译与安装

```sh
cd app/mobile/android
echo "sdk.dir=$HOME/Library/Android/sdk" > local.properties     # 不入库
./gradlew assembleDebug
adb install -r -t app/build/outputs/apk/debug/app-debug.apk
```

vivo 手机每次 adb 安装都要在手机上点「授权本次安装」。调试签名换了机器会对不上，需先 `adb uninstall com.example.myapplication00001`。

## 不接板卡时用 Mac 扮演板卡（仿真）

```sh
# Mac：仿真控制台 8770 + 网关 8090（.claude/launch.json 的 arm-sim、gw-dev）
adb reverse tcp:18090 tcp:8090     # 手机 127.0.0.1:18090 → Mac 网关
```

App「连接」页：网关地址 `http://127.0.0.1:8090`；跳板的板卡地址 `127.0.0.1`、板卡网关端口 `18090`；打开跳板。其他设备用 `http://<手机 Tailscale IP>:8090/` 访问。

## 接板卡

连不上时先在 Mac 上跑 `python3 dev/tools/check_links.py`，它逐跳告诉你断在哪（[36](../../../docs/atlas-200i-dk-a2/36-启动与链路检测指南.md)）；adb 单步见 `dev/tools/phone.sh`。

步骤见 [31 §2.1c](../../../docs/atlas-200i-dk-a2/31-板载端分离与中继通信.md)：板上跑 `app/scripts/board_beacon.py`（部署后在板上是 `/root/arm-app/scripts/board_beacon.py`），App 的板卡地址留空即可自动找到；手机插板卡前先 `adb tcpip 5555`，之后经 `adb connect <手机 Tailscale IP>:5555` 调试手机。

## 边界

- `local.properties`、构建输出、IDE 缓存不入库。
- 2026-11-18 只在仿真上联调过（数字见 31 §2.1b）；实机未测。动实机前按[硬件安全边界](../../../docs/05-板卡连接与硬件实测.md)。
- 竖屏布局；32 §4.1 设计的横屏未做。
