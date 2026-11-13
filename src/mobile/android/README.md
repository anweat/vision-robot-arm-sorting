# Android 手机端

手机作业台通过 WebView 打开 F1 网页，使用同一套概览、手动操作、告警和任务计划。首页右上角的「记录」打开 Android 原生页面，显示逐块记录、历史统计并导出 CSV 或 Excel。两处共用 F1 网关地址和口令。

## 先用样例日志检查页面

在电脑 PowerShell 中执行以下命令。样例写入单独的回放日志。

```powershell
.\.venv\Scripts\python.exe src/web/serve.py --lan --log data/web/f2-demo.jsonl --replay docs/samples/log_sim_8rounds.jsonl --speed 20
```

在仓库根目录运行。终端会显示本次访问口令。电脑可先访问 `http://127.0.0.1:8080/?token=本次口令`。手机和电脑连在可互访的网络上，手机 App 填 `http://电脑的WLAN-IPv4:8080/?token=本次口令`。Windows 可用 `ipconfig` 查看 WLAN IPv4 地址。每次用自动口令重启网关后，在 App 的「连接」里更新地址。仅在本机模拟器预览时可省略 `--lan`，在 App 输入 `http://10.0.2.2:8080/`。

样例只有日志和统计，8 轮回放完成后应显示 32 条逐块记录。分别导出 CSV 和 Excel，打开文件核对两份都有 32 条数据行。CSV 为 UTF-8 BOM。分拣成功率 = 成功 ÷（成功 + 夹空 + 跳过 + 失败）；挪开障碍单独计数。

## 在 Android Studio 运行

1. 打开 Android Studio，选择 **File → Open**，选中包含 `settings.gradle.kts` 的 `src/mobile/android` 目录。Windows 下将工程放在纯 ASCII 路径中，避免中文目录名导致 Gradle 同步失败。
2. 等待 Gradle Sync。工程使用 Android Gradle Plugin 9.1.0、Gradle 9.3.1、编译 SDK 36.1。若提示找不到 SDK，在 **Tools → SDK Manager** 检查 SDK 位置。
3. 真机开启开发者选项和 USB 调试，用数据线连接电脑，解锁并允许 USB 调试。顶部运行配置选 `app`，设备选手机，再点 **Run ▶**。也可在 Device Manager 启动 Android 模拟器。
4. 打开「机械臂垃圾分拣」，在 App 内填写网关地址，点「连接」，随后用底部网页页签或右上角「记录」。Android 模拟器可用 `http://10.0.2.2:8080/?token=本次口令`。

如果真机 Wi-Fi 无法访问电脑，可在电脑的 PowerShell 使用 USB 转发，再在 App 输入 `http://127.0.0.1:8080/?token=本次口令`。先将 Android SDK 的 `platform-tools` 加入 PATH。第二条仅供内嵌三维仿真使用。

```powershell
adb reverse tcp:8080 tcp:8080
adb reverse tcp:8770 tcp:8770
```

## 联调仿真与手机

结束样例网关后，在两个 PowerShell 窗口各切到仓库根目录，分别运行：

```powershell
.\.venv\Scripts\python.exe src/sim/sim_server.py --lan
```

```powershell
.\.venv\Scripts\python.exe src/web/serve.py --lan --console http://127.0.0.1:8770 --log data/sim/log.jsonl
```

电脑和 App 都打开 8080，并使用本次口令。在「手动操作」打开内嵌三维仿真，在「分拣日志」面板使用「开始分拣」和停止按钮。手机的「记录」与电脑网关都读取同一日志。若 8080 能连但内嵌画面打不开，在手机浏览器检查 `http://电脑WLAN-IPv4:8770/sim/`，或使用 USB 转发方式。

需要验证任务计划时，启动网关时追加 `--timers data/web/timers.json`。记录页也读取 `/api/status` 的待发状态，并通过 `/api/timers` 取消当前待发任务。正式页面或接口来自 [F1 PR #1](https://github.com/anweat/vision-robot-arm-sorting/pull/1) 对应的 `src/web`。这里的启停步骤针对仿真。
