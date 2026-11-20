package com.example.myapplication00001

import android.Manifest
import android.content.pm.PackageManager
import android.os.Build
import androidx.activity.compose.rememberLauncherForActivityResult
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.text.selection.SelectionContainer
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.Button
import androidx.compose.material3.HorizontalDivider
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.OutlinedTextField
import androidx.compose.material3.Switch
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.ui.text.input.PasswordVisualTransformation
import androidx.compose.ui.text.input.VisualTransformation
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.unit.dp
import androidx.core.content.ContextCompat
import com.example.myapplication00001.relay.RelayConfig
import com.example.myapplication00001.relay.RelayLog
import com.example.myapplication00001.relay.RelayService
import kotlinx.coroutines.delay

@Composable
fun ConnectScreen(onGatewayChanged: () -> Unit, modifier: Modifier = Modifier) {
    val context = LocalContext.current
    var url by remember { mutableStateOf(ArmClient.savedUrl(context)) }
    var token by remember { mutableStateOf(ArmClient.savedToken(context)) }
    var saved by remember { mutableStateOf("") }
    var showToken by remember { mutableStateOf(false) }

    val initial = remember { RelayConfig.load(context) }
    var boardHost by remember { mutableStateOf(initial.boardHost) }
    var gatewayRemote by remember { mutableStateOf(initial.gatewayRemote?.toString() ?: "") }
    var allowLan by remember { mutableStateOf(initial.allowLan) }
    var running by remember { mutableStateOf(RelayService.instance != null) }
    var snap by remember { mutableStateOf(RelayService.instance?.snapshot()) }
    var log by remember { mutableStateOf(RelayLog.snapshot()) }

    val notifyPermission = rememberLauncherForActivityResult(ActivityResultContracts.RequestPermission()) { }

    LaunchedEffect(Unit) {
        while (true) {
            running = RelayService.instance != null
            snap = RelayService.instance?.snapshot()
            log = RelayLog.snapshot()
            delay(1000)
        }
    }

    fun relayConfig() = RelayConfig(
        boardHost = boardHost.trim(),
        gatewayRemote = RelayConfig.validPort(gatewayRemote.trim().toIntOrNull()),
        allowLan = allowLan,
    )

    Column(
        modifier.fillMaxSize().verticalScroll(rememberScrollState()).padding(16.dp),
        verticalArrangement = Arrangement.spacedBy(10.dp),
    ) {
        Text("网关", style = MaterialTheme.typography.titleLarge)
        Text(
            "跳板手机自己用 http://127.0.0.1:8090；其他手机填跳板手机的 Tailscale 地址，如 http://100.x.x.x:8090。",
            style = MaterialTheme.typography.bodySmall,
        )
        OutlinedTextField(url, { url = it }, label = { Text("网关地址") }, singleLine = true, modifier = Modifier.fillMaxWidth())
        OutlinedTextField(
            token, { token = it },
            label = { Text("口令（网关用 --lan 时必填）") },
            singleLine = true,
            // 口令默认遮住：截屏、旁人看屏幕都不该看到
            visualTransformation = if (showToken) VisualTransformation.None else PasswordVisualTransformation(),
            trailingIcon = { TextButton(onClick = { showToken = !showToken }) { Text(if (showToken) "隐藏" else "显示") } },
            modifier = Modifier.fillMaxWidth(),
        )
        Button(onClick = {
            saved = try {
                ArmClient(url, token)
                ArmClient.save(context, url, token)
                onGatewayChanged()
                "已保存"
            } catch (e: Exception) {
                e.message ?: "地址无效"
            }
        }) { Text("保存并连接") }
        if (saved.isNotBlank()) Text(saved)

        HorizontalDivider()
        Row(verticalAlignment = Alignment.CenterVertically) {
            Text("跳板（本机经 USB 接板卡）", style = MaterialTheme.typography.titleLarge, modifier = Modifier.weight(1f))
            Switch(checked = running, onCheckedChange = { on ->
                if (on) {
                    relayConfig().save(context)
                    if (Build.VERSION.SDK_INT >= 33 && ContextCompat.checkSelfPermission(context, Manifest.permission.POST_NOTIFICATIONS)
                        != PackageManager.PERMISSION_GRANTED) {
                        notifyPermission.launch(Manifest.permission.POST_NOTIFICATIONS)
                    }
                    RelayService.start(context)
                } else {
                    RelayService.stop(context)
                }
            })
        }
        Text(
            "板卡地址留空：等板卡在 USB 共享链路上广播（scripts/board_beacon.py）。开发时可填 127.0.0.1，配合 Mac 上的 adb reverse。" +
                "网关端口留空：跟随广播里板卡实际在跑的网关端口，没收到广播时用 ${RelayConfig.DEFAULT_GATEWAY}；手填后固定用手填的。改设置后关掉再打开跳板生效。",
            style = MaterialTheme.typography.bodySmall,
        )
        OutlinedTextField(boardHost, { boardHost = it }, label = { Text("板卡地址（留空自动）") }, singleLine = true, modifier = Modifier.fillMaxWidth())
        val portInvalid = gatewayRemote.isNotBlank() && RelayConfig.validPort(gatewayRemote.trim().toIntOrNull()) == null
        OutlinedTextField(
            gatewayRemote, { gatewayRemote = it },
            label = { Text("板卡网关端口（留空自动）") },
            singleLine = true,
            isError = portInvalid,
            supportingText = if (portInvalid) ({ Text("不是 1–65535 的端口，保存时按留空（自动）处理") }) else null,
            modifier = Modifier.fillMaxWidth(),
        )
        Row(verticalAlignment = Alignment.CenterVertically) {
            Text("放行热点/同一 Wi-Fi 的设备", modifier = Modifier.weight(1f))
            Switch(checked = allowLan, onCheckedChange = { allowLan = it })
        }
        OutlinedButton(onClick = { relayConfig().save(context); saved = "跳板设置已保存（重启跳板生效）" }) { Text("保存跳板设置") }

        snap?.let { s ->
            val vpn = s.vpn.firstOrNull()
            SelectionContainer {
                Column(verticalArrangement = Arrangement.spacedBy(2.dp)) {
                    Text("虚拟网地址：${s.vpn.joinToString().ifBlank { "没有（先打开 Tailscale）" }}")
                    Text("本机网卡：${s.local.joinToString("；").ifBlank { "—" }}")
                    Text("板卡：${s.board ?: "未找到"}（${s.boardSource}）" + when {
                        s.boardMs >= 0 -> " · 网关建连 ${s.boardMs} ms"
                        s.boardMs == -1L -> " · 网关不可达"
                        else -> ""
                    })
                    Text("网关端口：:${s.gatewayPort}（${s.gatewaySource}）")
                    s.ports.forEach { Text("· $it") }
                    Text("当前连接 ${s.active} 条")
                    s.errors.forEach { Text(it, color = MaterialTheme.colorScheme.error) }
                    if (vpn != null) {
                        Text("其他设备：", style = MaterialTheme.typography.titleSmall)
                        Text("网页 http://$vpn:${RelayConfig().gatewayLocal}/", fontFamily = FontFamily.Monospace)
                        Text("ssh -p ${RelayConfig().sshLocal} root@$vpn", fontFamily = FontFamily.Monospace)
                    }
                }
            }
        }
        Text("跳板日志", style = MaterialTheme.typography.titleSmall)
        log.asReversed().forEach { Text(it, style = MaterialTheme.typography.bodySmall, fontFamily = FontFamily.Monospace) }
    }
}
