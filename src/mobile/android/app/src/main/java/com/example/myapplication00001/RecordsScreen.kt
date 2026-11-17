package com.example.myapplication00001

import android.net.Uri
import androidx.activity.compose.rememberLauncherForActivityResult
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.material3.Button
import androidx.compose.material3.Card
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableIntStateOf
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.rememberCoroutineScope
import androidx.compose.runtime.setValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.unit.dp
import kotlinx.coroutines.CancellationException
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext
import kotlin.math.ceil
import kotlin.math.roundToInt

@Composable
internal fun RecordsScreen(gateway: GatewayClient, inForeground: Boolean, modifier: Modifier = Modifier) {
    var logs by remember(gateway) { mutableStateOf<LogView?>(null) }
    var loadError by remember(gateway) { mutableStateOf("") }
    var exportMessage by remember(gateway) { mutableStateOf("") }
    var exportRows by remember(gateway) { mutableStateOf<List<BlockRecord>?>(null) }
    var refresh by remember { mutableIntStateOf(0) }
    var pendingTimer by remember(gateway) { mutableStateOf<PendingTimer?>(null) }
    var replay by remember(gateway) { mutableStateOf(false) }
    var cancelling by remember(gateway) { mutableStateOf(false) }
    var timerMessage by remember(gateway) { mutableStateOf("") }
    val context = LocalContext.current
    val scope = rememberCoroutineScope()

    fun save(uri: Uri?, excel: Boolean) {
        val snapshot = exportRows ?: return
        if (uri == null) {
            exportRows = null
            return
        }
        scope.launch {
            try {
                withContext(Dispatchers.IO) {
                    val output = context.contentResolver.openOutputStream(uri) ?: error("无法创建文件")
                    output.use { if (excel) writeXlsx(it, snapshot) else writeCsv(it, snapshot) }
                }
                exportMessage = "已导出 ${snapshot.size} 条记录"
            } catch (exception: CancellationException) {
                throw exception
            } catch (exception: Exception) {
                exportMessage = "导出失败：${exception.message ?: "请重试"}"
            } finally {
                exportRows = null
            }
        }
    }

    val csv = rememberLauncherForActivityResult(ActivityResultContracts.CreateDocument("text/csv")) {
        save(it, excel = false)
    }
    val xlsx = rememberLauncherForActivityResult(
        ActivityResultContracts.CreateDocument("application/vnd.openxmlformats-officedocument.spreadsheetml.sheet"),
    ) { save(it, excel = true) }

    LaunchedEffect(gateway, inForeground, refresh) {
        if (!inForeground) return@LaunchedEffect
        while (true) {
            try {
                logs = gateway.fetchRecords()
                loadError = ""
            } catch (exception: CancellationException) {
                throw exception
            } catch (exception: Exception) {
                loadError = exception.message ?: "暂时无法读取记录"
            }
            delay(if (loadError.isBlank()) 1000 else 5000)
        }
    }

    LaunchedEffect(gateway, inForeground) {
        pendingTimer = null
        if (!inForeground) return@LaunchedEffect
        while (true) {
            try {
                val status = gateway.recordsStatus()
                replay = status.replay
                pendingTimer = status.pendingTimer
            } catch (exception: CancellationException) {
                throw exception
            } catch (_: Exception) {
                pendingTimer = null
            }
            delay(800)
        }
    }

    Column(modifier.fillMaxSize().padding(horizontal = 16.dp)) {
        if (replay) {
            Text("样例日志回放", Modifier.padding(bottom = 8.dp), color = MaterialTheme.colorScheme.primary)
        }
        pendingTimer?.let { timer ->
            RecordCard {
                Text("${ceil(timer.seconds).toInt()} 秒后自动开始分拣", color = MaterialTheme.colorScheme.error)
                TextButton(enabled = !cancelling, onClick = {
                    cancelling = true
                    timerMessage = ""
                    scope.launch {
                        try {
                            gateway.cancelTimer(timer.id)
                            pendingTimer = null
                            timerMessage = "已取消定时任务"
                        } catch (exception: CancellationException) {
                            throw exception
                        } catch (exception: Exception) {
                            timerMessage = exception.message ?: "取消失败，请重试"
                        } finally {
                            cancelling = false
                        }
                    }
                }) { Text("取消这次") }
            }
        }
        if (timerMessage.isNotBlank()) Text(timerMessage, Modifier.padding(vertical = 4.dp))
        RecordList(logs, loadError, exportMessage, exportRows != null, { refresh++ }, onExport = { rows, excel ->
            exportRows = rows
            if (excel) xlsx.launch("sorting-records.xlsx") else csv.launch("sorting-records.csv")
        })
    }
}

@Composable
private fun RecordList(
    logs: LogView?,
    loadError: String,
    exportMessage: String,
    exporting: Boolean,
    onRetry: () -> Unit,
    onExport: (List<BlockRecord>, Boolean) -> Unit,
) {
    LazyColumn(
        modifier = Modifier.fillMaxSize(),
        verticalArrangement = Arrangement.spacedBy(12.dp),
    ) {
        if (loadError.isNotBlank()) {
            item {
                Text(
                    if (logs == null) loadError else "更新失败，显示上次读取的记录 · $loadError",
                    color = MaterialTheme.colorScheme.error,
                )
                TextButton(onClick = onRetry) { Text("重试") }
            }
        }
        val snapshot = logs
        if (snapshot == null) {
            if (loadError.isBlank()) item { Text("正在读取记录…", Modifier.padding(vertical = 16.dp)) }
            return@LazyColumn
        }
        val summary = snapshot.summary
        val rows = snapshot.allRecords
        val attempts = summary.sorted + summary.missed + summary.skipped + summary.failed
        val successRate = if (attempts == 0) "—" else "${(100.0 * summary.sorted / attempts).roundToInt()}%"
        item {
            RecordCard {
                Text("历史统计", style = MaterialTheme.typography.titleMedium)
                Text("${rows.size} 条逐块记录", style = MaterialTheme.typography.headlineSmall)
                Text("${summary.events} 条日志事件 · ${summary.roundsFinished} 轮已完成")
                Text("分拣成功率 $successRate")
                Text("分拣成功 ${summary.sorted} · 夹空 ${summary.missed} · 跳过 ${summary.skipped}")
                Text("失败 ${summary.failed} · 挪开障碍 ${summary.cleared}")
                summary.medianSeconds?.let { Text("分拣成功耗时中位数 ${"%.1f".format(it)} 秒") }
                Text("各类别已分拣")
                summary.sortedByCategory.forEach { (category, count) ->
                    Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceBetween) {
                        Text(category)
                        Text("$count")
                    }
                }
            }
        }
        item {
            Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                Button(enabled = rows.isNotEmpty() && !exporting, onClick = { onExport(rows, false) }) {
                    Text("导出 CSV")
                }
                OutlinedButton(enabled = rows.isNotEmpty() && !exporting, onClick = { onExport(rows, true) }) {
                    Text("导出 Excel")
                }
            }
            if (exportMessage.isNotBlank()) Text(exportMessage)
        }
        item {
            Text("逐块记录", style = MaterialTheme.typography.titleMedium)
            if (rows.isEmpty()) Text("暂无分拣记录", Modifier.padding(top = 8.dp))
        }
        items(rows.asReversed(), key = { it.seq }) { row ->
            RecordCard {
                Text(row.category, style = MaterialTheme.typography.titleMedium)
                Text("${row.result} · ${row.time}", color = MaterialTheme.colorScheme.primary)
                if (row.round > 0) Text("日志轮次 ${row.round}")
                Text("位置 ${row.cubeX}, ${row.cubeY} cm · 容器 ${row.bin.ifBlank { "—" }}")
                Text(if (row.seconds.isBlank()) "耗时 —" else "耗时 ${row.seconds} 秒")
                if (row.reason.isNotBlank()) Text(row.reason)
            }
        }
    }
}

@Composable
private fun RecordCard(content: @Composable () -> Unit) {
    Card(Modifier.fillMaxWidth(), shape = MaterialTheme.shapes.large) {
        Column(Modifier.padding(16.dp), verticalArrangement = Arrangement.spacedBy(6.dp)) {
            content()
        }
    }
}
