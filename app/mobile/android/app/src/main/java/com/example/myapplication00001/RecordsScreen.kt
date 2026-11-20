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
import kotlin.math.ceil
import kotlin.math.roundToInt
import kotlinx.coroutines.CancellationException
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext
import org.json.JSONObject

/*
 * 分拣记录页（移植自 PR #2 3b2f521 的 RecordsScreen）：逐块记录、历史统计、导出 CSV/Excel、定时任务待发取消。
 * 网关地址与口令用「连接」页保存的那一份（ArmClient），不另设地址。用到的网关接口（app/src/web/serve.py）：
 *   GET  /api/log/records、/api/log/summary   逐块记录与统计
 *   GET  /api/status                           replay（样例回放）与 timer（定时任务；未加 --timers 时为 enabled=false）
 *   POST /api/timers {"cancel": id}            取消一个定时任务；未启用时网关回 404
 */

/** 定时任务在记录页的三种显示。 */
internal sealed interface TimerState {
    /** 网关启动时没加 `--timers`，整个功能关着，取消接口也不在。 */
    data object Disabled : TimerState
    data object Idle : TimerState
    data class Pending(val id: String, val seconds: Double) : TimerState
}

internal data class RecordsStatus(val replay: Boolean, val timer: TimerState)

/** `/api/status` 里 `timer` 的判定，拆成纯函数便于单测（org.json 在本机单测里不可用）。 */
internal fun timerState(enabled: Boolean, pending: Boolean, id: String?, countdownS: Double?): TimerState = when {
    !enabled -> TimerState.Disabled
    pending && !id.isNullOrBlank() -> TimerState.Pending(id, (countdownS ?: 0.0).coerceAtLeast(0.0))
    else -> TimerState.Idle
}

/** 分拣成功率 = 成功 ÷（成功 + 夹空 + 跳过 + 失败）；挪开障碍不计入。没有尝试时为 null。 */
internal fun successRatePercent(sorted: Int, missed: Int, skipped: Int, failed: Int): Int? {
    val attempts = sorted + missed + skipped + failed
    return if (attempts == 0) null else (100.0 * sorted / attempts).roundToInt()
}

/** 网关错误换成可操作的提示；其余原样显示网关给的中文原因。 */
internal fun recordsErrorText(code: Int, message: String?): String = when (code) {
    401 -> "口令无效：到「连接」页填写网关本次启动时的口令"
    404 -> "网关没有这个接口：确认「连接」页填的是网关地址（serve.py），而不是控制台或仿真端口"
    else -> message?.takeIf { it.isNotBlank() } ?: "暂时无法读取记录"
}

private fun ArmClient.fetchRecords(): LogView {
    val records = get("/api/log/records", 5000)
    val summary = get("/api/log/summary", 5000)
    // 两次读取之间可能有新物块写入：数目对不上就再读一次记录，避免轮询时反复追日志尾部
    val current = if (records.getJSONArray("records").length() == summary.getInt("records")) records
    else get("/api/log/records", 5000)
    return parseLogView(summary, current)
}

private fun ArmClient.recordsStatus(): RecordsStatus {
    val status = get("/api/status")
    val timer = status.optJSONObject("timer")
    // 启用时网关不带 enabled 字段（只有 lead_s/pending/id…），未启用时是 {"enabled": false}
    val enabled = timer != null && timer.optBoolean("enabled", true)
    val state = timerState(
        enabled = enabled,
        pending = timer?.optBoolean("pending") ?: false,
        id = timer?.takeIf { !it.isNull("id") }?.optString("id"),
        countdownS = timer?.takeIf { !it.isNull("countdown_s") }?.optDouble("countdown_s"),
    )
    return RecordsStatus(status.optBoolean("replay"), state)
}

private fun ArmClient.cancelTimer(id: String) {
    val reply = post("/api/timers", JSONObject().put("cancel", id))
    check(reply.optBoolean("ok")) { "未能取消：任务可能已启动或已被取消" }
}

private fun errorText(exception: Exception): String =
    if (exception is ArmError) recordsErrorText(exception.code, exception.message)
    else exception.message ?: "无法连接网关，请检查网络"

@Composable
fun RecordsScreen(client: ArmClient?, inForeground: Boolean, openConnect: () -> Unit, modifier: Modifier = Modifier) {
    if (client == null) {
        Column(modifier.fillMaxSize().padding(16.dp), verticalArrangement = Arrangement.spacedBy(8.dp)) {
            Text("分拣记录", style = MaterialTheme.typography.titleLarge)
            Text("还没有可用的网关地址。")
            Button(onClick = openConnect) { Text("去「连接」页填写") }
        }
        return
    }
    var logs by remember(client) { mutableStateOf<LogView?>(null) }
    var loadError by remember(client) { mutableStateOf("") }
    var exportMessage by remember(client) { mutableStateOf("") }
    var exportRows by remember(client) { mutableStateOf<List<BlockRecord>?>(null) }
    var refresh by remember { mutableIntStateOf(0) }
    var timer by remember(client) { mutableStateOf<TimerState?>(null) }
    var replay by remember(client) { mutableStateOf(false) }
    var cancelling by remember(client) { mutableStateOf(false) }
    var timerMessage by remember(client) { mutableStateOf("") }
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

    LaunchedEffect(client, inForeground, refresh) {
        if (!inForeground) return@LaunchedEffect
        while (true) {
            try {
                logs = withContext(Dispatchers.IO) { client.fetchRecords() }
                loadError = ""
            } catch (exception: CancellationException) {
                throw exception
            } catch (exception: Exception) {
                loadError = errorText(exception)
            }
            delay(if (loadError.isBlank()) 1000 else 5000)
        }
    }

    LaunchedEffect(client, inForeground) {
        if (!inForeground) return@LaunchedEffect
        while (true) {
            try {
                val status = withContext(Dispatchers.IO) { client.recordsStatus() }
                replay = status.replay
                timer = status.timer
            } catch (exception: CancellationException) {
                throw exception
            } catch (_: Exception) {
                timer = null
            }
            delay(800)
        }
    }

    Column(modifier.fillMaxSize().padding(horizontal = 16.dp)) {
        Row(Modifier.fillMaxWidth().padding(vertical = 8.dp)) {
            Text("分拣记录", style = MaterialTheme.typography.titleLarge, modifier = Modifier.weight(1f))
            if (replay) Text("样例日志回放", color = MaterialTheme.colorScheme.primary)
        }
        when (val t = timer) {
            is TimerState.Pending -> RecordCard {
                Text("${ceil(t.seconds).toInt()} 秒后自动开始分拣", color = MaterialTheme.colorScheme.error)
                TextButton(enabled = !cancelling, onClick = {
                    cancelling = true
                    timerMessage = ""
                    scope.launch {
                        try {
                            withContext(Dispatchers.IO) { client.cancelTimer(t.id) }
                            timer = TimerState.Idle
                            timerMessage = "已取消定时任务"
                        } catch (exception: CancellationException) {
                            throw exception
                        } catch (exception: Exception) {
                            timerMessage = if (exception is ArmError && exception.code == 404) {
                                "定时任务不可用：网关未启用（启动时加 --timers）"
                            } else exception.message ?: "取消失败，请重试"
                        } finally {
                            cancelling = false
                        }
                    }
                }) { Text("取消这次") }
            }
            TimerState.Disabled -> Text(
                "定时任务：网关未启用（启动时加 --timers），无待发任务可取消",
                style = MaterialTheme.typography.bodySmall,
                color = MaterialTheme.colorScheme.onSurfaceVariant,
                modifier = Modifier.padding(bottom = 4.dp),
            )
            else -> {}
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
        val rate = successRatePercent(summary.sorted, summary.missed, summary.skipped, summary.failed)
        item {
            RecordCard {
                Text("历史统计", style = MaterialTheme.typography.titleMedium)
                Text("${rows.size} 条逐块记录", style = MaterialTheme.typography.headlineSmall)
                Text("${summary.events} 条日志事件 · ${summary.roundsFinished} 轮已完成")
                Text("分拣成功率 ${rate?.let { "$it%" } ?: "—"}")
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
