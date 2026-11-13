package com.example.myapplication00001

import android.graphics.Bitmap
import android.graphics.BitmapFactory
import android.net.Uri
import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.rememberLauncherForActivityResult
import androidx.activity.compose.setContent
import androidx.activity.enableEdgeToEdge
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.Image
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.Button
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedTextField
import androidx.compose.material3.Scaffold
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.rememberCoroutineScope
import androidx.compose.runtime.setValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.asImageBitmap
import androidx.compose.ui.layout.ContentScale
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.unit.dp
import com.example.myapplication00001.ui.theme.MyApplication00001Theme
import java.net.HttpURLConnection
import java.net.URI
import java.io.OutputStream
import java.io.OutputStreamWriter
import java.text.DateFormat
import java.util.Date
import java.util.Locale
import java.util.UUID
import java.util.zip.ZipEntry
import java.util.zip.ZipOutputStream
import kotlinx.coroutines.CancellationException
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext
import org.json.JSONObject

private const val PREFS_NAME = "simulation_connection"
private const val URL_KEY = "server_url"
private const val WEB_URL_KEY = "web_server_url"
private const val DEFAULT_URL = ""
private const val DEFAULT_WEB_URL = ""

class MainActivity : ComponentActivity() {
    private var inForeground by mutableStateOf(false)

    override fun onStart() {
        super.onStart()
        inForeground = true
    }

    override fun onStop() {
        inForeground = false
        super.onStop()
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        enableEdgeToEdge()
        val preferences = getSharedPreferences(PREFS_NAME, MODE_PRIVATE)
        setContent {
            MyApplication00001Theme {
                var showClassic by remember { mutableStateOf(false) }
                Scaffold(modifier = Modifier.fillMaxSize()) { padding ->
                    if (showClassic) {
                        Column(modifier = Modifier.fillMaxSize().padding(padding)) {
                            Button(onClick = { showClassic = false }) { Text("返回统一网页") }
                            StatusScreen(
                                initialUrl = preferences.getString(URL_KEY, DEFAULT_URL) ?: DEFAULT_URL,
                                saveUrl = { preferences.edit().putString(URL_KEY, it).apply() },
                                inForeground = inForeground,
                                modifier = Modifier.weight(1f),
                            )
                        }
                    } else {
                        WebConsoleScreen(
                            initialUrl = preferences.getString(WEB_URL_KEY, DEFAULT_WEB_URL) ?: DEFAULT_WEB_URL,
                            saveUrl = { preferences.edit().putString(WEB_URL_KEY, it).apply() },
                            openClassic = { showClassic = true },
                            modifier = Modifier.padding(padding),
                        )
                    }
                }
            }
        }
    }
}

private data class ArmStatus(
    val mode: String,
    val pose: String,
    val armed: Boolean,
    val busy: Boolean,
    val revision: Int,
    val error: String,
)

private data class LogSummary(
    val events: Int,
    val records: Int,
    val roundsFinished: Int,
    val sorted: Int,
    val missed: Int,
    val skipped: Int,
    val failed: Int,
    val cleared: Int,
    val medianSeconds: Double?,
)

internal data class BlockRecord(
    val seq: Int,
    val time: String,
    val timestamp: String,
    val round: Int,
    val run: String,
    val category: String,
    val cubeX: String,
    val cubeY: String,
    val bin: String,
    val seconds: String,
    val result: String,
    val reason: String,
    val source: String,
)

private data class LogView(val summary: LogSummary, val allRecords: List<BlockRecord>)

private data class AlertEvent(val seq: Int, val time: String, val title: String, val detail: String)

private data class EventPage(val alerts: List<AlertEvent>, val next: Int, val total: Int)

@Composable
private fun StatusScreen(
    initialUrl: String,
    saveUrl: (String) -> Unit,
    inForeground: Boolean,
    modifier: Modifier = Modifier,
) {
    var serverUrl by remember { mutableStateOf(initialUrl) }
    var status by remember { mutableStateOf<ArmStatus?>(null) }
    var message by remember { mutableStateOf("输入电脑地址后，点击“读取状态”。") }
    var loading by remember { mutableStateOf(false) }
    var connectedUrl by remember { mutableStateOf<String?>(null) }
    var cameraImage by remember { mutableStateOf<Bitmap?>(null) }
    var cameraMessage by remember { mutableStateOf("连接后显示腕部相机画面") }
    var selectedTab by remember { mutableStateOf(0) }
    var logView by remember { mutableStateOf<LogView?>(null) }
    var logMessage by remember { mutableStateOf("连接后读取分拣记录") }
    var alerts by remember { mutableStateOf<List<AlertEvent>>(emptyList()) }
    var alertMessage by remember { mutableStateOf("连接后读取异常事件") }
    var exportRows by remember { mutableStateOf<List<BlockRecord>>(emptyList()) }
    var exportMessage by remember { mutableStateOf("") }
    var simSortState by remember { mutableStateOf<SimSortState?>(null) }
    var simSortMessage by remember { mutableStateOf("仿真任务状态待读取") }
    var commandInFlight by remember { mutableStateOf(false) }
    val owner = remember { UUID.randomUUID().toString() }
    val scope = rememberCoroutineScope()
    val context = LocalContext.current

    fun saveRecords(uri: Uri?, excel: Boolean) {
        if (uri == null) return
        val snapshot = exportRows
        scope.launch {
            try {
                withContext(Dispatchers.IO) {
                    val output = context.contentResolver.openOutputStream(uri)
                        ?: throw IllegalStateException("无法创建文件")
                    output.use { if (excel) writeXlsx(it, snapshot) else writeCsv(it, snapshot) }
                }
                exportMessage = "已导出 ${snapshot.size} 条记录"
            } catch (exception: Exception) {
                exportMessage = "导出失败：${exception.message ?: "请重试"}"
            }
        }
    }

    val csvLauncher = rememberLauncherForActivityResult(ActivityResultContracts.CreateDocument("text/csv")) {
        saveRecords(it, excel = false)
    }
    val excelLauncher = rememberLauncherForActivityResult(
        ActivityResultContracts.CreateDocument("application/vnd.openxmlformats-officedocument.spreadsheetml.sheet"),
    ) {
        saveRecords(it, excel = true)
    }

    LaunchedEffect(connectedUrl, inForeground) {
        val url = connectedUrl ?: return@LaunchedEffect
        if (!inForeground) return@LaunchedEffect
        while (true) {
            try {
                status = fetchStatus(url)
                message = "连接成功 · 状态每秒刷新"
            } catch (exception: CancellationException) {
                throw exception
            } catch (exception: Exception) {
                status = null
                message = "连接中断：${exception.message ?: "请检查网络"}"
            }
            delay(1000)
        }
    }

    LaunchedEffect(connectedUrl, inForeground, status?.mode) {
        val url = connectedUrl ?: return@LaunchedEffect
        if (!inForeground || status?.mode != "simulation") return@LaunchedEffect
        while (true) {
            try {
                simSortState = fetchSimSortState(url)
                simSortMessage = if (simSortState?.running == true) "仿真分拣运行中"
                    else "仿真分拣未运行"
            } catch (exception: CancellationException) {
                throw exception
            } catch (exception: Exception) {
                simSortMessage = "任务状态不可用：${exception.message ?: "请检查服务"}"
            }
            delay(1000)
        }
    }

    LaunchedEffect(connectedUrl, inForeground, selectedTab) {
        val url = connectedUrl ?: return@LaunchedEffect
        if (!inForeground || selectedTab != 0) return@LaunchedEffect
        while (true) {
            try {
                cameraImage = fetchSnapshot(url)
                cameraMessage = "腕部相机 · 每 2 秒更新"
            } catch (exception: CancellationException) {
                throw exception
            } catch (exception: Exception) {
                cameraImage = null
                cameraMessage = "相机暂不可用：${exception.message ?: "请稍后再试"}"
            }
            delay(2000)
        }
    }

    LaunchedEffect(connectedUrl, inForeground, selectedTab) {
        val url = connectedUrl ?: return@LaunchedEffect
        if (!inForeground || selectedTab != 1) return@LaunchedEffect
        while (true) {
            try {
                logView = fetchLogView(url)
                logMessage = "分拣记录 · 每 2 秒更新"
            } catch (exception: CancellationException) {
                throw exception
            } catch (exception: Exception) {
                logMessage = "记录暂不可用：${exception.message ?: "请稍后再试"}"
            }
            delay(2000)
        }
    }

    LaunchedEffect(connectedUrl, inForeground, selectedTab) {
        val url = connectedUrl ?: return@LaunchedEffect
        if (!inForeground || selectedTab != 2) return@LaunchedEffect
        alerts = emptyList()
        var after = 0
        while (true) {
            try {
                var total: Int
                do {
                    val page = fetchEventPage(url, after)
                    alerts = (alerts + page.alerts).takeLast(100)
                    total = page.total
                    if (page.next <= after) break
                    after = page.next
                } while (after < total)
                alertMessage = "异常事件 · 每秒更新 · 最近 ${alerts.size} 条"
            } catch (exception: CancellationException) {
                throw exception
            } catch (exception: Exception) {
                alertMessage = "事件暂不可用：${exception.message ?: "请稍后再试"}"
            }
            delay(1000)
        }
    }

    Column(
        modifier = modifier.fillMaxSize().verticalScroll(rememberScrollState()).padding(20.dp),
        verticalArrangement = Arrangement.spacedBy(16.dp),
    ) {
        Text("机械臂仿真", style = MaterialTheme.typography.headlineMedium)
        OutlinedTextField(
            value = serverUrl,
            onValueChange = { serverUrl = it },
            label = { Text("电脑上的仿真服务地址") },
            supportingText = { Text("例如 http://电脑局域网IP:8770") },
            singleLine = true,
            modifier = Modifier.fillMaxWidth(),
        )
        Button(
            onClick = {
                loading = true
                connectedUrl = null
                status = null
                cameraImage = null
                logView = null
                logMessage = "正在读取分拣记录…"
                alerts = emptyList()
                alertMessage = "正在读取异常事件…"
                simSortState = null
                simSortMessage = "仿真任务状态待读取"
                message = "正在连接…"
                scope.launch {
                    try {
                        val result = fetchStatus(serverUrl)
                        status = result
                        val normalizedUrl = serverUrl.trim().trimEnd('/')
                        connectedUrl = normalizedUrl
                        saveUrl(normalizedUrl)
                        message = "连接成功 · 状态每秒刷新"
                    } catch (exception: Exception) {
                        message = "连接失败：${exception.message ?: "请检查地址和网络"}"
                    } finally {
                        loading = false
                    }
                }
            },
            enabled = !loading,
        ) {
            Text(if (loading) "连接中…" else "连接并自动刷新")
        }
        Text(message)
        Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            Button(onClick = { selectedTab = 0 }, enabled = selectedTab != 0) { Text("实时状态") }
            Button(onClick = { selectedTab = 1 }, enabled = selectedTab != 1) { Text("记录与统计") }
            Button(onClick = { selectedTab = 2 }, enabled = selectedTab != 2) { Text("异常") }
        }
        if (selectedTab == 0) {
            status?.let { current ->
                Text("运行模式：${if (current.mode == "simulation") "仿真" else current.mode}")
                Text("六轴角度：${current.pose}")
                Text("操作权：${if (current.armed) "已使能" else "未使能"}")
                Text("运行状态：${if (current.busy) "动作中" else "空闲"}")
                Text("状态版本：${current.revision}")
                if (current.error.isNotBlank()) Text("服务提示：${current.error}")
                if (current.mode == "simulation") {
                    Text("仿真分拣", style = MaterialTheme.typography.titleMedium)
                    Text(simSortMessage)
                    simSortState?.let { task ->
                        Text("日志事件：${task.logLines} 条")
                        if (!task.running && task.exitCode != null) Text("进程退出码：${task.exitCode}")
                        task.output.takeLast(3).forEach { Text(it.take(150)) }
                    }
                    Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                        Button(
                            onClick = {
                                connectedUrl?.let { url ->
                                    commandInFlight = true
                                    scope.launch {
                                        try {
                                            simSortState = sendSimSortCommand(url, owner, stop = false)
                                            simSortMessage = "已请求启动 1 轮仿真分拣"
                                        } catch (exception: Exception) {
                                            simSortMessage = "启动被拒绝：${exception.message ?: "请检查场景"}"
                                        } finally {
                                            commandInFlight = false
                                        }
                                    }
                                }
                            },
                            enabled = connectedUrl != null && !commandInFlight && simSortState?.running != true && !current.armed,
                        ) { Text("开始 1 轮") }
                        Button(
                            onClick = {
                                connectedUrl?.let { url ->
                                    commandInFlight = true
                                    scope.launch {
                                        try {
                                            simSortState = sendSimSortCommand(url, owner, stop = true)
                                            simSortMessage = "已请求停止仿真分拣"
                                        } catch (exception: Exception) {
                                            simSortMessage = "停止被拒绝：${exception.message ?: "请稍后重试"}"
                                        } finally {
                                            commandInFlight = false
                                        }
                                    }
                                }
                            },
                            enabled = connectedUrl != null && !commandInFlight && simSortState?.running == true,
                        ) { Text("停止任务") }
                    }
                }
            }
            Text(cameraMessage)
            cameraImage?.let { frame ->
                Image(
                    bitmap = frame.asImageBitmap(),
                    contentDescription = "腕部相机画面",
                    contentScale = ContentScale.Fit,
                    modifier = Modifier.fillMaxWidth().height(240.dp),
                )
            }
        } else if (selectedTab == 1) {
            Text(logMessage)
            logView?.let { logs ->
                val summary = logs.summary
                Text("事件 ${summary.events} 条 · 逐块记录 ${summary.records} 条")
                Text("已完成 ${summary.roundsFinished} 轮")
                Text("已分拣 ${summary.sorted} · 夹空 ${summary.missed} · 跳过 ${summary.skipped}")
                Text("失败 ${summary.failed} · 挪开障碍 ${summary.cleared}")
                summary.medianSeconds?.let { Text("成功分拣单块耗时中位数：${"%.1f".format(it)} 秒") }
                Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                    Button(onClick = {
                        exportRows = logs.allRecords
                        csvLauncher.launch("sorting-records.csv")
                    }) { Text("导出 CSV") }
                    Button(onClick = {
                        exportRows = logs.allRecords
                        excelLauncher.launch("sorting-records.xlsx")
                    }) { Text("导出 Excel") }
                }
                if (exportMessage.isNotBlank()) Text(exportMessage)
                val recentRecords = logs.allRecords.takeLast(20).asReversed()
                if (recentRecords.isEmpty()) {
                    Text("暂无逐块记录。可在电脑仿真页面的“分拣日志”面板开始分拣。")
                } else {
                    Text("最近 ${recentRecords.size} 条记录", style = MaterialTheme.typography.titleMedium)
                    recentRecords.forEach { record ->
                        Text("#${record.seq}  ${record.time}  ${record.result}  ${record.category}")
                        Text("来源 ${record.cubeX}, ${record.cubeY} cm → 目标 ${record.bin.ifBlank { "—" }} · 耗时 ${record.seconds.ifBlank { "—" }}")
                        if (record.reason.isNotBlank()) Text("原因：${record.reason}")
                    }
                }
            }
        } else {
            Text(alertMessage)
            if (alerts.isEmpty()) Text("暂无异常或跳过事件。")
            alerts.asReversed().forEach { alert ->
                Text("#${alert.seq}  ${alert.time}  ${alert.title}", style = MaterialTheme.typography.titleSmall)
                if (alert.detail.isNotBlank()) Text(alert.detail)
            }
        }
    }
}

private suspend fun fetchStatus(baseUrl: String): ArmStatus = withContext(Dispatchers.IO) {
    val connection = endpoint(baseUrl, "/api/status").openConnection() as HttpURLConnection
    connection.connectTimeout = 5000
    connection.readTimeout = 5000
    try {
        if (connection.responseCode != HttpURLConnection.HTTP_OK) {
            throw IllegalStateException("服务器返回 HTTP ${connection.responseCode}")
        }
        val json = connection.inputStream.bufferedReader().use { JSONObject(it.readText()) }
        val poseArray = json.getJSONArray("pose")
        val pose = (0 until poseArray.length()).joinToString("°, ", postfix = "°") { index ->
            if (poseArray.isNull(index)) "—" else poseArray.getString(index)
        }
        ArmStatus(
            mode = json.optString("mode", "未知"),
            pose = pose,
            armed = json.optBoolean("armed"),
            busy = json.optBoolean("busy"),
            revision = json.optInt("revision"),
            error = json.optString("error", ""),
        )
    } finally {
        connection.disconnect()
    }
}

private suspend fun fetchSnapshot(baseUrl: String): Bitmap = withContext(Dispatchers.IO) {
    val connection = endpoint(baseUrl, "/snapshot.jpg").openConnection() as HttpURLConnection
    connection.connectTimeout = 5000
    connection.readTimeout = 5000
    try {
        if (connection.responseCode != HttpURLConnection.HTTP_OK) {
            throw IllegalStateException("服务器返回 HTTP ${connection.responseCode}")
        }
        connection.inputStream.use { stream ->
            BitmapFactory.decodeStream(stream) ?: throw IllegalStateException("相机图片无法解码")
        }
    } finally {
        connection.disconnect()
    }
}

internal fun endpoint(baseUrl: String, path: String): java.net.URL {
    val uri = URI(baseUrl.trim().trimEnd('/'))
    require(uri.scheme in listOf("http", "https") && !uri.host.isNullOrBlank()) {
        "请输入完整地址，例如 http://电脑局域网IP:8770"
    }
    require(uri.path.isNullOrEmpty() && uri.query == null && uri.fragment == null) {
        "只需填写服务地址，不要加 /api/status"
    }
    return URI(uri.toString() + path).toURL()
}

private suspend fun fetchLogView(baseUrl: String): LogView = withContext(Dispatchers.IO) {
    val summaryJson = readJson(baseUrl, "/api/log/summary")
    val recordsJson = readJson(baseUrl, "/api/log/records")
    val byResult = summaryJson.getJSONObject("by_result")
    val summary = LogSummary(
        events = summaryJson.getInt("events"),
        records = summaryJson.getInt("records"),
        roundsFinished = summaryJson.getInt("rounds_finished"),
        sorted = byResult.optInt("sorted"),
        missed = byResult.optInt("missed"),
        skipped = byResult.optInt("skipped"),
        failed = byResult.optInt("failed"),
        cleared = byResult.optInt("cleared"),
        medianSeconds = if (summaryJson.isNull("seconds_per_sorted_block_median")) null
            else summaryJson.getDouble("seconds_per_sorted_block_median"),
    )
    val rows = recordsJson.getJSONArray("records")
    val timeFormat = DateFormat.getDateTimeInstance(DateFormat.SHORT, DateFormat.MEDIUM, Locale.CHINA)
    val allRecords = (0 until rows.length()).map { index ->
        val row = rows.getJSONObject(index)
        val cube = row.getJSONArray("cube")
        BlockRecord(
            seq = row.getInt("seq"),
            time = timeFormat.format(Date((row.getDouble("t") * 1000).toLong())),
            timestamp = row.getDouble("t").toString(),
            round = row.optInt("round"),
            run = if (row.isNull("run")) "" else row.getString("run"),
            category = categoryName(row.optString("category")),
            cubeX = cube.get(0).toString(),
            cubeY = cube.get(1).toString(),
            bin = if (row.isNull("bin")) "" else row.getString("bin"),
            seconds = if (row.isNull("seconds")) "" else row.getDouble("seconds").toString(),
            result = resultName(row.getString("result")),
            reason = if (row.isNull("reason")) "" else row.getString("reason"),
            source = if (row.isNull("source")) "" else row.getString("source"),
        )
    }
    LogView(summary, allRecords)
}

private suspend fun fetchEventPage(baseUrl: String, after: Int): EventPage = withContext(Dispatchers.IO) {
    val json = readJson(baseUrl, "/api/log?after=$after&limit=2000")
    val events = json.getJSONArray("events")
    val timeFormat = DateFormat.getDateTimeInstance(DateFormat.SHORT, DateFormat.MEDIUM, Locale.CHINA)
    val alerts = (0 until events.length()).mapNotNull { index ->
        val event = events.getJSONObject(index)
        val name = event.optString("event")
        val title = alertTitle(name)
        if (title == null) null else AlertEvent(
            seq = event.getInt("seq"),
            time = timeFormat.format(Date((event.getDouble("t") * 1000).toLong())),
            title = title,
            detail = event.optString("reason").takeIf { it.isNotBlank() }
                ?: event.optString("note").takeIf { it.isNotBlank() }
                ?: event.optString("bin").takeIf { it.isNotBlank() }
                ?: "",
        )
    }
    EventPage(alerts, json.getInt("next"), json.getInt("total"))
}

private fun alertTitle(event: String): String? = when (event) {
    "pipeline_missed" -> "夹空或搬运滑脱"
    "pipeline_skip" -> "跳过物块"
    "pipeline_approach_failed" -> "接近失败"
    "pipeline_put_failed" -> "投放失败"
    "pipeline_stuck" -> "物块互相阻挡"
    "gripper_not_closed" -> "夹爪未响应"
    "gripper_still_asleep" -> "夹爪唤醒失败"
    "gripper_wake_refused" -> "夹爪唤醒被拒绝"
    "stopped" -> "任务中止"
    "table_z_disagrees" -> "桌面高度不一致"
    "unknown_object" -> "未知物体"
    "bin_full" -> "容器已满"
    "stall" -> "舵机堵转"
    else -> null
}

internal fun readJson(baseUrl: String, path: String): JSONObject {
    val connection = endpoint(baseUrl, path).openConnection() as HttpURLConnection
    connection.connectTimeout = 5000
    connection.readTimeout = 5000
    try {
        if (connection.responseCode != HttpURLConnection.HTTP_OK) {
            throw IllegalStateException("服务器返回 HTTP ${connection.responseCode}")
        }
        return connection.inputStream.bufferedReader().use { JSONObject(it.readText()) }
    } finally {
        connection.disconnect()
    }
}

private fun categoryName(value: String): String = when (value) {
    "recyclable" -> "可回收物"
    "kitchen" -> "厨余垃圾"
    "hazardous" -> "有害垃圾"
    "other" -> "其他垃圾"
    else -> "未确定"
}

private fun resultName(value: String): String = when (value) {
    "sorted" -> "分拣成功"
    "missed" -> "夹空"
    "skipped" -> "跳过"
    "failed" -> "失败"
    "cleared" -> "挪开障碍"
    else -> value
}

private val exportHeaders = listOf(
    "序号", "时间", "时间戳(秒)", "轮次", "运行标识", "垃圾类别", "来源X(cm)",
    "来源Y(cm)", "目标容器", "耗时(秒)", "结果", "原因", "来源",
)

private fun BlockRecord.exportCells(): List<String> = listOf(
    seq.toString(), time, timestamp, round.toString(), run, category,
    cubeX, cubeY, bin, seconds, result, reason, source,
)

internal fun writeCsv(output: OutputStream, records: List<BlockRecord>) {
    val writer = OutputStreamWriter(output, Charsets.UTF_8)
    writer.write("\uFEFF")
    writer.write(exportHeaders.joinToString(",") { csvCell(it) } + "\r\n")
    records.forEach { record ->
        writer.write(record.exportCells().joinToString(",") { csvCell(it) } + "\r\n")
    }
    writer.flush()
}

private fun csvCell(value: String): String {
    val safe = if (value.firstOrNull() in listOf('=', '+', '-', '@') && value.toDoubleOrNull() == null) {
        "'$value"
    } else value
    return "\"${safe.replace("\"", "\"\"")}\""
}

internal fun writeXlsx(output: OutputStream, records: List<BlockRecord>) {
    ZipOutputStream(output).use { zip ->
        fun entry(name: String, contents: String) {
            zip.putNextEntry(ZipEntry(name))
            zip.write(contents.toByteArray(Charsets.UTF_8))
            zip.closeEntry()
        }

        entry(
            "[Content_Types].xml",
            """<?xml version="1.0" encoding="UTF-8"?>
<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">
<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>
<Default Extension="xml" ContentType="application/xml"/>
<Override PartName="/xl/workbook.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.sheet.main+xml"/>
<Override PartName="/xl/worksheets/sheet1.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.worksheet+xml"/>
</Types>""",
        )
        entry(
            "_rels/.rels",
            """<?xml version="1.0" encoding="UTF-8"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
<Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/officeDocument" Target="xl/workbook.xml"/>
</Relationships>""",
        )
        entry(
            "xl/workbook.xml",
            """<?xml version="1.0" encoding="UTF-8"?>
<workbook xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main"
xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships">
<sheets><sheet name="分拣记录" sheetId="1" r:id="rId1"/></sheets></workbook>""",
        )
        entry(
            "xl/_rels/workbook.xml.rels",
            """<?xml version="1.0" encoding="UTF-8"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
<Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/worksheet" Target="worksheets/sheet1.xml"/>
</Relationships>""",
        )

        val sheet = buildString {
            append("""<?xml version="1.0" encoding="UTF-8"?>
<worksheet xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main"><sheetData>""")
            val allRows = sequenceOf(exportHeaders) + records.asSequence().map { it.exportCells() }
            allRows.forEachIndexed { rowIndex, values ->
                append("<row r=\"${rowIndex + 1}\">")
                values.forEachIndexed { columnIndex, value ->
                    val address = "${'A' + columnIndex}${rowIndex + 1}"
                    val isNumber = rowIndex > 0 && columnIndex in setOf(0, 2, 3, 6, 7, 9) &&
                        value.toDoubleOrNull()?.isFinite() == true
                    if (isNumber) {
                        append("<c r=\"$address\"><v>$value</v></c>")
                    } else {
                        append("<c r=\"$address\" t=\"inlineStr\"><is><t xml:space=\"preserve\">")
                        append(xmlText(value))
                        append("</t></is></c>")
                    }
                }
                append("</row>")
            }
            append("</sheetData></worksheet>")
        }
        entry("xl/worksheets/sheet1.xml", sheet)
    }
}

private fun xmlText(value: String): String = buildString {
    value.forEach { character ->
        when (character) {
            '&' -> append("&amp;")
            '<' -> append("&lt;")
            '>' -> append("&gt;")
            '"' -> append("&quot;")
            '\'' -> append("&apos;")
            else -> if (character == '\t' || character == '\n' || character == '\r' || character.code >= 32) {
                append(character)
            }
        }
    }
}
