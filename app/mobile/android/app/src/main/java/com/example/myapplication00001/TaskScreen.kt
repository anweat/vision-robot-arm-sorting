package com.example.myapplication00001

import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.ExperimentalLayoutApi
import androidx.compose.foundation.layout.FlowRow
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.FilterChip
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.rememberCoroutineScope
import androidx.compose.runtime.setValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import kotlinx.coroutines.CancellationException
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext
import org.json.JSONObject

private val STEP_NAMES = linkedMapOf(
    "scan" to "扫描识别", "plan" to "解算验证", "grasp" to "抓取", "check" to "抓后检查",
    "retry" to "空爪重试", "place" to "放置", "verify" to "放置核对", "back_off" to "退回", "home" to "回观察位",
)
/** 要选块的步（文档 32 §2.2）。 */
private val NEEDS_BLOCK = setOf("plan", "grasp", "check", "retry", "place", "back_off")

private data class FlowView(
    val running: Boolean,
    val output: List<String>,
    val alarms: List<JSONObject>,
    val session: String?,
    val blocks: List<JSONObject>,
)

@OptIn(ExperimentalLayoutApi::class)
@Composable
fun TaskScreen(client: ArmClient?, foreground: Boolean, modifier: Modifier = Modifier) {
    var flow by remember { mutableStateOf<FlowView?>(null) }
    var steps by remember { mutableStateOf<List<String>>(emptyList()) }
    var block by remember { mutableStateOf<String?>(null) }
    var message by remember { mutableStateOf("") }
    val scope = rememberCoroutineScope()

    fun send(name: String, args: Map<String, Any?> = emptyMap(), done: String) {
        val c = client ?: return
        scope.launch {
            message = try {
                withContext(Dispatchers.IO) { c.cmd(name, args) }
                done
            } catch (e: CancellationException) {
                throw e
            } catch (e: Exception) {
                "被拒绝：${e.message}"
            }
        }
    }

    LaunchedEffect(client, foreground) {
        val c = client ?: return@LaunchedEffect
        if (!foreground) return@LaunchedEffect
        var after = -1
        while (true) {
            try {
                val j = withContext(Dispatchers.IO) { c.get("/api/flow") }
                val p = j.optJSONObject("process")
                val out = p?.optJSONArray("output")
                val alarms = j.optJSONArray("alarms")
                val blocks = j.optJSONArray("blocks")
                flow = FlowView(
                    running = p?.optBoolean("running") ?: false,
                    output = (0 until (out?.length() ?: 0)).map { out!!.optString(it) },
                    alarms = (0 until (alarms?.length() ?: 0)).map { alarms!!.getJSONObject(it) },
                    session = if (j.isNull("session")) null else j.optString("session"),
                    blocks = (0 until (blocks?.length() ?: 0)).map { blocks!!.getJSONObject(it) },
                )
                // 最近的 step 事件（日志契约 32 §2.4）：首次从末尾 60 条开始
                if (after < 0) after = maxOf(0, withContext(Dispatchers.IO) { c.get("/api/log?after=0&limit=1") }.optInt("total") - 60)
                val log = withContext(Dispatchers.IO) { c.get("/api/log?after=$after&limit=500") }
                val events = log.optJSONArray("events")
                val fresh = (0 until (events?.length() ?: 0)).map { events!!.getJSONObject(it) }
                    .filter { it.optString("event") in setOf("step", "alarm", "alarm_ack", "pipeline_sorted", "pipeline_missed", "pipeline_skip") }
                    .map(::eventLine)
                if (fresh.isNotEmpty()) steps = (steps + fresh).takeLast(30)
                after = log.optInt("next", after)
            } catch (e: CancellationException) {
                throw e
            } catch (e: Exception) {
                message = "读取失败：${e.message}"
            }
            delay(1000)
        }
    }

    val f = flow
    Column(modifier.fillMaxSize().verticalScroll(rememberScrollState()).padding(12.dp), verticalArrangement = Arrangement.spacedBy(10.dp)) {
        // 报警：未确认前完整流程不会被语音/手势/定时再次启动
        f?.alarms?.takeIf { it.isNotEmpty() }?.let { alarms ->
            Column(Modifier.fillMaxWidth().background(Color(0xFFFCE4E4)).padding(8.dp), verticalArrangement = Arrangement.spacedBy(4.dp)) {
                Text("⚠ 报警 ${alarms.size} 条", color = Color(0xFFB3261E), fontWeight = FontWeight.Bold)
                alarms.forEach { a ->
                    Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                        Text("${a.optString("kind")}${a.optString("block").takeIf { it.isNotBlank() && it != "null" }?.let { "（$it）" } ?: ""}：${a.optString("summary")}",
                            modifier = Modifier.weight(1f))
                        OutlinedButton(onClick = { send("alarm_ack", mapOf("kind" to a.optString("kind")), "已确认") }) { Text("知道了") }
                    }
                }
                if (alarms.size > 1) OutlinedButton(onClick = { send("alarm_ack", mapOf("kind" to "*"), "已全部确认") }) { Text("全部知道了") }
            }
        }

        Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            Button(onClick = { send("start", done = "已请求开始分拣") }, enabled = client != null && f?.running != true) { Text("开始分拣") }
            Button(
                onClick = { send("stop", done = "已停止") },
                enabled = client != null,
                colors = ButtonDefaults.buttonColors(containerColor = Color(0xFFB3261E)),
            ) { Text("停止") }
        }
        Text(
            if (f == null) "未连接" else if (f.running) "流程运行中" else "流程未运行" + (f.session?.let { " · 最近会话 $it" } ?: ""),
            style = MaterialTheme.typography.titleSmall,
        )
        if (message.isNotBlank()) Text(message)

        Text("单步执行", style = MaterialTheme.typography.titleMedium)
        val blocks = f?.blocks.orEmpty().filter { !it.optBoolean("in_bin") }
        if (blocks.isEmpty()) {
            Text("先执行「扫描识别」，才有可选的块。", style = MaterialTheme.typography.bodySmall)
        } else {
            FlowRow(horizontalArrangement = Arrangement.spacedBy(6.dp)) {
                blocks.forEach { b ->
                    val name = b.optString("name")
                    FilterChip(selected = block == name, onClick = { block = name },
                        label = { Text("$name ${b.optString("category").takeIf { it != "null" }.orEmpty()}") })
                }
            }
        }
        FlowRow(horizontalArrangement = Arrangement.spacedBy(6.dp), verticalArrangement = Arrangement.spacedBy(4.dp)) {
            STEP_NAMES.forEach { (step, label) ->
                val needs = step in NEEDS_BLOCK
                OutlinedButton(
                    onClick = { send("flow_step", mapOf("step" to step, "block" to if (needs) block else null), "已发出：$label") },
                    enabled = client != null && f?.running != true && (!needs || block != null),
                ) { Text(label) }
            }
        }

        Text("最近步骤与结果", style = MaterialTheme.typography.titleMedium)
        if (steps.isEmpty()) Text("还没有步骤事件。", style = MaterialTheme.typography.bodySmall)
        steps.asReversed().forEach { Text(it, style = MaterialTheme.typography.bodySmall) }
        f?.output?.takeIf { it.isNotEmpty() }?.let { out ->
            Text("流程输出（末几行）", style = MaterialTheme.typography.titleMedium)
            out.forEach { Text(it, style = MaterialTheme.typography.bodySmall) }
        }
    }
}

private fun eventLine(e: JSONObject): String {
    val t = java.text.SimpleDateFormat("HH:mm:ss", java.util.Locale.CHINA).format(java.util.Date((e.optDouble("t") * 1000).toLong()))
    fun s(k: String) = e.optString(k).takeIf { it.isNotBlank() && it != "null" }
    return when (e.optString("event")) {
        "step" -> "$t ${STEP_NAMES[s("step")] ?: s("step")} ${s("state").orEmpty()}" +
            (s("block")?.let { " [$it]" } ?: "") + (s("result")?.let { " → $it" } ?: "") + (s("summary")?.let { "：$it" } ?: "")
        "alarm" -> "$t ⚠ ${s("kind")}：${s("summary").orEmpty()}"
        "alarm_ack" -> "$t 报警已确认 ${s("kind")}（${s("by").orEmpty()}）"
        "pipeline_sorted" -> "$t ✓ 分拣成功 ${s("category").orEmpty()} → ${s("bin").orEmpty()}"
        "pipeline_missed" -> "$t ✗ 夹空 ${s("reason").orEmpty()}"
        "pipeline_skip" -> "$t – 跳过 ${s("reason").orEmpty()}"
        else -> "$t ${e.optString("event")}"
    }
}
