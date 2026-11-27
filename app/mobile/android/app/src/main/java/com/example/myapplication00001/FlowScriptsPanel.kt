package com.example.myapplication00001

import androidx.compose.foundation.background
import androidx.compose.foundation.layout.*
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.testTag
import androidx.compose.ui.unit.dp
import kotlinx.coroutines.*
import org.json.JSONArray
import org.json.JSONObject

internal data class ScriptStepState(val session: String, val path: String, val step: String, val state: String, val result: String = "")
internal data class ScriptProgress(val session: String? = null, val lines: Map<String, ScriptStepState> = emptyMap()) {
    fun accept(event: ScriptStepState): ScriptProgress {
        val previous = if (session != event.session || event.step == "flow" && event.state == "started") emptyMap() else lines
        return ScriptProgress(event.session, if (event.path.isBlank()) previous else previous + (event.path to event))
    }
}

internal data class ScriptLine(val path: String, val label: String)
internal fun scriptLines(actions: JSONArray, prefix: String = "steps"): List<ScriptLine> = buildList {
    for (i in 0 until actions.length()) {
        val action = actions.getJSONObject(i)
        val path = "$prefix[$i]"
        val key = listOf("step", "for_each", "retry", "repeat", "record", "alarm", "stop", "next_block").firstOrNull { action.has(it) } ?: "action"
        add(ScriptLine(path, if (key == "step") action.optString(key) else key))
        action.optJSONArray("do")?.let { addAll(scriptLines(it, "$path.do")) }
        for (kind in listOf("retry", "repeat")) action.optJSONObject(kind)?.let { nested ->
            for (branch in listOf("do", "else")) nested.optJSONArray(branch)?.let { addAll(scriptLines(it, "$path.$kind.$branch")) }
        }
        action.optJSONObject("on_result")?.let { branches ->
            branches.keys().forEach { result -> branches.optJSONArray(result)?.let { addAll(scriptLines(it, "$path.on_result.$result")) } }
        }
    }
}

@OptIn(ExperimentalLayoutApi::class)
@Composable
internal fun FlowScriptsPanel(client: ArmClient?, foreground: Boolean, modifier: Modifier = Modifier) {
    var scripts by remember(client) { mutableStateOf<List<JSONObject>>(emptyList()) }
    var selected by remember(client) { mutableStateOf<String?>(null) }
    var lines by remember(client) { mutableStateOf<List<ScriptLine>>(emptyList()) }
    var progress by remember(client) { mutableStateOf(ScriptProgress()) }
    var running by remember(client) { mutableStateOf(false) }
    var activeScript by remember(client) { mutableStateOf<String?>(null) }
    var alarms by remember(client) { mutableStateOf<List<JSONObject>>(emptyList()) }
    var message by remember(client) { mutableStateOf("") }
    val scope = rememberCoroutineScope()

    LaunchedEffect(client, foreground) {
        val c = client ?: return@LaunchedEffect
        if (!foreground) return@LaunchedEffect
        var after = 0
        var poll = 0
        while (true) {
            try {
                val result = withContext(Dispatchers.IO) {
                    val catalog = if (poll++ % 4 == 0) c.get("/api/scripts") else null
                    Triple(catalog, c.get("/api/flow"), c.get("/api/log?after=$after&limit=500"))
                }
                result.first?.optJSONArray("scripts")?.let { list ->
                    scripts = (0 until list.length()).map { list.getJSONObject(it) }
                    if (selected == null || scripts.none { it.optString("name") == selected }) selected = scripts.firstOrNull()?.optString("name")
                }
                val process = result.second.optJSONObject("process")
                running = process?.optBoolean("running") ?: false
                activeScript = process?.takeIf { !it.isNull("script") }?.optString("script")
                result.second.optJSONArray("alarms")?.let { list -> alarms = (0 until list.length()).map { list.getJSONObject(it) } }
                val log = result.third
                // A restarted gateway may have truncated its log; replay the new sequence instead of waiting forever.
                if (log.optInt("total") < after) { after = 0; progress = ScriptProgress() }
                else {
                    log.optJSONArray("events")?.let { events ->
                        for (i in 0 until events.length()) {
                            val e = events.getJSONObject(i)
                            if (e.optString("event") == "step") progress = progress.accept(ScriptStepState(
                                e.optString("session"), e.optString("path"), e.optString("step"), e.optString("state"), e.optString("result")))
                        }
                    }
                    after = log.optInt("next", after)
                }
                if (message.startsWith("读取失败")) message = ""
            } catch (e: CancellationException) { throw e }
            catch (e: Exception) { message = "读取失败：${e.message}" }
            delay(500)
        }
    }
    LaunchedEffect(client, selected) {
        lines = emptyList()
        val c = client ?: return@LaunchedEffect
        val name = selected ?: return@LaunchedEffect
        try {
            val detail = withContext(Dispatchers.IO) { c.get("/api/scripts/$name") }
            val actions = detail.optJSONObject("check")?.optJSONObject("script")?.optJSONArray("steps")
            lines = actions?.let { scriptLines(it) }.orEmpty()
        } catch (e: CancellationException) { throw e }
        catch (e: Exception) { message = "读取失败：${e.message}" }
    }
    fun command(name: String, args: Map<String, Any?>) {
        val c = client ?: return
        scope.launch {
            try {
                withContext(Dispatchers.IO) { c.cmd(name, args) }
                if (name == "run_script") { running = true; activeScript = selected; progress = ScriptProgress() }
                message = if (name == "alarm_ack") "已确认报警" else "已请求运行脚本"
            } catch (e: CancellationException) { throw e }
            catch (e: Exception) { message = "请求失败：${e.message}" }
        }
    }
    Column(modifier.fillMaxWidth(), verticalArrangement = Arrangement.spacedBy(6.dp)) {
        Text("流程脚本", style = MaterialTheme.typography.titleMedium)
        if (scripts.isEmpty()) Text(if (client == null) "请先连接网关" else "暂无脚本或尚未读取")
        FlowRow(horizontalArrangement = Arrangement.spacedBy(6.dp)) {
            scripts.forEach { script ->
                val name = script.optString("name")
                FilterChip(selected == name, onClick = { selected = name }, label = { Text(script.optString("title", name)) },
                    modifier = Modifier.testTag("flow_script_$name"))
            }
        }
        val option = scripts.firstOrNull { it.optString("name") == selected }
        if (option != null && !option.optBoolean("runnable")) Text("脚本不可运行：${option.optJSONArray("unavailable") ?: "请在网页检查脚本错误"}")
        Button(onClick = { command("run_script", mapOf("name" to selected)) },
            enabled = client != null && foreground && option?.optBoolean("runnable") == true && !running,
            modifier = Modifier.testTag("run_script")) { Text("运行所选脚本") }
        Text(if (running) "脚本运行中" else "脚本未运行", Modifier.testTag("script_running"))
        if (message.isNotBlank()) Text(message, Modifier.testTag("script_message"))
        lines.forEach { line ->
            val event = if (activeScript == selected) progress.lines[line.path] else null
            Text("${line.path} · ${line.label}${event?.let { " · ${it.state} ${it.result}" } ?: ""}",
                Modifier.fillMaxWidth().testTag("path_${line.path}")
                    .background(if (event?.state == "started") MaterialTheme.colorScheme.primaryContainer else MaterialTheme.colorScheme.surface))
        }
        alarms.forEach { alarm ->
            Text("报警：${alarm.optString("kind")} ${alarm.optString("summary")}")
            OutlinedButton(onClick = { command("alarm_ack", mapOf("kind" to alarm.optString("kind"))) },
                modifier = Modifier.testTag("script_alarm_ack")) { Text("确认此报警") }
        }
    }
}
