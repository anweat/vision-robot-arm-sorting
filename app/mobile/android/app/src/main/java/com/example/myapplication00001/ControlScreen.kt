package com.example.myapplication00001

import android.graphics.Bitmap
import androidx.compose.foundation.Canvas
import androidx.compose.foundation.Image
import androidx.compose.foundation.background
import androidx.compose.foundation.gestures.awaitEachGesture
import androidx.compose.foundation.gestures.awaitFirstDown
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.aspectRatio
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.FilterChip
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.Slider
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.DisposableEffect
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableFloatStateOf
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.rememberCoroutineScope
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.asImageBitmap
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.layout.ContentScale
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import java.util.concurrent.atomic.AtomicReference
import kotlin.math.abs
import kotlin.math.hypot
import kotlinx.coroutines.CancellationException
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext

/** 摇杆与旋腕的当前输入；UI 线程写，驱动循环读。 */
private class StickInput {
    val left = AtomicReference(Offset.Zero)   // x 右正，y 上正，各 −1..1
    val right = AtomicReference(Offset.Zero)
    val wrist = AtomicReference(0f)

    /** 控制台 `/api/drive` 的 v（文档 32 §1.5；分量含义见 motion_policy.drive_goal）。全 0 返回 null。 */
    fun vector(mode: String): Pair<List<Double>, Double>? {
        val l = left.get()
        val r = right.get()
        val w = wrist.get().toDouble()
        val v = if (mode == "joint") {
            // 左摇杆 J1（右推 = J1 减小，指尖向右）/ J2；右摇杆 J3 / J4
            listOf(-l.x, l.y, r.y, r.x)
        } else {
            // 左摇杆：上下 = 沿径向前后，左右 = 绕底座转；右摇杆：上下 = z，左右 = 俯仰
            listOf(l.y, -l.x, r.y, r.x)
        }.map { dead(it).toDouble() }
        return if (v.all { it == 0.0 } && w == 0.0) null else v to w
    }

    private fun dead(x: Float) = if (abs(x) < 0.08f) 0f else x.coerceIn(-1f, 1f)
}

@Composable
fun ControlScreen(client: ArmClient?, foreground: Boolean, modifier: Modifier = Modifier) {
    val input = remember { StickInput() }
    var mode by remember { mutableStateOf("cartesian") }
    var view by remember { mutableStateOf<ArmView?>(null) }
    var sim by remember { mutableStateOf<Boolean?>(null) }
    var frame by remember { mutableStateOf<Bitmap?>(null) }
    var linkText by remember { mutableStateOf("未连接") }
    var reject by remember { mutableStateOf("") }
    var speedDraft by remember { mutableFloatStateOf(-1f) }
    var pushing by remember { mutableStateOf(false) }
    val scope = rememberCoroutineScope()

    fun send(name: String, args: Map<String, Any?> = emptyMap(), after: () -> Unit = {}) {
        val c = client ?: return
        scope.launch {
            try {
                withContext(Dispatchers.IO) { c.cmd(name, args) }
                reject = ""
                after()
            } catch (e: CancellationException) {
                throw e
            } catch (e: Exception) {
                reject = e.message ?: "指令失败"
            }
        }
    }

    // 状态 2 Hz
    LaunchedEffect(client, foreground) {
        val c = client ?: return@LaunchedEffect
        if (!foreground) return@LaunchedEffect
        sim = runCatching { withContext(Dispatchers.IO) { c.get("/api/health") }.optBoolean("console_sim") }.getOrNull()
        while (true) {
            try {
                val t0 = System.nanoTime()
                val status = withContext(Dispatchers.IO) { c.get("/api/status") }
                view = ArmView.parse(status)
                linkText = "往返 ${(System.nanoTime() - t0) / 1_000_000} ms"
            } catch (e: CancellationException) {
                throw e
            } catch (e: Exception) {
                view = null
                linkText = "连接中断：${e.message}"
            }
            delay(500)
        }
    }

    // 相机：上一帧到了再取下一帧，最多约 5 帧/秒
    LaunchedEffect(client, foreground) {
        val c = client ?: return@LaunchedEffect
        if (!foreground) return@LaunchedEffect
        while (true) {
            frame = try {
                withContext(Dispatchers.IO) { c.frame() }
            } catch (e: CancellationException) {
                throw e
            } catch (_: Exception) {
                null
            }
            delay(if (frame == null) 1500 else 200)
        }
    }

    // 页面可见期间续租（网关只在最近有心跳时替页面保住控制台操作权，文档 32 §1.2）
    LaunchedEffect(client, foreground) {
        val c = client ?: return@LaunchedEffect
        if (!foreground) return@LaunchedEffect
        while (true) {
            runCatching { withContext(Dispatchers.IO) { c.cmd("heartbeat", timeoutMs = 3000) } }
            delay(800)
        }
    }

    // 摇杆驱动 10 Hz；松手后补发一次全 0，控制台立即停在回读位
    LaunchedEffect(client, foreground, mode) {
        val c = client ?: return@LaunchedEffect
        if (!foreground) return@LaunchedEffect
        var wasActive = false
        while (true) {
            val v = input.vector(mode)
            if (v != null || wasActive) {
                val (vec, wrist) = v ?: (listOf(0.0, 0.0, 0.0, 0.0) to 0.0)
                try {
                    withContext(Dispatchers.IO) {
                        c.cmd("drive", mapOf("mode" to mode, "v" to vec, "wrist" to wrist), timeoutMs = 3000)
                    }
                    if (v != null) reject = ""
                } catch (e: CancellationException) {
                    throw e
                } catch (e: Exception) {
                    reject = e.message ?: "摇杆指令失败"
                }
            }
            wasActive = v != null
            delay(100)
        }
    }

    // 离开页面/退后台：只停续租（网关的 release），不打断正在跑的流程
    DisposableEffect(client, foreground) {
        onDispose {
            val c = client
            if (c != null) Thread { runCatching { c.cmd("release", timeoutMs = 2000) } }.start()
            input.left.set(Offset.Zero)
            input.right.set(Offset.Zero)
            input.wrist.set(0f)
        }
    }

    val v = view
    Column(modifier.fillMaxSize().padding(8.dp), verticalArrangement = Arrangement.spacedBy(6.dp)) {
        // 顶栏：实机/仿真、速度、停止
        Row(verticalAlignment = Alignment.CenterVertically, horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            val badge = when (sim) { true -> "仿真"; false -> "实机"; null -> "—" }
            Text(
                badge,
                color = Color.White,
                modifier = Modifier.background(if (sim == false) Color(0xFFB3261E) else Color(0xFF3B6EA8)).padding(6.dp, 2.dp),
            )
            Column(Modifier.weight(1f)) {
                val shown = if (speedDraft >= 0) speedDraft.toInt() else v?.speed ?: 0
                Text("速度 $shown °/s", style = MaterialTheme.typography.labelMedium)
                Slider(
                    value = if (speedDraft >= 0) speedDraft else (v?.speed ?: 20).toFloat(),
                    onValueChange = { speedDraft = it },
                    onValueChangeFinished = {
                        val target = speedDraft.toInt()
                        send("speed", mapOf("speed" to target)) { speedDraft = -1f }
                    },
                    valueRange = (v?.speedMin ?: 3).toFloat()..(v?.speedMax ?: 40).toFloat(),
                    enabled = v != null,
                )
            }
            Button(
                onClick = { send("stop") },
                enabled = client != null,
                colors = ButtonDefaults.buttonColors(containerColor = Color(0xFFB3261E)),
            ) { Text("停止", fontWeight = FontWeight.Bold) }
        }

        // 相机画面 + 拒绝原因
        Box(Modifier.fillMaxWidth().aspectRatio(4f / 3f).background(Color.Black)) {
            frame?.let {
                Image(it.asImageBitmap(), "腕部相机", Modifier.fillMaxSize(), contentScale = ContentScale.Fit)
            } ?: Text("没有画面", color = Color.Gray, modifier = Modifier.align(Alignment.Center))
            // 红条只放「当前」的拒绝：本机刚发的指令被拒，或正推摇杆时控制台的原因。
            // drive.reason 与 status.error 在动作成功后都不清空（error 只在取得操作权时清），不能当成当前故障。
            val reason = reject.ifBlank { if (pushing) v?.driveReason.orEmpty() else "" }
            if (reason.isNotBlank()) {
                Text(
                    reason,
                    color = Color.White,
                    modifier = Modifier.align(Alignment.BottomCenter).fillMaxWidth()
                        .background(Color(0xCCB3261E)).padding(6.dp),
                )
            }
        }
        Text(
            buildString {
                append(linkText)
                v?.let {
                    it.clearanceCm?.let { c -> append(" · 余量 %.1f cm".format(c)) }
                    if (it.busy) append(" · 运动中")
                    if (it.flowRunning) append(" · 流程运行中")
                }
            },
            style = MaterialTheme.typography.bodySmall,
        )
        v?.let {
            Text(
                it.joints.mapIndexed { i, j -> "J${i + 1} ${j?.toString() ?: "—"}" }.joinToString("  "),
                style = MaterialTheme.typography.bodyMedium,
                fontWeight = FontWeight.Medium,
            )
            if (it.error.isNotBlank()) {
                Text("控制台上次故障：${it.error}", style = MaterialTheme.typography.bodySmall, color = Color.Gray)
            }
        }

        // 摇杆区：左右摇杆按宽度自适应，中间一窄列放旋腕与模式
        Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(6.dp), verticalAlignment = Alignment.CenterVertically) {
            Joystick(if (mode == "joint") "J1 / J2" else "前后 / 转向", Modifier.weight(1f)) {
                input.left.set(it)
                pushing = it != Offset.Zero || input.right.get() != Offset.Zero
            }
            Column(Modifier.width(76.dp), horizontalAlignment = Alignment.CenterHorizontally, verticalArrangement = Arrangement.spacedBy(6.dp)) {
                HoldButton("J5 ⟲") { input.wrist.set(if (it) 1f else 0f); pushing = it }
                HoldButton("J5 ⟳") { input.wrist.set(if (it) -1f else 0f); pushing = it }
                FilterChip(selected = mode == "cartesian", onClick = { mode = "cartesian" }, label = { Text("直角") })
                FilterChip(selected = mode == "joint", onClick = { mode = "joint" }, label = { Text("关节") })
            }
            Joystick(if (mode == "joint") "J3 / J4" else "上下 / 俯仰", Modifier.weight(1f)) {
                input.right.set(it)
                pushing = it != Offset.Zero || input.left.get() != Offset.Zero
            }
        }
        Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(8.dp), verticalAlignment = Alignment.CenterVertically) {
            OutlinedButton(onClick = { send("gripper", mapOf("action" to "wide")) }, enabled = client != null, modifier = Modifier.weight(1f)) { Text("全开") }
            OutlinedButton(onClick = { send("gripper", mapOf("action" to "open")) }, enabled = client != null, modifier = Modifier.weight(1f)) { Text("张开") }
            OutlinedButton(onClick = { send("gripper", mapOf("action" to "grip")) }, enabled = client != null, modifier = Modifier.weight(1f)) { Text("合爪") }
        }
        v?.gripResult?.let { Text("夹爪：$it", style = MaterialTheme.typography.bodySmall) }
        if (client == null) Text("先在「连接」页填网关地址。", color = MaterialTheme.colorScheme.error)
    }
}

/** 圆形摇杆：按下即生效，松手回中并报 (0,0)。 */
@Composable
private fun Joystick(label: String, modifier: Modifier = Modifier, onChange: (Offset) -> Unit) {
    var knob by remember { mutableStateOf(Offset.Zero) }
    Column(modifier, horizontalAlignment = Alignment.CenterHorizontally) {
        Box(
            Modifier.fillMaxWidth().aspectRatio(1f).pointerInput(Unit) {
                awaitEachGesture {
                    val down = awaitFirstDown()
                    fun update(p: Offset) {
                        val r = size.width / 2f
                        var d = Offset((p.x - r) / r, (p.y - r) / r)
                        val len = hypot(d.x, d.y)
                        if (len > 1f) d = Offset(d.x / len, d.y / len)
                        knob = d
                        onChange(Offset(d.x, -d.y))
                    }
                    update(down.position)
                    down.consume()
                    while (true) {
                        val change = awaitPointerEvent().changes.firstOrNull { it.id == down.id } ?: break
                        if (!change.pressed) break
                        update(change.position)
                        change.consume()
                    }
                    knob = Offset.Zero
                    onChange(Offset.Zero)
                }
            },
        ) {
            Canvas(Modifier.fillMaxSize()) {
                val r = size.minDimension / 2f
                drawCircle(Color(0x223B6EA8), r)
                drawCircle(Color(0xFF3B6EA8), r, style = Stroke(2f))
                drawCircle(Color(0xFF3B6EA8), r * 0.32f, center = center + Offset(knob.x * r * 0.68f, knob.y * r * 0.68f))
            }
        }
        Text(label, style = MaterialTheme.typography.labelSmall)
    }
}

/** 按住为真、松开为假的按钮（旋腕）。 */
@Composable
private fun HoldButton(label: String, onHold: (Boolean) -> Unit) {
    var held by remember { mutableStateOf(false) }
    Box(
        Modifier.width(64.dp).background(if (held) Color(0xFF3B6EA8) else Color(0x333B6EA8)).padding(vertical = 10.dp)
            .pointerInput(Unit) {
                awaitEachGesture {
                    awaitFirstDown().consume()
                    held = true
                    onHold(true)
                    while (true) {
                        val event = awaitPointerEvent()
                        if (event.changes.none { it.pressed }) break
                    }
                    held = false
                    onHold(false)
                }
            },
        contentAlignment = Alignment.Center,
    ) { Text(label, color = if (held) Color.White else Color.Unspecified) }
}
