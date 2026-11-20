package com.example.myapplication00001

import android.content.Context
import android.graphics.Bitmap
import android.graphics.BitmapFactory
import java.net.HttpURLConnection
import java.net.URI
import java.net.URL
import org.json.JSONArray
import org.json.JSONObject

/** 网关拒绝时的中文原因（409/501 等），界面原样显示。 */
class ArmError(message: String, val code: Int) : Exception(message)

/**
 * 板载网关（src/web/serve.py）的客户端。所有指令走 `POST /api/cmd`，来源记为 mobile（日志契约 `source`）。
 * 不做任何限位或安全判断：那些在控制台里，网关拒绝什么就显示什么（文档 32 §1）。
 */
class ArmClient(baseUrl: String, private val token: String? = null) {
    val base: String = baseUrl.trim().trimEnd('/')

    init {
        val uri = URI(base)
        require(uri.scheme in listOf("http", "https") && !uri.host.isNullOrBlank()) { "地址应为 http://主机:端口" }
    }

    private fun open(path: String, timeoutMs: Int): HttpURLConnection {
        val c = URL(base + path).openConnection() as HttpURLConnection
        c.connectTimeout = timeoutMs
        c.readTimeout = timeoutMs
        c.useCaches = false
        token?.takeIf { it.isNotBlank() }?.let { c.setRequestProperty("X-Token", it) }
        return c
    }

    private fun read(c: HttpURLConnection): JSONObject {
        val code = c.responseCode
        val text = (if (code in 200..299) c.inputStream else c.errorStream)?.bufferedReader()?.use { it.readText() }.orEmpty()
        val json = runCatching { JSONObject(text) }.getOrNull()
        if (code !in 200..299) {
            val reason = json?.optString("error")?.takeIf { it.isNotBlank() } ?: "网关返回 HTTP $code"
            throw ArmError(reason, code)
        }
        return json ?: JSONObject()
    }

    fun get(path: String, timeoutMs: Int = 4000): JSONObject {
        val c = open(path, timeoutMs)
        try {
            return read(c)
        } finally {
            c.disconnect()
        }
    }

    fun cmd(name: String, args: Map<String, Any?> = emptyMap(), timeoutMs: Int = 8000): JSONObject {
        val body = JSONObject().put("cmd", name).put("source", "mobile")
        args.forEach { (k, v) -> body.put(k, if (v is List<*>) JSONArray(v) else v ?: JSONObject.NULL) }
        return post("/api/cmd", body, timeoutMs)
    }

    /** 网关的其他 POST 接口（如 `/api/timers`）；同样要求 JSON 请求体。 */
    fun post(path: String, body: JSONObject, timeoutMs: Int = 8000): JSONObject {
        val c = open(path, timeoutMs)
        c.requestMethod = "POST"
        c.doOutput = true
        c.setRequestProperty("Content-Type", "application/json; charset=utf-8")
        try {
            c.outputStream.use { it.write(body.toString().toByteArray(Charsets.UTF_8)) }
            return read(c)
        } finally {
            c.disconnect()
        }
    }

    fun frame(timeoutMs: Int = 4000): Bitmap {
        val c = open("/api/frame", timeoutMs)
        try {
            if (c.responseCode != 200) throw ArmError("画面不可用（HTTP ${c.responseCode}）", c.responseCode)
            return c.inputStream.use { BitmapFactory.decodeStream(it) } ?: throw ArmError("画面无法解码", 0)
        } finally {
            c.disconnect()
        }
    }

    companion object {
        private const val PREFS = "gateway"

        /** 跳板手机自己连板卡：经本机转发端口；其他手机填跳板的 Tailscale 地址。 */
        const val DEFAULT_URL = "http://127.0.0.1:8090"

        fun savedUrl(context: Context): String =
            context.getSharedPreferences(PREFS, Context.MODE_PRIVATE).getString("url", DEFAULT_URL) ?: DEFAULT_URL

        fun savedToken(context: Context): String =
            context.getSharedPreferences(PREFS, Context.MODE_PRIVATE).getString("token", "") ?: ""

        fun save(context: Context, url: String, token: String) {
            context.getSharedPreferences(PREFS, Context.MODE_PRIVATE).edit()
                .putString("url", url.trim().trimEnd('/')).putString("token", token.trim()).apply()
        }
    }
}

/** `GET /api/status` 里控制页要用的部分。 */
data class ArmView(
    val attached: Boolean,
    val joints: List<Int?>,
    val speed: Int,
    val speedMin: Int,
    val speedMax: Int,
    val driving: Boolean,
    val driveReason: String,
    val busy: Boolean,
    val error: String,
    val clearanceCm: Double?,
    val tipCm: List<Double>,
    val gripResult: String?,
    val flowRunning: Boolean,
) {
    companion object {
        fun parse(status: JSONObject): ArmView? {
            val arm = status.optJSONObject("arm") ?: return null
            val speed = arm.optJSONObject("speed")
            val drive = arm.optJSONObject("drive")
            val joints = arm.optJSONArray("joints_deg")
            val tip = arm.optJSONArray("tip_cm")
            return ArmView(
                attached = arm.optBoolean("attached"),
                joints = (0 until (joints?.length() ?: 0)).map { if (joints!!.isNull(it)) null else joints.getInt(it) },
                speed = speed?.optInt("value", 20) ?: 20,
                speedMin = speed?.optInt("min", 3) ?: 3,
                speedMax = speed?.optInt("max", 40) ?: 40,
                driving = drive?.optBoolean("active") ?: false,
                driveReason = drive?.optString("reason").orEmpty(),
                busy = arm.optBoolean("busy"),
                error = arm.optString("error_text").ifBlank { if (arm.isNull("error")) "" else arm.optString("error") },
                clearanceCm = if (arm.isNull("clearance_cm")) null else arm.optDouble("clearance_cm"),
                tipCm = (0 until (tip?.length() ?: 0)).map { tip!!.getDouble(it) },
                gripResult = if (arm.isNull("grip_result")) null else arm.optString("grip_result"),
                flowRunning = status.optJSONObject("flow")?.optBoolean("running") ?: false,
            )
        }
    }
}
