package com.example.myapplication00001

import android.net.Uri
import java.net.HttpURLConnection
import java.net.URI
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.withContext
import org.json.JSONObject

internal class GatewayClient private constructor(
    private val origin: URI,
    private val token: String?,
    val pageUrl: String,
) {
    suspend fun check() = withContext(Dispatchers.IO) {
        val health = request("/api/health")
        require(health.optBoolean("ok") && health.has("contract_version") && health.has("console")) {
            "请填写 F1 网页服务地址（默认 8080）"
        }
    }

    suspend fun fetchRecords(): LogView = withContext(Dispatchers.IO) {
        val records = request("/api/log/records")
        val summary = request("/api/log/summary")
        // 两次读取之间可能有新物块写入。对齐一次，避免轮询时反复追逐日志尾部。
        val currentRecords = if (records.getJSONArray("records").length() == summary.getInt("records")) {
            records
        } else {
            request("/api/log/records")
        }
        parseLogView(summary, currentRecords)
    }

    suspend fun recordsStatus(): RecordsStatus = withContext(Dispatchers.IO) {
        val status = request("/api/status")
        val timer = status.optJSONObject("timer")
        val pending = if (timer != null && timer.optBoolean("pending") && !timer.isNull("id")) {
            PendingTimer(timer.getString("id"), timer.optDouble("countdown_s", 0.0).coerceAtLeast(0.0))
        } else null
        RecordsStatus(status.optBoolean("replay"), pending)
    }

    suspend fun cancelTimer(id: String) = withContext(Dispatchers.IO) {
        val reply = request("/api/timers", JSONObject().put("cancel", id))
        check(reply.optBoolean("ok")) { "未能取消，任务可能已启动或已取消" }
    }

    private fun request(path: String, body: JSONObject? = null): JSONObject {
        val connection = origin.resolve(path).toURL().openConnection() as HttpURLConnection
        connection.instanceFollowRedirects = false
        connection.connectTimeout = 5000
        connection.readTimeout = 5000
        if (!token.isNullOrEmpty()) connection.setRequestProperty("X-Token", token)
        try {
            if (body != null) {
                connection.requestMethod = "POST"
                connection.setRequestProperty("Content-Type", "application/json")
                connection.doOutput = true
                connection.outputStream.use { it.write(body.toString().toByteArray(Charsets.UTF_8)) }
            }
            when (val status = connection.responseCode) {
                200 -> return connection.inputStream.bufferedReader(Charsets.UTF_8).use {
                    JSONObject(it.readText())
                }
                401 -> error("访问口令无效，请填写本次启动时终端显示的口令")
                404 -> error("找不到接口，请确认填写的是 F1 网页服务地址（默认 8080）")
                else -> {
                    val message = connection.errorStream?.bufferedReader(Charsets.UTF_8)?.use {
                        runCatching { JSONObject(it.readText()).optString("error") }.getOrNull()
                    }
                    error(message?.takeIf { it.isNotBlank() } ?: "服务返回 HTTP $status")
                }
            }
        } finally {
            connection.disconnect()
        }
    }

    companion object {
        fun fromUrl(value: String): GatewayClient {
            val address = value.trim()
            val uri = runCatching { URI(address) }.getOrNull()
            require(uri != null && uri.scheme in listOf("http", "https") &&
                uri.host != null && uri.rawUserInfo == null &&
                (uri.port == -1 || uri.port in 1..65535) &&
                uri.path in listOf("", "/", "/index.html")
            ) {
                "请输入完整的网页服务地址，例如 http://电脑IP:8080/?token=口令"
            }
            val parsed = Uri.parse(address)
            val page = if (uri.path.isEmpty()) parsed.buildUpon().path("/").build() else parsed
            val origin = URI(uri.scheme, null, uri.host, uri.port, "/", null, null)
            return GatewayClient(origin, page.getQueryParameter("token"), page.toString())
        }
    }
}

internal data class PendingTimer(val id: String, val seconds: Double)
internal data class RecordsStatus(val replay: Boolean, val pendingTimer: PendingTimer?)
