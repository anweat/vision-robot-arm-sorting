package com.example.myapplication00001

import java.net.HttpURLConnection
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.withContext
import org.json.JSONObject

internal data class SimSortState(
    val running: Boolean,
    val exitCode: Int?,
    val logLines: Int,
    val output: List<String>,
)

internal suspend fun fetchSimSortState(baseUrl: String): SimSortState = withContext(Dispatchers.IO) {
    parseSimSortState(readJson(baseUrl, "/api/sim/sort"))
}

internal suspend fun sendSimSortCommand(baseUrl: String, owner: String, stop: Boolean): SimSortState =
    withContext(Dispatchers.IO) {
        require(readJson(baseUrl, "/api/status").optString("mode") == "simulation") {
            "当前服务不是仿真，不能发送仿真分拣命令"
        }
        val token = readJson(baseUrl, "/api/bootstrap").getString("token")
        val path = if (stop) "/api/sim/sort/stop" else "/api/sim/sort"
        val request = JSONObject().put("owner", owner)
        if (!stop) request.put("rounds", 1)
        val connection = endpoint(baseUrl, path).openConnection() as HttpURLConnection
        connection.requestMethod = "POST"
        connection.setRequestProperty("Content-Type", "application/json; charset=utf-8")
        connection.setRequestProperty("X-Console-Token", token)
        connection.connectTimeout = 5000
        connection.readTimeout = 15000
        connection.doOutput = true
        try {
            connection.outputStream.use { it.write(request.toString().toByteArray(Charsets.UTF_8)) }
            val statusCode = connection.responseCode
            val response = (if (statusCode in 200..299) connection.inputStream else connection.errorStream)
                ?.bufferedReader()?.use { it.readText() }.orEmpty()
            if (statusCode !in 200..299) {
                val error = runCatching { JSONObject(response).optString("error") }.getOrNull()
                throw IllegalStateException(error?.takeIf { it.isNotBlank() } ?: "服务器返回 HTTP $statusCode")
            }
            parseSimSortState(JSONObject(response))
        } finally {
            connection.disconnect()
        }
    }

private fun parseSimSortState(json: JSONObject): SimSortState {
    val output = json.optJSONArray("output")
    return SimSortState(
        running = json.getBoolean("running"),
        exitCode = if (json.isNull("exit_code")) null else json.getInt("exit_code"),
        logLines = json.optInt("log_lines"),
        output = if (output == null) emptyList() else (0 until output.length()).map { output.getString(it) },
    )
}
