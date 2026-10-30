package com.example.myapplication00001

import android.net.Uri
import android.webkit.WebView
import android.webkit.WebViewClient
import android.webkit.WebSettings
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.material3.Button
import androidx.compose.material3.OutlinedTextField
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableIntStateOf
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.unit.dp
import androidx.compose.ui.viewinterop.AndroidView

/** 手机与电脑共用 src/web/ 的同一份页面，App 只负责保存服务地址。 */
@Composable
fun WebConsoleScreen(
    initialUrl: String,
    saveUrl: (String) -> Unit,
    openClassic: () -> Unit,
    modifier: Modifier = Modifier,
) {
    var input by remember { mutableStateOf(initialUrl) }
    var activeUrl by remember { mutableStateOf(initialUrl) }
    var loadVersion by remember { mutableIntStateOf(0) }
    var error by remember { mutableStateOf("") }

    Column(modifier = modifier.fillMaxSize(), verticalArrangement = Arrangement.spacedBy(4.dp)) {
        Row(modifier = Modifier.fillMaxWidth().padding(horizontal = 8.dp)) {
            OutlinedTextField(
                value = input,
                onValueChange = { input = it },
                label = { Text("网页服务地址（8080，可含 ?token=）") },
                singleLine = true,
                modifier = Modifier.weight(1f),
            )
            Button(onClick = {
                val candidate = input.trim()
                val parsed = Uri.parse(candidate)
                if (parsed.scheme !in listOf("http", "https") || parsed.host.isNullOrBlank()) {
                    error = "请输入完整地址，例如 http://电脑IP:8080/?token=口令"
                } else {
                    error = ""
                    // 只补齐空路径；不能在 ?token= 后面拼接斜杠，否则口令会被改写。
                    activeUrl = if (parsed.path.isNullOrEmpty()) {
                        parsed.buildUpon().path("/").build().toString()
                    } else {
                        candidate
                    }
                    loadVersion++
                    saveUrl(activeUrl)
                }
            }, modifier = Modifier.padding(start = 6.dp, top = 10.dp)) { Text("打开") }
        }
        if (error.isNotEmpty()) Text(error, modifier = Modifier.padding(horizontal = 12.dp))
        Button(onClick = openClassic, modifier = Modifier.padding(horizontal = 8.dp)) {
            Text("旧版状态与导出")
        }
        if (activeUrl.isBlank()) {
            Text("输入电脑上的网页服务地址，再点击“打开”。", modifier = Modifier.padding(horizontal = 12.dp))
        } else AndroidView(
            factory = { context ->
                WebView(context).apply {
                    settings.javaScriptEnabled = true
                    settings.domStorageEnabled = true
                    settings.cacheMode = WebSettings.LOAD_NO_CACHE
                    settings.allowFileAccess = false
                    settings.allowContentAccess = false
                    settings.setSupportMultipleWindows(false)
                    webViewClient = WebViewClient()
                }
            },
            update = { view ->
                val request = "$activeUrl#$loadVersion"
                if (view.tag != request) {
                    view.tag = request
                    view.loadUrl(activeUrl)
                }
            },
            modifier = Modifier.fillMaxWidth().weight(1f),
        )
    }
}
