package com.example.myapplication00001

import android.webkit.WebResourceError
import android.webkit.WebResourceRequest
import android.webkit.WebSettings
import android.webkit.WebView
import android.webkit.WebViewClient
import androidx.activity.compose.BackHandler
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.text.KeyboardOptions
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.Button
import androidx.compose.material3.Card
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedTextField
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableIntStateOf
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.rememberCoroutineScope
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.text.input.KeyboardType
import androidx.compose.ui.unit.dp
import androidx.compose.ui.viewinterop.AndroidView
import kotlinx.coroutines.CancellationException
import kotlinx.coroutines.launch

@Composable
fun WebConsoleScreen(
    activeUrl: String,
    saveUrl: (String) -> Unit,
    openClassic: () -> Unit,
    inForeground: Boolean,
    modifier: Modifier = Modifier,
) {
    var input by remember(activeUrl) { mutableStateOf(activeUrl) }
    var revision by remember { mutableIntStateOf(0) }
    var showConnection by rememberSaveable { mutableStateOf(activeUrl.isBlank()) }
    var showRecords by rememberSaveable { mutableStateOf(false) }
    var validating by remember { mutableStateOf(false) }
    var connectionError by remember { mutableStateOf("") }
    var pageError by remember(activeUrl) { mutableStateOf("") }
    val scope = rememberCoroutineScope()
    val gateway = remember(activeUrl) { runCatching { GatewayClient.fromUrl(activeUrl) }.getOrNull() }

    LaunchedEffect(gateway, inForeground, revision) {
        if (gateway == null || !inForeground) return@LaunchedEffect
        try {
            gateway.check()
            connectionError = ""
        } catch (exception: CancellationException) {
            throw exception
        } catch (exception: Exception) {
            connectionError = exception.message ?: "无法连接，请检查地址和网络"
        }
    }

    BackHandler(enabled = activeUrl.isNotBlank() && (showConnection || showRecords) && !validating) {
        if (showConnection) showConnection = false else showRecords = false
    }

    Column(modifier.fillMaxSize()) {
        Row(
            modifier = Modifier.fillMaxWidth().padding(horizontal = 16.dp, vertical = 4.dp),
            verticalAlignment = Alignment.CenterVertically,
            horizontalArrangement = Arrangement.SpaceBetween,
        ) {
            Text(
                if (showRecords && !showConnection) "分拣记录" else "机械臂作业台",
                style = MaterialTheme.typography.titleLarge,
                modifier = Modifier.weight(1f),
            )
            if (gateway != null && !showConnection) {
                TextButton(onClick = { showRecords = !showRecords }) {
                    Text(if (showRecords) "作业台" else "记录")
                }
                TextButton(onClick = { showConnection = true }) { Text("连接") }
            }
        }
        when {
            showConnection || gateway == null -> Column(
                Modifier.fillMaxWidth().verticalScroll(rememberScrollState()).padding(16.dp),
                verticalArrangement = Arrangement.spacedBy(16.dp),
            ) {
                Card(shape = MaterialTheme.shapes.large) {
                    Column(Modifier.padding(16.dp), verticalArrangement = Arrangement.spacedBy(12.dp)) {
                        Text("连接电脑", style = MaterialTheme.typography.headlineSmall)
                        OutlinedTextField(
                            value = input,
                            onValueChange = {
                                input = it
                                connectionError = ""
                            },
                            enabled = !validating,
                            label = { Text("网页服务地址") },
                            placeholder = { Text("http://电脑IP:8080/?token=口令") },
                            keyboardOptions = KeyboardOptions(keyboardType = KeyboardType.Uri),
                            singleLine = true,
                            modifier = Modifier.fillMaxWidth(),
                        )
                        if (connectionError.isNotBlank()) {
                            Text(connectionError, color = MaterialTheme.colorScheme.error)
                        }
                        Button(
                            enabled = input.isNotBlank() && !validating,
                            onClick = {
                                val candidate = input
                                validating = true
                                connectionError = ""
                                scope.launch {
                                    try {
                                        val client = GatewayClient.fromUrl(candidate)
                                        client.check()
                                        saveUrl(client.pageUrl)
                                        revision++
                                        showConnection = false
                                    } catch (exception: CancellationException) {
                                        throw exception
                                    } catch (exception: Exception) {
                                        connectionError = exception.message ?: "无法连接，请检查地址和网络"
                                    } finally {
                                        validating = false
                                    }
                                }
                            },
                        ) { Text(if (validating) "连接中…" else "连接") }
                    }
                }
                TextButton(onClick = openClassic, enabled = !validating) { Text("独立仿真入口") }
            }
            showRecords -> RecordsScreen(gateway, inForeground, Modifier.weight(1f))
            else -> {
                val error = connectionError.ifBlank { pageError }
                if (error.isNotBlank()) {
                    Text(error, Modifier.padding(horizontal = 16.dp), color = MaterialTheme.colorScheme.error)
                    TextButton(onClick = {
                        pageError = ""
                        revision++
                    }) { Text("重试") }
                }
                AndroidView(
                    factory = { context ->
                        WebView(context).apply {
                            settings.javaScriptEnabled = true
                            settings.domStorageEnabled = true
                            settings.cacheMode = WebSettings.LOAD_NO_CACHE
                            settings.allowFileAccess = false
                            settings.allowContentAccess = false
                            settings.setSupportMultipleWindows(false)
                            webViewClient = object : WebViewClient() {
                                override fun onReceivedError(
                                    view: WebView, request: WebResourceRequest, error: WebResourceError,
                                ) {
                                    if (request.isForMainFrame) pageError = "网页加载失败，请重试或检查连接"
                                }

                            }
                        }
                    },
                    update = { view ->
                        val request = gateway.pageUrl to revision
                        if (view.tag != request) {
                            view.tag = request
                            view.loadUrl(gateway.pageUrl)
                        }
                    },
                    onRelease = { view -> view.destroy() },
                    modifier = Modifier.fillMaxWidth().weight(1f),
                )
            }
        }
    }
}
