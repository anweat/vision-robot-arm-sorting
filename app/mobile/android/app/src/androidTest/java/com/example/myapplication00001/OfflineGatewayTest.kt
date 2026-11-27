package com.example.myapplication00001

import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.MaterialTheme
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.asAndroidBitmap
import androidx.compose.ui.test.*
import androidx.compose.ui.test.junit4.createComposeRule
import androidx.test.platform.app.InstrumentationRegistry
import org.junit.Assert.*
import org.junit.Rule
import org.junit.Test
import org.json.JSONArray
import java.io.ByteArrayOutputStream
import java.io.ByteArrayInputStream
import java.util.zip.ZipInputStream
import javax.xml.parsers.DocumentBuilderFactory

/** Only the explicitly passed emulator loopback gateway is accepted; no board URL defaults. */
class OfflineGatewayTest {
    @get:Rule val compose = createComposeRule()
    private fun client(): ArmClient {
        val url = InstrumentationRegistry.getArguments().getString("gatewayUrl") ?: error("gatewayUrl is required")
        require(url.startsWith("http://127.0.0.1:"))
        return ArmClient(url)
    }
    private fun screenshot(name: String) {
        val image = compose.onRoot().captureToImage().asAndroidBitmap()
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        context.openFileOutput(name, 0).use { image.compress(android.graphics.Bitmap.CompressFormat.PNG, 100, it) }
    }

    @Test fun scriptListRunPathHighlightAndAlarmAcknowledgement() {
        val client = client()
        compose.setContent { MaterialTheme { Column(Modifier.verticalScroll(rememberScrollState())) { FlowScriptsPanel(client, true) } } }
        compose.waitUntil(15000) { compose.onAllNodesWithTag("flow_script_mobile-demo").fetchSemanticsNodes().isNotEmpty() }
        compose.onNodeWithTag("flow_script_mobile-demo").performClick()
        compose.waitUntil(10000) { compose.onAllNodesWithTag("path_steps[1]").fetchSemanticsNodes().isNotEmpty() }
        compose.onNodeWithTag("run_script").performScrollTo().assertIsEnabled().performClick()
        compose.waitUntil(15000) {
            compose.onAllNodesWithText("steps[1] · wait · started", substring=true).fetchSemanticsNodes().isNotEmpty()
        }
        compose.onNodeWithTag("path_steps[0]").assertTextContains("done", substring=true)
        compose.waitUntil(15000) { compose.onAllNodesWithTag("script_alarm_ack").fetchSemanticsNodes().isNotEmpty() }
        compose.onNodeWithTag("path_steps[1]").assertTextContains("done", substring=true)
        screenshot("offline-script.png")
        compose.onNodeWithTag("script_alarm_ack").performScrollTo().performClick()
        compose.waitUntil(10000) { compose.onAllNodesWithTag("script_alarm_ack").fetchSemanticsNodes().isEmpty() }
        val events = client.get("/api/log?after=1000&limit=500").getJSONArray("events")
        assertTrue((0 until events.length()).any { events.getJSONObject(it).optString("event")=="interaction" && events.getJSONObject(it).optString("source")=="mobile" })
        compose.onNodeWithTag("flow_script_voice-unavailable").performScrollTo().performClick()
        compose.onNodeWithTag("run_script").assertIsNotEnabled()
    }

    @Test fun thousandRecordSnapshotAndRealCsvXlsxWritersAgree() {
        val client = client()
        val records = client.get("/api/log/records")
        val summary = client.get("/api/log/summary")
        val view = parseLogView(summary, records)
        assertEquals(1000, view.allRecords.size)
        assertEquals(1000, view.summary.records)
        val csv = ByteArrayOutputStream().also { writeCsv(it, view.allRecords) }.toByteArray()
        assertTrue(csv.take(3)==listOf(0xEF.toByte(),0xBB.toByte(),0xBF.toByte()))
        assertTrue(csv.toString(Charsets.UTF_8).contains("中文"))
        val xlsx = ByteArrayOutputStream().also { writeXlsx(it, view.allRecords) }
        var sheet: ByteArray? = null
        ZipInputStream(ByteArrayInputStream(xlsx.toByteArray())).use { zip ->
            while (true) { val entry = zip.nextEntry ?: break; if (entry.name=="xl/worksheets/sheet1.xml") sheet=zip.readBytes(); zip.closeEntry() }
        }
        val xml = DocumentBuilderFactory.newInstance().apply { isNamespaceAware=true }.newDocumentBuilder().parse(ByteArrayInputStream(sheet!!))
        assertEquals(1001, xml.getElementsByTagNameNS("http://schemas.openxmlformats.org/spreadsheetml/2006/main","row").length)
        // Retain actual bytes so the desktop runner can compare the complete exported row set.
        val context = InstrumentationRegistry.getInstrumentation().targetContext
        context.openFileOutput("offline-records.csv", 0).use { it.write(csv) }
        context.openFileOutput("offline-records.xlsx", 0).use { it.write(xlsx.toByteArray()) }
        compose.setContent { MaterialTheme { RecordsScreen(client, true, {}) } }
        compose.waitUntil(15000) { compose.onAllNodesWithText("1000 条逐块记录").fetchSemanticsNodes().isNotEmpty() }
        compose.onNodeWithText("导出 CSV").assertIsEnabled()
        compose.onNodeWithText("导出 Excel").assertIsEnabled()
        screenshot("offline-records.png")
    }

    @Test fun nestedScriptPathsMatchTheGatewayInterpreter() {
        val lines = scriptLines(JSONArray("""[{"for_each":"blocks","do":[{"retry":{"times":1,"do":[{"step":"check","on_result":{"empty":[{"step":"back_off"}]}}]}}]}]"""))
        assertTrue(lines.any { it.path=="steps[0].do[0].retry.do[0].on_result.empty[0]" })
    }
}
