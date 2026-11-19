package com.example.myapplication00001

import java.io.ByteArrayInputStream
import java.io.ByteArrayOutputStream
import java.util.zip.ZipInputStream
import javax.xml.parsers.DocumentBuilderFactory
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class ExportFormatTest {
    private val record = BlockRecord(
        seq = 7,
        time = "2026-10-20 22:30",
        timestamp = "1792506600.0",
        round = 2,
        run = "sim-run",
        category = "可回收物",
        cubeX = "-4.5",
        cubeY = "12.0",
        bin = "40",
        seconds = "3.2",
        result = "分拣成功",
        reason = "小于 & 大于, \"测试\"",
        source = "mobile",
    )

    @Test
    fun csvHasBomAndOneDataRow() {
        val output = ByteArrayOutputStream()
        writeCsv(output, listOf(record))
        val bytes = output.toByteArray()
        assertTrue(bytes.take(3) == listOf(0xEF.toByte(), 0xBB.toByte(), 0xBF.toByte()))
        val text = bytes.toString(Charsets.UTF_8)
        assertEquals(2, text.trim().lines().size)
        assertTrue(text.contains("\"小于 & 大于, \"\"测试\"\"\""))
    }

    @Test
    fun excelWorksheetIsWellFormedAndContainsOneDataRow() {
        val output = ByteArrayOutputStream()
        writeXlsx(output, listOf(record))
        val entries = mutableMapOf<String, ByteArray>()
        ZipInputStream(ByteArrayInputStream(output.toByteArray())).use { zip ->
            while (true) {
                val entry = zip.nextEntry ?: break
                entries[entry.name] = zip.readBytes()
                zip.closeEntry()
            }
        }
        assertTrue(entries.keys.containsAll(listOf("[Content_Types].xml", "_rels/.rels", "xl/workbook.xml", "xl/worksheets/sheet1.xml")))
        val factory = DocumentBuilderFactory.newInstance().apply { isNamespaceAware = true }
        val worksheet = factory.newDocumentBuilder().parse(ByteArrayInputStream(entries.getValue("xl/worksheets/sheet1.xml")))
        val ns = "http://schemas.openxmlformats.org/spreadsheetml/2006/main"
        assertEquals(2, worksheet.getElementsByTagNameNS(ns, "row").length)
        assertTrue(worksheet.documentElement.textContent.contains("小于 & 大于, \"测试\""))
    }
}
