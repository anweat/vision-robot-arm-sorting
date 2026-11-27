import csv
import io
import os
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from offline_fixture import FixtureGateway
from playwright.sync_api import sync_playwright, expect


class WebRecordsBrowserTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        cls.root = Path(os.environ.get("OFFLINE_ACCEPTANCE_OUT", cls.temp.name)) / "browser"
        cls.root.mkdir(parents=True, exist_ok=True)
        cls.pw = sync_playwright().start()
        cls.browser = cls.pw.chromium.launch(headless=True)
        cls.addClassCleanup(cls.cleanup)

    @classmethod
    def cleanup(cls):
        cls.browser.close()
        cls.pw.stop()
        cls.temp.cleanup()

    def setUp(self):
        self.folder = self.root / self._testMethodName
        self.fixture = FixtureGateway(self.folder)
        self.context = self.browser.new_context(accept_downloads=True)
        self.page = self.context.new_page()
        self.page.goto(self.fixture.url + "/#logs")
        self.page.locator("#lRound").select_option("all")
        expect(self.page.locator("#lPage")).to_contain_text("共 1000 条", timeout=15000)
        self.addCleanup(self.fixture.close)
        self.addCleanup(self.context.close)

    def download(self, selector, name):
        with self.page.expect_download() as info:
            self.page.locator(selector).click()
        path = self.folder / name
        info.value.save_as(path)
        return path

    def test_thousand_rows_pagination_filters_and_exports(self):
        seen = set()
        for i in range(10):
            expect(self.page.locator("#lPage")).to_contain_text(f"第 {i+1}/10 页")
            seen.update(self.page.locator("#lRecords tr td:first-child").all_text_contents())
            if i < 9: self.page.locator("#lNext").click()
        self.assertEqual(len(seen), 1000)
        csv_path = self.download("#lCsv", "all.csv")
        self.assertTrue(csv_path.read_bytes().startswith(b"\xef\xbb\xbf"))
        rows = list(csv.reader(io.StringIO(csv_path.read_text(encoding="utf-8-sig"))))
        self.assertEqual(len(rows), 1001)
        self.assertEqual(len({r[0] for r in rows[1:]}), 1000)
        self.assertTrue(any('中文, "引号"\n换行 & <内容>' == r[-1] for r in rows))
        self.assertTrue(any(r[-1] == "'=SUM(1,2)" for r in rows))
        xml_path = self.download("#lExcel", "all.xml")
        tree = ET.parse(xml_path)
        self.assertEqual(len(tree.findall(".//{urn:schemas-microsoft-com:office:spreadsheet}Row")), 1001)
        self.page.locator("#lCategory").select_option("kitchen")
        self.page.locator("#lResult").select_option("sorted")
        expect(self.page.locator("#lPage")).to_contain_text("共 50 条")
        filtered = self.download("#lCsv", "filtered.csv")
        selected = list(csv.reader(io.StringIO(filtered.read_text(encoding="utf-8-sig"))))
        self.assertEqual(len(selected), 51)
        self.assertTrue(all(r[3] == "厨余垃圾" and r[8] == "已分拣" for r in selected[1:]))
        self.page.screenshot(path=str(self.folder / "records.png"), full_page=True)

    def test_error_keeps_snapshot_and_retry_recovers_empty_data(self):
        self.page.route("**/api/log/records*", lambda route: route.fulfill(status=503, content_type="application/json", body='{"error":"离线测试断连"}'))
        expect(self.page.locator("#lError")).to_contain_text("保留上次记录", timeout=15000)
        expect(self.page.locator("#lPage")).to_contain_text("共 1000 条")
        self.page.unroute("**/api/log/records*")
        self.fixture.state.truncate_log()
        self.page.locator("#lRetry").click()
        expect(self.page.locator("#lPage")).to_contain_text("共 0 条", timeout=15000)
        expect(self.page.locator("#lCsv")).to_be_disabled()
        expect(self.page.locator("#lError")).to_be_empty()

    def test_sse_reconnect_preserves_exact_record_count(self):
        self.context.set_offline(True)
        e = dict(t=__import__('time').time(), event="pipeline_sorted", cube=[0, 10], category="kitchen", bin="bin2", seconds=1)
        self.fixture.state.append(e)
        self.context.set_offline(False)
        expect(self.page.locator("#lPage")).to_contain_text("共 1001 条", timeout=20000)
        download = self.download("#lCsv", "reconnected.csv")
        self.assertEqual(len(list(csv.reader(io.StringIO(download.read_text(encoding="utf-8-sig"))))), 1002)
