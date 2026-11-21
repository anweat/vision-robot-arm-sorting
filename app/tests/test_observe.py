"""Observation scripts (src/pipeline/observe.py) and the capture rule (src/adapters/capture.py), docs/current/42."""
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
for sub in ("pipeline", "adapters", "tools", "tasks", "flow"):
    sys.path.insert(0, str(ROOT / "src" / sub))

import capture as CAP  # noqa: E402
import observe as OB  # noqa: E402

OLD_FIELD = [[100, 125, 0, 0], [70, 125, 0, 0], [110, 105, 15, 0], [80, 120, 0, 0], [40, 90, 30, 0],
             [62, 106, 5, 0], [90, 105, 5, 0], [118, 106, 5, 0]]


class ScriptTest(unittest.TestCase):
    def test_the_field_script_is_the_old_scan(self):
        sc = OB.load("field")
        self.assertEqual([v["pose"] for v in sc["views"]], OLD_FIELD)
        self.assertEqual(sc["on_blocked"], "skip")
        self.assertTrue(all(v["board"] for v in sc["views"]))
        import field_scan as FS
        self.assertEqual(FS.SCAN_POSES, OLD_FIELD)

    def test_every_script_in_the_folder_loads_and_its_poses_pass_offline(self):
        for name in OB.names():
            rows = OB.check(name)
            self.assertTrue(all(r["ok"] for r in rows), (name, [r for r in rows if not r["ok"]]))

    def test_shape_problems_are_named(self):
        bad = OB.problems(dict(views=[dict(pose=[1, 2, 3])], on_blocked="later", colour=1))
        self.assertTrue(any("pose" in b for b in bad))
        self.assertTrue(any("on_blocked" in b for b in bad))
        self.assertTrue(any("colour" in b for b in bad))
        with self.assertRaises(ValueError):
            OB.load(dict(views=[]))

    def test_a_view_can_override_the_defaults_and_carry_its_own_keys(self):
        sc = OB.load(dict(name="t", settle_s=0.5, views=[dict(pose=[90, 105, 5, 5]),
                                                         dict(pose=[90, 105, 5, 5, 120], settle_s=1.2, bin=2)]))
        self.assertEqual([v["settle_s"] for v in sc["views"]], [0.5, 1.2])
        self.assertEqual(OB.command(sc["views"][1]), [90, 105, 5, 5, 120])
        self.assertEqual(sc["views"][1]["bin"], 2)

    def test_a_path_through_the_platform_is_refused_offline(self):
        rows = OB.check(dict(views=[dict(pose=[90, 40, 10, 10])]))
        self.assertFalse(rows[0]["ok"])


class FakeResponse:
    def __init__(self, body, headers):
        self.body, self.headers = body, headers

    def read(self):
        return self.body

    def __enter__(self):
        return self

    def __exit__(self, *a):
        return False


class CaptureTest(unittest.TestCase):
    def test_a_frame_older_than_the_stop_is_not_used(self):
        stamps = iter([9.0, 9.5, 10.4])                 # the still readback was at 10.0
        frames = lambda *a, **k: FakeResponse(b"\xff\xd8jpg", {"X-Frame-Time": str(next(stamps)), "X-Frame-Seq": "3"})  # noqa: E731
        with patch.object(CAP.urllib.request, "urlopen", side_effect=frames), patch.object(CAP.time, "sleep"):
            jpeg, meta = CAP.frame("http://x", after=10.0)
        self.assertEqual(meta["stamp"], 10.4)

    def test_an_old_console_without_the_header_gives_its_frame(self):
        with patch.object(CAP.urllib.request, "urlopen", return_value=FakeResponse(b"\xff\xd8", {})):
            jpeg, meta = CAP.frame("http://x", after=10.0)
        self.assertEqual(meta, {})

    def test_still_needs_two_agreeing_readbacks(self):
        seq = iter([dict(pose=[90, 105, 5, 5, 95, 30], busy=True, observed_at=1),
                    dict(pose=[90, 105, 5, 5, 95, 30], busy=False, observed_at=2),
                    dict(pose=[90, 105, 5, 5, 95, 30], busy=False, observed_at=3)])
        with patch.object(CAP, "status", side_effect=lambda *_: next(seq)), patch.object(CAP.time, "sleep"):
            first, now = CAP.wait_still("http://x")
        self.assertEqual((first["observed_at"], now["observed_at"]), (2, 3))


if __name__ == "__main__":
    unittest.main()
