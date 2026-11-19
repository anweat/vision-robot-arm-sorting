import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src' / 'teleop'))

try:
    import cv2
    import numpy as np
except ImportError:  # pragma: no cover
    cv2 = None
import localization as L


@unittest.skipIf(cv2 is None, 'OpenCV not installed')
class AnnotationTest(unittest.TestCase):
    def test_gripper_points_follow_forward_kinematics(self):
        points = L.gripper_map_points([90, 90, 90, 90, 90, 30])
        self.assertEqual(len(points), 3)
        # upright arm: all points on the base axis, i.e. the map point under base (0, 0)
        from map_registration import TableMap
        dx, dy, _ = TableMap().offset
        for x, y in points:
            self.assertAlmostEqual(x, -dx, places=3)
            self.assertAlmostEqual(y, -dy, places=3)
        self.assertEqual(L.gripper_map_points(None), [])

    def test_annotate_skips_hidden_rejections_and_separates_labels(self):
        image = np.full((480, 640, 3), 255, np.uint8)
        result = dict(objects=[dict(label='red', bbox=[100, 100, 40, 40], map_cm=[1, 2]),
                               dict(label='green', bbox=[110, 104, 40, 40], map_cm=[3, 4])],
                      rejected=[dict(label='blue', bbox=[300, 300, 30, 30], reason='地图外', draw=False)])
        out = L.annotate(image.copy(), result)
        self.assertTrue((out[300:330, 300:330] == 255).all())  # hidden rejection not drawn
        self.assertFalse((out[100:140, 100:140] == 255).all())


@unittest.skipIf(cv2 is None, 'OpenCV not installed')
class PhoneParallaxTest(unittest.TestCase):
    """2026-09-26: four blocks on marker centres; table projection put them 2.2-3.0 cm away from the phone."""

    def test_blocks_on_marker_centres_within_a_centimetre(self):
        import json
        d = Path(__file__).resolve().parents[1] / 'data' / 'captures' / '2026-09-26'
        if not (d / 'phone_blocks_on_markers.json').exists():
            self.skipTest('needs the recorded frames under data/captures (not in the repository)')
        meta = json.loads((d / 'phone_blocks_on_markers.json').read_text())
        layout = json.loads((Path(__file__).resolve().parents[1] / 'config' / 'map_a4_v2.json').read_text())['layout_m']
        result = L.locate_phone((d / meta['image']).read_bytes(), meta['arm_pose'])
        self.assertIsNotNone(result['phone_camera'])
        found = {o['label']: o['map_cm'] for o in result['objects']}
        for colour, marker in meta['truth'].items():
            centre = np.mean(np.asarray(layout[str(marker)]), axis=0)[:2] * 100
            self.assertLess(float(np.hypot(*(np.array(found[colour]) - centre))), 1.0, colour)


if __name__ == '__main__':
    unittest.main()


@unittest.skipIf(cv2 is None, 'OpenCV not installed')
class BoardSplitTest(unittest.TestCase):
    def test_board_localization_terminates_on_real_frames(self):
        """2026-09-26: splitting merged blobs appended to the list being iterated and hung the locator."""
        import signal
        d = Path(__file__).resolve().parents[1] / 'data' / 'captures' / '2026-09-25' / 'map_handeye'
        if not (d / 'views.json').exists():
            self.skipTest('needs the recorded frames under data/captures (not in the repository)')
        views = __import__('json').loads((d / 'views.json').read_text())
        signal.signal(signal.SIGALRM, lambda *a: (_ for _ in ()).throw(TimeoutError('locate_board hung')))
        for v in views[:3]:
            signal.alarm(10)
            try:
                L.locate_board((d / v['image']).read_bytes(), v['readback'])
            except ValueError:
                pass
            finally:
                signal.alarm(0)


@unittest.skipIf(cv2 is None, 'OpenCV not installed')
class BoardPerHueTest(unittest.TestCase):
    def test_red_next_to_yellow_and_background_is_found(self):
        import json
        d = Path(__file__).resolve().parents[1] / 'data' / 'captures' / '2026-09-26'
        if not (d / 'board_red_near_yellow.json').exists():
            self.skipTest('needs the recorded frames under data/captures (not in the repository)')
        meta = json.loads((d / 'board_red_near_yellow.json').read_text())
        result = L.locate_board((d / meta['image']).read_bytes(), meta['servos'])
        red = [o for o in result['objects'] if o['label'] == 'red']
        self.assertEqual(len(red), 1)
        self.assertLess(float(np.hypot(*(np.array(red[0]['map_cm']) - meta['expect']['red']))), 1.0)
        self.assertEqual(red[0]['method'], 'top-face')
