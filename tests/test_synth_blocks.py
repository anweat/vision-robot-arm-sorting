import math
import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
for sub in ('pipeline', 'tasks', 'tools', 'spatial', 'vision'):
    sys.path.insert(0, str(ROOT / 'dev' / sub))
    sys.path.insert(0, str(ROOT / 'app' / 'src' / sub))
import synth_blocks as SY  # noqa: E402
import eval_scene as E  # noqa: E402
import run as R  # noqa: E402


class CubeGeometryTest(unittest.TestCase):
    def test_the_six_faces_are_30_mm_squares_with_outward_normals(self):
        faces = SY.cube_faces(0.05, 0.12, 20)
        self.assertEqual(len(faces), 5)                     # the top and four sides; the base is never seen
        for corners, normal in faces:
            sides = [np.linalg.norm(corners[i] - corners[(i + 1) % 4]) for i in range(4)]
            for s in sides:
                self.assertAlmostEqual(s, SY.BLOCK, places=6)
            centre = corners.mean(axis=0)
            self.assertAlmostEqual(float(np.linalg.norm(centre - np.array([0.05, 0.12, SY.BLOCK / 2]))),
                                   SY.BLOCK / 2, places=6)
            self.assertAlmostEqual(float(np.dot(normal, centre - np.array([0.05, 0.12, SY.BLOCK / 2]))),
                                   SY.BLOCK / 2, places=6)
        self.assertTrue(np.allclose(faces[0][1], [0, 0, 1]))

    def test_corners_sit_on_the_table_and_one_block_up(self):
        c = SY.corners(0.0, 0.1, 0)
        self.assertEqual(len(c), 8)
        self.assertAlmostEqual(c[:, 2].min(), 0.0)
        self.assertAlmostEqual(c[:, 2].max(), SY.BLOCK)

    def test_every_block_has_six_faces_including_its_colour(self):
        if not (SY.FACES_DIR / 'index.json').exists():
            self.skipTest('needs data/reference/block_faces (Release assets-v3)')
        blocks = SY.load_blocks()
        self.assertEqual(sorted(blocks), sorted(SY.CLASSES))
        for cat, faces in blocks.items():
            self.assertEqual(len(faces), 6, cat)


class YawErrorTest(unittest.TestCase):
    def test_a_square_repeats_every_90_degrees(self):
        self.assertAlmostEqual(E.yaw_error(5, 95), 0.0)
        self.assertAlmostEqual(E.yaw_error(-44, 44), 2.0)      # across the +-45 wrap
        self.assertAlmostEqual(E.yaw_error(0, 45), 45.0)       # the worst a square can be


class RefineTriggerTest(unittest.TestCase):
    def cube(self, **kw):
        return dict(dict(coverage=1.0, score=0.9, detector=None), **kw)

    def test_a_block_seen_only_in_clipped_boxes_is_looked_at_again(self):
        # 2026-09-26: these are the map-edge blocks, smeared on the top-plane orthophoto
        self.assertTrue(R.needs_refine(self.cube(detector=dict(approx=True))))
        self.assertTrue(R.needs_refine(self.cube(score=0.35)))
        self.assertTrue(R.needs_refine(self.cube(coverage=0.9)))
        self.assertFalse(R.needs_refine(self.cube(detector=dict(approx=False))))


if __name__ == '__main__':
    unittest.main()
