import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
for sub in ('pipeline', 'tasks', 'tools', 'spatial', 'vision'):
    sys.path.insert(0, str(ROOT / 'dev' / sub))
    sys.path.insert(0, str(ROOT / 'app' / 'src' / sub))
import collect as K  # noqa: E402


class TumbleGeometryTest(unittest.TestCase):
    def test_roll_and_flatten_tilt_the_held_block_past_45(self):
        # gripped at pitch 135 / J5 92, then J5 180 and pitch 120: the old bottom face turns 68 deg
        self.assertAlmostEqual(K.tilt_deg(K.held_rotation(135, 92, 120, 180)), 68, delta=1)
        self.assertAlmostEqual(K.tilt_deg(K.held_rotation(135, 92, 135, 92)), 0, delta=1e-6)

    def test_lowest_corner(self):
        self.assertAlmostEqual(K.lowest_corner(K.held_rotation(150, 92, 150, 92)), 0.015, places=6)
        self.assertGreater(K.lowest_corner(K.held_rotation(135, 92, 120, 180)), 0.02)


class MatchTest(unittest.TestCase):
    TRACK = [dict(category='kitchen', map_cm=[-6, 13]), dict(category='other', map_cm=[12, 13])]

    def scene(self, *cubes):
        return dict(cubes=[dict(map_cm=m, colour=c) for m, c in cubes])

    def test_the_tumbled_block_is_found_near_its_drop_point(self):
        lab, why = K.match(self.scene(([-5.5, 13.2], 'green'), ([3, 10], None)), self.TRACK, 1, [2, 12])
        self.assertEqual(why, '')
        self.assertEqual({tuple(c['map_cm']): c['category'] for c in lab}, {(-5.5, 13.2): 'kitchen', (3, 10): 'other'})

    def test_a_colour_that_disagrees_or_a_lost_block_gives_no_labels(self):
        self.assertIsNone(K.match(self.scene(([-6, 13], 'red'), ([12, 13], None)), self.TRACK, None, None)[0])
        self.assertIsNone(K.match(self.scene(([-6, 13], None),), self.TRACK, None, None)[0])
        self.assertIsNone(K.match(self.scene(([-6, 13], None), ([12, 13], None), ([0, 17], None)), self.TRACK, None, None)[0])


if __name__ == '__main__':
    unittest.main()
