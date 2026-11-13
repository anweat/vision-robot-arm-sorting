import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "spatial"))

import dofbot_kinematics as kin  # noqa: E402
import gripper_model as gm  # noqa: E402

MODEL = gm.load()


class JawCurveTest(unittest.TestCase):
    """The jaw table is the only calibration the project has for the gripper; guard its shape."""

    def test_endpoints_come_from_the_official_table(self):
        self.assertAlmostEqual(gm.jaw_gap_m(0.0), 0.064, places=6)
        self.assertAlmostEqual(gm.jaw_gap_m(180.0), 0.0, places=6)

    def test_a_3cm_block_grips_at_134_deg_officially(self):
        # documented: 3 cm -> 134 deg; this arm gripped a 3 cm block at command 133 deg
        self.assertAlmostEqual(gm.servo6_for_gap(0.030), 134.0, delta=0.5)

    def test_the_curve_never_opens_when_the_angle_grows(self):
        gaps = [gm.jaw_gap_m(a) for a in range(0, 181)]
        self.assertTrue(all(b <= a + 1e-9 for a, b in zip(gaps, gaps[1:])))

    def test_the_angles_the_project_actually_uses_clear_a_3cm_block_plus_one_centimetre(self):
        # open_deg 30 and open_wide_deg 11 from config/arm_limits.json
        for angle in (11, 30):
            self.assertTrue(gm.fits(0.030, angle, 0.010), msg=f"{angle} deg")
        # least-open command that still clears 30 + 10 mm comes from the table's 40 mm row
        self.assertAlmostEqual(gm.open_angle_for(0.030, 0.010), 115.0, delta=0.5)

    def test_out_of_range_commands_clamp_instead_of_extrapolating(self):
        self.assertAlmostEqual(gm.jaw_gap_m(-20), 0.064, places=6)
        self.assertAlmostEqual(gm.jaw_gap_m(200), 0.0, places=6)

    def test_the_table_stays_within_the_course_servo_range(self):
        angles = [a for a, _ in MODEL["jaw"]["points"]]
        self.assertEqual(min(angles), 0.0)
        self.assertEqual(max(angles), 180.0)


class PadGeometryTest(unittest.TestCase):
    def test_pads_are_mirror_images_and_their_inner_faces_are_one_jaw_gap_apart(self):
        for angle in (11, 30, 90, 134):
            left, right = gm.pad_boxes(angle)
            gap = gm.jaw_gap_m(angle)
            self.assertAlmostEqual(right["inner_face_x_m"] - left["inner_face_x_m"], gap, places=9)
            self.assertAlmostEqual(left["x_m"][0], -right["x_m"][1], places=9)

    def test_the_gap_between_the_pads_is_the_same_along_the_pad(self):
        # the USD four-bar is a parallelogram, so the pads translate instead of tilting
        left, right = gm.pad_boxes(30)
        self.assertEqual(left["z_m"], right["z_m"])
        self.assertLess(left["z_m"][0], left["z_m"][1])

    def test_the_envelope_stays_below_the_documented_fingertip(self):
        for angle in (0, 30, 180):
            zs = [p[2] for p in gm.envelope_points(angle)]
            self.assertLessEqual(max(zs), MODEL["geometry"]["j5_to_fingertip_m"] + 1e-9)

    def test_the_fully_closed_envelope_is_narrower_than_the_fully_open_one(self):
        def width(angle):
            xs = [p[0] for p in gm.envelope_points(angle)]
            return max(xs) - min(xs)
        self.assertLess(width(180), width(0))
        self.assertGreater(width(0), MODEL["envelope"]["outer_span_max_m"] * 0.9)


class TcpTest(unittest.TestCase):
    """Three 'tips' exist; mixing them is exactly how a grasp ends up a centimetre off."""

    def setUp(self):
        self.servos = [107, 60, 26, 30, 92, 30]

    def test_fingertip_mode_reproduces_the_upstream_forward_kinematics_tip(self):
        expected = kin.forward(self.servos[:4])["tip"]
        got = gm.grasp_point(self.servos, mode="fingertip")
        for a, b in zip(expected, got):
            self.assertAlmostEqual(a, b, delta=1e-4)   # forward() rounds to 0.1 mm

    def test_the_empirical_point_sits_the_measured_distance_from_the_fk_tip(self):
        tip = gm.grasp_point(self.servos, mode="fingertip")
        empirical = gm.grasp_point(self.servos, mode="empirical")
        d = math.dist(tip, empirical)
        self.assertAlmostEqual(d, 0.0115, delta=1e-6)
        # the offset lives in the gripper cross-section, so it turns with J5
        other = gm.grasp_point([107, 60, 26, 30, 152, 30], mode="empirical")
        base = gm.grasp_point([107, 60, 26, 30, 92, 30], mode="fingertip")
        self.assertGreater(math.dist(other, base), 0.005)

    def test_the_geometric_pad_centre_is_about_a_centimetre_behind_the_fk_tip(self):
        tip = gm.grasp_point(self.servos, mode="fingertip")
        centre = gm.grasp_point(self.servos, mode="geometric")
        self.assertAlmostEqual(math.dist(tip, centre), 0.0134, delta=1e-6)

    def test_an_unknown_mode_fails_loudly(self):
        with self.assertRaises(ValueError):
            gm.grasp_point(self.servos, mode="almost")


class SummaryTest(unittest.TestCase):
    def test_summary_is_json_safe(self):
        s = gm.summary()
        self.assertEqual(s["grip_3cm_at_deg"], 134.0)
        self.assertTrue(s["jaw_evidence"])


if __name__ == "__main__":
    unittest.main()
