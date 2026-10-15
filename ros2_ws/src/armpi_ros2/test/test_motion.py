import math
import unittest
from armpi_ros2.motion import (
    Geometry, MotionLease, encoder_delta, integrate_pose,
    rotate_vector, wheel_commands, wheel_displacement,
)


def legacy_output(velocity, direction, angular):
    velocity, angular = -velocity, -angular
    vx = velocity * math.cos(math.radians(direction))
    vy = velocity * math.sin(math.radians(direction))
    vp = angular * 207.5
    v1, v2, v3, v4 = vy - vx + vp, vy + vx - vp, vy - vx - vp, vy + vx + vp
    return [int(v / (math.pi * 96.5) * 7832 * 0.01) for v in (-v1, v4, v2, -v3)]


class KinematicsTests(unittest.TestCase):
    def test_forward_matches_legacy(self):
        self.assertEqual(wheel_commands(0.06, 0, 0), legacy_output(60, 90, 0))

    def test_back_matches_legacy(self):
        self.assertEqual(wheel_commands(-0.06, 0, 0), legacy_output(60, 270, 0))

    def test_left_matches_legacy(self):
        self.assertEqual(wheel_commands(0, 0.06, 0), legacy_output(60, 180, 0))

    def test_right_matches_legacy(self):
        self.assertEqual(wheel_commands(0, -0.06, 0), legacy_output(60, 0, 0))

    def test_left_and_right_turn_match_legacy(self):
        for omega in (0.2, -0.2):
            self.assertEqual(wheel_commands(0, 0, omega), legacy_output(0, 0, omega))

    def test_combined_translation_rotation(self):
        for x, y, omega in ((.02, .01, .1), (-.03, .025, -.13), (.06, -.04, .1)):
            direction = math.degrees(math.atan2(x, -y)) % 360
            expected = legacy_output(math.hypot(x, y) * 1000, direction, omega)
            self.assertEqual(wheel_commands(x, y, omega), expected)

    def test_signed_output_saturation(self):
        self.assertLessEqual(max(abs(x) for x in wheel_commands(10, 10, 10)), 100)

    def test_nonfinite_and_invalid_geometry_rejected(self):
        for value in (float('nan'), float('inf'), -float('inf')):
            with self.assertRaises(ValueError):
                wheel_commands(value, 0, 0)
        with self.assertRaises(ValueError):
            Geometry(wheel_diameter=0)
        with self.assertRaises(ValueError):
            wheel_commands(0, 0, 0, max_pulse=128)

    def test_encoder_int32_wrap(self):
        self.assertEqual(encoder_delta(-2147483648, 2147483647), 1)
        self.assertEqual(encoder_delta(2147483647, -2147483648), -1)

    def test_encoder_forward_and_left_and_turn(self):
        dx, dy, da = wheel_displacement([100, -100, -100, 100])
        self.assertGreater(dx, 0)
        self.assertEqual((dy, da), (0, 0))
        dx, dy, da = wheel_displacement([100, 100, 100, 100])
        self.assertGreater(dy, 0)
        self.assertEqual((dx, da), (0, 0))
        dx, dy, da = wheel_displacement([100, -100, 100, -100])
        self.assertGreater(da, 0)
        self.assertEqual((dx, dy), (0, 0))

    def test_encoder_real_bench_captures(self):
        # Raw board feedback, not synthetic encoder assumptions from ROS 1.
        dx, dy, da = wheel_displacement([646, -641, -646, 647])
        self.assertGreater(dx, .02)
        self.assertLess(abs(dy), .001)
        self.assertLess(abs(da), .001)
        dx, dy, da = wheel_displacement([651, 646, 647, 648])
        self.assertGreater(dy, .02)
        self.assertLess(abs(dx), .001)
        self.assertLess(abs(da), .001)
        dx, dy, da = wheel_displacement([420, -411, 423, -420])
        self.assertGreater(da, .07)
        self.assertLess(math.hypot(dx, dy), .001)

    def test_encoder_inverse_combined_motion(self):
        g = Geometry()
        for x, y, yaw in ((.02, .01, .1), (-.03, .025, -.13), (.06, -.04, .1)):
            turn = yaw * (g.a + g.b)
            metres = (x + y + turn, -x + y - turn, -x + y + turn, x + y - turn)
            counts = [m / g.metres_per_pulse for m in metres]
            actual = wheel_displacement(counts, g)
            for result, expected in zip(actual, (x, y, yaw)):
                self.assertAlmostEqual(result, expected)

    def test_encoder_channel_and_body_polarity(self):
        g = Geometry()
        x, y, yaw = wheel_displacement([100, 100, -100, 100], g,
                                      channel_signs=(1, -1, 1, 1))
        self.assertAlmostEqual(x, 100 * g.metres_per_pulse)
        self.assertEqual((y, yaw), (0, 0))
        reversed_axes = wheel_displacement([100, -100, -100, 100], g, signs=(-1, 1, -1))
        self.assertAlmostEqual(reversed_axes[0], -x)

    def test_encoder_invalid_signs_and_deltas(self):
        for deltas, axes, channels in (
                ([1, 2, 3], (1, 1, 1), (1, 1, 1, 1)),
                ([1, 2, 3, float('nan')], (1, 1, 1), (1, 1, 1, 1)),
                ([1, 2, 3, 4], (1, 1), (1, 1, 1, 1)),
                ([1, 2, 3, 4], (1, 1, 0), (1, 1, 1, 1)),
                ([1, 2, 3, 4], (1, 1, 1), (1, 1, 1)),
                ([1, 2, 3, 4], (1, 1, 1), (1, 1, float('inf'), 1))):
            with self.assertRaises(ValueError):
                wheel_displacement(deltas, signs=axes, channel_signs=channels)

    def test_forward_and_strafe_arc(self):
        p = integrate_pose((0, 0, 0), 1, 0, math.pi / 2)
        self.assertAlmostEqual(p[0], 2 / math.pi)
        self.assertAlmostEqual(p[1], 2 / math.pi)
        p = integrate_pose((0, 0, 0), 0, 1, math.pi / 2)
        self.assertAlmostEqual(p[0], -2 / math.pi)
        self.assertAlmostEqual(p[1], 2 / math.pi)

    def test_pose_rotates_body_displacement(self):
        p = integrate_pose((0, 0, math.pi / 2), 1, 0, 0)
        self.assertAlmostEqual(p[0], 0)
        self.assertAlmostEqual(p[1], 1)

    def test_imu_mount_yaw90(self):
        result = rotate_vector((1, 0, .1), 0, 0, math.pi / 2)
        self.assertAlmostEqual(result[0], 0)
        self.assertAlmostEqual(result[1], 1)
        self.assertAlmostEqual(result[2], .1)


class LeaseTests(unittest.TestCase):
    def test_timeout_discards_command(self):
        lease = MotionLease()
        lease.accept(.06, 0, 0, 10)
        self.assertEqual(lease.output(10.2), (.06, 0, 0))
        self.assertEqual(lease.output(10.4), (0, 0, 0))
        self.assertIsNone(lease.received_at)

    def test_limits_preserve_translation_direction(self):
        lease = MotionLease()
        lease.accept(1, 1, 10, 1)
        x, y, w = lease.output(1)
        self.assertAlmostEqual(math.hypot(x, y), .08)
        self.assertAlmostEqual(x, y)
        self.assertEqual(w, .2)

    def test_stop_and_estop(self):
        lease = MotionLease()
        lease.accept(.02, 0, 0, 1)
        lease.stop(latch=True)
        self.assertFalse(lease.accept(.03, 0, 0, 2))
        self.assertEqual(lease.output(2), (0, 0, 0))
        lease.reset()
        self.assertEqual(lease.output(2), (0, 0, 0))
        self.assertTrue(lease.accept(.03, 0, 0, 3))

    def test_invalid_input_cancels_previous_output(self):
        lease = MotionLease()
        lease.accept(.02, 0, 0, 1)
        self.assertFalse(lease.accept(float('nan'), 0, 0, 1.1))
        self.assertEqual(lease.output(1.1), (0, 0, 0))

    def test_time_reversal_cancels_output(self):
        lease = MotionLease()
        lease.accept(.02, 0, 0, 10)
        self.assertEqual(lease.output(9), (0, 0, 0))

    def test_zero_command_is_zero(self):
        lease = MotionLease()
        lease.accept(.02, 0, 0, 1)
        lease.accept(0, 0, 0, 1.1)
        self.assertEqual(lease.output(1.1), (0, 0, 0))


if __name__ == '__main__':
    unittest.main()
