"""The spec tables must agree with the code that actually drives the servos."""

import math
import unittest

from vla_controller import spec


class JointTableTest(unittest.TestCase):
    def test_matches_the_driving_package(self):
        """A drift here would aim every action at the wrong servo."""
        servo_protocol = __import__('armpi_ros2.servo_protocol', fromlist=['JOINTS'])
        self.assertEqual(servo_protocol.JOINTS, spec.ARM_PI_JOINTS)

    def test_tick_conversion_round_trips(self):
        for name in spec.POSE_JOINTS:
            low, high = spec.joint_limits(name)
            for tick in (low, (low + high) // 2, high):
                radians = spec.tick_to_radians(name, tick)
                ident, back = spec.radians_to_tick(name, radians)
                self.assertEqual(back, tick, name)
                self.assertEqual(ident, spec.ARM_PI_JOINTS[name][0])

    def test_out_of_range_target_is_refused(self):
        with self.assertRaises(ValueError):
            spec.radians_to_tick('joint2', math.pi)

    def test_non_finite_angle_is_refused(self):
        with self.assertRaises(ValueError):
            spec.radians_to_tick('joint1', float('nan'))


class ActionVectorTest(unittest.TestCase):
    def test_order_is_stable(self):
        self.assertEqual(spec.ACTION_COMPONENTS,
                         ('dx', 'dy', 'dz', 'drx', 'dry', 'drz', 'gripper'))

    def test_vector_to_mapping_is_named(self):
        mapping = spec.action_from_vector([1, 2, 3, 4, 5, 6, 7])
        self.assertEqual(mapping['drx'], 4.0)
        self.assertEqual(mapping['gripper'], 7.0)

    def test_wrong_length_and_nan_are_refused(self):
        with self.assertRaises(ValueError):
            spec.action_from_vector([0] * 6)
        with self.assertRaises(ValueError):
            spec.action_from_vector([0, 0, 0, 0, 0, 0, float('nan')])

    def test_zero_action_keeps_the_gripper_where_it_is(self):
        action = spec.zero_action(0.4)
        self.assertEqual(action[:6], (0.0,) * 6)
        self.assertEqual(action[6], 0.4)
        with self.assertRaises(ValueError):
            spec.zero_action(1.5)


class GripperModelTest(unittest.TestCase):
    def test_open_and_closed_are_the_measured_ticks(self):
        self.assertAlmostEqual(spec.gripper_tick_to_opening(spec.GRIPPER_TICK_OPEN), 1.0)
        self.assertAlmostEqual(spec.gripper_tick_to_opening(spec.GRIPPER_TICK_CLOSED), 0.0)

    def test_smaller_ticks_mean_wider_jaws(self):
        self.assertGreater(spec.gripper_tick_to_opening(300),
                           spec.gripper_tick_to_opening(400))

    def test_out_of_band_ticks_saturate_instead_of_extrapolating(self):
        self.assertEqual(spec.gripper_tick_to_opening(0), 1.0)
        self.assertEqual(spec.gripper_tick_to_opening(700), 0.0)

    def test_opening_to_tick_round_trips(self):
        for tick in (spec.GRIPPER_TICK_OPEN, 300, 400, spec.GRIPPER_TICK_CLOSED):
            opening = spec.gripper_tick_to_opening(tick)
            self.assertEqual(spec.gripper_opening_to_tick(opening), tick)


class RealizableMaskTest(unittest.TestCase):
    def test_pitch_is_realisable_and_roll_yaw_are_not(self):
        self.assertEqual(spec.REALIZABLE_MASK, (1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 1.0))
        self.assertEqual(spec.NON_REALIZABLE_COMPONENTS, ('dry', 'drz'))


if __name__ == '__main__':
    unittest.main()
