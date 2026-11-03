"""The adapter is the only place where a delta becomes a servo tick.

The pose used throughout is a real measured one (arm unfolded in the air,
``j1=495 j2=644 j3=696 j4=166``), so a regression in the vendor mapping or in
this adapter shows up as a changed number rather than as a plausible one.
"""

import math
import unittest

from vla_controller.adapter import AdapterError, TcpDeltaAdapter
from vla_controller import spec

RAW = {'joint1': 495, 'joint2': 644, 'joint3': 696, 'joint4': 166, 'joint5': 499}


def action(dx=0.0, dy=0.0, dz=0.0, drx=0.0, dry=0.0, drz=0.0, gripper=0.5):
    return (dx, dy, dz, drx, dry, drz, gripper)


class PoseTest(unittest.TestCase):
    def setUp(self):
        self.adapter = TcpDeltaAdapter()

    def test_measured_pose_is_reproduced(self):
        pose = self.adapter.pose_from_raw(RAW)
        self.assertAlmostEqual(pose.x, 0.0027, places=4)
        self.assertAlmostEqual(pose.y, 0.1266, places=4)
        self.assertAlmostEqual(pose.z, 0.2323, places=4)
        self.assertAlmostEqual(pose.pitch_deg, -92.64, places=1)
        self.assertAlmostEqual(pose.yaw_deg, -1.20, places=1)

    def test_missing_feedback_is_named(self):
        with self.assertRaises(AdapterError) as caught:
            self.adapter.pose_from_raw({'joint1': 500})
        self.assertIn('joint2', str(caught.exception))

    def test_out_of_range_feedback_is_refused(self):
        with self.assertRaises(AdapterError):
            self.adapter.pose_from_raw(dict(RAW, joint3=1200))


class DirectionTest(unittest.TestCase):
    def setUp(self):
        self.adapter = TcpDeltaAdapter()

    def test_every_axis_solves_at_full_scale(self):
        for delta in ((0.010, 0, 0, 0), (-0.010, 0, 0, 0), (0, 0.010, 0, 0),
                      (0, -0.010, 0, 0), (0, 0, 0.010, 0), (0, 0, -0.010, 0)):
            solution = self.adapter.solve(RAW, action(*delta))
            self.assertAlmostEqual(solution.scale, 1.0, places=6, msg=str(delta))
            self.assertLess(solution.residual_m, 0.002, str(delta))

    def test_reported_residual_is_below_servo_quantisation_scale(self):
        """One tick is 0.24 deg; at this lever arm that is already ~1 mm."""
        solution = self.adapter.solve(RAW, action(dy=0.010))
        self.assertLess(solution.residual_m, 0.0015)

    def test_lateral_motion_turns_the_base_joint(self):
        solution = self.adapter.solve(RAW, action(dx=0.010))
        self.assertNotEqual(solution.targets['joint1'], RAW['joint1'])

    def test_wrist_roll_is_never_commanded(self):
        solution = self.adapter.solve(RAW, action(dy=0.010))
        self.assertEqual(set(solution.targets), set(spec.POSE_JOINTS))
        self.assertNotIn('joint5', solution.targets)


class ScalingTest(unittest.TestCase):
    def test_tight_tick_budget_shrinks_the_whole_step(self):
        adapter = TcpDeltaAdapter(max_step_ticks=10)
        solution = adapter.solve(RAW, action(dx=0.010))
        self.assertLess(solution.scale, 1.0)
        self.assertLessEqual(
            max(abs(solution.targets[name] - RAW[name]) for name in spec.POSE_JOINTS), 10)

    def test_scaling_keeps_the_direction(self):
        loose = TcpDeltaAdapter().solve(RAW, action(dy=0.010))
        tight = TcpDeltaAdapter(max_step_ticks=10).solve(RAW, action(dy=0.010))
        loose_delta = [loose.targets[n] - RAW[n] for n in spec.POSE_JOINTS]
        tight_delta = [tight.targets[n] - RAW[n] for n in spec.POSE_JOINTS]
        # Same sign on every axis that moved materially; magnitude shrunk.
        for a, b in zip(loose_delta, tight_delta):
            if abs(a) >= 5:
                self.assertEqual(math.copysign(1, a), math.copysign(1, b) or 1)
                self.assertLessEqual(abs(b), abs(a) + 1)

    def test_impossible_step_is_refused_not_silently_dropped(self):
        with self.assertRaises(AdapterError) as caught:
            self.adapter_tight().solve(RAW, action(dy=0.400))
        self.assertIn('No realisable step', str(caught.exception))

    def adapter_tight(self):
        return TcpDeltaAdapter(max_step_ticks=5)


class RefusalTest(unittest.TestCase):
    def setUp(self):
        self.adapter = TcpDeltaAdapter()

    def test_roll_reaching_the_adapter_is_a_bug_not_a_no_op(self):
        with self.assertRaises(AdapterError) as caught:
            self.adapter.solve(RAW, action(dry=0.01))
        self.assertIn('realisable', str(caught.exception))

    def test_yaw_reaching_the_adapter_is_a_bug_not_a_no_op(self):
        with self.assertRaises(AdapterError):
            self.adapter.solve(RAW, action(drz=0.01))

    def test_pitch_delta_is_refused_away_from_the_model_axis(self):
        turned = dict(RAW, joint1=800)          # ~72 deg of base yaw
        with self.assertRaises(AdapterError) as caught:
            self.adapter.solve(turned, action(drx=0.01))
        self.assertIn('yaw', str(caught.exception))

    def test_pitch_delta_near_zero_yaw_is_allowed(self):
        solution = self.adapter.solve(RAW, action(drx=math.radians(5.0)))
        self.assertLess(solution.residual_m, 0.002)

    def test_wrong_component_count_is_refused(self):
        with self.assertRaises(AdapterError):
            self.adapter.solve(RAW, (0.0, 0.0, 0.0))


class OutputTest(unittest.TestCase):
    def setUp(self):
        self.adapter = TcpDeltaAdapter()

    def test_radian_targets_match_the_tick_targets(self):
        solution = self.adapter.solve(RAW, action(dy=0.010))
        radians = self.adapter.radian_targets(solution)
        for name, tick in solution.targets.items():
            self.assertAlmostEqual(radians[name], spec.tick_to_radians(name, tick), places=12)

    def test_radian_targets_are_inside_the_hardware_range(self):
        solution = self.adapter.solve(RAW, action(dy=0.010))
        for name, value in self.adapter.radian_targets(solution).items():
            ident, back = spec.radians_to_tick(name, value)
            self.assertEqual(back, solution.targets[name])

    def test_wrist_roll_hold_can_be_pinned(self):
        adapter = TcpDeltaAdapter(wrist_roll_hold_tick=500)
        self.assertEqual(adapter.wrist_roll_target({'joint5': 499}), 500)
        self.assertEqual(TcpDeltaAdapter().wrist_roll_target({'joint5': 499}), 499)

    def test_describe_lists_the_held_joint(self):
        described = self.adapter.describe()
        self.assertEqual(described['held_joint'], 'joint5')
        self.assertIn('scales', described)


class ConfigurationTest(unittest.TestCase):
    def test_tick_budget_cannot_exceed_the_arm_node_limit(self):
        with self.assertRaises(ValueError):
            TcpDeltaAdapter(max_step_ticks=101)

    def test_scales_are_sorted_descending(self):
        adapter = TcpDeltaAdapter(scales=(0.25, 1.0, 0.5))
        self.assertEqual(adapter.scales, (1.0, 0.5, 0.25))

    def test_scale_above_one_is_refused(self):
        with self.assertRaises(ValueError):
            TcpDeltaAdapter(scales=(1.5, 0.5))


if __name__ == '__main__':
    unittest.main()
