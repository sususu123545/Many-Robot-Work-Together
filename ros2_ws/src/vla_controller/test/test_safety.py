"""What the filter must refuse, and what it must merely slow down."""

import math
import unittest

from vla_controller import safety
from vla_controller.safety import Code, Context, Limits, SafetyFilter, Workspace
from vla_controller.spec import ACTION_FRAME, ACTION_SPACE_ID, REALIZABLE_MASK

GOOD_POSE = (0.0, 0.15, 0.20)
GRIPPER = 0.5


def context(**overrides):
    values = dict(now=100.0, observation_stamp=99.9, chunk_stamp=99.8,
                  chunk_validity_s=1.0, tcp_position=GOOD_POSE,
                  gripper_opening=GRIPPER)
    values.update(overrides)
    return Context(**values)


def action(dx=0.0, dy=0.0, dz=0.0, drx=0.0, dry=0.0, drz=0.0, gripper=GRIPPER):
    return (dx, dy, dz, drx, dry, drz, gripper)


class PreconditionTest(unittest.TestCase):
    def setUp(self):
        self.filter = SafetyFilter(Limits())

    def test_healthy_and_parked_passes(self):
        verdict = self.filter.check(action(dy=0.005), context())
        self.assertTrue(verdict.accepted, verdict.reason)

    def test_latched_state_refuses_before_anything_else(self):
        verdict = self.filter.check(action(dy=0.005), context(latched=True))
        self.assertFalse(verdict.accepted)
        self.assertEqual(verdict.code, Code.ESTOP)

    def test_unhealthy_arm_refuses(self):
        verdict = self.filter.check(
            action(dy=0.005),
            context(arm_ready=False, arm_detail='fault_latched'))
        self.assertEqual(verdict.code, Code.HEALTH)
        self.assertIn('fault_latched', verdict.reason)

    def test_moving_base_refuses(self):
        """The delta is relative to arm_base; a moving base invalidates it."""
        verdict = self.filter.check(
            action(dy=0.005), context(base_parked=False, base_detail='base motion detected'))
        self.assertEqual(verdict.code, Code.BASE_MOVING)


class IdentityTest(unittest.TestCase):
    def setUp(self):
        self.filter = SafetyFilter(Limits())

    def test_wrong_frame_is_refused(self):
        verdict = self.filter.check(action(dy=0.005), context(frame='base_link'))
        self.assertEqual(verdict.code, Code.FRAME)

    def test_wrong_action_space_is_refused(self):
        verdict = self.filter.check(action(dy=0.005), context(action_space_id='other_v2'))
        self.assertEqual(verdict.code, Code.SPACE)

    def test_frame_check_names_both_sides(self):
        verdict = self.filter.check(action(), context(frame='base_link'))
        self.assertIn(ACTION_FRAME, verdict.reason)
        self.assertIn('base_link', verdict.reason)


class FreshnessTest(unittest.TestCase):
    def setUp(self):
        self.filter = SafetyFilter(Limits())

    def test_expired_chunk_is_refused(self):
        verdict = self.filter.check(action(dy=0.005),
                                    context(chunk_stamp=98.0, chunk_validity_s=1.0))
        self.assertEqual(verdict.code, Code.EXPIRED)

    def test_chunk_validity_overrides_a_generous_platform_cap(self):
        verdict = self.filter.check(action(dy=0.005),
                                    context(chunk_stamp=99.5, chunk_validity_s=0.1))
        self.assertEqual(verdict.code, Code.EXPIRED)

    def test_stale_observation_is_refused(self):
        verdict = self.filter.check(action(dy=0.005), context(observation_stamp=90.0))
        self.assertEqual(verdict.code, Code.STALE)

    def test_small_negative_skew_is_tolerated(self):
        """A plan timestamped marginally ahead of the judging clock is normal."""
        verdict = self.filter.check(action(dy=0.005),
                                    context(chunk_stamp=100.2, observation_stamp=100.2,
                                            chunk_validity_s=1.0))
        self.assertTrue(verdict.accepted, verdict.reason)

    def test_absurd_future_stamp_is_refused(self):
        verdict = self.filter.check(action(dy=0.005),
                                    context(chunk_stamp=1000.0, observation_stamp=1000.0,
                                            chunk_validity_s=1.0))
        self.assertEqual(verdict.code, Code.EXPIRED)


class RealizabilityTest(unittest.TestCase):
    def test_commanded_roll_is_refused_by_default(self):
        filter_ = SafetyFilter(Limits())
        verdict = filter_.check(action(dry=0.01), context())
        self.assertEqual(verdict.code, Code.REALIZABLE)
        self.assertIn('dry', verdict.reason)

    def test_commanded_yaw_is_refused(self):
        filter_ = SafetyFilter(Limits())
        verdict = filter_.check(action(drz=0.01), context())
        self.assertEqual(verdict.code, Code.REALIZABLE)

    def test_projection_drops_the_component_and_says_so(self):
        filter_ = SafetyFilter(Limits(), allow_projection=True)
        verdict = filter_.check(action(dy=0.005, dry=0.01, drz=0.02), context())
        self.assertTrue(verdict.accepted, verdict.reason)
        self.assertEqual(verdict.action[4], 0.0)
        self.assertEqual(verdict.action[5], 0.0)
        self.assertEqual(verdict.action[1], 0.005)
        self.assertEqual(set(verdict.dropped), {'dry', 'drz'})

    def test_zero_roll_is_not_a_violation(self):
        filter_ = SafetyFilter(Limits())
        verdict = filter_.check(action(dry=0.0), context())
        self.assertTrue(verdict.accepted, verdict.reason)

    def test_numeric_noise_is_not_treated_as_a_command(self):
        filter_ = SafetyFilter(Limits())
        verdict = filter_.check(action(dry=1e-12), context())
        self.assertTrue(verdict.accepted, verdict.reason)


class EnvelopeTest(unittest.TestCase):
    def setUp(self):
        self.limits = Limits(max_step_m=0.010, max_step_rad=math.radians(5.0))
        self.filter = SafetyFilter(self.limits)

    def test_oversized_step_is_scaled_not_rejected(self):
        verdict = self.filter.check(action(dy=0.020), context())
        self.assertTrue(verdict.accepted, verdict.reason)
        self.assertAlmostEqual(verdict.scale, 0.5, places=6)
        self.assertAlmostEqual(verdict.action[1], 0.010, places=9)

    def test_scaling_preserves_direction(self):
        verdict = self.filter.check(action(dx=0.012, dy=0.016), context())
        self.assertTrue(verdict.accepted, verdict.reason)
        self.assertAlmostEqual(verdict.scale, 0.5, places=6)
        self.assertAlmostEqual(verdict.action[0] / verdict.action[1], 0.012 / 0.016, places=9)

    def test_scale_below_floor_is_rejected_rather_than_crawled(self):
        """A 10x overshoot means the policy's timing is wrong, not that it is fast."""
        verdict = self.filter.check(action(dy=0.100), context())
        self.assertEqual(verdict.code, Code.SPEED)

    def test_rotation_is_limited_on_its_own_axis(self):
        verdict = self.filter.check(action(drx=math.radians(10.0)), context())
        self.assertTrue(verdict.accepted, verdict.reason)
        self.assertAlmostEqual(verdict.action[3], math.radians(5.0), places=9)

    def test_the_tighter_of_position_and_rotation_wins(self):
        verdict = self.filter.check(action(dy=0.020, drx=math.radians(2.0)), context())
        self.assertAlmostEqual(verdict.scale, 0.5, places=6)
        self.assertAlmostEqual(verdict.action[3], math.radians(1.0), places=9)

    def test_unreachable_box_is_rejected(self):
        verdict = self.filter.check(action(dy=0.200), context())
        self.assertIn(verdict.code, (Code.SPEED, Code.WORKSPACE))

    def test_target_outside_envelope_is_rejected_even_when_small(self):
        workspace = Workspace(x=(-0.05, 0.05), y=(0.10, 0.20), z=(0.15, 0.25))
        filter_ = SafetyFilter(Limits(workspace=workspace))
        verdict = filter_.check(action(dx=0.004), context(tcp_position=(0.049, 0.15, 0.20)))
        self.assertEqual(verdict.code, Code.WORKSPACE)

    def test_gripper_target_outside_unit_range_is_rejected(self):
        verdict = self.filter.check(action(gripper=1.4), context())
        self.assertEqual(verdict.code, Code.GRIPPER)

    def test_gripper_jump_is_rejected(self):
        verdict = self.filter.check(action(gripper=0.95), context(gripper_opening=0.1))
        self.assertEqual(verdict.code, Code.GRIPPER_STEP)

    def test_gripper_is_not_scaled_with_the_arm(self):
        verdict = self.filter.check(action(dy=0.020, gripper=0.6), context())
        self.assertTrue(verdict.accepted, verdict.reason)
        self.assertAlmostEqual(verdict.action[6], 0.6, places=9)


class MalformedInputTest(unittest.TestCase):
    def setUp(self):
        self.filter = SafetyFilter(Limits())

    def test_wrong_length_is_refused(self):
        verdict = self.filter.check([0, 0, 0], context())
        self.assertEqual(verdict.code, Code.DIMENSION)

    def test_nan_is_refused(self):
        verdict = self.filter.check(action(dy=float('nan')), context())
        self.assertEqual(verdict.code, Code.NONFINITE)

    def test_infinity_is_refused(self):
        verdict = self.filter.check(action(dz=float('inf')), context())
        self.assertEqual(verdict.code, Code.NONFINITE)

    def test_non_numeric_is_refused_without_raising(self):
        verdict = self.filter.check(['a', 'b', 'c', 'd', 'e', 'f', 'g'], context())
        self.assertEqual(verdict.code, Code.DIMENSION)


class LimitsValidationTest(unittest.TestCase):
    def test_control_period_below_one_bus_move_is_refused(self):
        """20 Hz is in the design document and cannot be executed by this arm."""
        with self.assertRaises(ValueError):
            Limits(control_dt=0.05)

    def test_implied_speed_is_reported(self):
        limits = Limits(max_step_m=0.010, control_dt=0.5)
        self.assertAlmostEqual(limits.max_speed_mps, 0.02)

    def test_inconsistent_workspace_is_refused(self):
        with self.assertRaises(ValueError):
            Workspace(max_radius=0.02, min_radius=0.05)

    def test_describe_covers_every_limit(self):
        described = SafetyFilter(Limits()).describe()
        for key in ('action_space_id', 'action_frame', 'realizable_mask',
                    'max_step_m', 'workspace', 'min_scale'):
            self.assertIn(key, described)


if __name__ == '__main__':
    unittest.main()
