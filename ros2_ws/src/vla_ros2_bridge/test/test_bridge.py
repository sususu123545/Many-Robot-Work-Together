"""Tests for the goal contract: observation + policy output -> chunk."""

import math
import unittest

from vla_controller import spec
from vla_controller.normalization import NormalizationStats
from vla_controller.safety import Limits
from vla_ros2_bridge.bridge import BridgeError, BridgePlanner, Code
from vla_ros2_bridge.policy import (POLICY_OUTPUT_NORMALIZED,
                                    POLICY_OUTPUT_PHYSICAL, PolicyOutput,
                                    ScriptedPolicy)
from vla_ros2_bridge.observation import (ObservationBuilder, SensorSnapshot)

TICKS = {'joint1': 495, 'joint2': 644, 'joint3': 696, 'joint4': 166,
         'joint5': 499, 'r_joint': 339}
NOW = 100.4


def fake_pose(_ticks):
    return {'position': (0.002, 0.127, 0.232),
            'quaternion': (0.0, 0.0, 0.0, 1.0)}


def observation(stamp=100.0):
    snap = SensorSnapshot(dict(TICKS), stamp, b'\xff\xd8img', 'jpeg',
                          stamp + 0.02, 'pick up the red ball')
    return ObservationBuilder(pose_provider=fake_pose).build(snap)


def planner(**overrides):
    values = dict(limits=Limits(), execute_steps=4)
    values.update(overrides)
    return BridgePlanner(**values)


def physical(actions=None, **overrides):
    values = dict(actions=actions or [(0, 0.01, 0, 0, 0, 0, 0.5)] * 8,
                  policy_id='p0', control_dt=0.35, validity_s=1.0,
                  output_kind=POLICY_OUTPUT_PHYSICAL)
    values.update(overrides)
    return PolicyOutput(**values)


def good_stats(space=spec.ACTION_SPACE_ID):
    return NormalizationStats(space, 'mean_std', (0.0,) * 7,
                              (0.01, 0.01, 0.01, 0.05, 0.05, 0.05, 0.5))


class TestHappyPath(unittest.TestCase):

    def test_scripted_policy_closes_the_head(self):
        obs = observation()
        out = ScriptedPolicy(horizon=16, control_dt=0.35, validity_s=1.0).predict(obs)
        goal = planner().plan(obs, out, NOW)
        self.assertEqual(goal.action_count, 16)
        kwargs = goal.as_goal_kwargs()
        self.assertIn('observation', kwargs)
        self.assertIn('chunk', kwargs)

        chunk = kwargs['chunk']
        self.assertEqual(chunk['action_frame'], spec.ACTION_FRAME)
        self.assertEqual(chunk['action_space_id'], spec.ACTION_SPACE_ID)
        self.assertEqual(chunk['policy_id'], 'scripted_tcp_walk_v0')
        self.assertEqual(chunk['realizable_mask'], list(spec.REALIZABLE_MASK))
        self.assertEqual(chunk['control_dt'], 0.35)
        self.assertEqual(chunk['validity'], 1.0)
        self.assertEqual(chunk['header']['frame_id'], spec.ACTION_FRAME)
        self.assertEqual(list(chunk['actions'][0]), list(spec.ACTION_COMPONENTS))

    def test_chunk_and_observation_keep_their_own_stamps(self):
        goal = planner().plan(observation(100.0), physical(), NOW)
        self.assertEqual(goal.chunk['header']['stamp'], NOW)
        self.assertEqual(goal.observation['header']['stamp'], 100.0)

    def test_actions_are_dictionaries_in_canonical_order(self):
        goal = planner().plan(observation(), physical(), NOW)
        action = goal.chunk['actions'][0]
        self.assertEqual(tuple(action.keys()), spec.ACTION_COMPONENTS)
        self.assertAlmostEqual(action['dy'], 0.01)

    def test_normalized_output_is_denormalized_with_matching_stats(self):
        out = PolicyOutput([(0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0)] * 8,
                           'remote_v1', 0.35, 1.0, POLICY_OUTPUT_NORMALIZED)
        goal = planner(stats=good_stats()).plan(observation(), out, NOW)
        # centre 0, dy scale 0.01 -> 0.01 m; gripper scale 0.5 -> 0.0 opening
        self.assertAlmostEqual(goal.chunk['actions'][0]['dy'], 0.01, places=9)
        self.assertAlmostEqual(goal.chunk['actions'][0]['gripper'], 0.0, places=9)


class TestRefusals(unittest.TestCase):

    def assert_code(self, code, **plan_overrides):
        with self.assertRaises(BridgeError) as caught:
            planner(**plan_overrides).plan(observation(), physical(), NOW)
        self.assertEqual(caught.exception.code, code)

    def test_stale_observation(self):
        with self.assertRaises(BridgeError) as caught:
            planner().plan(observation(), physical(), 102.0)
        self.assertEqual(caught.exception.code, Code.STALE)

    def test_future_observation(self):
        with self.assertRaises(BridgeError) as caught:
            planner().plan(observation(200.0), physical(), NOW)
        self.assertEqual(caught.exception.code, Code.FUTURE)

    def test_control_dt_must_match_the_executor(self):
        out = physical(control_dt=0.5)
        with self.assertRaises(BridgeError) as caught:
            planner().plan(observation(), out, NOW)
        self.assertEqual(caught.exception.code, Code.CONTROL_DT)

    def test_control_dt_below_bus_minimum(self):
        # Limits() refuses such a value at construction, so use a stub to reach
        # the bridge's own second line of defence.
        class StubLimits:
            control_dt = 0.1
            max_observation_age_s = 0.75
            max_chunk_age_s = 1.5

        out = physical(control_dt=0.1)
        with self.assertRaises(BridgeError) as caught:
            planner(limits=StubLimits()).plan(observation(), out, NOW)
        self.assertEqual(caught.exception.code, Code.CONTROL_DT)

    def test_validity_cannot_outlive_the_executor_window(self):
        out = physical(validity_s=3.0)
        with self.assertRaises(BridgeError) as caught:
            planner().plan(observation(), out, NOW)
        self.assertEqual(caught.exception.code, Code.VALIDITY)

    def test_short_horizon(self):
        with self.assertRaises(BridgeError) as caught:
            planner().plan(observation(), physical([(0,) * 7] * 2), NOW)
        self.assertEqual(caught.exception.code, Code.HORIZON)

    def test_normalized_without_stats_is_a_hard_refusal(self):
        out = PolicyOutput([(0.0,) * 7] * 8, 'remote_v1', 0.35, 1.0,
                           POLICY_OUTPUT_NORMALIZED)
        with self.assertRaises(BridgeError) as caught:
            planner().plan(observation(), out, NOW)
        self.assertEqual(caught.exception.code, Code.STATS)

    def test_stats_from_another_action_space(self):
        out = PolicyOutput([(0.0,) * 7] * 8, 'remote_v1', 0.35, 1.0,
                           POLICY_OUTPUT_NORMALIZED)
        with self.assertRaises(BridgeError) as caught:
            planner(stats=good_stats('some_other_space')).plan(observation(), out, NOW)
        self.assertEqual(caught.exception.code, Code.STATS_SPACE)

    def test_unrealizable_rotation(self):
        out = physical([(0, 0.01, 0, 0, 0.05, 0, 0.5)] * 8)
        with self.assertRaises(BridgeError) as caught:
            planner().plan(observation(), out, NOW)
        self.assertEqual(caught.exception.code, Code.UNREALIZABLE)

    def test_projection_is_opt_in(self):
        out = physical([(0, 0.01, 0, 0, 0.05, 0, 0.5)] * 8)
        goal = planner(allow_projection=True).plan(observation(), out, NOW)
        self.assertAlmostEqual(goal.chunk['actions'][0]['dry'], 0.05)

    def test_gripper_out_of_range(self):
        out = physical([(0, 0.01, 0, 0, 0, 0, 1.2)] * 8)
        with self.assertRaises(BridgeError) as caught:
            planner().plan(observation(), out, NOW)
        self.assertEqual(caught.exception.code, Code.GRIPPER)

    def test_broken_stats_cannot_even_be_constructed(self):
        # NormalizationStats refuses non-finite centres before the bridge sees
        # them, so the bridge's post-de-normalisation finiteness check is a
        # defensive second line, not a reachable branch.  Assert the upstream
        # guarantee instead of pretending to test unreachable code.
        from vla_controller.normalization import NormalizationError

        with self.assertRaises(NormalizationError):
            NormalizationStats(spec.ACTION_SPACE_ID, 'mean_std',
                               (float('nan'),) * 7, (0.1,) * 7)

    def test_wrong_input_type(self):
        with self.assertRaises(BridgeError) as caught:
            planner().plan(observation(), [(0,) * 7], NOW)
        self.assertEqual(caught.exception.code, Code.TYPE)


class TestDescribe(unittest.TestCase):

    def test_describe_exposes_the_single_source_of_truth(self):
        info = planner().describe()
        self.assertEqual(info['control_dt'], Limits().control_dt)
        self.assertEqual(info['execute_steps'], 4)
        self.assertIsNone(info['stats'])
        self.assertFalse(info['allow_projection'])


if __name__ == '__main__':
    unittest.main()
