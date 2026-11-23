#!/usr/bin/env python3
"""Pin the planning and scoring rules of the pose-sweep capture.

Two of these decide whether a nine-group session is usable: how an absolute move
is split into legal single commands, and how a frame is scored against the arm
status and the transform.  Both are easy to get subtly wrong and impossible to
notice from the output alone, so they are tested here.

Pure Python: no ROS, no bridge, no robot.

Usage:  python3 -B tools/test_capture_pose_sweep.py
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import capture_pose_sweep as sweep  # noqa: E402


class TestTickMath(unittest.TestCase):

    def test_fifteen_degrees_is_about_sixty_two_ticks(self):
        # 15 deg / 0.24 deg-per-tick = 62.5, and the plan rounds to 62.
        self.assertEqual(sweep.deg_to_ticks(15.0), 62)

    def test_one_tick_is_a_quarter_degree(self):
        self.assertAlmostEqual(sweep.ticks_to_deg(1), 0.24, places=6)

    def test_plan_span_matches_what_is_achieved(self):
        plan = sweep.build_plan(206, 15.0, (30, 40, 48))
        self.assertAlmostEqual(plan['span_deg_planned'], 29.76, places=2)


class TestSplitSteps(unittest.TestCase):

    def test_short_move_is_one_step(self):
        self.assertEqual(sweep.split_steps(20), [20])

    def test_move_is_capped_at_the_servo_limit(self):
        self.assertEqual(sweep.split_steps(250), [100, 100, 50])

    def test_negative_move_keeps_its_sign(self):
        self.assertEqual(sweep.split_steps(-120), [-100, -20])

    def test_zero_moves_nothing(self):
        self.assertEqual(sweep.split_steps(0), [])

    def test_no_step_ever_exceeds_the_cap(self):
        for delta in (1, 99, 100, 101, 999, -999):
            for step in sweep.split_steps(delta):
                self.assertLessEqual(abs(step), sweep.MAX_STEP_TICKS)
                self.assertNotEqual(step, 0)
            self.assertEqual(sum(sweep.split_steps(delta)), delta)


class TestClamp(unittest.TestCase):

    def test_joint2_respects_its_narrow_window(self):
        self.assertEqual(sweep.clamp_tick('joint2', 100)[0], 125)
        self.assertEqual(sweep.clamp_tick('joint2', 900)[0], 875)

    def test_gripper_window_ends_at_700(self):
        self.assertEqual(sweep.clamp_tick('r_joint', 750)[0], 700)

    def test_in_range_value_is_untouched(self):
        tick, clamped = sweep.clamp_tick('joint4', 206)
        self.assertEqual((tick, clamped), (206, False))


class TestPlan(unittest.TestCase):

    def test_one_centre_and_two_extremes(self):
        plan = sweep.build_plan(206, 15.0, (30, 40, 48))
        self.assertEqual([g['joint4_label'] for g in plan['groups']],
                         ['minus'] * 3 + ['centre'] * 3 + ['plus'] * 3)
        self.assertEqual(plan['groups'][3]['joint4_requested_tick'], 206)

    def test_nine_groups_and_three_openings(self):
        plan = sweep.build_plan(206)
        self.assertEqual(len(plan['groups']), 9)
        self.assertEqual([g['opening_target_mm'] for g in plan['groups'][:3]],
                         [48, 40, 30])

    def test_plan_carries_no_gripper_tick(self):
        # The tick that produces a given opening is measured on the bench, not
        # planned: a plan that invented one would be silently wrong.
        plan = sweep.build_plan(206)
        self.assertNotIn('r_joint_tick', plan['groups'][0])

    def test_low_theta0_clamps_the_minus_arm(self):
        plan = sweep.build_plan(50, 15.0)
        minus = plan['groups'][0]
        self.assertEqual(minus['joint4_requested_tick'], 0)
        self.assertTrue(minus['joint4_clamped'])

    def test_rejects_zero_span(self):
        with self.assertRaises(ValueError):
            sweep.build_plan(206, 0.0)

    def test_rejects_duplicate_openings(self):
        with self.assertRaises(ValueError):
            sweep.build_plan(206, 15.0, (30, 30))


class TestNearestByStamp(unittest.TestCase):
    """Stamps are robot-clock nanoseconds, around 1.79e18 in practice."""

    BASE = 1_795_415_000_000_000_000

    def setUp(self):
        # Four samples 500 ms apart, matching the 2 Hz arm status stream.
        self.samples = [{'stamp_ns': self.BASE + n * 500_000_000} for n in range(4)]

    def test_picks_the_closest_sample(self):
        index, _ = sweep.nearest_by_stamp(self.samples, self.BASE + 520_000_000)
        self.assertEqual(index, 1)

    def test_skew_is_reported_in_milliseconds(self):
        # 12 ms after sample 1 must read as about 12 ms, not 12 ns.
        index, skew = sweep.nearest_by_stamp(self.samples, self.BASE + 500_000_000 + 12_000_000)
        self.assertEqual(index, 1)
        self.assertAlmostEqual(skew, 12.0, places=6)

    def test_quarter_second_off_midpoint_still_resolves(self):
        index, skew = sweep.nearest_by_stamp(self.samples, self.BASE + 250_000_000)
        self.assertIn(index, (0, 1))
        self.assertAlmostEqual(skew, 250.0, places=6)

    def test_empty_input(self):
        self.assertEqual(sweep.nearest_by_stamp([], 5), (None, None))

    def test_samples_without_stamps_are_skipped(self):
        index, skew = sweep.nearest_by_stamp(
            [{'stamp_ns': None}, {'stamp_ns': self.BASE}], self.BASE + 1_000_000)
        self.assertEqual(index, 1)
        self.assertAlmostEqual(skew, 1.0, places=6)


class TestGroupScoring(unittest.TestCase):

    def test_group_within_target_passes(self):
        verdict = sweep.judge_group([10.0, 20.0, 30.0], 50.0)
        self.assertTrue(verdict['skew_ok'])
        self.assertAlmostEqual(verdict['worst_skew_ms'], 30.0)

    def test_one_bad_frame_fails_the_whole_group(self):
        verdict = sweep.judge_group([10.0, 80.0, 20.0], 50.0)
        self.assertFalse(verdict['skew_ok'])
        self.assertAlmostEqual(verdict['worst_skew_ms'], 80.0)

    def test_no_frames_is_a_failure_not_an_absence(self):
        verdict = sweep.judge_group([], 50.0)
        self.assertFalse(verdict['skew_ok'])
        self.assertEqual(verdict['reason'], 'no frames captured')


class TestSummarise(unittest.TestCase):

    def record(self, index, joint4, mm, skews, tick=200):
        return {'index': index, 'joint4_tick': joint4, 'measured_mm': mm,
                'r_joint_tick': tick, 'skews_ms': skews}

    def test_clean_set_reports_the_achieved_span(self):
        records = [self.record(1, 144, 48.0, [5.0]),
                   self.record(2, 206, 40.0, [6.0]),
                   self.record(3, 268, 30.0, [7.0])]
        result = sweep.summarise(records, 50.0)
        self.assertTrue(result['clean'])
        self.assertEqual(result['distinct_joint4_ticks'], [144, 206, 268])
        self.assertEqual(result['joint4_step_ticks'], [62, 62])
        self.assertAlmostEqual(result['span_deg_achieved'], 29.76, places=2)

    def test_missing_caliper_reading_is_flagged(self):
        records = [self.record(1, 206, None, [5.0])]
        result = sweep.summarise(records, 50.0)
        self.assertFalse(result['clean'])
        self.assertEqual(result['problems'][0]['reason'], 'missing caliper reading')

    def test_missing_gripper_tick_is_flagged(self):
        records = [self.record(1, 206, 40.0, [5.0], tick=None)]
        result = sweep.summarise(records, 50.0)
        self.assertFalse(result['clean'])
        self.assertIn('missing commanded r_joint tick',
                      [p['reason'] for p in result['problems']])

    def test_bad_skew_is_flagged_with_its_index(self):
        records = [self.record(7, 206, 40.0, [200.0])]
        result = sweep.summarise(records, 50.0)
        self.assertEqual(result['problems'][0]['index'], 7)

    def test_openings_are_collected_from_measurements(self):
        records = [self.record(1, 206, 40.2, [1.0]),
                   self.record(2, 206, 29.7, [1.0])]
        result = sweep.summarise(records, 50.0)
        self.assertEqual(result['openings_mm'], [29.7, 40.2])


if __name__ == '__main__':
    unittest.main(verbosity=2)
