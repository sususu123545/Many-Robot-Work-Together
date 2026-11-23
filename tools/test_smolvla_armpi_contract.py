#!/usr/bin/env python3
"""Pin the SmolVLA-to-ArmPi joint contract.

The numbers here are the ones a dataset recording decision depends on, so a
silent change to them would quietly invalidate every recorded episode.  Pure
Python: no ROS 2, no torch, no robot.

Usage:  python3 -B tools/test_smolvla_armpi_contract.py
"""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import smolvla_armpi_contract as contract  # noqa: E402
from vla_controller import spec  # noqa: E402


class TestDecimation(unittest.TestCase):

    def test_thirty_hertz_needs_every_ninth_step(self):
        timing = contract.decimation(30.0, 50)
        self.assertEqual(timing['decimation_factor'], 9)
        self.assertAlmostEqual(timing['execution_hz'], 30.0 / 9, places=6)

    def test_decimation_thins_waypoints_instead_of_slowing_the_plan(self):
        timing = contract.decimation(30.0, 50)
        # 50 steps keep every 9th -> indices 0,9,...,45 -> 6 waypoints spanning
        # 45 policy periods (1.5 s).  Wall time to send 6 commands at 0.3 s is
        # 1.8 s, close to the plan's own horizon.
        self.assertEqual(timing['decimated_steps'], 6)
        self.assertAlmostEqual(timing['covered_intent_s'], 1.5, places=6)
        self.assertAlmostEqual(timing['wall_time_s'], 1.8, places=6)

    def test_naive_streaming_is_the_slow_option(self):
        timing = contract.decimation(30.0, 50)
        self.assertAlmostEqual(timing['naive_stream_duration_s'], 15.0, places=6)
        # The whole point: naive streaming is 9x slower than the plan, which is
        # why it must not be confused with decimating.
        self.assertAlmostEqual(timing['naive_stream_duration_s'] / timing['wall_time_s'],
                               15.0 / 1.8, places=6)

    def test_a_slow_policy_is_left_alone(self):
        timing = contract.decimation(2.0, 50)
        self.assertEqual(timing['decimation_factor'], 1)
        self.assertTrue(timing['fits_one_command_period'])

    def test_policy_already_at_the_bus_floor_needs_no_decimation(self):
        bus_hz = 1.0 / spec.MIN_MOVE_DURATION_S
        timing = contract.decimation(bus_hz, 50)
        self.assertEqual(timing['decimation_factor'], 1)

    def test_rejects_nonsense_rates(self):
        for hz in (0.0, -1.0, float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                contract.decimation(hz, 50)
        for steps in (0, -3, 2.5, '50'):
            with self.assertRaises(ValueError):
                contract.decimation(30.0, steps)


class TestTickBudget(unittest.TestCase):

    def test_one_command_may_travel_a_hundred_ticks(self):
        budget = contract.tick_budget(spec.MIN_MOVE_DURATION_S)
        self.assertEqual(budget['max_ticks'], spec.DEFAULT_MAX_STEP_TICKS)
        self.assertAlmostEqual(budget['max_degrees'], 24.0, places=4)

    def test_the_cap_does_not_grow_with_duration(self):
        short = contract.tick_budget(0.3)
        long = contract.tick_budget(5.0)
        self.assertEqual(short['max_ticks'], long['max_ticks'])
        self.assertLess(long['max_ticks_per_second'], short['max_ticks_per_second'])

    def test_a_five_second_move_is_slower_per_second(self):
        budget = contract.tick_budget(5.0)
        self.assertAlmostEqual(budget['max_degrees_per_second'], 4.8, places=4)


class TestRoundTripThroughTicks(unittest.TestCase):

    def test_joint3_sign_is_applied(self):
        # spec declares joint3 centre 500 with sign -1, so a tick above centre
        # must read as a negative radian and come back to the same tick.
        radians = spec.tick_to_radians('joint3', 907)
        self.assertLess(radians, 0.0)
        _, raw = spec.radians_to_tick('joint3', radians)
        self.assertEqual(raw, 907)

    def test_every_joint_round_trips_across_its_limits(self):
        for name in contract.JOINT_ORDER:
            low, high = spec.joint_limits(name)
            for tick in (low, (low + high) // 2, high):
                radians = spec.tick_to_radians(name, tick)
                _, back = spec.radians_to_tick(name, radians)
                self.assertEqual(back, tick, '%s at %d' % (name, tick))


class TestCheckTargets(unittest.TestCase):

    def state(self, offsets):
        return [spec.tick_to_radians(name, 500 + offsets[name])
                for name in contract.JOINT_ORDER]

    def test_small_move_is_feasible(self):
        start = self.state({name: 0 for name in contract.JOINT_ORDER})
        targets = self.state({name: 10 for name in contract.JOINT_ORDER})
        result = contract.check_targets(targets, start, spec.MIN_MOVE_DURATION_S)
        self.assertTrue(result['feasible'])
        self.assertEqual(result['offenders'], [])
        self.assertEqual(result['max_delta_tick'], 10)

    def test_one_travelling_joint_rejects_the_whole_command(self):
        start = self.state({name: 0 for name in contract.JOINT_ORDER})
        offsets = {name: 0 for name in contract.JOINT_ORDER}
        offsets['joint3'] = -150
        targets = self.state(offsets)
        result = contract.check_targets(targets, start, spec.MIN_MOVE_DURATION_S)
        self.assertFalse(result['feasible'])
        self.assertEqual(result['offenders'][0]['joint'], 'joint3')
        self.assertEqual(result['offenders'][0]['delta_tick'], -150)

    def test_exactly_at_the_cap_is_allowed(self):
        start = self.state({name: 0 for name in contract.JOINT_ORDER})
        targets = self.state({name: spec.DEFAULT_MAX_STEP_TICKS
                              for name in contract.JOINT_ORDER})
        result = contract.check_targets(targets, start, spec.MIN_MOVE_DURATION_S)
        self.assertTrue(result['feasible'])


class TestNormalizer(unittest.TestCase):

    def test_round_trip_is_exact(self):
        samples = [[0.1 * i + 0.01 * j for j in range(6)] for i in range(5)]
        normalizer = contract.Normalizer.fit(samples)
        for row in samples:
            back = normalizer.denormalize(normalizer.normalize(row))
            for a, b in zip(row, back):
                self.assertAlmostEqual(a, b, places=9)

    def test_mean_and_std_match_a_hand_computation(self):
        samples = [[0.0] * 6, [2.0] * 6]
        normalizer = contract.Normalizer.fit(samples)
        for mean, std in zip(normalizer.mean, normalizer.std):
            self.assertAlmostEqual(mean, 1.0, places=9)
            self.assertAlmostEqual(std, 1.0, places=9)

    def test_constant_joint_gets_zero_std_and_is_reported(self):
        # Only joint1 varies; joints 2..6 must come out with zero std.
        samples = [[float(i), 0.0, 0.0, 0.0, 0.0, 0.0] for i in range(4)]
        normalizer = contract.Normalizer.fit(samples)
        self.assertGreater(normalizer.std[0], 0.0)
        for index in range(1, 6):
            self.assertEqual(normalizer.std[index], 0.0)
        report = normalizer.clamp_report(samples[0])
        flagged = {entry['joint'] for entry in report['outside']}
        self.assertIn('joint2', flagged)
        self.assertNotIn('joint1', flagged)

    def test_zero_std_does_not_divide_by_zero(self):
        samples = [[1.0] * 6]
        normalizer = contract.Normalizer.fit(samples)
        normalized = normalizer.normalize(samples[0])
        self.assertTrue(all(math.isfinite(v) for v in normalized))

    def test_rejects_wrong_dimensions(self):
        with self.assertRaises(ValueError):
            contract.Normalizer.fit([[0.0] * 5])
        with self.assertRaises(ValueError):
            contract.Normalizer([0.0] * 5, [1.0] * 6)

    def test_rejects_empty_input(self):
        with self.assertRaises(ValueError):
            contract.Normalizer.fit([])


class TestJointOrder(unittest.TestCase):

    def test_order_matches_the_joint_states_convention(self):
        self.assertEqual(contract.JOINT_ORDER,
                         ('joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'r_joint'))

    def test_missing_joint_is_rejected(self):
        with self.assertRaises(ValueError):
            contract._vector_from_names({'joint1': 0.0})

    def test_non_finite_value_is_rejected(self):
        with self.assertRaises(ValueError):
            contract._vector_from_names([0.0, 0.0, float('nan'), 0.0, 0.0, 0.0])


class TestContractPayload(unittest.TestCase):

    def test_contract_states_the_recording_rate(self):
        payload = contract.contract()
        self.assertAlmostEqual(payload['recording_fps_required'], 30.0 / 9, places=6)
        self.assertAlmostEqual(payload['bus']['max_command_hz'],
                               1.0 / spec.MIN_MOVE_DURATION_S, places=6)

    def test_contract_payload_is_json_serialisable(self):
        import json
        json.dumps(contract.contract())


if __name__ == '__main__':
    unittest.main(verbosity=2)
