#!/usr/bin/env python3
"""Verify that descent steps shrink as the tool approaches the floor guard.

Pure Python, no ROS 2 and no robot: forward kinematics is stubbed with a chosen
tool height, which is the only input descent_step_cap() actually uses.

Usage:  python3 -B tools/test_descent_step_cap.py
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import grasp_sequence as gs  # noqa: E402


class TestDescentStepCap(unittest.TestCase):

    def setUp(self):
        self.original = gs.forward_tool_pose_raw
        self.height = 0.0
        gs.forward_tool_pose_raw = lambda _ticks: {
            'translation_m': (0.0, 0.2, self.height)}

    def tearDown(self):
        gs.forward_tool_pose_raw = self.original

    def cap(self, height, base):
        self.height = height
        return gs.descent_step_cap({}, base)

    def test_high_above_floor_keeps_the_full_step(self):
        # Boot pose: tool ~+0.09 m, ~0.20 m of room -> the cap must not bind.
        # base 40 is below the unconstrained cap, so it must pass through
        self.assertEqual(self.cap(0.09, 30), 30)

    def test_mid_descent_is_limited(self):
        # 99 mm of room at 0.20 -> 19 ticks
        self.assertEqual(self.cap(0.020, 60), 19)

    def test_near_the_guard_converges_to_one_deadband_step(self):
        # Target for a ball on the floor is roughly 12 mm above the guard line.
        self.assertEqual(self.cap(-0.067, 60), gs.MIN_JOG_TICKS)
        self.assertEqual(self.cap(-0.067, 25), gs.MIN_JOG_TICKS)

    def test_below_the_guard_never_goes_negative_or_zero(self):
        for height in (-0.079, -0.090, -0.120):
            self.assertGreaterEqual(self.cap(height, 60), gs.MIN_JOG_TICKS)

    def test_monotonic_with_height(self):
        heights = (0.09, 0.02, -0.02, -0.05, -0.067, -0.079)
        caps = [self.cap(height, 30) for height in heights]
        self.assertEqual(caps, sorted(caps, reverse=True))
        self.assertEqual(caps[0], 30)

    def test_small_base_is_never_increased(self):
        self.assertEqual(self.cap(0.09, 12), 12)
        self.assertEqual(self.cap(0.09, 25), 25)


if __name__ == '__main__':
    unittest.main(verbosity=2)
