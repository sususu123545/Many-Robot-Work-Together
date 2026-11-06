#!/usr/bin/env python3
"""Offline tests for the rotation gate (no ROS, no car).

Run: python tools/test_lidar_rotation_gate.py
"""
import math
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import scan_clean


def ring(points):
    """Build a full 455-point scan that returns `points` and null elsewhere."""
    inc = math.radians(360.0 / 455.0)
    ranges = [None] * 455
    for angle_deg, distance in points:
        index = int(round(math.radians(angle_deg) / inc)) % 455
        ranges[index] = distance
    return ranges, inc


def scan_of(points):
    ranges, inc = ring(points)
    return scan_clean.clean_scan(ranges, inc)


class TestSelfRadius(unittest.TestCase):

    def test_chassis_return_inside_self_radius_is_ignored(self):
        # The exact cluster that blocked every rotation on 2026-11-06:
        # 0-12 deg at 0.133-0.181 m, which turned out to be the wheel-tip corner.
        pts = [(a, 0.133 + 0.004 * a) for a in range(0, 13)]
        _ranges, _inc = ring(pts)
        cleaned = scan_clean.clean_scan(_ranges, _inc)
        ok, offenders = scan_clean.rotation_clearance(cleaned)
        self.assertTrue(ok, 'own chassis must not block a rotation')
        self.assertEqual(offenders, [])

    def test_real_obstacle_still_blocks(self):
        # 0.24 m is outside the skin (0.182) and inside the sweep radius (0.25).
        cleaned = scan_of([(90.0, 0.24)])
        ok, offenders = scan_clean.rotation_clearance(cleaned)
        self.assertFalse(ok)
        self.assertEqual(len(offenders), 1)

    def test_obstacle_just_outside_the_radius_is_allowed(self):
        cleaned = scan_of([(90.0, 0.26)])
        ok, _ = scan_clean.rotation_clearance(cleaned)
        self.assertTrue(ok)

    def test_self_radius_bounds_are_respected(self):
        # exactly at the near edge counts; a hair below does not.
        cleaned = scan_of([(90.0, scan_clean.SELF_RADIUS_M)])
        ok, _ = scan_clean.rotation_clearance(cleaned)
        self.assertFalse(ok)
        cleaned = scan_of([(90.0, scan_clean.SELF_RADIUS_M - 0.002)])
        ok, _ = scan_clean.rotation_clearance(cleaned)
        self.assertTrue(ok)

    def test_agreed_needs_two_votes(self):
        blocked = scan_of([(90.0, 0.22)])
        clear = scan_of([])
        ok, _ = scan_clean.rotation_clearance_agreed([blocked, clear, clear])
        self.assertTrue(ok, 'one noisy frame must not decide')
        ok, _ = scan_clean.rotation_clearance_agreed([blocked, blocked, clear])
        self.assertFalse(ok, 'a real object shows up in every frame')

    def test_agreed_propagates_a_custom_self_radius(self):
        cleaned = scan_of([(90.0, 0.22)])
        ok, _ = scan_clean.rotation_clearance_agreed(
            [cleaned, cleaned], self_radius_m=0.24)
        self.assertTrue(ok, 'the custom near edge must reach rotation_clearance')

    def test_min_return_unchanged_so_proximity_paths_keep_seeing_close(self):
        # SELF_RADIUS_M deliberately does NOT raise MIN_RETURN_M: reverse and
        # push-detection still need the 0.13-0.19 m band.
        self.assertEqual(scan_clean.MIN_RETURN_M, 0.13)
        self.assertGreater(scan_clean.SELF_RADIUS_M, scan_clean.MIN_RETURN_M)
        # 100 deg sits outside every SELF_SECTORS entry, so this isolates the
        # min-return floor from the sector table.
        cleaned = scan_of([(100.0, 0.16)])
        self.assertTrue(cleaned, 'clean_scan must keep a 0.16 m return')


if __name__ == '__main__':
    unittest.main(verbosity=2)
