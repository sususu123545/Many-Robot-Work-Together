#!/usr/bin/env python3
"""Offline tests for the --map-frame wiring in find_and_grasp.py.

Run: python tools/test_map_frame_wiring.py
"""
import os
import subprocess
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, 'perception'))

import find_and_grasp as fg                                     # noqa: E402

FOUND_OUTPUT = """
sweeping j1 by +/-200 ticks
FOUND at bearing 90 deg (relative to the start heading)
  ball [0.0493, 0.5910, -0.0798]  -> 0.591 m ahead, 28.4 mm tall
"""
POSE = {'x': 0.0, 'y': 0.0, 'yaw': -2.73}


class TestMapFrameWiring(unittest.TestCase):

    def test_default_is_odom_so_existing_calls_are_unchanged(self):
        sighting = fg.sighting_from(FOUND_OUTPUT, POSE)
        self.assertEqual(sighting['frame'], 'odom')
        self.assertAlmostEqual(sighting['pose']['yaw'], 87.27, places=2)
        self.assertAlmostEqual(sighting['range'], 0.591)

    def test_map_frame_is_propagated(self):
        sighting = fg.sighting_from(FOUND_OUTPUT, POSE, 'map')
        self.assertEqual(sighting['frame'], 'map')

    def test_no_pose_means_no_memory_write(self):
        """When /amcl_pose is unavailable, never stamp an odom pose as map."""
        self.assertIsNone(fg.sighting_from(FOUND_OUTPUT, None, 'map'))

    def test_no_found_line_means_no_sighting(self):
        self.assertIsNone(fg.sighting_from('not found in 360 degrees', POSE))

    def test_flag_is_opt_in(self):
        out = subprocess.run(
            [sys.executable, os.path.join(HERE, 'find_and_grasp.py'), '--help'],
            capture_output=True, text=True, timeout=30).stdout
        self.assertIn('--map-frame', out)
        self.assertIn('amcl', out)


if __name__ == '__main__':
    unittest.main(verbosity=2)
