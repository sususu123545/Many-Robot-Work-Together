#!/usr/bin/env python3
"""Offline tests for the arm/rear_camera transforms (no ROS, no car).

Run: python tools/test_arm_tf.py
"""
import math
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, 'perception'))
sys.path.insert(0, os.path.join(REPO, 'ros2_ws', 'src', 'armpi_ros2', 'armpi_ros2'))

import arm_tf
from arm_kinematics import forward_tool_pose_raw

CALIB = os.path.join(HERE, 'runtime', 'calib', 'rear_camera_hand_eye.json')

# A pose the arm actually stood in on 2026-11-06 (the reset pose).
RAW = {'joint1': 478, 'joint2': 687, 'joint3': 916, 'joint4': 194}


class TestArmTf(unittest.TestCase):

    def setUp(self):
        self.hand_eye = arm_tf.load_hand_eye(CALIB)

    def test_quaternion_round_trip(self):
        # A real rotation matrix from the FK, not a rounded literal: rounding to
        # 4 decimals destroys orthonormality and the round trip then fails by
        # 4e-5 for reasons that have nothing to do with the conversion.
        matrix = forward_tool_pose_raw(RAW)['rotation_matrix']
        quaternion = arm_tf.quaternion_from_matrix(matrix)
        back = arm_tf.matrix_from_quaternion(*quaternion)
        for i in range(3):
            for j in range(3):
                self.assertAlmostEqual(matrix[i][j], back[i][j], places=9)
        self.assertAlmostEqual(sum(v * v for v in quaternion), 1.0, places=9)

    def test_camera_matches_the_hand_eye_chain(self):
        """The published camera pose must reproduce ball_in_arm_base exactly."""
        translation, quaternion = arm_tf.camera_pose_in_arm_base(
            RAW, self.hand_eye, forward_tool_pose_raw)
        # Independent recomputation with the same arithmetic ball_in_arm_base
        # uses: camera -> tool -> arm_base on the point (0, 0, 0).
        rotation_tool_camera, translation_tool_camera, _ = self.hand_eye
        tool = forward_tool_pose_raw(RAW)
        expected = tuple(
            sum(tool['rotation_matrix'][i][k] * translation_tool_camera[k]
                for k in range(3)) + tool['translation_m'][i] for i in range(3))
        for i in range(3):
            self.assertAlmostEqual(translation[i], expected[i], places=9)
        self.assertAlmostEqual(sum(v * v for v in quaternion), 1.0, places=9)

    def test_camera_sits_near_the_wrist_not_the_fingertip(self):
        """Sanity: the camera is ~19 cm from the arm base, not 36 cm away."""
        translation, _ = arm_tf.camera_pose_in_arm_base(
            RAW, self.hand_eye, forward_tool_pose_raw)
        radius = math.sqrt(sum(v * v for v in translation))
        self.assertGreater(radius, 0.15)
        self.assertLess(radius, 0.25)

    def test_urdf_camera_joint_still_disagrees_by_about_two_centimetres(self):
        """Pins the known conflict so neither side drifts silently.

        If someone fixes the URDF, this test fails and that is correct: it means
        the two sources finally agree (or moved), and the note in arm_tf.py plus
        this assertion should be reviewed rather than left to rot.
        """
        hand_eye_offset = arm_tf.camera_offset_in_link4(self.hand_eye)
        urdf = arm_tf.URDF_CAMERA_JOINT_M
        delta = tuple(hand_eye_offset[i] - urdf[i] for i in range(3))
        self.assertAlmostEqual(delta[2], 0.0218, delta=0.005,
                               msg='camera z disagreement changed: %r' % (delta,))
        self.assertLess(abs(delta[0]), 0.010)
        self.assertLess(abs(delta[1]), 0.010)

    def test_arm_base_offset_matches_the_urdf(self):
        self.assertEqual(arm_tf.BASE_LINK_TO_ARM_BASE_M, (0.0, 0.062, 0.094))

    def test_hand_eye_calibration_was_accepted(self):
        _, _, data = self.hand_eye
        self.assertTrue(data['quality']['accepted'])
        self.assertEqual(data['camera_frame'], 'rear_camera')
        self.assertEqual(data['arm_base_frame'], 'arm_base')
        self.assertLess(data['quality']['translation_rms_m'], 0.005)


if __name__ == '__main__':
    unittest.main(verbosity=2)
