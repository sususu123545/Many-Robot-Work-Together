"""Tests for the observation contract (no ROS 2, no robot)."""

import math
import unittest

from vla_controller import spec
from vla_ros2_bridge.observation import (Code, ObservationBuilder,
                                         ObservationError, SensorSnapshot,
                                         quaternion_from_matrix)

TICKS = {'joint1': 495, 'joint2': 644, 'joint3': 696, 'joint4': 166,
         'joint5': 499, 'r_joint': 339}


def fake_pose(ticks):
    return {'position': (0.002, 0.127, 0.232),
            'quaternion': (0.0, 0.0, 0.0, 1.0)}


def builder(**overrides):
    values = dict(pose_provider=fake_pose, max_skew_s=0.75)
    values.update(overrides)
    return ObservationBuilder(**values)


def snapshot(**overrides):
    values = dict(joint_ticks=dict(TICKS), joint_stamp=100.0,
                  wrist_image=b'\xff\xd8jpegbytes',
                  wrist_image_format='jpeg', wrist_stamp=100.05,
                  instruction='pick up the red ball')
    values.update(overrides)
    return SensorSnapshot(**values)


class TestQuaternion(unittest.TestCase):

    def test_identity(self):
        q = quaternion_from_matrix(((1, 0, 0), (0, 1, 0), (0, 0, 1)))
        self.assertAlmostEqual(abs(q[3]), 1.0, places=9)
        self.assertAlmostEqual(math.sqrt(sum(v * v for v in q)), 1.0, places=9)

    def test_unit_norm_for_a_real_rotation(self):
        angle = math.radians(37.0)
        c, s = math.cos(angle), math.sin(angle)
        q = quaternion_from_matrix(((c, -s, 0), (s, c, 0), (0, 0, 1)))
        self.assertAlmostEqual(math.sqrt(sum(v * v for v in q)), 1.0, places=9)
        self.assertAlmostEqual(q[2] / q[3], math.tan(angle / 2), places=9)

    def test_bad_matrix_is_rejected(self):
        with self.assertRaises(ObservationError):
            quaternion_from_matrix(((1, 0), (0, 1)))
        with self.assertRaises(ObservationError):
            quaternion_from_matrix(((float('nan'), 0, 0), (0, 1, 0), (0, 0, 1)))


class TestObservationBuilder(unittest.TestCase):

    def test_valid_snapshot_produces_the_message_fields(self):
        obs = builder().build(snapshot())
        data = obs.as_msg_kwargs()
        self.assertEqual(data['instruction'], 'pick up the red ball')
        self.assertEqual(data['robot_config_id'], spec.ROBOT_CONFIG_ID)
        self.assertEqual(data['header']['frame_id'], spec.ACTION_FRAME)
        self.assertEqual(data['header']['stamp'], 100.0)
        self.assertEqual(data['tcp_position_m'], (0.002, 0.127, 0.232))
        self.assertEqual(len(data['joint_names']), 6)
        self.assertEqual(len(data['joint_positions_rad']), 6)
        self.assertEqual(data['front_image']['data'], b'')
        self.assertEqual(data['front_image_stamp'], 0.0)
        self.assertEqual(data['wrist_image']['data'], b'\xff\xd8jpegbytes')
        self.assertEqual(obs.stamp, 100.0)

    def test_joint_radians_come_from_the_same_ticks(self):
        obs = builder().build(snapshot())
        pairs = dict(zip(obs['joint_names'], obs['joint_positions_rad']))
        for name in TICKS:
            self.assertAlmostEqual(pairs[name], spec.tick_to_radians(name, TICKS[name]),
                                   places=12)

    def test_gripper_opening_uses_hardware_model(self):
        open_ticks = dict(TICKS, r_joint=spec.GRIPPER_TICK_OPEN)
        closed_ticks = dict(TICKS, r_joint=spec.GRIPPER_TICK_CLOSED)
        self.assertAlmostEqual(builder().build(snapshot(joint_ticks=open_ticks))
                               ['gripper_opening'], 1.0, places=9)
        self.assertAlmostEqual(builder().build(snapshot(joint_ticks=closed_ticks))
                               ['gripper_opening'], 0.0, places=9)

    def test_pose_provider_sees_all_joints(self):
        seen = {}

        def probe(ticks):
            seen.update(ticks)
            return fake_pose(ticks)

        builder(pose_provider=probe).build(snapshot())
        self.assertEqual(seen, TICKS)

    def test_instruction_is_verbatim(self):
        text = '  Pick THE red ball!  '
        obs = builder().build(snapshot(instruction=text))
        self.assertEqual(obs['instruction'], text)

    def test_missing_joint(self):
        ticks = dict(TICKS)
        del ticks['joint3']
        with self.assertRaises(ObservationError) as caught:
            builder().build(snapshot(joint_ticks=ticks))
        self.assertEqual(caught.exception.code, Code.MISSING_JOINT)

    def test_out_of_range_joint_is_not_clamped(self):
        ticks = dict(TICKS, joint3=1200)
        with self.assertRaises(ObservationError) as caught:
            builder().build(snapshot(joint_ticks=ticks))
        self.assertEqual(caught.exception.code, Code.JOINT_RANGE)

    def test_nonfinite_joint(self):
        ticks = dict(TICKS, joint2=float('nan'))
        with self.assertRaises(ObservationError) as caught:
            builder().build(snapshot(joint_ticks=ticks))
        self.assertEqual(caught.exception.code, Code.NONFINITE)

    def test_sensor_skew_is_rejected(self):
        with self.assertRaises(ObservationError) as caught:
            builder().build(snapshot(wrist_stamp=101.0))
        self.assertEqual(caught.exception.code, Code.SKEW)

    def test_empty_image_is_a_dead_camera_not_a_dark_frame(self):
        with self.assertRaises(ObservationError) as caught:
            builder().build(snapshot(wrist_image=b''))
        self.assertEqual(caught.exception.code, Code.IMAGE)

    def test_missing_image_can_be_waived_explicitly(self):
        obs = builder(require_wrist_image=False).build(snapshot(wrist_image=b''))
        self.assertEqual(obs['wrist_image']['data'], b'')

    def test_empty_instruction(self):
        with self.assertRaises(ObservationError) as caught:
            builder().build(snapshot(instruction=''))
        self.assertEqual(caught.exception.code, Code.INSTRUCTION)

    def test_bad_stamps(self):
        for value in (0.0, -1.0, float('inf')):
            with self.assertRaises(ObservationError) as caught:
                builder().build(snapshot(joint_stamp=value))
            self.assertEqual(caught.exception.code, Code.STAMP)

    def test_pose_failure_is_reported(self):
        def broken(_ticks):
            return {'position': (0, 0)}  # wrong dimension

        with self.assertRaises(ObservationError) as caught:
            builder(pose_provider=broken).build(snapshot())
        self.assertEqual(caught.exception.code, Code.POSE)

    def test_builder_rejects_bad_skew_limit(self):
        with self.assertRaises(ValueError):
            builder(max_skew_s=0.0)


if __name__ == '__main__':
    unittest.main()
