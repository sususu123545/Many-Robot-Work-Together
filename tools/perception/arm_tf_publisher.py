#!/usr/bin/env python3
"""Publish the two frames the car does not have: arm_base and rear_camera.

The car publishes `odom -> base_link` plus two latched statics
(`base_link -> base_laser`, `base_link -> imu_link`).  Nothing publishes
`arm_base` or `rear_camera`, because there is no robot_state_publisher on the
car and the camera rides the wrist, so its pose is a function of joint feedback
(see arm_tf.py for why the hand-eye number, not the URDF's, is used).

What this node does
-------------------
  * joint feedback          ->  arm_base -> rear_camera  on /tf, ~10 Hz
  * once at startup         ->  base_link -> arm_base    on /tf_static

Feedback sources
----------------
`/joint_states` is the primary source: it is standard, always present, and
already used by every tool in tools/.  It runs at 2.2 Hz on this car (defect
D6), which is enough for TF at 10 Hz because the pose is only recomputed when a
new reading arrives -- publishing the same pose ten times a second is correct,
not wasteful: tf2 needs repeated broadcast to keep a frame from going stale.

`/arm_controller/status` was tried first and abandoned: its JSON contains a
`{"joint1": 1, "joint2": 1, ...}` block whose values are servo IDS, not ticks, so
a schema-agnostic search for four joint keys locked onto it and fed garbage to
the forward kinematics.  `/joint_states` has one unambiguous shape, so it is the
only source.

Run it:
    python3 arm_tf_publisher.py \
        --calibration /opt/armpi-calibration/rear_camera_hand_eye.json
"""
import argparse
import json
import math
import os
import sys
import threading

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.dirname(HERE))

import arm_tf                                                   # noqa: E402

TICK_RAD = math.radians(240.0) / 1000.0
SIGNS = {'joint1': 1.0, 'joint2': 1.0, 'joint3': -1.0, 'joint4': 1.0}
JOINTS = ('joint1', 'joint2', 'joint3', 'joint4')


def ticks_from_radians(positions_by_name):
    """Radians from /joint_states -> the raw ticks arm_kinematics expects."""
    ticks = {}
    for name in JOINTS:
        if name not in positions_by_name:
            return None
        ticks[name] = int(round(500.0 + SIGNS[name] * positions_by_name[name] / TICK_RAD))
    return ticks


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--calibration', default=os.environ.get(
        'ARM_HAND_EYE', '/opt/armpi-calibration/rear_camera_hand_eye.json'))
    parser.add_argument('--rate', type=float, default=10.0)
    parser.add_argument('--base-frame', default='base_link')
    parser.add_argument('--arm-base-frame', default='arm_base')
    parser.add_argument('--camera-frame', default='rear_camera')
    args = parser.parse_args()

    if not os.path.exists(args.calibration):
        print('hand-eye calibration missing: %s' % args.calibration, file=sys.stderr)
        print('the camera rides the wrist, so without it there is no honest '
              'arm_base -> rear_camera transform. Refusing to guess.', file=sys.stderr)
        return 2

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import TransformStamped
    from sensor_msgs.msg import JointState
    from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

    # On the car the module lives inside the `armpi_ros2` package, so the import
    # has to be qualified; from tools/ the package directory itself is on the
    # path and the bare name works.  Support both rather than guessing.
    try:
        from armpi_ros2.arm_kinematics import forward_tool_pose_raw
    except ImportError:
        from arm_kinematics import forward_tool_pose_raw

    hand_eye = arm_tf.load_hand_eye(args.calibration)

    rclpy.init()
    node = Node('arm_tf_publisher')

    state = {'ticks': None, 'source': '', 'lock': threading.Lock(), 'warned': False}

    def adopt(ticks, source):
        if not ticks:
            return
        with state['lock']:
            first = state['ticks'] is None
            state['ticks'] = ticks
            state['source'] = source
        if first:
            node.get_logger().info('first %s reading: %s' % (source, ticks))

    def on_joint_states(message):
        positions = dict(zip(message.name, message.position))
        adopt(ticks_from_radians(positions), 'joint_states')

    static = StaticTransformBroadcaster(node)
    base_to_arm = TransformStamped()
    base_to_arm.header.frame_id = args.base_frame
    base_to_arm.child_frame_id = args.arm_base_frame
    base_to_arm.transform.translation.x = arm_tf.BASE_LINK_TO_ARM_BASE_M[0]
    base_to_arm.transform.translation.y = arm_tf.BASE_LINK_TO_ARM_BASE_M[1]
    base_to_arm.transform.translation.z = arm_tf.BASE_LINK_TO_ARM_BASE_M[2]
    base_to_arm.transform.rotation.w = 1.0
    static.sendTransform(base_to_arm)
    node.get_logger().info('published %s -> %s (static, %s)'
                           % (args.base_frame, args.arm_base_frame,
                              arm_tf.BASE_LINK_TO_ARM_BASE_M))

    dynamic = TransformBroadcaster(node)

    def publish():
        with state['lock']:
            ticks = dict(state['ticks']) if state['ticks'] else None
            source = state['source']
        if ticks is None:
            if not state['warned']:
                node.get_logger().warn('no joint feedback yet -- is the arm up?')
                state['warned'] = True
            return
        try:
            translation, quaternion = arm_tf.camera_pose_in_arm_base(
                ticks, hand_eye, forward_tool_pose_raw)
        except ValueError as error:
            node.get_logger().warn('arm feedback unusable: %s' % error)
            return
        message = TransformStamped()
        message.header.stamp = node.get_clock().now().to_msg()
        message.header.frame_id = args.arm_base_frame
        message.child_frame_id = args.camera_frame
        message.transform.translation.x = translation[0]
        message.transform.translation.y = translation[1]
        message.transform.translation.z = translation[2]
        message.transform.rotation.x = quaternion[0]
        message.transform.rotation.y = quaternion[1]
        message.transform.rotation.z = quaternion[2]
        message.transform.rotation.w = quaternion[3]
        dynamic.sendTransform(message)

    node.create_subscription(JointState, '/joint_states', on_joint_states,
                             qos_profile_sensor_data)
    node.create_timer(1.0 / max(args.rate, 1.0), publish)

    node.get_logger().info('publishing %s -> %s at %.0f Hz'
                           % (args.arm_base_frame, args.camera_frame, args.rate))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
