"""Publish the dynamic arm_base-to-camera TF from measured ROS 2 feedback."""
import json
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from armpi_ros2.arm_kinematics import forward_tool_pose_raw
from armpi_ros2.hand_eye_calibration import _rotation_to_quaternion


JOINTS = ('joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'r_joint')


def matrix_from_calibration(data):
    translation = data['translation_m']
    quaternion = data['quaternion_xyzw']
    if len(translation) != 3 or len(quaternion) != 4:
        raise ValueError('Extrinsic must have translation xyz and quaternion xyzw')
    x, y, z, w = map(float, quaternion)
    values = (x, y, z, w, *map(float, translation))
    if not all(math.isfinite(value) for value in values):
        raise ValueError('Extrinsic contains a non-finite value')
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if abs(norm - 1.0) > 1e-3:
        raise ValueError('Extrinsic quaternion is not normalized')
    rotation = np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ], dtype=np.float64)
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = translation
    return matrix


class ArmCameraTransform(Node):
    def __init__(self):
        super().__init__('arm_camera_transform')
        self.declare_parameter('status_topic', '/arm_controller/status')
        self.declare_parameter('status_output_topic', '/grasp/camera_tf/status')
        self.declare_parameter('calibration_file', '')
        self.declare_parameter('max_feedback_age_sec', 0.5)
        self.declare_parameter('joint5_tolerance_ticks', 8)
        self.arm_base_frame = 'arm_base'
        self.camera_frame = 'rear_camera'
        self.tool_T_camera = None
        self.joint5_reference = None
        self.latest_status_time = None
        self.br = TransformBroadcaster(self)
        self.status_pub = self.create_publisher(
            String, self.get_parameter('status_output_topic').value, 10)
        self.create_subscription(String, self.get_parameter('status_topic').value,
                                 self.on_status, 10)
        path = str(self.get_parameter('calibration_file').value).strip()
        if path:
            try:
                with open(path, 'r', encoding='utf-8') as stream:
                    calibration = json.load(stream)
                if calibration.get('schema') != 'armpi_ros2_hand_eye_v1':
                    raise ValueError('Unsupported hand-eye calibration schema')
                if not calibration.get('quality', {}).get('accepted'):
                    raise ValueError('Hand-eye calibration quality is not accepted')
                self.arm_base_frame = str(calibration['arm_base_frame'])
                self.camera_frame = str(calibration['camera_frame'])
                self.joint5_reference = int(calibration['joint5_raw_reference'])
                self.tool_T_camera = matrix_from_calibration(
                    calibration['tool_T_camera'])
                self.get_logger().info(
                    'Loaded accepted eye-in-hand calibration: %s -> %s'
                    % (self.arm_base_frame, self.camera_frame))
            except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError) as error:
                self.get_logger().error('No usable camera extrinsic: ' + str(error))
        if self.tool_T_camera is None:
            self.get_logger().warning(
                'Waiting for an accepted hand-eye calibration file; no camera TF will be published')

    def report(self, valid, reason, **extra):
        message = String()
        message.data = json.dumps({'valid': bool(valid), 'reason': str(reason),
                                   **extra}, separators=(',', ':'), allow_nan=False)
        self.status_pub.publish(message)

    def on_status(self, message):
        now = time.monotonic()
        self.latest_status_time = now
        if self.tool_T_camera is None:
            self.report(False, 'hand_eye_calibration_missing')
            return
        try:
            status = json.loads(message.data)
            if (status.get('estop') or status.get('health_state') != 'READY'
                    or status.get('fault')
                    or not status.get('feedback_real')
                    or not status.get('feedback_complete')):
                raise ValueError('arm feedback is not healthy READY')
            joints = status.get('joints') or {}
            ages = status.get('feedback_age_seconds') or {}
            max_age = float(self.get_parameter('max_feedback_age_sec').value)
            if (set(joints) != set(JOINTS) or set(ages) != set(JOINTS)
                    or any(not 0 <= float(ages[name]) <= max_age for name in JOINTS)):
                raise ValueError('arm joint feedback is incomplete or stale')
            raw = {name: int(joints[name]['raw']) for name in JOINTS}
            if abs(raw['joint5'] - self.joint5_reference) > int(
                    self.get_parameter('joint5_tolerance_ticks').value):
                raise ValueError('wrist-roll joint5 differs from the calibrated fixed position')
            tool = forward_tool_pose_raw(raw)
            base_T_tool = np.eye(4, dtype=np.float64)
            base_T_tool[:3, :3] = tool['rotation_matrix']
            base_T_tool[:3, 3] = tool['translation_m']
            base_T_camera = base_T_tool @ self.tool_T_camera
            q = _rotation_to_quaternion(base_T_camera[:3, :3])
            stamp = status.get('stamp') or {}
            transform = TransformStamped()
            transform.header.stamp.sec = int(stamp['sec'])
            transform.header.stamp.nanosec = int(stamp['nanosec'])
            transform.header.frame_id = self.arm_base_frame
            transform.child_frame_id = self.camera_frame
            translation = base_T_camera[:3, 3]
            transform.transform.translation.x = float(translation[0])
            transform.transform.translation.y = float(translation[1])
            transform.transform.translation.z = float(translation[2])
            (transform.transform.rotation.x, transform.transform.rotation.y,
             transform.transform.rotation.z, transform.transform.rotation.w) = q
            self.br.sendTransform(transform)
            self.report(True, 'camera_tf_published',
                        parent_frame=self.arm_base_frame,
                        camera_frame=self.camera_frame,
                        wrist_roll_raw=raw['joint5'])
        except (ValueError, KeyError, TypeError, json.JSONDecodeError,
                OverflowError) as error:
            self.report(False, str(error))


def main(args=None):
    rclpy.init(args=args)
    node = ArmCameraTransform()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
