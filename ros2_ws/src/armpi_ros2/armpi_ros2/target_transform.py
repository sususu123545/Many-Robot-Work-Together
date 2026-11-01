"""Transform a detected ball point into the arm base frame using live ROS 2 TF.

This is a passive coordinate-chain component. It never publishes actuator,
gripper, chassis, reset, or safety commands. Missing camera/arm TF is reported
as unavailable; no guessed extrinsic is substituted.
"""
import json
import math

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


def transform_xyz(point_xyz, transform):
    """Apply a TransformStamped to xyz, validating the quaternion and result."""
    x, y, z = (float(value) for value in point_xyz)
    values = (x, y, z)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('Point contains a non-finite coordinate')

    rotation = transform.transform.rotation
    qx, qy, qz, qw = map(float, (rotation.x, rotation.y, rotation.z, rotation.w))
    qnorm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if not math.isfinite(qnorm) or abs(qnorm - 1.0) > 1e-3:
        raise ValueError('TF quaternion is not normalized')

    # q * [p, 0] * q^-1, using the numerically compact cross-product form.
    tx = 2.0 * (qy*z - qz*y)
    ty = 2.0 * (qz*x - qx*z)
    tz = 2.0 * (qx*y - qy*x)
    rx = x + qw*tx + (qy*tz - qz*ty)
    ry = y + qw*ty + (qz*tx - qx*tz)
    rz = z + qw*tz + (qx*ty - qy*tx)

    translation = transform.transform.translation
    result = (rx + float(translation.x),
              ry + float(translation.y),
              rz + float(translation.z))
    if not all(math.isfinite(value) for value in result):
        raise ValueError('Transformed point is not finite')
    return result


class GraspTargetTransform(Node):
    def __init__(self, **kwargs):
        super().__init__('grasp_target_transform', **kwargs)
        self.declare_parameter('input_topic', '/grasp/target_camera')
        self.declare_parameter('output_topic', '/grasp/target_base')
        self.declare_parameter('status_topic', '/grasp/transform/status')
        self.declare_parameter('target_frame', 'arm_base')
        self.declare_parameter('max_point_age_sec', 1.0)
        self.declare_parameter('max_future_sec', 0.25)
        self.declare_parameter('max_tf_lag_sec', 0.75)

        self.target_frame = str(self.get_parameter('target_frame').value)
        self.max_age = float(self.get_parameter('max_point_age_sec').value)
        self.max_tf_lag = float(self.get_parameter('max_tf_lag_sec').value)
        self.max_future = float(self.get_parameter('max_future_sec').value)
        if not self.target_frame or self.max_age <= 0 or self.max_future < 0:
            raise ValueError('Invalid target transform configuration')

        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.target_pub = self.create_publisher(
            PointStamped, self.get_parameter('output_topic').value, 5)
        self.status_pub = self.create_publisher(
            String, self.get_parameter('status_topic').value, 5)
        self.point_sub = self.create_subscription(
            PointStamped, self.get_parameter('input_topic').value,
            self.on_target, 5)
        self.get_logger().info(
            'Passive target transform ready; waiting for timestamped camera-to-base TF')

    def report(self, valid, reason, source_frame='', stamp_sec=0,
               point_base=None, tf_lag_sec=None):
        payload = {'valid': bool(valid), 'reason': str(reason),
                   'source_frame': source_frame,
                   'target_frame': self.target_frame,
                   'stamp_sec': int(stamp_sec),
                   'point_base_m': list(point_base) if point_base else None}
        if tf_lag_sec is not None:
            payload['tf_lag_sec'] = round(float(tf_lag_sec), 4)
        msg = String()
        msg.data = json.dumps(payload, separators=(',', ':'), allow_nan=False)
        self.status_pub.publish(msg)

    def on_target(self, msg):
        source_frame = msg.header.frame_id.strip()
        stamp = Time.from_msg(msg.header.stamp)
        stamp_sec = stamp.nanoseconds // 1_000_000_000
        if not source_frame:
            self.report(False, 'missing_source_frame', stamp_sec=stamp_sec)
            return
        if stamp.nanoseconds <= 0:
            self.report(False, 'missing_source_timestamp', source_frame, stamp_sec)
            return

        now_ns = self.get_clock().now().nanoseconds
        age_sec = (now_ns - stamp.nanoseconds) * 1e-9
        if age_sec > self.max_age:
            self.report(False, 'stale_target', source_frame, stamp_sec)
            return
        if age_sec < -self.max_future:
            self.report(False, 'future_target_stamp', source_frame, stamp_sec)
            return

        tf_lag = 0.0
        if self.tf_buffer.can_transform(
                self.target_frame, source_frame, stamp,
                timeout=Duration(seconds=0.0)):
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.target_frame, source_frame, stamp,
                    timeout=Duration(seconds=0.0))
            except (TransformException, ValueError, TypeError) as error:
                self.report(False, 'transform_error: ' + str(error),
                            source_frame, stamp_sec)
                return
            reason = 'transformed'
        else:
            # Arm feedback runs at 2 Hz while the camera delivers ~8 Hz, so a
            # fresh image stamp is almost always NEWER than the newest
            # arm_base->camera TF. tf2 refuses to extrapolate into the future, so
            # an exact-time lookup dropped EVERY point and the chain never
            # produced /grasp/target_base. Waiting cannot help either: this
            # callback blocks a single-threaded executor, including the TF
            # listener that would otherwise fill the buffer. Fall back to the
            # newest transform while it is still within one arm-feedback period,
            # and report the skew so a slow arm is never silently hidden.
            try:
                latest = self.tf_buffer.lookup_transform(
                    self.target_frame, source_frame, Time())
            except (TransformException, ValueError, TypeError) as error:
                self.report(False, 'waiting_for_camera_to_target_tf: ' + str(error),
                            source_frame, stamp_sec)
                return
            lag = (stamp.nanoseconds
                   - Time.from_msg(latest.header.stamp).nanoseconds) * 1e-9
            # A NEGATIVE lag means the newest TF is NEWER than the image, which
            # is the harmless direction: the arm pose is at most a few hundred
            # microseconds "after" the picture it is applied to. Refusing it as
            # out-of-range was wrong -- it threw away perfectly good points
            # whenever the 2 Hz arm feedback happened to land just after the
            # frame. Only bound how far into the future the TF may be, using the
            # same tolerance already applied to the point stamp.
            # Read live so the budget can be tightened at runtime (and tested).
            max_lag = float(self.get_parameter('max_tf_lag_sec').value)
            if lag > max_lag or lag < -self.max_future:
                self.report(False, 'tf_lag_out_of_range', source_frame, stamp_sec,
                            tf_lag_sec=lag)
                return
            transform = latest
            tf_lag = lag
            reason = 'transformed_with_latest_tf'
        try:
            point_base = transform_xyz(
                (msg.point.x, msg.point.y, msg.point.z), transform)
        except (ValueError, TypeError) as error:
            self.report(False, 'transform_error: ' + str(error),
                        source_frame, stamp_sec)
            return

        result = PointStamped()
        result.header = msg.header
        result.header.frame_id = self.target_frame
        result.point.x, result.point.y, result.point.z = point_base
        self.target_pub.publish(result)
        self.report(True, reason, source_frame, stamp_sec, point_base,
                    tf_lag_sec=tf_lag)


def main(args=None):
    rclpy.init(args=args)
    node = GraspTargetTransform()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
