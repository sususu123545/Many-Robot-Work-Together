"""Native ROS 2 /odom + odom->base_link from encoder counts and fresh BMI gyro."""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32MultiArray
from tf2_ros import TransformBroadcaster

from .motion import Geometry, encoder_delta, finite, integrate_pose, rotate_vector, wheel_displacement


class OdomNode(Node):
    def __init__(self):
        super().__init__('mecanum_odom')
        for key, value in {'a': 0.110, 'b': 0.0975, 'wheel_diameter': 0.0965,
                           'pulses_per_rev': 7832.0, 'imu_topic': '/bmi270/imu_raw',
                           'imu_frame': 'imu_link', 'imu_yaw_weight': 0.70,
                           'imu_timeout': 0.25, 'imu_roll': 0.0, 'imu_pitch': 0.0,
                           'imu_yaw': math.pi / 2,
                           'encoder_signs': [1.0, 1.0, 1.0],
                           'encoder_channel_signs': [1.0, 1.0, 1.0, 1.0], 'max_dt': 0.5,
                           'max_encoder_speed': 0.5, 'max_encoder_yaw': 3.0,
                           'initial_x': 0.0, 'initial_y': 0.0, 'initial_yaw': 0.0}.items():
            self.declare_parameter(key, value)
        p = lambda key: self.get_parameter(key).value
        self.geometry = Geometry(*(p(k) for k in ('a', 'b', 'wheel_diameter', 'pulses_per_rev')))
        self.weight, self.timeout = p('imu_yaw_weight'), p('imu_timeout')
        self.rpy = [p('imu_roll'), p('imu_pitch'), p('imu_yaw')]
        self.signs = p('encoder_signs')
        self.channel_signs = p('encoder_channel_signs')
        self.imu_frame = p('imu_frame')
        self.max_dt = p('max_dt')
        self.max_speed, self.max_yaw = p('max_encoder_speed'), p('max_encoder_yaw')
        if (not 0 <= self.weight <= 1 or self.timeout <= 0 or len(self.rpy) != 3
                or len(self.signs) != 3 or len(self.channel_signs) != 4
                or any(s not in (-1.0, 1.0) for s in (*self.signs, *self.channel_signs))
                or min(self.max_dt, self.max_speed, self.max_yaw) <= 0
                or not finite(*self.rpy, *self.signs, *self.channel_signs, self.weight, self.timeout,
                              self.max_dt, self.max_speed, self.max_yaw)):
            raise ValueError('Invalid odometry/IMU parameters')
        self.counts, self.last_time = None, None
        self.pose = tuple(p(k) for k in ('initial_x', 'initial_y', 'initial_yaw'))
        if not finite(*self.pose):
            raise ValueError('Initial odometry pose must be finite')
        self.gyro = None
        self.pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Int32MultiArray, '/chassis_control/encoder_counts', self.on_counts, 10)
        self.create_subscription(Imu, p('imu_topic'), self.on_imu, qos_profile_sensor_data)

    def on_imu(self, msg):
        if msg.header.frame_id != self.imu_frame:
            self.get_logger().warning('IMU frame mismatch; wheel yaw only', throttle_duration_sec=5.0)
            self.gyro = None
            return
        vector = (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)
        now_ns = self.get_clock().now().nanoseconds
        stamp_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        age = (now_ns - stamp_ns) / 10**9
        if not finite(*vector) or stamp_ns <= 0 or not 0 <= age <= self.timeout:
            self.gyro = None
            return
        self.gyro = (rotate_vector(vector, *self.rpy)[2], time.monotonic(), age)

    def on_counts(self, msg):
        counts, now = list(msg.data), time.monotonic()
        if len(counts) != 4:
            return
        if self.counts is None:
            self.counts, self.last_time = counts, now
            self.publish((0.0, 0.0, 0.0))
            return
        dt = now - self.last_time
        deltas = [encoder_delta(n, o) for n, o in zip(counts, self.counts)]
        self.counts, self.last_time = counts, now
        if not 0 < dt <= self.max_dt:
            self.get_logger().warning('Encoder gap; discarding discontinuity', throttle_duration_sec=5.0)
            return
        dx, dy, da = wheel_displacement(deltas, self.geometry, self.signs, self.channel_signs)
        if math.hypot(dx, dy) / dt > self.max_speed or abs(da) / dt > self.max_yaw:
            self.get_logger().error('Implausible encoder step; rejected', throttle_duration_sec=2.0)
            return
        if self.gyro is not None:
            gz, received, initial_age = self.gyro
            if 0 <= now - received + initial_age <= self.timeout:
                da = (1 - self.weight) * da + self.weight * gz * dt
        self.pose = integrate_pose(self.pose, dx, dy, da)
        self.publish((dx / dt, dy / dt, da / dt))

    def publish(self, velocity):
        x, y, yaw = self.pose
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id, odom.child_frame_id = 'odom', 'base_link'
        odom.pose.pose.position.x, odom.pose.pose.position.y = x, y
        odom.pose.pose.orientation.z = math.sin(yaw / 2)
        odom.pose.pose.orientation.w = math.cos(yaw / 2)
        odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.angular.z = velocity
        for i, value in zip((0, 7, 14, 21, 28, 35), (0.01, 0.01, 1e6, 1e6, 1e6, 0.05)):
            odom.pose.covariance[i] = value
            odom.twist.covariance[i] = value
        self.pub.publish(odom)
        transform = TransformStamped()
        transform.header = odom.header
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x, transform.transform.translation.y = x, y
        transform.transform.rotation = odom.pose.pose.orientation
        self.tf.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = OdomNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
