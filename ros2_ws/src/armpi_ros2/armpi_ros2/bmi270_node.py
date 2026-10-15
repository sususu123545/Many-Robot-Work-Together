import math
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from .bmi270_transport import find_ch341_bus, make_sensor, read_sample


class Bmi270Node(Node):
    def __init__(self):
        super().__init__('bmi270_imu')
        for key, value in {'library_path': '/opt/bmi270_py', 'rate': 50.0,
                           'frame_id': 'imu_link', 'retry_interval': 5.0,
                           'gyro_bias': [0.002870237, 0.002160416, -0.000748247]}.items():
            self.declare_parameter(key, value)
        p = lambda key: self.get_parameter(key).value
        self.library_path, self.frame = p('library_path'), p('frame_id')
        self.bias, self.retry = p('gyro_bias'), p('retry_interval')
        if (len(self.bias) != 3 or not all(math.isfinite(b) for b in self.bias)
                or not math.isfinite(p('rate')) or p('rate') <= 0
                or not math.isfinite(self.retry) or self.retry <= 0):
            raise ValueError('Invalid BMI270 parameters')
        self.sensor, self.next_connect = None, 0.0
        self.pub = self.create_publisher(Imu, '/bmi270/imu_raw', qos_profile_sensor_data)
        self.create_timer(1.0 / p('rate'), self.tick)

    def close_sensor(self):
        if self.sensor is not None:
            self.sensor.bus.close()
            self.sensor = None

    def tick(self):
        try:
            if self.sensor is None:
                if time.monotonic() < self.next_connect:
                    return
                self.next_connect = time.monotonic() + self.retry
                self.sensor = make_sensor(find_ch341_bus(), self.library_path)
                self.get_logger().info('BMI270 initialized; publishing sensor-frame SI data')
            accel, gyro = read_sample(self.sensor)
            message = Imu()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = self.frame
            message.orientation_covariance[0] = -1.0  # No absolute orientation estimate.
            for axis, a, g, bias in zip('xyz', accel, gyro, self.bias):
                setattr(message.linear_acceleration, axis, a)
                setattr(message.angular_velocity, axis, g - bias)
            for i in (0, 4, 8):
                message.angular_velocity_covariance[i] = 0.01
                message.linear_acceleration_covariance[i] = 0.04
            self.pub.publish(message)
        except (OSError, RuntimeError, ImportError) as error:
            self.close_sensor()
            self.next_connect = time.monotonic() + self.retry
            self.get_logger().error(f'BMI270 unavailable: {error}', throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = Bmi270Node()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close_sensor()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
