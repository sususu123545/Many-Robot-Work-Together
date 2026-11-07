#!/usr/bin/env python3
"""Integrate the BMI270 gyro into a yaw angle, published as /imu/yaw (degrees).

WHY THIS RUNS ON THE CAR
------------------------
The workstation reads sensors through rosbridge -- a single websocket where
publishing cmd_vel and receiving telemetry interleave.  The chassis MotionLease
drops motion when cmd_vel gaps exceed 0.35 s, so the workstation can never
block on a 50 Hz sensor stream without stalling the car.  Integrating here, on
the car, keeps the 50 Hz DDS stream entirely local; the workstation only ever
reads one small Float32.

CALIBRATION
-----------
BMI270's gyro zero-bias is what turns "stationary" into "0.0002 rad/s" instead
of exactly zero.  On startup the node averages CALIB_SECONDS of samples while
the car must be held still.  Bias drifts slowly with temperature; call
/imu_yaw/recalibrate (std_srvs/Trigger) any time to redo it -- the caller must
keep the car still again while it runs.

OUTPUT
------
/imu/yaw, std_msgs/Float32, DEGREES, roughly 10 Hz.  yaw starts at 0 at node
startup (or at the last recalibrate) and increases counterclockwise viewed
from above (ROS convention).  It is RELATIVE to startup -- no magnetometer, no
absolute heading.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from std_srvs.srv import Trigger

IMU_HZ = 50.0          # measured 2026-11-05
PUBLISH_INTERVAL = 0.1  # ~10 Hz is plenty for a heading scalar


class ImuYawNode(Node):

    def __init__(self):
        super().__init__('imu_yaw_node')
        self.declare_parameter('calib_seconds', 3.0)
        self.calib_seconds = float(self.get_parameter('calib_seconds').value)

        self.bias = None          # set once calibration completes
        self.samples = []
        self.yaw = 0.0            # degrees, relative to startup
        self.last_t = None
        self.last_pub = 0.0

        self.sub = self.create_subscription(
            Imu, '/bmi270/imu_raw', self.on_imu, qos_profile_sensor_data)
        self.pub = self.create_publisher(Float32, '/imu/yaw', 10)
        self.srv = self.create_service(
            Trigger, '/imu_yaw/recalibrate', self.recalibrate)
        self.get_logger().info(
            'calibrating gyro bias: hold the car STILL for %.1f s'
            % self.calib_seconds)

    def on_imu(self, msg):
        stamp = msg.header.stamp
        now = stamp.sec + stamp.nanosec * 1e-9
        wz = msg.angular_velocity.z

        if self.bias is None:
            self.samples.append(wz)
            needed = int(self.calib_seconds * IMU_HZ)
            if len(self.samples) >= needed:
                self.bias = sum(self.samples) / len(self.samples)
                self.last_t = now
                self.get_logger().info(
                    'bias = %+.5f rad/s over %d samples; integrating from now'
                    % (self.bias, len(self.samples)))
            return

        if self.last_t is None:
            self.last_t = now
            return
        dt = now - self.last_t
        self.last_t = now
        if 0.0 < dt < 0.5:
            self.yaw += math.degrees((wz - self.bias) * dt)

        if now - self.last_pub >= PUBLISH_INTERVAL:
            out = Float32()
            out.data = float(self.yaw)
            self.pub.publish(out)
            self.last_pub = now

    def recalibrate(self, _request, response):
        self.bias = None
        self.samples = []
        self.last_t = None
        self.yaw = 0.0
        response.success = True
        response.message = ('bias cleared; hold the car still, calibration '
                            'restarts automatically (%.1f s)' % self.calib_seconds)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ImuYawNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
