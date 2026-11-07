#!/usr/bin/env python3
"""Acceptance test: does nav2_amcl converge from a global seed while turning?

Why a single process
--------------------
The obvious manual sequence -- seed, then rotate, then read /amcl_pose -- has a
trap: publishing /initialpose again RESETS the particle filter, so reading the
pose with the same tool that seeds it throws away everything the motion just
taught the filter (measured: sigma_yaw went 196 deg -> 134 deg only because a
fresh seed had re-spread it).  Seed once, move, then only listen.

What it does
------------
  1. publish ONE broad /initialpose (global search)
  2. record the pose and sigma
  3. drive /cmd_vel at a slow fixed yaw rate for N degrees, sending a zero
     command afterwards -- the chassis has NO deadman, the last command it
     hears is the one it keeps obeying
  4. record the pose and sigma again, without reseeding
  5. report whether sigma collapsed and whether the yaw moved by the amount the
     car actually turned (taken from /odom, which is the only trustworthy
     record of rotation on this chassis)

Run INSIDE the core container, with the localisation stack already active:
    source /opt/ros/jazzy/setup.bash
    python3 /opt/armpi-ros2-localization/amcl_converge_test.py --degrees 120
"""
import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

DEFAULT_X = -0.131
DEFAULT_Y = 0.188
VAR_XY = 1.8
VAR_YAW = math.pi ** 2


def yaw_of(quaternion):
    return math.atan2(2.0 * (quaternion.w * quaternion.z),
                      1.0 - 2.0 * quaternion.z ** 2)


def wrap_deg(value):
    while value > 180.0:
        value -= 360.0
    while value <= -180.0:
        value += 360.0
    return value


class Driver(Node):
    def __init__(self):
        super().__init__('amcl_converge_test')
        self.cmd = self.create_publisher(Twist, '/cmd_vel', QoSProfile(depth=10))
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.amcl = None
        self.odom_yaw = None
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl, qos)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos)
        self.seed_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', QoSProfile(depth=1))

    def on_amcl(self, message):
        self.amcl = message

    def on_odom(self, message):
        self.odom_yaw = yaw_of(message.pose.pose.orientation)

    def seed(self, x, y, yaw=0.0):
        message = PoseWithCovarianceStamped()
        message.header.frame_id = 'map'
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.pose.position.x = x
        message.pose.pose.position.y = y
        message.pose.pose.orientation.z = math.sin(yaw / 2.0)
        message.pose.pose.orientation.w = math.cos(yaw / 2.0)
        covariance = [0.0] * 36
        covariance[0] = VAR_XY
        covariance[7] = VAR_XY
        covariance[35] = VAR_YAW
        message.pose.covariance = covariance
        self.seed_pub.publish(message)


def spin_for(node, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)


def report(label, node):
    if node.amcl is None:
        print('%s: no /amcl_pose' % label)
        return None
    pose = node.amcl.pose.pose
    cov = node.amcl.pose.covariance
    sigma_xy = math.sqrt(max(0.0, cov[0]))
    sigma_yaw = math.degrees(math.sqrt(max(0.0, cov[35])))
    print('%s: x=%+.3f y=%+.3f yaw=%+.1f deg | sigma_xy=%.3f m sigma_yaw=%.1f deg'
          % (label, pose.position.x, pose.position.y,
             math.degrees(yaw_of(pose.orientation)), sigma_xy, sigma_yaw))
    return (math.degrees(yaw_of(pose.orientation)), sigma_xy, sigma_yaw)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--x', type=float, default=DEFAULT_X)
    parser.add_argument('--y', type=float, default=DEFAULT_Y)
    parser.add_argument('--degrees', type=float, default=120.0)
    parser.add_argument('--rate', type=float, default=0.25,
                        help='yaw rate rad/s; 0.25 measured reliable here')
    parser.add_argument('--settle', type=float, default=6.0,
                        help='seconds to let the filter absorb the last scans')
    args = parser.parse_args()

    rclpy.init()
    node = Driver()
    spin_for(node, 1.5)

    # Do NOT wait for /amcl_pose before seeding: nav2_amcl publishes nothing at
    # all until it has been given an initial pose (publishAmclPose() returns
    # early while initial_pose_is_known_ is false).  Seed first, then wait.
    print('seeding globally at (%+.3f, %+.3f) sigma_xy=%.2f m sigma_yaw=%.0f deg'
          % (args.x, args.y, math.sqrt(VAR_XY),
             math.degrees(math.sqrt(VAR_YAW))))
    node.seed(args.x, args.y, 0.0)

    deadline = time.monotonic() + 30.0
    while node.amcl is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.amcl is None:
        print('FAILED: still no /amcl_pose 30 s after seeding. Check '
              '`ros2 lifecycle get /amcl` and that map_server published /map.')
        rclpy.shutdown()
        return 1

    spin_for(node, args.settle)
    before = report('before ', node)

    start_yaw = node.odom_yaw
    accumulated = 0.0
    previous = node.odom_yaw
    target = abs(args.degrees)
    direction = 1.0 if args.degrees >= 0 else -1.0
    command = Twist()
    command.angular.z = direction * args.rate
    # degrees / (rad/s) is not a time.  Convert first, or the loop runs for
    # 480-odd seconds and gets killed by whatever timeout wrapped it.
    expected_s = math.radians(target) / abs(args.rate)
    deadline = time.monotonic() + expected_s * 2.0 + 10.0
    try:
        while accumulated < target and time.monotonic() < deadline:
            node.cmd.publish(command)
            spin_for(node, 0.05)
            if node.odom_yaw is not None and previous is not None:
                accumulated += abs(wrap_deg(node.odom_yaw - previous))
                previous = node.odom_yaw
    finally:
        stop = Twist()                       # no deadman: stop explicitly
        for _ in range(10):
            node.cmd.publish(stop)
            time.sleep(0.05)
    spin_for(node, args.settle)
    after = report('after  ', node)

    turned = wrap_deg(node.odom_yaw - start_yaw)
    print('---')
    print('car actually turned (odom): %+.1f deg' % turned)
    if before is None or after is None:
        rclpy.shutdown()
        return 1
    amcl_turned = wrap_deg(after[0] - before[0])
    print('amcl yaw changed         : %+.1f deg' % amcl_turned)
    print('agreement                : %.1f deg apart'
          % abs(wrap_deg(amcl_turned - turned)))
    converged = after[1] < 0.30 and after[2] < 20.0
    tracked = abs(wrap_deg(amcl_turned - turned)) < 15.0
    print('verdict                  : sigma %s, tracking %s'
          % ('collapsed' if converged else 'STILL BROAD',
             'OK' if tracked else 'MISMATCHED'))
    rclpy.shutdown()
    return 0 if (converged and tracked) else 2


if __name__ == '__main__':
    sys.exit(main())
