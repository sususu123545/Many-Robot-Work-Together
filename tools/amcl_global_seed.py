#!/usr/bin/env python3
"""Give nav2_amcl a broad initial pose so it localises globally.

Why this is needed at all
-------------------------
The plan for this stage said "nav2_amcl does global localisation, no initial
pose required".  That is wrong, and the source says so plainly: in
`AmclNode::publishAmclPose()` (nav2_amcl/src/amcl_node.cpp, Jazzy):

    if (!initial_pose_is_known_) {
      RCLCPP_WARN(get_logger(), "AMCL cannot publish a pose or update the
                  transform. Please set the initial pose...");
      return;
    }

`set_initial_pose: true` does not help either -- it feeds a default-constructed
PoseWithCovarianceStamped, i.e. ZERO covariance, so every particle starts on one
point.  That is a known start pose, not a global search.

The way to actually get global localisation is to publish one `/initialpose`
with a LARGE covariance.  `handleInitialPose()` passes that covariance straight
into `pf_init()`, which samples the particle cloud from it -- so a 1.3 m sigma
on x/y and a pi sigma on yaw spreads the particles over the whole map and lets
the first few scans collapse them.  That is what this script does.

Default centre is the centroid of the map's free cells, computed from
maps/workshop.pgm (x=-0.131, y=0.188).  Override with --x/--y.

Run inside the core container, with map_server + amcl already active:
    source /opt/ros/jazzy/setup.bash
    python3 /tmp/amcl_global_seed.py
"""
import argparse
import math
import sys

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

try:                                   # only present in the car's container
    from nav2_msgs.msg import ParticleCloud
except ImportError:                    # keeps this file importable on the PC
    ParticleCloud = None

DEFAULT_X = -0.131
DEFAULT_Y = 0.188

# variance, not sigma: 1.8 -> sigma 1.34 m, which reaches ~2.7 m at 2 sigma and
# therefore covers this 4.45 x 5.60 m patch.  yaw gets pi^2, i.e. effectively
# uniform over a full turn, because a corridor is symmetric and we must not
# assume which way the car was pointing after a power cycle.
VAR_XY = 1.8
VAR_YAW = math.pi ** 2


class Seeder(Node):
    def __init__(self, var_xy=VAR_XY, var_yaw=VAR_YAW):
        super().__init__('amcl_global_seed')
        self.var_xy = var_xy
        self.var_yaw = var_yaw
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', QoSProfile(depth=1))
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.pose = None
        self.particles = None
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.on_pose, qos)
        if ParticleCloud is not None:
            self.create_subscription(
                ParticleCloud, '/particle_cloud', self.on_particles, qos)

    def on_pose(self, message):
        self.pose = message

    def on_particles(self, message):
        self.particles = len(message.particles)

    def seed(self, x, y, yaw=0.0):
        message = PoseWithCovarianceStamped()
        message.header.frame_id = 'map'
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.pose.position.x = x
        message.pose.pose.position.y = y
        message.pose.pose.orientation.z = math.sin(yaw / 2.0)
        message.pose.pose.orientation.w = math.cos(yaw / 2.0)
        covariance = [0.0] * 36
        covariance[0] = self.var_xy
        covariance[7] = self.var_xy
        covariance[35] = self.var_yaw
        message.pose.covariance = covariance
        self.publisher.publish(message)


def spin_for(node, seconds):
    import time
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--x', type=float, default=DEFAULT_X)
    parser.add_argument('--y', type=float, default=DEFAULT_Y)
    parser.add_argument('--yaw', type=float, default=0.0,
                        help='seed yaw, radians (0 = +x, map frame)')
    parser.add_argument('--wait', type=float, default=20.0,
                        help='seconds to wait for /amcl_pose per attempt')
    parser.add_argument('--attempts', type=int, default=3)
    parser.add_argument('--sigma-xy', type=float, default=math.sqrt(VAR_XY),
                        help='std dev in metres (default = broad global search). '
                             'Use a small value, e.g. 0.1, with a KNOWN pose to '
                             'test tracking separately from the global search.')
    parser.add_argument('--sigma-yaw', type=float, default=math.sqrt(VAR_YAW),
                        help='std dev in radians')
    args = parser.parse_args()

    rclpy.init()
    node = Seeder(args.sigma_xy ** 2, args.sigma_yaw ** 2)
    spin_for(node, 1.0)          # let the publisher discover subscribers
    if node.publisher.get_subscription_count() == 0:
        print('WARNING: nothing is subscribed to /initialpose -- '
              'is amcl running and active?')

    ok = False
    for attempt in range(1, args.attempts + 1):
        print('attempt %d: seeding (%+.3f, %+.3f, %.2f rad) '
              'sigma_xy=%.2f m sigma_yaw=%.2f rad'
              % (attempt, args.x, args.y, args.yaw,
                 args.sigma_xy, args.sigma_yaw), flush=True)
        node.seed(args.x, args.y, args.yaw)
        spin_for(node, args.wait)
        if node.pose is not None:
            ok = True
            break

    if not ok:
        print('FAILED: no /amcl_pose after %d attempts. amcl is active but not '
              'converging -- check that /scan reaches it and that the map frame '
              'matches.' % args.attempts)
        rclpy.shutdown()
        return 1

    pose = node.pose.pose.pose
    covariance = node.pose.pose.covariance
    sigma_x = math.sqrt(max(0.0, covariance[0]))
    sigma_y = math.sqrt(max(0.0, covariance[7]))
    sigma_yaw = math.sqrt(max(0.0, covariance[35]))
    print('---')
    print('pose        : x=%+.3f  y=%+.3f  yaw=%+.1f deg'
          % (pose.position.x, pose.position.y,
             math.degrees(math.atan2(
                 2.0 * (pose.orientation.w * pose.orientation.z),
                 1.0 - 2.0 * pose.orientation.z ** 2))))
    print('sigma       : x=%.3f m  y=%.3f m  yaw=%.1f deg  (particles=%s)'
          % (sigma_x, sigma_y, math.degrees(sigma_yaw),
             node.particles if node.particles is not None else 'n/a'))
    converged = sigma_x < 0.30 and sigma_y < 0.30 and sigma_yaw < math.radians(20)
    print('verdict     : %s' % ('CONVERGED' if converged else
                                'NOT converged -- needs motion or more particles'))
    rclpy.shutdown()
    return 0 if converged else 2


if __name__ == '__main__':
    sys.exit(main())
