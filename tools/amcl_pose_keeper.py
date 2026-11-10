#!/usr/bin/env python3
"""Persist the last converged map pose so the next boot can start warm.

Why this exists
---------------
2026-11-09: seeding amcl with a broad cloud (sigma 1.34 m, uniform yaw) is not
reliable in a cluttered room -- three attempts, one convergence; the failures
parked at sigma 0.9-1.0 m with the yaw estimate flipping between hypotheses.
Seeding at a *known* nearby pose with a tight covariance converged immediately
(sigma 0.075 m / 5.6 deg), because the particle cloud has a small, correct place
to collapse into.

The robot usually does not move while it is powered off, so the pose it had when
it last shut down is the best possible prior.  This node writes that pose out
whenever it is trustworthy; `amcl_global_seed.py` reads it back and verifies it
against the live scan before believing it.

Only poses with a tight covariance are written, so a half-converged filter can
never poison the next boot.

Run inside the core container:
    source /opt/ros/jazzy/setup.bash
    python3 /opt/armpi-maps/amcl_pose_keeper.py
"""
import json
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

PATH = '/opt/armpi-maps/last_pose.json'
MAX_SIGMA_XY = 0.30          # metres; amcl's own result, not the seed
MAX_SIGMA_YAW = math.radians(20.0)
MAX_AGE_S = 5.0              # a pose older than this is not "current"
WRITE_EVERY_S = 3.0


def map_id(grid):
    """A short fingerprint of the geometry a pose is expressed in.

    A pose is only meaningful in the coordinate frame of one specific map.  On
    2026-11-10 the map was rebuilt after the furniture moved; the stored pose
    then pointed into empty space in the new frame.  Recording the geometry and
    refusing a mismatch makes "the map changed under the pose" impossible to
    ignore.
    """
    origin = grid.info.origin.position
    return '%dx%d@%.3f,%.3f' % (grid.info.width, grid.info.height,
                                origin.x, origin.y)


class Keeper(Node):
    def __init__(self):
        super().__init__('amcl_pose_keeper')
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.pose = None
        self.received_at = 0.0
        self.map_token = None
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.on_pose, qos)
        self.create_subscription(
            OccupancyGrid, '/map', self.on_map,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        self.last_written = 0.0
        self.writes = 0
        self.create_timer(WRITE_EVERY_S, self.maybe_write)
        self.get_logger().info('keeper up, writing to %s' % PATH)

    def on_map(self, message):
        self.map_token = map_id(message)

    def on_pose(self, message):
        self.pose = message
        self.received_at = time.monotonic()

    def maybe_write(self):
        if self.pose is None:
            return
        if self.map_token is None:          # a pose without a frame is useless
            return
        if time.monotonic() - self.received_at > MAX_AGE_S:
            return
        covariance = self.pose.pose.covariance
        sigma_xy = max(math.sqrt(max(0.0, covariance[0])),
                       math.sqrt(max(0.0, covariance[7])))
        sigma_yaw = math.sqrt(max(0.0, covariance[35]))
        if sigma_xy > MAX_SIGMA_XY or sigma_yaw > MAX_SIGMA_YAW:
            return
        if time.monotonic() - self.last_written < WRITE_EVERY_S:
            return

        pose = self.pose.pose.pose
        yaw = math.atan2(2.0 * (pose.orientation.w * pose.orientation.z),
                         1.0 - 2.0 * pose.orientation.z ** 2)
        payload = {
            'x': pose.position.x,
            'y': pose.position.y,
            'yaw': yaw,
            'sigma_xy': sigma_xy,
            'sigma_yaw_deg': math.degrees(sigma_yaw),
            'map': self.map_token,
            'saved_at': time.strftime('%Y-%m-%dT%H:%M:%S'),
        }
        temporary = PATH + '.tmp'
        try:
            with open(temporary, 'w') as handle:
                json.dump(payload, handle)
            os.replace(temporary, PATH)     # atomic: a reader never sees half
            self.last_written = time.monotonic()
            self.writes += 1
            if self.writes == 1:
                self.get_logger().info(
                    'first pose saved: (%.3f, %.3f, %.1f deg) sigma %.3f m / %.1f deg'
                    % (payload['x'], payload['y'], math.degrees(yaw),
                       sigma_xy, payload['sigma_yaw_deg']))
        except OSError as error:
            self.get_logger().warn('cannot write %s: %s' % (PATH, error))


def main():
    rclpy.init()
    node = Keeper()
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
