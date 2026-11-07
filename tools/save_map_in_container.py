#!/usr/bin/env python3
"""Save the CURRENT /map from INSIDE the ROS 2 container.

Why a container-side saver exists alongside tools/save_occupancy_map.py
----------------------------------------------------------------------
slam_toolbox publishes /map with transient-local (latched) durability.
rosbridge read back the FIRST frame and kept returning it: after two full
revolutions the PC-side saver still saw 134 occupied cells, exactly the same as
before the survey.  A native rclpy subscriber with a matching QoS gets the real
current map.  Both savers emit the same standard PGM + YAML format.

Run inside the core container:
    source /opt/ros/jazzy/setup.bash
    python3 /tmp/save_map_in_container.py --output /tmp/workshop
"""
import argparse
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy)
from nav_msgs.msg import OccupancyGrid


def pixel(value):
    if value < 0:
        return 205
    if value == 0:
        return 254
    return max(0, min(254, int(round((100.0 - value) * 254.0 / 100.0))))


class Once(Node):
    def __init__(self, output):
        super().__init__('armpi_map_saver_once')
        self.output = output
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.sub = self.create_subscription(OccupancyGrid, '/map', self.on_map, qos)
        self.done = False

    def on_map(self, msg):
        if self.done:
            return
        self.done = True
        width, height = msg.info.width, msg.info.height
        data = msg.data
        header = ('P5\n%d %d\n255\n' % (width, height)).encode('ascii')
        pixels = bytearray()
        for row in reversed(range(height)):
            base = row * width
            for col in range(width):
                pixels.append(pixel(data[base + col]))
        with open(self.output + '.pgm', 'wb') as handle:
            handle.write(header + bytes(pixels))

        q = msg.info.origin.orientation
        yaw = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y),
                                      1 - 2 * (q.y ** 2 + q.z ** 2)))
        with open(self.output + '.yaml', 'w', encoding='utf-8') as handle:
            handle.write(
                'image: %s.pgm\nresolution: %.6f\n'
                'origin: [%.6f, %.6f, %.6f]\n'
                'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n'
                'mode: trinary\n'
                % (self.output.rsplit('/', 1)[-1], msg.info.resolution,
                   msg.info.origin.position.x, msg.info.origin.position.y, yaw))

        unknown = sum(1 for v in data if v < 0)
        free = sum(1 for v in data if v == 0)
        occupied = sum(1 for v in data if v > 0)
        print('MAP %dx%d res=%.3f -> %.2f x %.2f m  origin=(%.2f,%.2f)'
              % (width, height, msg.info.resolution,
                 width * msg.info.resolution, height * msg.info.resolution,
                 msg.info.origin.position.x, msg.info.origin.position.y))
        print('CELLS unknown=%d free=%d occupied=%d known=%d'
              % (unknown, free, occupied, free + occupied))
        print('SAVED %s.pgm %s.yaml' % (self.output, self.output))
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', default='/tmp/workshop')
    parser.add_argument('--timeout', type=float, default=20.0)
    args = parser.parse_args()
    rclpy.init()
    node = Once(args.output)
    import time
    started = time.monotonic()
    while rclpy.ok() and not node.done and time.monotonic() - started < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.3)
    if not node.done:
        print('NO /map received in %.1f s' % args.timeout, file=sys.stderr)
        return 2
    return 0


if __name__ == '__main__':
    sys.exit(main())
