#!/usr/bin/env python3
"""Watch /map and say whether it is actually GROWING.

Why this exists
---------------
The first field run produced a map that never changed: unknown 9740 / free 2411
/ occupied 134, identical at every read.  Counting those numbers once cannot
tell "the mapper is alive but the car is standing still" from "the mapper is
dead" -- you have to watch them over time.

slam_toolbox publishes /map with transient-local durability, so a late
subscriber gets the latched copy first; that copy can be arbitrarily old.  The
interesting question is whether NEW maps arrive and whether the free space in
them grows.

Prints one line per received map with the deltas, then a verdict.

Run inside the core container:
    source /opt/ros/jazzy/setup.bash
    python3 /tmp/watch_map_growth.py --seconds 25
"""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid

OCCUPIED_MIN = 65


def census(message):
    data = message.data
    unknown = sum(1 for value in data if value < 0)
    occupied = sum(1 for value in data if value >= OCCUPIED_MIN)
    free = len(data) - unknown - occupied
    return unknown, free, occupied


class Watcher(Node):
    def __init__(self):
        super().__init__('map_growth_watcher')
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(OccupancyGrid, '/map', self.on_map, qos)
        self.started = time.monotonic()
        self.count = 0
        self.first = None
        self.last = None
        self.size = None

    def on_map(self, message):
        self.count += 1
        unknown, free, occupied = census(message)
        if self.first is None:
            self.first = (unknown, free, occupied)
        previous = self.last or (unknown, free, occupied)
        self.last = (unknown, free, occupied)
        self.size = (message.info.width, message.info.height,
                     message.info.resolution)
        print('#%02d t=%+5.1fs %dx%d res=%.3f unknown=%-6d free=%-6d '
              'occ=%-5d d_free=%+6d d_occ=%+5d'
              % (self.count, time.monotonic() - self.started,
                 message.info.width, message.info.height,
                 message.info.resolution, unknown, free, occupied,
                 free - previous[1], occupied - previous[2]), flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=float, default=25.0)
    args = parser.parse_args()

    rclpy.init()
    node = Watcher()
    started = time.monotonic()
    while rclpy.ok() and time.monotonic() - started < args.seconds:
        rclpy.spin_once(node, timeout_sec=0.1)

    print('---')
    if node.first is None:
        print('VERDICT: /map never arrived in %.0f s '
              '(no latched copy, no new map)' % args.seconds)
    else:
        first = node.first
        last = node.last
        print('maps received   : %d' % node.count)
        print('first           : unknown=%d free=%d occ=%d' % first)
        print('last            : unknown=%d free=%d occ=%d' % last)
        print('growth          : free %+d, occ %+d' % (last[1] - first[1],
                                                      last[2] - first[2]))
        if node.count == 1:
            print('VERDICT: only the latched copy arrived -- the mapper is NOT '
                  'publishing newer maps')
        elif last[1] <= first[1] and last[2] <= first[2]:
            print('VERDICT: newer maps arrive but nothing grew -- the mapper is '
                  'running but rejecting/discarding scans')
        else:
            print('VERDICT: the map IS growing -- mapping works')
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
