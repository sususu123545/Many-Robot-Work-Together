#!/usr/bin/env python3
"""Measure topic rates the way the tools actually consume them: over rosbridge.

Why not `ros2 topic hz`
-----------------------
On this car the ros2cli reading is not trustworthy.  On 2026-11-07 three runs
inside the container gave 5.08, 8.42 and 10.78 Hz for the very same
/rear_camera/image_raw, and `ros2 topic info -v /rear_camera/image_raw` showed
no publisher while the topic was demonstrably publishing.  DDS discovery here
is restricted (ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST, ROS_DOMAIN_ID=42) and
the CLI subscribes from yet another process.  rosbridge is the path every tool
in tools/ actually uses, so that is the number that matters.

Each topic gets its OWN connection.  Subscribing to several topics on one
rosbridge connection interleaves and cannibalises messages (measured), which
would understate the rate.

Duplicate header stamps are reported: a doubled stream must not be mistaken
for a fast one.

Usage:
    python tools/measure_topic_rates.py [--seconds 6] [--host 127.0.0.1] [--port 19091]
"""
import argparse
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, 'perception'))

from rosbridge_client import Rosbridge  # noqa: E402

TOPICS = (
    ('/rear_camera/image_raw/compressed', 'sensor_msgs/msg/CompressedImage'),
    ('/joint_states', 'sensor_msgs/msg/JointState'),
    ('/scan', 'sensor_msgs/msg/LaserScan'),
    ('/odom', 'nav_msgs/msg/Odometry'),
)


def measure(topic, msg_type, seconds, host, port):
    bridge = Rosbridge(host, port, timeout=10)
    bridge.subscribe(topic, msg_type)
    bridge.drain(0.4)
    stamps = []
    started = time.monotonic()
    while time.monotonic() - started < seconds:
        message = bridge.wait_for_topic(topic, timeout=0.5)
        if message is None:
            continue
        stamp = message.get('header', {}).get('stamp', {}) if isinstance(message, dict) else {}
        stamps.append((stamp.get('sec'), stamp.get('nanosec')))
    elapsed = time.monotonic() - started
    bridge.close()
    unique = len(set(stamps))
    return {
        'count': len(stamps),
        'unique': unique,
        'hz': len(stamps) / elapsed,
        'unique_hz': unique / elapsed,
        'duplicates': len(stamps) - unique,
        'elapsed': elapsed,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=float, default=6.0)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=19091)
    args = parser.parse_args()
    for topic, msg_type in TOPICS:
        result = measure(topic, msg_type, args.seconds, args.host, args.port)
        print('%-42s %6.2f Hz  (%2d msgs, %2d unique, %d dup)'
              % (topic, result['hz'], result['count'], result['unique'],
                 result['duplicates']), flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
