#!/usr/bin/env python3
"""Back the car up a requested distance at low speed, then report what the
odometry measured.

Why not just publish cmd_vel blindly: the wheel dead zone is real -- at
0.03 m/s the car covered 0.019 m (calibrated 2026-11-03) -- and mecanum
slippage makes open-loop distances lies. The encoder/odom readback is the
only trustworthy number.
"""

import argparse
import math
import sys
import os
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from rosbridge_client import Rosbridge  # noqa: E402


def yaw_from(msg):
    q = msg.get('pose', {}).get('pose', {}).get('orientation', {})
    return math.atan2(2.0 * (q.get('w', 1.0) * q.get('z', 0.0)),
                      1.0 - 2.0 * (q.get('z', 0.0) ** 2))


def xy_from(msg):
    p = msg.get('pose', {}).get('pose', {}).get('position', {})
    return p.get('x', 0.0), p.get('y', 0.0)


def read_odom(bridge, timeout=6.0):
    # drain first: wait_for_topic(newest=True) only sees the newest message in
    # the *client* inbox; everything still queued on the socket is older and a
    # match on it returns pre-motion data (2026-11-03: a 0.25 m move read 0.000).
    bridge.drain(0.4)
    message = bridge.wait_for_topic('/odom', timeout, newest=True)
    if message is None:
        raise SystemExit('no odometry within %.1f s' % timeout)
    return xy_from(message), yaw_from(message)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--distance', type=float, default=0.25,
                        help='meters to reverse (positive number)')
    parser.add_argument('--speed', type=float, default=0.06,
                        help='m/s; below ~0.05 the wheels do not move at all')
    parser.add_argument('--port', type=int, default=19091)
    args = parser.parse_args()

    duration = args.distance / args.speed
    print('plan: reverse %.2f m at %.2f m/s (~%.1f s)'
          % (args.distance, args.speed, duration), flush=True)

    bridge = Rosbridge('127.0.0.1', args.port, timeout=8)
    try:
        (x0, y0), yaw0 = read_odom(bridge)
        print('odom before: x=%.3f y=%.3f yaw=%.2f rad' % (x0, y0, yaw0), flush=True)

        # back means -x in the body frame
        twist = {'linear': {'x': -args.speed, 'y': 0.0, 'z': 0.0},
                 'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}}
        period = 0.1
        sent = 0
        deadline = time.monotonic() + duration
        next_tick = time.monotonic()
        while time.monotonic() < deadline:
            bridge.publish('/cmd_vel', twist, 'geometry_msgs/msg/Twist')
            sent += 1
            next_tick += period
            wait = next_tick - time.monotonic()
            if wait > 0:
                time.sleep(wait)
        print('published %d cmd_vel frames' % sent, flush=True)

        twist_stop = {'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                      'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}}
        for _ in range(3):
            bridge.publish('/cmd_vel', twist_stop, 'geometry_msgs/msg/Twist')
            time.sleep(0.05)

        time.sleep(0.6)                      # let the final odom sample land
        (x1, y1), yaw1 = read_odom(bridge)
        moved = math.hypot(x1 - x0, y1 - y0)
        print('odom after:  x=%.3f y=%.3f yaw=%.2f rad' % (x1, y1, yaw1), flush=True)
        print('RESULT: moved %.3f m (target %.2f m), yaw drift %.3f rad'
              % (moved, args.distance, abs(yaw1 - yaw0)), flush=True)
    finally:
        bridge.close()


if __name__ == '__main__':
    main()
