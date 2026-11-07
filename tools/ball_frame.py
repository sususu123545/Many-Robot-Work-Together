#!/usr/bin/env python3
"""Frame the ball: report where it is in the image, then (optionally) rotate
the chassis to bring it to the horizontal centre.

The camera is body-mounted on the tail; moving the ARM cannot change where the
ball sits in the frame, only the chassis can. Horizontal framing = yaw; the
vertical position of the ball in frame is set by DISTANCE (the camera pitches
down, so farther = higher in frame), not by rotation.

Rotation direction is not assumed: the camera looks backwards, so image-left /
image-right mapping to chassis yaw is easy to get wrong. This tool takes one
small probe rotation first, measures which way the blob moved, and only then
applies the corrective rotation (two-probe method, same as car_approach.py).
"""

import argparse
import math
import os
import sys
import time
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from PIL import Image                                   # noqa: E402
from car_approach import CX, locate_ball                # noqa: E402
from rosbridge_client import Rosbridge                  # noqa: E402

SNAPSHOT = ('http://127.0.0.1:19099/snapshot'
            '?topic=/rear_camera/image_raw&type=ros_compressed')

TARGET_U = 320.0          # horizontal centre of the 640-wide frame
YAW_RATE = 0.18           # rad/s, calibrated 2026-11-03 ("转向 0.18 rad/s 准")


def grab_frame(timeout=12.0):
    with urllib.request.urlopen(SNAPSHOT, timeout=timeout) as response:
        data = response.read()
    from io import BytesIO
    return Image.open(BytesIO(data)).convert('RGB')


def see():
    image = grab_frame()
    blob, reason = locate_ball(image)
    if blob is None:
        print('  detection failed: %s' % reason)
        return None
    print('  ball: u=%.0f v=%.0f  r=%.1f px  distance=%.2f m  bearing=%+.1f deg'
          % (blob['center_x'], blob['center_y'], blob['radius_px'],
             blob['distance_m'], blob['bearing_deg']))
    return blob


def rotate(bridge, angular, seconds):
    twist = {'linear': {'x': 0.0, 'y': 0.0, 'z': float(angular)},
             'angular': {'x': 0.0, 'y': 0.0, 'z': float(angular)}}
    # note: linear.z is bogus for a ground robot; angular carries the turn
    twist['linear']['z'] = 0.0
    period = 0.1
    deadline = time.monotonic() + seconds
    next_tick = time.monotonic()
    while time.monotonic() < deadline:
        bridge.publish('/cmd_vel', twist, 'geometry_msgs/msg/Twist')
        next_tick += period
        wait = next_tick - time.monotonic()
        if wait > 0:
            time.sleep(wait)
    for _ in range(3):
        bridge.publish('/cmd_vel', {'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                                    'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}},
                       'geometry_msgs/msg/Twist')
        time.sleep(0.05)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--port', type=int, default=19091)
    parser.add_argument('--probe-rad', type=float, default=0.12,
                        help='first probe rotation (always positive; the sign '
                             'of the blob response gives the real mapping)')
    args = parser.parse_args()

    bridge = Rosbridge('127.0.0.1', args.port, timeout=8)
    try:
        bridge.advertise('/cmd_vel', 'geometry_msgs/msg/Twist')

        print('step 1: initial frame', flush=True)
        blob = see()
        if blob is None:
            return 1
        u0 = blob['center_x']

        print('step 2: probe rotation %+0.2f rad' % args.probe_rad, flush=True)
        rotate(bridge, +args.probe_rad, args.probe_rad / YAW_RATE)
        time.sleep(0.6)
        blob = see()
        if blob is None:
            return 1
        u1 = blob['center_x']
        response = u1 - u0
        print('  blob moved %+0.f px for a +yaw probe' % response, flush=True)
        if abs(response) < 4:
            print('  no measurable response; aborting rather than guessing')
            return 1

        gain = response / args.probe_rad          # px per rad, signed
        error_rad = math.atan2(u1 - TARGET_U, 541.286)
        correction = -error_rad * (541.286 / gain)   # px-per-rad -> rad
        print('step 3: corrective rotation %+.3f rad (%.1f s)'
              % (correction, abs(correction) / YAW_RATE), flush=True)
        rotate(bridge, correction, abs(correction) / YAW_RATE)
        time.sleep(0.6)
        blob = see()
        if blob is None:
            return 1
        residual = blob['center_x'] - TARGET_U
        print('RESULT: u=%.0f (target %.0f, residual %+.0f px, ~%+.1f deg)'
              % (blob['center_x'], TARGET_U, residual,
                 math.degrees(math.atan2(residual, 541.286))), flush=True)
        return 0 if abs(residual) < 25 else 2
    finally:
        bridge.close()


if __name__ == '__main__':
    sys.exit(main())
