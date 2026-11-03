#!/usr/bin/env python3
"""Drive up to the red ball using the camera as the only sensor.

Why this is not just "publish a forward velocity"
------------------------------------------------
Neither the sign of ``/cmd_vel`` nor the sign of a rotation can be assumed on
this robot. The repository's own notes say the hardware mapping differs from the
legacy names, and today's log already contains two wrong conclusions about
whether the base had moved. So this file *measures* both signs with one short
probe each, then uses them. If a probe cannot resolve the sign it refuses to
move rather than guessing.

Detection uses red-channel dominance rather than the HSV rule in ``car_scan``.
Measured on this ball (940 pixels, ``snapshots/backed_off.jpg``): median
saturation 235, and 90% of its pixels would pass the on-board detector's
``S >= 75``. So the repository's colour thresholds are *not* the problem -- the
earlier miss was in this file's own bounding-box logic, not in the parameters.
Dominance is still the better rule here because the ball sits on bare white
tile, where "red beats green and blue" is unambiguous, and it does not care
whether the red is saturated or washed out by exposure.

Control is step-and-confirm: move a short distance, stop, let the base settle,
measure again. Continuous servoing would be faster but every measurement here
comes from a 4 Hz JPEG stream over a tunnel, and a controller that acts on stale
pixels drives into things.

Usage:
    python3 tools/car_approach.py --dry-run     # locate and probe only
    python3 tools/car_approach.py               # approach to ~0.27 m
"""

import argparse
import base64
import io
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)

from PIL import Image, ImageChops  # noqa: E402

from car_scan import connected_components  # noqa: E402
from rosbridge_client import Rosbridge  # noqa: E402

# Calibrated intrinsics of the rear camera (fisheye, equidistant model).
FX, CX, CY = 541.286, 308.972, 220.808
BALL_RADIUS_M = 0.0175          # 35 mm ball

MIN_DIFF = 30                   # R must beat both G and B by this much
MIN_RED = 85                    # and be reasonably bright
FLOOR_FROM = 0.35               # ignore the upper part of the frame


def redness_mask(image):
    """Pixels where red clearly dominates green and blue."""
    red, green, blue = image.convert('RGB').split()
    rg = ImageChops.subtract(red, green)
    rb = ImageChops.subtract(red, blue)
    dominance = ImageChops.darker(rg, rb).point(
        lambda value: 255 if value >= MIN_DIFF else 0)
    bright = red.point(lambda value: 255 if value >= MIN_RED else 0)
    return ImageChops.multiply(dominance, bright)


def locate_ball(image):
    """Return the most ball-like red blob on the floor, or (None, reason)."""
    height = image.size[1]
    mask = redness_mask(image)
    scored = []
    for component in connected_components(mask):
        if not 80 <= component['area'] <= 8000:
            continue
        if component['center_y'] < height * FLOOR_FROM:
            continue
        aspect = component['width'] / float(max(1, component['height']))
        # Wide tolerance on purpose: as the base nears the ball its own chassis
        # clips the lower half, so a circle becomes a horizontal sliver. On this
        # floor nothing else is red, so shape is a weak gate and fill carries it.
        if not 0.45 <= aspect <= 2.20:
            continue
        if component['fill'] < 0.45:
            continue
        component['aspect'] = round(aspect, 2)
        # A sphere projects to a filled disc: aspect 1 and fill ~pi/4.
        scored.append((abs(math.log(aspect)) + abs(component['fill'] - 0.785),
                       component))
    if not scored:
        return None, 'no_floor_candidate'
    scored.sort(key=lambda item: item[0])
    best = scored[0][1]
    best['distance_m'] = FX * BALL_RADIUS_M / max(1.0, best['radius_px'])
    best['bearing_deg'] = math.degrees(math.atan2(best['center_x'] - CX, FX))
    return best, ''


class Robot:
    def __init__(self, bridge):
        self.bridge = bridge
        self.bridge.subscribe('/rear_camera/image_raw/compressed',
                              'sensor_msgs/msg/CompressedImage', throttle_rate=200)
        bridge.advertise('/cmd_vel', 'geometry_msgs/msg/Twist')

    def drive(self, linear=0.0, angular=0.0):
        self.bridge.publish('/cmd_vel', {
            'linear': {'x': float(linear), 'y': 0.0, 'z': 0.0},
            'angular': {'x': 0.0, 'y': 0.0, 'z': float(angular)}})

    def run(self, linear, angular, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.drive(linear, angular)
            time.sleep(0.05)
        self.stop()

    def stop(self, repeats=6):
        for _ in range(repeats):
            self.drive(0.0, 0.0)
            time.sleep(0.03)

    def look(self, settle=0.45, timeout=8.0):
        """Fresh frame after the base has had time to stop moving."""
        time.sleep(settle)
        self.bridge.drain(0.25)
        message = self.bridge.wait_for_topic('/rear_camera/image_raw/compressed',
                                             timeout)
        if message is None:
            return None
        image = Image.open(io.BytesIO(base64.b64decode(message['data'])))
        image.load()
        return image.convert('RGB')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--port', type=int, default=19091)
    parser.add_argument('--dry-run', action='store_true')
    parser.add_argument('--target-m', type=float, default=0.27)
    parser.add_argument('--max-radial-px', type=float, default=70.0,
                        help='abort if the ball gets this big (too close)')
    parser.add_argument('--linear', type=float, default=0.06)
    parser.add_argument('--angular', type=float, default=0.12)
    parser.add_argument('--seconds', type=float, default=75.0)
    args = parser.parse_args(argv)

    if args.linear > 0.08 or args.angular > 0.20:
        raise SystemExit('refusing: exceeds the chassis limits (0.08 m/s, 0.20 rad/s)')

    bridge = Rosbridge('127.0.0.1', args.port, timeout=8)
    robot = Robot(bridge)
    deadline = time.monotonic() + args.seconds

    image = robot.look(settle=0.3)
    if image is None:
        print('no frames', flush=True)
        return 1
    ball, reason = locate_ball(image)
    if ball is None:
        print('cannot see the ball: %s' % reason, flush=True)
        return 1
    print('ball at (%.0f,%.0f) r=%.0f px -> %.2f m, bearing %+.1f deg'
          % (ball['center_x'], ball['center_y'], ball['radius_px'],
             ball['distance_m'], ball['bearing_deg']), flush=True)

    if args.dry_run:
        print('dry run: no motion', flush=True)
        bridge.close()
        return 0

    # -- sign probes -------------------------------------------------------
    # Turn one way; if the bearing error shrinks, that sign centres the ball.
    before = ball['bearing_deg']
    robot.run(0.0, args.angular, 0.6)
    probe, reason = locate_ball(robot.look())
    if probe is None:
        print('lost the ball during the rotation probe: %s' % reason, flush=True)
        bridge.close()
        return 1
    after = probe['bearing_deg']
    turn_sign = 1.0 if abs(after) < abs(before) else -1.0
    print('rotation probe: %+.1f -> %+.1f deg, so centring uses wz %+.0f'
          % (before, after, turn_sign), flush=True)

    # Drive the other way; if the ball grows, that sign approaches.
    radius_before = probe['radius_px']
    robot.run(args.linear, 0.0, 0.9)
    probe2, reason = locate_ball(robot.look())
    if probe2 is None:
        print('lost the ball during the drive probe: %s' % reason, flush=True)
        bridge.close()
        return 1
    drive_sign = 1.0 if probe2['radius_px'] > radius_before else -1.0
    print('drive probe: r %.0f -> %.0f px, so approaching uses vx %+.0f'
          % (radius_before, probe2['radius_px'], drive_sign), flush=True)

    # -- closed loop -------------------------------------------------------
    ball = probe2
    cycles = 0
    while time.monotonic() < deadline:
        cycles += 1
        if ball is None:
            print('ball lost; stopping', flush=True)
            break
        if ball['radius_px'] > args.max_radial_px:
            print('too close (r=%.0f px); stopping' % ball['radius_px'], flush=True)
            break
        if ball['distance_m'] <= args.target_m:
            print('arrived: %.2f m (target %.2f m)' % (ball['distance_m'],
                                                      args.target_m), flush=True)
            break

        bearing = ball['bearing_deg']
        if abs(bearing) > 6.0:
            # Aim the camera at it before driving: a mecanum base pushed along a
            # bearing it is not pointing at slides sideways and loses the ball.
            seconds = min(1.2, max(0.25, abs(bearing) / math.degrees(args.angular)))
            robot.run(0.0, turn_sign * args.angular, seconds)
        else:
            step = min(0.12, max(0.03, ball['distance_m'] - args.target_m))
            robot.run(drive_sign * args.linear, 0.0, step / args.linear)

        ball, reason = locate_ball(robot.look())
        if ball is None:
            print('lost the ball after cycle %d: %s' % (cycles, reason), flush=True)
            break
        print('  cycle %2d: r=%2.0f px  %.2f m  bearing %+5.1f deg'
              % (cycles, ball['radius_px'], ball['distance_m'],
                 ball['bearing_deg']), flush=True)

    final = robot.look(settle=0.6)
    if final is not None:
        final.save(os.path.join(REPO, 'snapshots', 'approach_final.jpg'))
        locate, _ = locate_ball(final)
        if locate:
            print('\nfinal: %.2f m, bearing %+.1f deg, r=%.0f px'
                  % (locate['distance_m'], locate['bearing_deg'], locate['radius_px']),
                  flush=True)
    robot.stop()
    bridge.unadvertise('/cmd_vel')
    bridge.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
