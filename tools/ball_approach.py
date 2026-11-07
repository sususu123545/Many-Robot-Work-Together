#!/usr/bin/env python3
"""Drive the car toward the red ball while the ARM keeps it framed.

The camera rides on the arm (verified 2026-11-03: joint1 pans the image 1:1),
so every chassis yaw also yaws the camera. The loop therefore does three
coupled things each step:

1. aim   -- if the ball is off the car's axis, yaw the chassis toward it and
   undo that yaw on joint1 so the image does not swing;
2. track -- joint1 centres the ball horizontally, joint2 holds the vertical
   position (both axes measured ~2.1 px per *commanded* tick, which already
   includes the usual servo dead-band undershoot);
3. approach -- drive straight at 0.06 m/s (above the measured wheel dead zone)
   until the ball's projected distance reaches the arm's reach margin.

Distance comes from the blob radius: d = FX * BALL_RADIUS / r_px. It assumes
the ball is a 35 mm sphere seen broadside -- good enough to stop on.
"""

import argparse
import math
import os
import subprocess
import sys
import time
import urllib.request
from io import BytesIO

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from PIL import Image                                    # noqa: E402
from car_approach import CX, FX, locate_ball             # noqa: E402

SNAPSHOT = ('http://127.0.0.1:19099/snapshot'
            '?topic=/rear_camera/image_raw&type=ros_compressed')

BALL_RADIUS_M = 0.0175
PX_PER_CMD_TICK = 2.1          # measured on both axes, dead-band included
TICK_DEG = 0.24                # 1000 ticks = 240 deg
JOINT1_REF = 512               # startup-pose tick where the camera looked
                               # along the car axis (calibrated pose)
TARGET_U = 320.0
TARGET_V = 215.0               # hold the ball mid-frame while approaching
STOP_D = 0.38                  # arm reach margin
SPEED = 0.06
YAW_RATE = 0.18                # rad/s, calibrated
MAX_STEP_M = 0.12
MAX_PAN_TICKS = 45
MAX_TILT_TICKS = 35
MAX_AIM_DEG = 11.0
AIM_DEADZONE_DEG = 8.0
MAX_LOOPS = 14


def grab():
    with urllib.request.urlopen(SNAPSHOT, timeout=12) as response:
        return Image.open(BytesIO(response.read())).convert('RGB')


def see():
    image = grab()
    blob, reason = locate_ball(image)
    if blob is None:
        return None, reason
    return blob, ''


def publish_twist(bridge, linear, angular, seconds):
    twist = {'linear': {'x': float(linear), 'y': 0.0, 'z': 0.0},
             'angular': {'x': 0.0, 'y': 0.0, 'z': float(angular)}}
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


def jog(joint, delta, duration=0.8):
    """Return True when the servo measurably moved."""
    result = subprocess.run(
        [sys.executable, os.path.join(HERE, 'arm_jog.py'),
         '--joint', joint, '--delta-ticks', str(int(delta)),
         '--duration', str(duration)],
        capture_output=True, text=True, timeout=120)
    return 'moved as commanded' in result.stdout


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--port', type=int, default=19091)
    parser.add_argument('--stop', type=float, default=STOP_D,
                        help='stop when the ball is this close (m, camera frame)')
    args = parser.parse_args()

    bridge = Rosbridge_bridge(args.port)
    misses = 0
    try:
        for step in range(1, MAX_LOOPS + 1):
            blob, reason = see()
            if blob is None:
                misses += 1
                print('step %d: ball lost (%s), miss %d/2' % (step, reason, misses),
                      flush=True)
                if misses >= 2:
                    print('STOP: cannot see the ball; halting for safety')
                    return 1
                time.sleep(1.0)
                continue
            misses = 0

            u, v, r = blob['center_x'], blob['center_y'], blob['radius_px']
            distance = FX * BALL_RADIUS_M / max(1.0, r)
            print('step %d: u=%.0f v=%.0f r=%.0f -> d=%.2f m' % (step, u, v, r, distance),
                  flush=True)
            if distance <= args.stop:
                print('STOP: within reach (%.2f m <= %.2f m)' % (distance, args.stop))
                return 0

            # --- aim: point the car at the ball (chassis yaw + pan undo) ----
            bearing_cam = math.degrees(math.atan2(u - CX, FX))
            joint1_ticks, _ = read_joint1()
            if joint1_ticks is None:
                joint1_ticks = JOINT1_REF
            pan_from_ref = (joint1_ticks - JOINT1_REF) * TICK_DEG
            bearing_car = bearing_cam - pan_from_ref
            if abs(bearing_car) > AIM_DEADZONE_DEG:
                aim_deg = max(-MAX_AIM_DEG, min(MAX_AIM_DEG, bearing_car))
                aim_rad = math.radians(aim_deg)
                seconds = abs(aim_rad) / YAW_RATE
                print('  aim: car off by %+.1f deg -> yaw %+.2f rad over %.1f s'
                      % (bearing_car, -aim_rad, seconds), flush=True)
                publish_twist(bridge, 0.0, -aim_rad, seconds)
                # the camera yawed with the car; undo it on the pan joint.
                # Car turned right (aim_rad<0) -> ball drifted LEFT in the
                # image -> pan it back right with POSITIVE ticks. Sign verified
                # on the real car 2026-11-03: getting this wrong doubled the
                # error and flung the ball to u=56.
                undo = aim_deg / TICK_DEG
                print('  pan undo: joint1 %+0.f ticks' % undo, flush=True)
                jog('joint1', undo)

            # --- track: keep the ball framed on both axes -------------------
            pan = max(-MAX_PAN_TICKS, min(MAX_PAN_TICKS,
                                          round(-(u - TARGET_U) / PX_PER_CMD_TICK)))
            if abs(pan) >= 6:
                print('  track: joint1 %+d ticks (du=%+.0f px)' % (pan, u - TARGET_U),
                      flush=True)
                jog('joint1', pan)
            tilt = max(-MAX_TILT_TICKS, min(MAX_TILT_TICKS,
                                            round(-(v - TARGET_V) / PX_PER_CMD_TICK)))
            if abs(tilt) >= 6:
                print('  track: joint2 %+d ticks (dv=%+.0f px)' % (tilt, v - TARGET_V),
                      flush=True)
                jog('joint2', tilt)

            # --- approach ----------------------------------------------------
            closing = min(MAX_STEP_M, distance - args.stop)
            seconds = closing / SPEED
            print('  drive: forward %.2f m over %.1f s' % (closing, seconds),
                  flush=True)
            publish_twist(bridge, SPEED, 0.0, seconds)
            time.sleep(0.5)

        print('DONE: loop budget spent; re-run to close further')
        return 0
    finally:
        bridge.close()


def Rosbridge_bridge(port):
    from rosbridge_client import Rosbridge
    return Rosbridge('127.0.0.1', port, timeout=8)


def read_joint1():
    from rosbridge_client import Rosbridge
    try:
        from arm_jog import read_ticks
        bridge = Rosbridge('127.0.0.1', 19091, timeout=8)
        try:
            ticks, _ = read_ticks(bridge)
            return (ticks or {}).get('joint1'), None
        finally:
            bridge.close()
    except Exception as error:                          # noqa: BLE001
        return None, str(error)


if __name__ == '__main__':
    sys.exit(main())
