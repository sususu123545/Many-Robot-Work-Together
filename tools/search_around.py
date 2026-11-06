#!/usr/bin/env python3
"""Look for the ball all the way around the car, not just in front.

Why this is needed
------------------
find_ball.py sweeps j1, which covers roughly 130 degrees ahead.  On 2026-11-05 a
ball got nudged out of that sector and the sweep came back empty at both a high
and a low arm pose -- the machine could see 130 degrees and the ball was not in
them.  Closing that gap is a chassis rotation: turn 90 degrees, sweep again.

Safety: the arm is folded up before each turn.  Sweeping the body with the arm
extended out at floor height is how you sweep a coffee cup off a table.

What it reports
---------------
The bearing (degrees relative to the starting heading) and range of the ball, or
a clean "not found in 360 degrees".  It does not drive towards the ball; that is
approach_ball.py's job.

Usage
    python3 tools/search_around.py                 # 4 quadrants, 90 deg each
    python3 tools/search_around.py --quadrants 6   # coarser, faster
    python3 tools/search_around.py --no-rotate     # just sweep the current heading
"""

import argparse
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, 'perception'))

import scan_clean                    # noqa: E402
import grasp_sequence as gs            # noqa: E402
from rosbridge_client import Rosbridge  # noqa: E402

TUCK_J2 = 790          # shoulder high enough that the fingers clear the floor

# The documented search pose from arm_poses.py: covers 0.22-0.88 m of floor with
# the camera 0.18 m up and pitched 40 degrees down.  Every sweep starts here.
# Sweeping from whatever pose the last grasp happened to leave behind makes the
# covered area different every run -- a low shoulder sees 0.17-0.30 m, a high one
# sees 0.22-0.88 m, and neither covers the other.  The operator asked the right
# question: "小车怎么不先进入找东西的姿态".
SEARCH_POSE = {'joint2': 694, 'joint3': 930, 'joint4': 200}


def log(message):
    print(message, flush=True)


def goto_search_pose(bridge):
    """Walk the arm into SEARCH_POSE and report the resulting ticks."""
    ticks = gs.read_ticks(bridge)
    if ticks is None:
        return None
    for joint, value in SEARCH_POSE.items():
        for _ in range(12):
            delta = value - ticks[joint]
            if abs(delta) <= 10:
                break
            gs.jog(joint, max(-55, min(55, delta)), duration=1.6)
            fresh = gs.read_ticks(bridge)
            if fresh is None:
                return None
            ticks = fresh
    log('    search pose reached: j2=%d j3=%d j4=%d'
        % (ticks['joint2'], ticks['joint3'], ticks['joint4']))
    return ticks


def spin(bridge, angular, seconds):
    """Rotate in place.  The mecanum base turns reliably; translation does not."""
    twist = {'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
             'angular': {'x': 0.0, 'y': 0.0, 'z': float(angular)}}
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        bridge.publish('/cmd_vel', twist, 'geometry_msgs/msg/Twist')
        time.sleep(0.05)
    for _ in range(3):
        bridge.publish('/cmd_vel', {'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                                    'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}},
                       'geometry_msgs/msg/Twist')
        time.sleep(0.05)


def tuck(bridge):
    """Fold the arm up so a body rotation cannot sweep anything off the table."""
    ticks = gs.read_ticks(bridge)
    if ticks is None:
        return None
    for _ in range(8):
        if ticks['joint2'] >= TUCK_J2:
            break
        gs.jog('joint2', 45, duration=1.8)
        fresh = gs.read_ticks(bridge)
        if fresh is None:
            return None
        ticks = fresh
    log('    arm folded at j2=%d' % ticks['joint2'])
    return ticks


def sweep_heading(bridge, span, step, settle):
    """j1 sweep at the current heading.  Returns (ticks, located) or (ticks, None)."""
    ticks = gs.read_ticks(bridge)
    if ticks is None:
        return None, None
    home = ticks['joint1']
    offsets = [0]
    for distance in range(step, span + 1, step):
        offsets.extend((distance, -distance))

    for offset in offsets:
        target = home + offset
        low, high = gs.SAFE['joint1']
        if not (low + 10 <= target <= high - 10):
            continue
        current = gs.read_ticks(bridge)
        if current is None:
            return None, None
        delta = target - current['joint1']
        if abs(delta) >= gs.MIN_JOG_TICKS:
            gs.jog('joint1', max(-55, min(55, delta)), duration=1.5)
            time.sleep(settle)
        ticks = gs.read_ticks(bridge)
        if ticks is None:
            return None, None
        located, reason = gs.ball_in_base(ticks)
        if located is None:
            continue
        height = located['object_height_mm']
        row = located['pixel'][1]
        # Wide enough to accept a DISTANT ball.  The height estimate degrades with
        # distance: a real 35 mm ball at 0.5 m came back as 26.7 mm.  With the old
        # 30-40 mm window the sweep saw the ball, computed where it was, called it
        # "not a ball" and carried on rotating -- exactly what the operator
        # reported: "摄像头出现了小球，但是后续还是转圈在寻找".  The basket that
        # must stay rejected reads 236 mm, so 20-45 still separates them cleanly.
        if 20.0 <= height <= 45.0 and row < 462:
            return ticks, located
        log('      j1=%+4d  red thing but not a ball (h=%.0f, v=%.0f)'
            % (offset, height, row))
    return ticks, None


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--quadrants', type=int, default=4)
    parser.add_argument('--span', type=int, default=200, help='j1 sweep either side (ticks)')
    parser.add_argument('--step', type=int, default=50)
    parser.add_argument('--settle', type=float, default=0.7)
    parser.add_argument('--no-rotate', action='store_true')
    parser.add_argument('--rate', type=float, default=0.5, help='rotation rad/s')
    parser.add_argument('--ensure-gate', action='store_true')
    args = parser.parse_args()

    bridge = gs.bind_bridge(Rosbridge('127.0.0.1', 19091, timeout=12))
    bridge.subscribe('/scan', 'sensor_msgs/msg/LaserScan')
    try:
        if args.ensure_gate:
            if not gs.arm_gate(bridge):
                log('cannot authorise the arm')
                return 2

        turns = 1 if args.no_rotate else args.quadrants
        degrees_each = 360.0 / turns if turns > 1 else 0.0

        for index in range(turns):
            if index > 0:
                log('[turn %d] folding the arm, then rotating %.0f deg'
                    % (index + 1, degrees_each))
                if tuck(bridge) is None:
                    log('    lost feedback while folding -- stopping')
                    return 2
                # Rotation clearance (2026-11-06): the car scraped a table
                # corner spinning blindly, and later that same day the check
                # itself started refusing every turn because the robot's own
                # wheels were being counted as obstacles.  Two changes: the
                # self-occlusion table now covers the wheels, and the decision
                # needs 2 of 3 consecutive scans to agree -- the self-returns
                # are intermittent, so one frame used to flip the answer.
                scans = []
                for _ in range(3):
                    bridge.drain(0.4)
                    scan_msg = bridge.wait_for_topic('/scan', timeout=3)
                    if scan_msg is None:
                        continue
                    scans.append(scan_clean.clean_scan(
                        scan_msg['ranges'], scan_msg['angle_increment']))
                    time.sleep(0.15)
                if scans:
                    clear, offenders = scan_clean.rotation_clearance_agreed(scans)
                    log('    rotation clearance over %d scans: %s'
                        % (len(scans), 'clear' if clear else 'BLOCKED'))
                    if not clear:
                        nearest = min(offenders, key=lambda p: p[1])
                        log('    rotation BLOCKED: %d points inside %.2f m '
                            '(nearest %.2f m at %.0f deg) -- skipping this heading'
                            % (len(offenders), scan_clean.ROTATION_RADIUS_M,
                               nearest[1], nearest[0]))
                        continue
                else:
                    log('    no scan available -- refusing to spin blind')
                    continue
                seconds = math.radians(degrees_each) / args.rate
                spin(bridge, args.rate, seconds)
                time.sleep(1.2)

            log('[heading %d/%d] entering the search pose' % (index + 1, turns))
            if goto_search_pose(bridge) is None:
                log('    lost feedback on the way to the search pose')
                return 2
            log('    sweeping j1 by +/-%d ticks' % args.span)
            ticks, located = sweep_heading(bridge, args.span, args.step, args.settle)
            if ticks is None:
                log('    lost joint feedback -- stopping')
                return 2
            if located is not None:
                bearing = degrees_each * index
                ball = located['ball']
                log('')
                log('FOUND at bearing %.0f deg (relative to the start heading)' % bearing)
                log('  ball [%.4f, %.4f, %.4f]  -> %.3f m ahead, %.1f mm tall'
                    % (ball[0], ball[1], ball[2], ball[1], located['object_height_mm']))
                log('  in the CAR frame it is now roughly y=%.3f m ahead' % ball[1])
                log('  next: approach_ball.py to close the distance, then the grasp')
                return 0

        log('')
        log('not found in %.0f degrees of sweep' % (360.0 if turns > 1 else 0.0))
        log('either it left the floor or it is outside this camera\'s usable range')
        return 2
    finally:
        bridge.close()


if __name__ == '__main__':
    sys.exit(main())
