#!/usr/bin/env python3
"""Look for the ball all the way around the car, not just in front.

Why this is needed
------------------
find_ball.py sweeps j1, which covers roughly 130 degrees ahead.  On 2026-11-05 a
ball got nudged out of that sector and the sweep came back empty at both a high
and a low arm pose -- the machine could see 130 degrees and the ball was not in
them.  Closing that gap is a chassis rotation: turn 90 degrees, sweep again.

Safety: the arm is returned to its boot pose before each turn.  Sweeping the
body with the arm extended out at floor height is how you sweep a coffee cup off
a table.  (Until 2026-11-06 this was a dedicated "fold" that raised the shoulder
until the fingers cleared the floor; it also laid the upper arm across the
lidar's scan plane, so the car kept seeing its own arm and refusing to turn.  It
is gone -- the boot pose is both compact and transparent to the lidar.)

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

# Pose used before every chassis rotation: the boot / reset pose, verbatim from
# car_setup/ros2/arm_startup_pose.json ("target_raw").  There used to be a
# dedicated fold here (TUCK_J2 = 790, later tried at 700): drive the shoulder up
# until "the fingers clear the floor".  It was deleted on the operator's
# instruction on 2026-11-06, and the numbers say he was right -- see
# RESET_POSE_NOTE below.
RESET_POSE = {'joint1': 483, 'joint2': 686, 'joint3': 907, 'joint4': 199}

# Why the reset pose replaces the old fold, in one paragraph of forward
# kinematics (link lengths from grasp_sequence: shoulder 0.0646 m up, upper arm
# 0.1005 m, forearm 0.0947 m, tool 0.1633 m, angles measured from vertical):
#
#   reset  (j2=686, j3=907, j4=199): elbow  70 mm fwd / 136 mm up
#                                    tool  139 mm BEHIND / 99 mm up
#   fold   (j2=822, j3=930, j4=200): elbow  98 mm fwd /  87 mm up   <-- in the beam
#                                    tool  105 mm BEHIND / 195 mm up
#
# Both poses are compact -- the fingers sit behind the arm base and inside
# ROTATION_RADIUS_M (0.25 m) either way, which is the property that actually
# matters: anything the fingers could sweep into already refuses the rotation.
# The difference is the UPPER ARM.  The fold swings it out to 98 mm forward at
# 87 mm up, i.e. straight through the lidar's only scan plane (measured 90-150 mm
# off the floor) and only ~40 mm from the sensor, so the car sees its own arm.
# The reset pose keeps the upper arm above that window.
#
# Measured on the real car before deleting the fold (raw returns under 0.13 m /
# cleaned points inside the 0.25 m rotation radius):
#   j2=740  59 / 1    rotation refused
#   j2=703  40 / 12   rotation refused
#   j2=681  27 / 7    rotation refused
#   j2=661  31 / 0    rotation allowed
#   reset   0 returns under 0.09 m, rotation allowed
# The 700 I picked first was inside the refused band; the operator's "just use
# the reset pose" is the value the measurements actually support.
RESET_POSE_NOTE = 'boot pose, fingers retracted inside the rotation radius'

# The documented search pose from arm_poses.py: covers 0.22-0.88 m of floor with
# the camera 0.18 m up and pitched 40 degrees down.  Every sweep starts here.
# Sweeping from whatever pose the last grasp happened to leave behind makes the
# covered area different every run -- a low shoulder sees 0.17-0.30 m, a high one
# sees 0.22-0.88 m, and neither covers the other.  The operator asked the right
# question: "小车怎么不先进入找东西的姿态".
SEARCH_POSE = {'joint2': 694, 'joint3': 930, 'joint4': 200}


def log(message):
    print(message, flush=True)


def goto_pose(bridge, pose, label):
    """Walk the arm into `pose` (joint -> target tick) and report the ticks.

    Steps are capped at 55 ticks and re-read every iteration, because a single
    >100 tick trajectory is silently dropped by the arm controller and a fixed
    step overshoots: the deleted tuck() jumped a constant +45 and drove j2 from
    694 to 735 when the target was 700.  Using the remaining delta instead
    cannot overshoot by more than the servo deadband.
    """
    ticks = gs.read_ticks(bridge)
    if ticks is None:
        return None
    for joint, value in pose.items():
        for _ in range(12):
            delta = value - ticks[joint]
            if abs(delta) <= 10:
                break
            gs.jog(joint, max(-55, min(55, delta)), duration=1.6)
            fresh = gs.read_ticks(bridge)
            if fresh is None:
                return None
            ticks = fresh
    log('    %s reached: %s' % (label, ' '.join(
        '%s=%d' % (joint, ticks[joint]) for joint in sorted(pose))))
    return ticks


def goto_search_pose(bridge):
    """Walk the arm into SEARCH_POSE and report the resulting ticks."""
    return goto_pose(bridge, SEARCH_POSE, 'search pose')


def yaw_from_odom(message):
    """Yaw in degrees from an Odometry message's quaternion."""
    q = message['pose']['pose']['orientation']
    return math.degrees(math.atan2(2.0 * (q['w'] * q['z'] + q['x'] * q['y']),
                                   1.0 - 2.0 * (q['y'] ** 2 + q['z'] ** 2)))


def wrap_deg(angle):
    return (angle + 180.0) % 360.0 - 180.0


def turn(bridge, degrees, rate=0.25, timeout_s=None):
    """Rotate in place by `degrees`, closed on the wheel encoders.

    Two things are wrong with the old open-loop `spin(bridge, rate, seconds)`:

    1. It published /cmd_vel on the SAME rosbridge connection that already
       subscribed to /scan.  Measured 2026-11-06: a 90 deg turn at 0.5 rad/s
       delivered 25.7 deg (29%), then 17.7 deg, then -0.2 deg, while the
       identical command sent from a dedicated connection moved /odom at
       0.205 rad/s against a 0.18 command (114%).  The scan subscription was
       eating the publish budget, and nothing in the code could see it.
    2. It assumed the car delivers what /cmd_vel asks for.  On this mecanum
       base it does not: measured delivery ratios across sessions are 48, 60,
       81, 101, 112, 127 and 31 percent.  Encoders are the only trustworthy
       displacement source on this chassis, so the turn is driven until the
       yaw has actually changed, not until a timer expires.

    Connections are dedicated and short-lived: one to publish, one to read
    /odom.  Subscribing to two topics on one connection cannibalises messages.

    Returns the delivered turn in degrees (signed, +ve = left), or None if /odom
    never answered.
    """
    cmd = Rosbridge('127.0.0.1', 19091, timeout=10)
    odom = Rosbridge('127.0.0.1', 19091, timeout=10)
    odom.subscribe('/odom', 'nav_msgs/msg/Odometry')
    odom.drain(0.5)
    start = odom.wait_for_topic('/odom', timeout=4)
    if start is None:
        cmd.close()
        odom.close()
        return None
    start_yaw = yaw_from_odom(start)

    target = abs(float(degrees))
    direction = 1.0 if degrees >= 0 else -1.0
    limit = timeout_s if timeout_s is not None else (target / max(rate, 0.05)) * 3.0 + 3.0
    twist = {'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
             'angular': {'x': 0.0, 'y': 0.0, 'z': direction * float(rate)}}
    deadline = time.monotonic() + limit
    travelled = 0.0
    signed = 0.0
    try:
        while time.monotonic() < deadline:
            cmd.publish('/cmd_vel', twist, 'geometry_msgs/msg/Twist')
            message = odom.wait_for_topic('/odom', timeout=0.08)
            if message is not None:
                # Signed, so the log cannot report a -45 deg turn as +45: the
                # caller decides the direction and must be able to see if the
                # car went the other way.
                signed = wrap_deg(yaw_from_odom(message) - start_yaw)
                travelled = abs(signed)
                if travelled >= target:
                    break
            time.sleep(0.02)
    finally:
        for _ in range(4):
            cmd.publish('/cmd_vel', {'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                                     'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}},
                        'geometry_msgs/msg/Twist')
            time.sleep(0.05)
        cmd.close()
        odom.close()
    log('    turned %+.1f of %+.1f deg requested' % (signed, degrees))
    return signed


def goto_reset_pose(bridge):
    """Return the arm to the boot pose before a chassis rotation.

    This is the whole of the old tuck(), and it is deliberately just "go back to
    reset": the reset pose already retracts the fingers behind the arm base
    (139 mm back, 99 mm up) and inside the 0.25 m rotation radius, while keeping
    the upper arm above the lidar's scan plane.  The dedicated fold made the
    lidar worse, not better -- see the note on RESET_POSE.
    """
    return goto_pose(bridge, RESET_POSE, 'reset pose')


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
    parser.add_argument('--rate', type=float, default=0.25,
                        help='rotation rad/s (0.25 measured reliable; 0.5 slipped)')
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
                # Reset pose, not a fold: the arm goes back to the boot pose, the
                # one the lidar was measured clear in.  Then the rotation is
                # gated on 2-of-3 scans seeing nothing inside 0.25 m.
                if goto_reset_pose(bridge) is None:
                    log('    lost feedback going to the reset pose -- stopping')
                    return 2
                log('[turn %d] rotating %.0f deg from the reset pose'
                    % (index + 1, degrees_each))
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
                if turn(bridge, degrees_each, args.rate) is None:
                    log('    /odom never answered -- cannot confirm the turn, '
                        'stopping rather than sweeping a heading we did not reach')
                    return 2
                time.sleep(1.0)

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
