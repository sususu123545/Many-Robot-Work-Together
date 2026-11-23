#!/usr/bin/env python3
"""Creep the chassis toward the ball, keeping it in the camera the whole way.

Why the chassis has to move at all
----------------------------------
The arm solves IK up to about 0.34 m.  A ball at 0.333 m is technically inside
that, but only at the very top of the workspace: lowering the tool point to 40 mm
above the ball has NO solution, so it cannot be grasped.  Bringing the ball to
0.27 m puts the whole descent inside the reachable set.  Measuring that by hand
("move it a palm closer") is what this replaces.

Why the arm is tucked first
---------------------------
On 2026-11-04 driving forward 100 mm made the ball vanish, and backing off
150 mm revealed it 77 mm FARTHER away than before -- the outstretched arm had
pushed it.  With the shoulder at j2=694 the fingertips hang around y=0.14 with
the ball at 0.33, so any forward motion above about 10 cm puts the fingers
behind the ball and collects it.  Tucking raises the shoulder until the tool
point sits well behind the ball, and only then does the car move.

Why the wrist keeps moving
--------------------------
Driving closer moves the ball DOWN the frame (closer to the camera).  Left alone
it walks off the bottom edge at v=480 in roughly 6 cm, and the run becomes blind.
Each step therefore re-measures the ball and, if it is drifting toward the edge,
tilts the wrist to bring it back to about v=320.

Usage
    python3 tools/approach_ball.py                  # stop at y = 0.27 m
    python3 tools/approach_ball.py --target 0.26 --max-steps 14
    python3 tools/approach_ball.py --dry-run        # measure, do not drive
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

import grasp_sequence as gs          # noqa: E402   (setup, jog, drive, FK, solver)
from rosbridge_client import Rosbridge  # noqa: E402
from scan_clean import clean_scan, nearest_in  # noqa: E402

LIMIT_TOP = 0.34        # hard IK ceiling measured on 2026-11-04
# The band is on the BALL, but what has to stay reachable is the aim point, and
# that sits TOOL_OFFSET_Y further forward.  With a 40 mm offset, a ball at 0.284 m
# puts the aim at 0.324 m where 20 mm-above-the-ball has no IK solution at all.
# Keeping the ball at or below 0.26 m leaves the aim inside the workspace.
GOOD_BAND = (0.20, 0.26)
FRAME_TOP = 250         # keep the ball between these rows...
FRAME_BOTTOM = 420      # ...so it never reaches the 480 edge

AIM_TOL_DEG = 8.0
CENTRE_STEP_DEG = 12.0  # never aim in one bite larger than this
# turn_to() breaks out immediately when the command is <= its 5 deg tolerance,
# so a smaller aim step is silently a no-op: measured 2026-11-10, seven
# consecutive "+5.0 deg" commands left the pixel at 220 every single time.
CENTRE_MIN_DEG = 8.0    # must stay above turn_to's tolerance
MAX_CENTRE_STEPS = 6

# Centre the ball on the IMAGE, not on the 3-D bearing.
# Measured 2026-11-10: the ground estimator reported -68 deg for a ball sitting
# at the left edge of a fisheye whose half-field is 34 deg (f=541 px, half-width
# 320 px), i.e. a factor-of-two error.  Acting on that number turned the ball out
# of frame in BOTH directions (-68 and +68 each lost it, and undoing either
# brought it straight back), which is the signature of a doubled bearing.  The
# pixel column has no such failure mode: it is literally where the camera saw
# the ball, so the aim loop closes on px-320 instead.
PIXEL_CENTRE = 234.0
# NOT the middle of the image.  The camera rides on the wrist, roughly 0.17 m to
# one side of the gripper's centre line, so a ball that looks centred is a ball
# the fingers are NOT over.  Measured 2026-11-11 from two grasps:
#     ball at image column 236  ->  arm_base x = +0.005  ->  GRASPED
#     ball at image column 307  ->  arm_base x = +0.168  ->  closed on nothing
# Those two points give x ~= 0.00228 * (column - 234), so the column that puts the
# ball on the gripper's centre line is ~234, not 320.  Aiming at 320 left the car
# 0.17 m off, and at that reach the wrist is twisted, which is why every grasp
# with the ball off to one side failed while the centred one worked.
PIXEL_TOL = 55.0        # one 8 deg aim step is ~112 px, so this is as good as
                        # the chassis loop can honestly settle: it stops the car
                        # within about 0.12 m of the gripper's centre line, and
                        # the arm's own re-measure does the last few centimetres.
# Pixels per degree, measured on the car: a 12 deg chassis turn moved the ball
# ~168 px (14 px/deg), against the 9.4 px/deg a pinhole f=541 would predict --
# the fisheye spreads the periphery, so the empirical number is the safe one.
PIXELS_PER_DEG = 14.0
CENTRE_GAIN = 0.6       # damped: a full correction overshoots on this lens
# Direction that worked last; the loop below learns it on the first step.
_AIM_SIGN = [1.0]

# --- coming back out of the dead zone -------------------------------------
# On 2026-11-10 the approach itself nudged the ball to 0.129 m.  The old code
# printed "too close now -- stopping" and returned 0 (success), so grasp then
# ran with the ball 129 mm out and the floor guard aborted three times in a row
# ("tool point is only 9 mm above the surface").  There was no way back: the
# only exits were "grasp from a distance that cannot work" or "give up".
# So: below BACKOFF_TRIGGER, reverse in short bursts until BACKOFF_TARGET.
BACKOFF_TRIGGER_M = 0.185   # closer than this and the grasp has no room
BACKOFF_TARGET_M = 0.240    # where to come back to before trying again
BACKOFF_BURST_M = 0.020     # one reverse burst; re-measure after every burst
BACKOFF_MAX_BURSTS = 8      # 8 x 20 mm = 160 mm of travel, hard cap
BACKOFF_REAR_CLEAR_M = 0.30  # never reverse into something this close


def log(message):
    print(message, flush=True)


def turn_by_imu(bridge, angle_deg, rate=0.15, timeout=30):
    """Rotate the chassis by angle_deg, closed-loop on the IMU gyro.

    Two hard chassis constraints discovered 2026-11-05:

    1. MotionLease silently drops everything when cmd_vel gaps exceed
       command_timeout (0.35 s).  The first version interleaved publishing with
       a 0.3 s blocking IMU read, so the effective publish gap was exactly
       0.35 s -- the lease expired between every pair of commands and the car
       sat still while the integrator counted silence.
    2. max_angular = 0.2 rad/s.  MotionLease.accept CLAMPS wz rather than
       rejecting it, so 0.5 rad/s actually drives at 0.2 -- but rate=0.15 keeps
       everything comfortably inside instead of leaning on the clamp.

    The BMI270 publishes raw angular_velocity at 50 Hz with no orientation
    fusion, so yaw comes from integrating |wz| using each message's own header
    stamp.  Between publishes the whole IMU backlog is drained, so no samples
    are lost and the integration stays honest.
    """
    target = math.radians(abs(angle_deg))
    sign = 1.0 if angle_deg > 0 else -1.0
    twist = {'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
             'angular': {'x': 0.0, 'y': 0.0, 'z': sign * rate}}
    stop = {'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}}
    bridge.subscribe('/bmi270/imu_raw', 'sensor_msgs/msg/Imu')
    bridge.drain(0.5)   # never integrate the pre-subscription backlog
    accumulated = 0.0
    last_t = None
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and accumulated < target:
        bridge.publish('/cmd_vel', twist, 'geometry_msgs/msg/Twist')
        # Drain the IMU backlog with a tiny timeout; this inner loop exits as
        # soon as the queue is empty, so the gap between cmd_vel publishes
        # stays well under the 0.35 s lease timeout.
        inner_deadline = time.monotonic() + 0.12
        while time.monotonic() < inner_deadline:
            m = bridge.wait_for_topic('/bmi270/imu_raw', timeout=0.03)
            if m is None:
                break
            stamp = m['header']['stamp']
            t = stamp['sec'] + stamp['nanosec'] * 1e-9
            if last_t is not None:
                dt = t - last_t
                if 0.0 < dt < 0.5:
                    accumulated += abs(m['angular_velocity']['z']) * dt
            last_t = t
    for _ in range(4):
        bridge.publish('/cmd_vel', stop, 'geometry_msgs/msg/Twist')
        time.sleep(0.05)
    return math.degrees(accumulated)


def drive_angular(bridge, wz, seconds):
    """Rotate for `seconds` at wz rad/s, publishing at 20 Hz so the lease lives.

    MotionLease zeroes everything if the publish gap exceeds 0.35 s, so any
    helper that blocks on feedback between publishes will stall the car.  This
    loop publishes continuously and touches nothing else.
    """
    twist = {'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
             'angular': {'x': 0.0, 'y': 0.0, 'z': float(wz)}}
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        bridge.publish('/cmd_vel', twist, 'geometry_msgs/msg/Twist')
        time.sleep(0.05)
    for _ in range(3):
        bridge.publish('/cmd_vel', {'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                                    'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}},
                       'geometry_msgs/msg/Twist')
        time.sleep(0.05)


def drive_lateral(bridge, vy, seconds):
    """Strafe sideways at vy m/s (y+ = left) for `seconds`, 20 Hz publishing.

    Mecanum drive can move without rotating -- the escape hatch when the car
    sits nose-in against furniture: strafing changes position without changing
    heading, so the rotation-clearance check stays satisfied the whole way.
    Calibrated 2026-11-05: 0.06 m/s walked 0.241 m in 4 s with no yaw drift.
    """
    twist = {'linear': {'x': 0.0, 'y': float(vy), 'z': 0.0},
             'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}}
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        bridge.publish('/cmd_vel', twist, 'geometry_msgs/msg/Twist')
        time.sleep(0.05)
    for _ in range(3):
        bridge.publish('/cmd_vel', {'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                                    'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}},
                       'geometry_msgs/msg/Twist')
        time.sleep(0.05)


def read_yaw(bridge):
    """Yaw in degrees from /odom's quaternion (mekanum odometry).

    drain() before every read: without it wait_for_topic hands back the stale
    socket-buffered sample, and four consecutive turns all measured 0.0 deg
    while the car was actually rotating (2026-11-05, the third time this trap
    fired today).
    """
    bridge.drain(0.4)
    m = bridge.wait_for_topic('/odom', timeout=2)
    if m is None:
        return None
    o = m['pose']['pose']['orientation']
    x, y, z, w = o['x'], o['y'], o['z'], o['w']
    return math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def turn_to(bridge, target_deg, max_rounds=5, tolerance=5.0):
    """Turn the chassis by target_deg: adaptive gain + odometry closure.

    Rotation efficiency is NOT a constant: 2026-11-05 evening measured 31%
    (low battery, heavy slip), the 2026-11-06 morning run measured 127%
    (fresh cells, the drive over-shoots its 0.15 rad/s pacing).  A fixed
    gain is therefore wrong by 4x in either direction.  Instead: start at
    gain 1.0, measure the realised angle with odometry after every burst,
    and update the gain from reality.  Bursts are capped at 45 deg of
    COMMANDED rotation so a wrong gain cannot throw the car far off.
    """
    sign = 1.0 if target_deg > 0 else -1.0
    remaining = abs(target_deg)
    gain = 1.0
    history = []
    for round_index in range(max_rounds):
        if remaining <= tolerance:
            break
        burst_cmd = min(remaining * gain, 45.0)
        y0 = read_yaw(bridge)
        drive_angular(bridge, sign * 0.15, burst_cmd / 8.6)  # 0.15 rad/s = 8.6 deg/s
        time.sleep(0.8)
        y1 = read_yaw(bridge)
        if y0 is None or y1 is None:
            history.append({'round': round_index + 1, 'error': 'no odometry'})
            break
        actual = (y1 - y0) * sign
        if actual < 0:
            actual += 360.0
        if actual > 5.0 and burst_cmd > 5.0:
            gain = max(0.3, min(3.0, gain * burst_cmd / actual))
        remaining -= actual
        history.append({'round': round_index + 1, 'cmd_deg': round(burst_cmd, 1),
                        'actual_deg': round(actual, 1),
                        'gain': round(gain, 2),
                        'remaining_deg': round(remaining, 1)})
        if remaining < 0:            # overshot -- report honestly
            history.append({'note': 'overshot by %.1f deg' % -remaining})
            break
    return history


def measure(bridge):
    ticks = gs.read_ticks(bridge)
    if ticks is None:
        return None, None, 'no joint feedback'
    located, reason = gs.ball_in_base(ticks)
    if located is None:
        return ticks, None, reason
    return ticks, located, ''


def tuck(bridge, ticks, ball, margin=0.12):
    """Raise the shoulder until the fingertips are clear behind the ball.

    Returns (ticks, ok).  Without this the car drives the fingers into the ball.
    """
    for _ in range(8):
        tool = gs.forward_tool_pose_raw(ticks)['translation_m']
        clearance = ball[1] - tool[1]
        if clearance >= margin:
            log('    tucked: tool point is %.0f mm behind the ball' % (clearance * 1000))
            return ticks, True
        if ticks['joint2'] >= gs.SAFE['joint2'][1] - 20:
            break
        gs.jog('joint2', 45, duration=2.0)
        fresh = gs.read_ticks(bridge)
        if fresh is None:
            break
        ticks = fresh
    tool = gs.forward_tool_pose_raw(ticks)['translation_m']
    log('    tuck ran out of shoulder travel: only %.0f mm clearance'
        % ((ball[1] - tool[1]) * 1000))
    return ticks, False


def keep_in_view(bridge, ticks, located):
    """Nudge the wrist so the ball stays well inside the frame.

    Driving closer drags the ball toward the bottom edge; pitching the wrist one
    way lifts it back.  Which way was measured, not assumed: at j3=917 raising
    j4 moved the ball from v=462 up to v=320.
    """
    row = located['pixel'][1]
    if FRAME_TOP <= row <= FRAME_BOTTOM:
        return ticks, False
    # Direction measured, not assumed.  Lowering j4 lifts the ball UP the frame:
    # j4 429 -> 214 moved the ball from v=462 to v=320.  So a ball that has sunk
    # towards the bottom edge needs j4 DECREASED.  This used to be the other way
    # round, and on 2026-11-05 a ball at v=421 got j4 +20 and left the frame
    # entirely -- the sweep then reported "lost the ball" and stopped.
    step = -20 if row > FRAME_BOTTOM else 20
    if not (gs.SAFE['joint4'][0] + 25 <= ticks['joint4'] + step <= gs.SAFE['joint4'][1] - 25):
        return ticks, False
    log('    ball at v=%.0f, outside %d..%d -> wrist %+d'
        % (row, FRAME_TOP, FRAME_BOTTOM, step))
    gs.jog('joint4', step, duration=1.5)
    fresh = gs.read_ticks(bridge)
    return (fresh or ticks), True


def rear_clearance(bridge):
    """Nearest obstacle behind the car, in metres, or None if unreadable.

    Reversing is the one motion this file never used to do, so it had no lidar
    guard at all.  Backing into a chair leg 100 mm behind the tail would be
    silent damage: the chassis has no bumper switch.
    """
    bridge.subscribe('/scan', 'sensor_msgs/msg/LaserScan')
    bridge.drain(0.5)
    m = bridge.wait_for_topic('/scan', timeout=4)
    if m is None:
        return None
    points = clean_scan(m['ranges'], m['angle_increment'])
    # Lidar angle 180 deg = straight behind.  +/-25 deg covers the tail width
    # at the 0.3 m distance we care about.
    return nearest_in(points, 180.0, 25.0)


def back_off(bridge, args):
    """Reverse in 20 mm bursts until the ball is back in the comfortable band.

    Every burst re-measures, because the whole point is that the distance was
    wrong -- stepping blind would just overshoot in the other direction.
    Returns (ticks, located, reason); located is None when the ball is gone.
    """
    located = None
    for burst in range(1, BACKOFF_MAX_BURSTS + 1):
        ticks, located, reason = measure(bridge)
        if located is None:
            return ticks, None, reason
        y = located['ball'][1]
        if y >= BACKOFF_TARGET_M:
            log('    backed out to y=%.3f m (%d burst(s))' % (y, burst - 1))
            return ticks, located, ''
        rear = rear_clearance(bridge)
        if rear is None:
            return ticks, located, 'cannot read the lidar to check behind'
        if rear < BACKOFF_REAR_CLEAR_M:
            log('    only %.2f m of floor behind (need %.2f) -- not reversing'
                % (rear, BACKOFF_REAR_CLEAR_M))
            return ticks, located, 'no room behind to back out'
        log('  back %d: y=%.3f m, rear clear %.2f m -- reversing %.0f mm'
            % (burst, y, rear, BACKOFF_BURST_M * 1000))
        gs.drive(bridge, -args.speed, BACKOFF_BURST_M / args.speed)
        time.sleep(1.0)
    ticks, located, reason = measure(bridge)
    return ticks, located, reason


def centre_ball(bridge, args):
    """Turn the chassis until the ball's image column is near the centre.

    Hill-climbing on the pixel error, not trigonometry.  The sign of a chassis
    turn and the honesty of an edge-of-fisheye bearing are both things this car
    has been wrong about, so the loop notices and corrects itself:
      * pixel error shrank  -> keep turning the same way;
      * pixel error grew    -> reverse and keep the flipped sign;
      * ball left the frame -> undo that step, reverse, look again.
    Returns (ticks, located, reason); located is None when both directions have
    been tried and the ball still cannot be found.
    """
    sign = _AIM_SIGN[0]
    last = None
    reversals = 0
    best = None
    for attempt in range(1, MAX_CENTRE_STEPS + 1):
        ticks, located, reason = measure(bridge)
        if located is None:
            if reversals >= 2:
                return ticks, None, reason
            reversals += 1
            sign = -sign
            step = sign * CENTRE_STEP_DEG
            log('    ball is not in view -- trying %+.0f deg' % step)
            turn_to(bridge, step)
            continue
        px = located['pixel'][0]
        error = px - PIXEL_CENTRE
        if abs(error) <= PIXEL_TOL:
            _AIM_SIGN[0] = sign
            log('    centred after %d step(s): pixel x=%.0f' % (attempt - 1, px))
            return ticks, located, ''
        if best is None or abs(error) < best:
            best = abs(error)
        if last is not None and abs(error) > abs(last) + 5.0:
            sign = -sign
            log('    pixel error grew %+.0f -> %+.0f -- turning the other way'
                % (last, error))
        last = error
        # Proportional, not fixed: a fixed 12 deg moves the ball ~170 px, four
        # times the residual error it is trying to remove, so a fixed step can
        # only ping-pong (measured 2026-11-10: 275 -> 106 -> 263 -> 443 -> 273,
        # a clean limit cycle that spun the car in place until it timed out).
        step = min(CENTRE_STEP_DEG,
                   max(CENTRE_MIN_DEG, abs(error) / PIXELS_PER_DEG * CENTRE_GAIN))
        log('    centring: pixel x=%.0f (error %+.0f), turning %+.1f'
            % (px, error, sign * step))
        turn_to(bridge, sign * step)
    ticks, located, reason = measure(bridge)
    if located is not None and best is not None:
        log('    could not centre (best error %+.0f px) -- carrying on'
            % best)
        _AIM_SIGN[0] = sign
    return ticks, located, reason


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    # 0.22 m, not 0.24: measured 2026-11-11, grasps at 0.211-0.231 m land
    # within 3-9 mm of the aim, while at 0.2525 m the IK branch sits 19-39 mm
    # too low and the fingers close under the ball.  Closer is more accurate
    # on this arm, so stop earlier in the band.
    parser.add_argument('--target', type=float, default=0.22,
                        help='stop when the ball is this far ahead (m)')
    parser.add_argument('--step-seconds', type=float, default=0.6)
    parser.add_argument('--speed', type=float, default=0.08)
    parser.add_argument('--max-steps', type=int, default=14)
    parser.add_argument('--margin', type=float, default=0.12,
                        help='how far the tool point must stay behind the ball')
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args()

    bridge = gs.bind_bridge(Rosbridge('127.0.0.1', 19091, timeout=10))
    try:
        ticks, located, reason = measure(bridge)
        if located is None:
            log('cannot see the ball: %s' % reason)
            return 2
        ball = located['ball']
        log('ball at y=%.3f m, %.0f mm above the floor'
            % (ball[1], (ball[2] + 0.094) * 1000))
        if ball[1] > LIMIT_TOP:
            log('ball is already past the %.2f m ceiling -- it will need to come '
                'closer somehow' % LIMIT_TOP)

        log('[1] tucking the arm so the fingers cannot reach the ball')
        if args.dry_run:
            tool = gs.forward_tool_pose_raw(ticks)['translation_m']
            log('    dry run: clearance now %.0f mm'
                % ((ball[1] - tool[1]) * 1000))
            return 0
        ticks, ok = tuck(bridge, ticks, ball, args.margin)
        if not ok:
            log('    refusing to drive: the arm is still in the way')
            return 2

        log('[2] creeping forward, re-measuring after every step')
        history = [ball[1]]
        blind_steps = 0
        for step in range(1, args.max_steps + 1):
            ticks, located, reason = measure(bridge)
            if located is None:
                # Blind creep: when the ball dives off the frame bottom it is
                # usually just short of the grasp band (measured 2026-11-06:
                # lost at y=0.276, grasp then saw 0.301 -- past low-gear IK
                # reach).  Two 15 mm blind steps bring it back into view AND
                # into reach.  Only allowed from the near band, never blind-
                # driving around the room.
                if (history and 0.24 <= history[-1] <= 0.33
                        and blind_steps < 2):
                    blind_steps += 1
                    log('    ball dove off-frame at y=%.3f -- blind creep '
                        '15 mm (%d/2)' % (history[-1], blind_steps))
                    gs.drive(bridge, 0.08, 0.15 / 0.08)
                    time.sleep(1.0)
                    continue
                log('    lost the ball (%s) -- stopping' % reason)
                return 2
            ball = located['ball']
            y = ball[1]
            log('  step %d: y=%.3f m  x=%+.3f  v=%.0f  (want %.2f)'
                % (step, y, ball[0], located['pixel'][1], args.target))

            # AIM FIRST.  "Inside the distance band" is not the same as
            # "graspable": on 2026-11-10 the ball sat at arm_base
            # (-0.60, +0.24) -- 0.24 m ahead but 68 deg off to the side.
            # The old order returned "done" on y alone, so grasp ran and IK
            # refused with "no IK at y=0.240 m" eight rounds in a row, and
            # nothing ever turned the car to face the ball.
            offset_deg = math.degrees(math.atan2(ball[0], ball[1]))
            pixel_error = located['pixel'][0] - PIXEL_CENTRE
            centred = abs(pixel_error) <= PIXEL_TOL

            if centred and GOOD_BAND[0] <= y <= GOOD_BAND[1]:
                log('    inside the comfortable band and centred -- done')
                return 0
            if not centred:
                log('    aiming: ball is %+.0f px off centre (estimator says '
                    '%+.0f deg)' % (pixel_error, offset_deg))
                ticks, located, reason = centre_ball(bridge, args)
                if located is None:
                    log('    could not get the ball in front (%s) -- stopping'
                        % reason)
                    return 2
                ball = located['ball']
                y = ball[1]
                log('    after aim: y=%.3f  x=%+.3f' % (y, ball[0]))
                # Turning changes the geometry, so the push guard must not
                # compare this reading with the pre-turn history.
                history = [y]
                continue
            if y < BACKOFF_TRIGGER_M:
                # Deep in the dead zone: the grasp cannot work from here, so
                # reverse instead of handing a doomed distance to grasp.
                log('    %.3f m is inside the dead zone -- backing out' % y)
                ticks, located, reason = back_off(bridge, args)
                if located is None:
                    log('    lost the ball while backing out (%s) -- stopping'
                        % reason)
                    return 2
                ball = located['ball']
                y = ball[1]
                log('    after back-out: y=%.3f  x=%+.3f' % (y, ball[0]))
                history = [y]
                ticks, moved = keep_in_view(bridge, ticks, located)
                if moved:
                    time.sleep(0.6)
                if GOOD_BAND[0] <= y <= GOOD_BAND[1]:
                    log('    inside the comfortable band -- done')
                    return 0
                if y < BACKOFF_TRIGGER_M:
                    log('    still only %.3f m away and there is no more room to '
                        'reverse -- stopping' % y)
                    return 2
                continue
            if y < GOOD_BAND[0]:
                log('    too close now -- stopping')
                return 0
            # A rising y after a forward command means the car pushed the ball.
            # Stop immediately rather than chasing it across the floor.
            # 20 mm, not 5: a real push moves the ball by centimetres, while
            # 5 mm was inside the localisation noise and fired on ordinary
            # approaches (2026-11-06, right after a 39-deg aim).
            if len(history) >= 2 and y > history[-1] + 0.020:
                log('    y went from %.3f to %.3f after driving forward -- the arm '
                    'is pushing the ball.  Stopping.' % (history[-1], y))
                return 2
            history.append(y)

            ticks, moved = keep_in_view(bridge, ticks, located)
            if moved:
                time.sleep(0.6)

            # AIM used to live here, after the push guard and keep_in_view.
            # It now runs at the top of the loop, before any "done" decision,
            # because a ball can sit inside the distance band while being far
            # enough off to the side that IK has no solution (2026-11-10).

            gap = y - args.target
            seconds = min(args.step_seconds, max(0.25, gap / args.speed))

            # Never advance further than the fingers can go without touching the
            # ball.  The tuck only guarantees MIN clearance, and on 2026-11-05 it
            # reported "tool point is 35 mm behind the ball" -- then this line
            # drove 72 mm and shoved the ball out of the camera.  Cap every step
            # at (clearance - 50 mm) so the fingers always stop short.
            tool_now = gs.forward_tool_pose_raw(ticks)['translation_m']
            clearance = y - tool_now[1]
            allowed = clearance - 0.050
            if allowed < 0.010:
                log('    fingers are only %.0f mm behind the ball -- cannot creep '
                    'closer without hitting it.  Stopping here.' % (clearance * 1000))
                return 0
            if seconds * args.speed > allowed:
                seconds = allowed / args.speed
                log('    step capped at %.0f mm to keep the fingers off the ball'
                    % (allowed * 1000))

            # Cap each step so the ball cannot walk off the bottom of the frame.
            # At 0.08 m/s a 0.6 s burst covers 48 mm, which drags the ball's image
            # row from 374 past the 480 edge -- and then the very next measurement
            # reports "lost the ball" and the run stops.  25 mm keeps it in view.
            if seconds * args.speed > 0.025:
                seconds = 0.025 / args.speed
                log('    step capped at 25 mm to keep the ball in frame')
            log('    driving %.0f mm at %.2f m/s for %.2f s'
                % (seconds * args.speed * 1000, args.speed, seconds))
            gs.drive(bridge, args.speed, seconds)
            time.sleep(1.0)

        log('ran out of steps at y=%.3f m' % history[-1])
        return 2
    finally:
        bridge.close()


if __name__ == '__main__':
    sys.exit(main())
