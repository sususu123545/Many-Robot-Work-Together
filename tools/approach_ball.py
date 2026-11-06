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

LIMIT_TOP = 0.34        # hard IK ceiling measured on 2026-11-04
# The band is on the BALL, but what has to stay reachable is the aim point, and
# that sits TOOL_OFFSET_Y further forward.  With a 40 mm offset, a ball at 0.284 m
# puts the aim at 0.324 m where 20 mm-above-the-ball has no IK solution at all.
# Keeping the ball at or below 0.26 m leaves the aim inside the workspace.
GOOD_BAND = (0.20, 0.26)
FRAME_TOP = 250         # keep the ball between these rows...
FRAME_BOTTOM = 420      # ...so it never reaches the 480 edge


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


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--target', type=float, default=0.27,
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
            log('  step %d: y=%.3f m  v=%.0f  (want %.2f)'
                % (step, y, located['pixel'][1], args.target))

            if GOOD_BAND[0] <= y <= GOOD_BAND[1]:
                log('    inside the comfortable band -- done')
                return 0
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

            # AIM: rotate the chassis so the ball sits dead ahead before walking.
            # arm_base: y = forward distance, x = lateral.  Beyond 8 degrees the
            # wrist twist wastes travel and risks pushing the ball sideways, so
            # hand the turn to the chassis (turn_to, odometry-corrected).
            # SIGN NOTE 2026-11-05: the x-axis polarity is confirmed on the first
            # live run -- if the ball drifts FARTHER off-centre after this turn,
            # negate theta below.
            # SIGN CONFIRMED 2026-11-06: x<0 means the ball is LEFT, and a left
            # correction is a POSITIVE chassis turn (turn_to(+) = left, verified
            # by odometry).  The first live run sent atan2's raw -32 and the car
            # turned AWAY from the ball -- it left the frame.  So negate.
            theta = -math.degrees(math.atan2(located['ball'][0], located['ball'][1]))
            if abs(theta) > 8.0:
                log('    aiming: ball sits %+.0f deg off-heading, turning %+0.0f'
                    % (theta, theta))
                turn_to(bridge, theta)
                time.sleep(1.0)
                ticks, located, reason = measure(bridge)
                if located is None:
                    log('    lost the ball while aiming (%s) -- stopping' % reason)
                    return 2
                ball = located['ball']
                y = ball[1]
                log('    after aim: y=%.3f  x=%+.3f' % (y, ball[0]))
                # Turning changes the geometry: the next forward step
                # almost always reads a larger y, which is NOT pushing.
                # Reset the baseline so the push guard compares like
                # with like.
                history = [y]

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
