#!/usr/bin/env python3
"""Drive ONE servo back inside its configured raw limit. Runs INSIDE armpi-ros2.

WHY THIS EXISTS
---------------
joint3 (servo 4) droops past raw 1000 while the robot is unpowered. The ROS 2 arm
node then cannot compute a legal angle, latches
    "Arm feedback fault (latched): Invalid physical feedback: joint3"
permanently, and refuses EVERY reset -- so the boot pose can never run. No ROS-side
path can recover it: the node's own move validation needs a valid current position,
which is exactly what is missing. The only way out is the bus itself.

SAFETY CONTRACT
---------------
* Only these commands are ever sent: 0x01 move (to a target INSIDE the limit) and
  reads 0x05 / 0x09. Torque is NEVER loaded or unloaded, and no stop packet is sent.
* One small increment at a time, re-reading position after every message.
* Aborts on: over-temperature, no progress for N messages, movement AWAY from the
  target, or the message cap. An abort leaves the arm where it is -- it never
  tries to "recover" by guessing.

MODE=plan just reads and prints the plan and sends NO move.
MODE=run performs the increments and additionally requires CONFIRM=AREA_CLEAR.
"""
import json
import math
import os
import sys
import time

from armpi_ros2.servo_protocol import ServoBoard, JOINTS

DEVICE = '/dev/rrc'
BAUD = 1000000


def env_int(name, default):
    return int(os.environ.get(name, str(default)))


def env_float(name, default):
    return float(os.environ.get(name, str(default)))


SERVO = env_int('SERVO', 4)            # joint3 == servo 4
FINAL = env_int('FINAL', 907)          # captured joint3 pose, with upper-limit margin
STEP = env_int('STEP', 25)             # raw ticks per message (~6 deg)
DURATION = env_float('DURATION', 1.5)
SETTLE = env_float('SETTLE', 1.8)
MESSAGE_CAP = env_int('MESSAGE_CAP', 12)
TEMP_ABORT = env_int('TEMP_ABORT', 55)
STALL_LIMIT = env_int('STALL_LIMIT', 3)
REGRESS_LIMIT = env_int('REGRESS_LIMIT', 3)
TOLERANCE = env_int('TOLERANCE', 12)
MODE = os.environ.get('MODE', 'plan')
CONFIRM = os.environ.get('CONFIRM', '')

NAME = {v[0]: k for k, v in JOINTS.items()}.get(SERVO, 'servo%d' % SERVO)
LOW, HIGH = {v[0]: (v[3], v[4]) for v in JOINTS.values()}[SERVO]


def sample(board):
    """Position + temperature for the single servo under control."""
    raw = board.position(SERVO)
    temp = board.temperature(SERVO)
    return raw, temp


def next_target(current):
    """One increment closer to FINAL, never outside [LOW, HIGH].

    The bus itself refuses an out-of-limit target: movement_payload() raises
    'Invalid servo ID or raw hardware position' for anything above HIGH. joint3
    sits at 1034, so the FIRST legal message has to be HIGH itself (1000) -- a
    34-tick jump rather than the nominal STEP. Clamping LOW matters too: a single
    STEP must not overshoot past FINAL.
    """
    return min(HIGH, max(FINAL, current - STEP))


def main():
    out = {'servo': SERVO, 'joint': NAME, 'limits': [LOW, HIGH],
           'final_target': FINAL, 'mode': MODE}
    if MODE not in ('plan', 'run'):
        print('REFUSED: MODE must be plan or run', flush=True)
        return 2
    if (SERVO != 4 or not 1 <= STEP <= 25 or not 1 <= MESSAGE_CAP <= 12
            or not 1.5 <= DURATION <= 5 or not math.isfinite(SETTLE) or SETTLE < DURATION
            or not 1 <= TEMP_ABORT <= 55 or not 900 <= FINAL <= 950):
        print('REFUSED: joint3-only bounded slow recovery parameters', flush=True)
        return 2
    if MODE == 'run' and CONFIRM != 'AREA_CLEAR':
        print('REFUSED: MODE=run needs CONFIRM=AREA_CLEAR', flush=True)
        return 2
    if not LOW <= FINAL <= HIGH:
        print('REFUSED: FINAL %d outside servo %d limits [%d,%d]'
              % (FINAL, SERVO, LOW, HIGH), flush=True)
        return 2

    board = ServoBoard(DEVICE, BAUD)
    try:
        if board.limits(SERVO) != [LOW, HIGH]:
            print('REFUSED: hardware limits differ from captured configuration', flush=True)
            return 2
        raw, temp = sample(board)
        print('START %s servo=%d raw=%d temp=%dC limits=[%d,%d] target=%d step=%d'
              % (NAME, SERVO, raw, temp, LOW, HIGH, FINAL, STEP), flush=True)
        out['start_raw'], out['start_temp'] = raw, temp

        # Only the measured near-limit droop band is eligible. No arbitrary
        # unknown position, global limit expansion or blind first move.
        if raw > HIGH + 40 or temp >= TEMP_ABORT or not 0 <= temp < 100:
            print('REFUSED: initial position/temperature outside recovery band', flush=True)
            return 2

        if LOW <= raw <= HIGH and raw <= FINAL + TOLERANCE:
            print('already safely below the upper-limit band; nothing to do', flush=True)
            out.update(reached=True, messages=0, final_raw=raw)
        elif raw < LOW:
            print('REFUSED: raw %d is BELOW the lower limit %d; this script only '
                  'recovers an over-travel, it will not guess a direction' % (raw, LOW),
                  flush=True)
            return 2
        elif MODE == 'plan':
            steps, cursor = [], raw
            while cursor > FINAL and len(steps) < MESSAGE_CAP:
                cursor = next_target(cursor)
                steps.append(cursor)
            print('PLAN %d messages: %s' % (len(steps), steps), flush=True)
            print('PLAN no motion was sent (MODE=plan)', flush=True)
            out.update(planned_messages=len(steps), planned_targets=steps,
                       sent_any_motion=False)
        else:
            messages = 0
            stalled = regressed = 0
            previous = raw
            while messages < MESSAGE_CAP:
                # Read again BEFORE each command, not just after it.
                raw, temp = sample(board)
                if (not LOW <= raw <= HIGH + 40 or not 0 <= temp < TEMP_ABORT
                        or abs(raw - previous) > 3):
                    out['abort'] = 'unstable_or_unsafe_feedback'
                    break
                previous = raw
                target = next_target(previous)
                if target == previous:
                    break
                if abs(target - previous) > (40 if messages == 0 else STEP):
                    out['abort'] = 'step_too_large'
                    break
                board.move(DURATION, [(SERVO, target)])
                messages += 1
                time.sleep(SETTLE)
                raw, temp = sample(board)
                print('  msg %d commanded=%d actual=%d temp=%dC'
                      % (messages, target, raw, temp), flush=True)
                if temp >= TEMP_ABORT:
                    print('ABORT: servo %d reached %dC (limit %dC)'
                          % (SERVO, temp, TEMP_ABORT), flush=True)
                    out['abort'] = 'overtemperature'
                    break
                if raw > previous + 2:
                    regressed += 1
                    print('  moved AWAY from target (%d -> %d), regress %d/%d'
                          % (previous, raw, regressed, REGRESS_LIMIT), flush=True)
                    out['abort'] = 'moved_away'
                    break
                elif abs(raw - previous) <= 2:
                    stalled += 1
                    print('  no movement, stall %d/%d' % (stalled, STALL_LIMIT), flush=True)
                    if stalled >= STALL_LIMIT:
                        out['abort'] = 'no_progress'
                        break
                else:
                    stalled = regressed = 0
                if LOW <= raw <= HIGH and abs(raw - FINAL) <= TOLERANCE:
                    print('  within %d ticks of target; stopping' % TOLERANCE, flush=True)
                    break
                previous = raw
            out.update(messages=messages, final_raw=raw, final_temp=temp)
    finally:
        try:
            board.close()
        except Exception:  # noqa: BLE001
            pass

    if 'final_raw' in out:
        final = out['final_raw']
        out['in_range'] = bool(LOW <= final <= HIGH)
        out['reached'] = bool(out.get('reached') or
                              (out['in_range'] and abs(final - FINAL) <= TOLERANCE))
    print('RESULT_JSON=' + json.dumps(out, sort_keys=True), flush=True)
    return 0 if out.get('reached') else 3


if __name__ == '__main__':
    sys.exit(main())
