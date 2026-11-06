#!/usr/bin/env python3
"""Authorise the arm and jog one joint, verifying that it actually moved.

The arm cannot move straight away: after boot it sits in
``RECOVERED_AWAIT_RESET`` with ``stop_reasons: ['startup_confirmation']``, and
``enable_arm_motion`` is off. Two steps clear that, in this order:

1. ``/arm_controller/reset_estop`` -- clears the boot latch. It also forces both
   motion gates back to false, so authorising first and resetting second would
   silently undo the authorisation.
2. ``/arm_controller/set_parameters`` with ``enable_arm_motion: true`` -- the
   node's own callback re-checks health and refuses if anything is stale.

Verification is the point of this tool. This arm has previously accepted a
command, reported no fault, and not moved (the gripper, measured 2026-11-01:
``reached=False``, ``fault_count=0``). "The service returned success" is not
evidence that a servo turned, so every jog reads the joint feedback back and
compares.

Deliberately conservative: one joint at a time, 60 ticks maximum per call, and a
refusal if the requested target leaves the mechanical range.
"""

import argparse
import json
import math
import sys
import time

import os
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from rosbridge_client import Rosbridge  # noqa: E402

RAD_PER_TICK = math.radians(1.0 / (1000.0 / 240.0))
JOINTS = {  # name: (centre tick, sign, low, high)
    'joint1': (500, 1, 0, 1000),
    'joint2': (500, 1, 125, 875),
    'joint3': (500, -1, 0, 1000),
    'joint4': (500, 1, 0, 1000),
    'joint5': (500, 1, 0, 1000),
    'r_joint': (700, 1, 0, 700),
}


def tick_to_rad(name, tick):
    centre, sign, _, _ = JOINTS[name]
    return sign * (tick - centre) * RAD_PER_TICK


def rad_to_tick(name, radians):
    centre, sign, _, _ = JOINTS[name]
    return int(round(centre + sign * radians / RAD_PER_TICK))


def read_ticks(bridge, timeout=6.0):
    bridge.drain(0.3)
    message = bridge.wait_for_topic('/joint_states', timeout)
    if message is None:
        return None, None
    ticks = {}
    for name, value in zip(message['name'], message['position']):
        if name in JOINTS:
            ticks[name] = rad_to_tick(name, value)
    return ticks, message.get('header', {}).get('stamp')


def status(bridge, timeout=8.0, attempts=3):
    """Read the arm status JSON. Never returns a silent {}: a missed read here
    once made the tool skip the estop reset and the authorisation was refused
    with 'No healthy, explicitly reset feedback' (2026-11-03)."""
    last_error = 'no message within %.1f s' % timeout
    for attempt in range(attempts):
        bridge.drain(0.3)
        message = bridge.wait_for_topic('/arm_controller/status', timeout)
        if message is None:
            last_error = 'no message within %.1f s (attempt %d/%d)' % (
                timeout, attempt + 1, attempts)
            continue
        try:
            return json.loads(message['data'])
        except (KeyError, ValueError, TypeError) as error:
            last_error = 'bad status payload: %s' % error
    raise SystemExit('cannot read /arm_controller/status: %s' % last_error)


def reset(bridge):
    values = bridge.call_service('/arm_controller/reset_estop', [], timeout=20)
    print('  reset_estop -> success=%s  %s'
          % (values.get('success'), values.get('message', '')[:110]), flush=True)
    return bool(values.get('success'))


def authorise(bridge, timeout=15.0):
    # rosbridge maps each positional arg onto one request field. SetParameters has
    # a single field, ``parameters``, which is itself a list -- so the outer list
    # is the call and the inner list is the field value. Getting this wrong makes
    # rosbridge answer "msg is not a list type, but a <class 'dict'>".
    values = bridge.call_service(
        '/arm_controller/set_parameters',
        [[{'name': 'enable_arm_motion', 'value': {'type': 1, 'bool_value': True}}]],
        timeout=timeout)
    results = values.get('results') or [{}]
    first = results[0]
    print('  enable_arm_motion -> successful=%s  %s'
          % (first.get('successful'), str(first.get('reason', ''))[:110]), flush=True)
    return bool(first.get('successful'))


def stamp_from(header_stamp):
    """Robot-clock stamp for the command header, taken from fresh feedback."""
    if not header_stamp:
        return None
    return {'sec': int(header_stamp['sec']), 'nanosec': int(header_stamp['nanosec'])}


def jog(bridge, joint, target_tick, duration=1.0):
    ticks, stamp = read_ticks(bridge)
    if ticks is None or joint not in ticks:
        print('  no fresh feedback for %s' % joint, flush=True)
        return False
    current = ticks[joint]
    low, high = JOINTS[joint][2], JOINTS[joint][3]
    if not low <= target_tick <= high:
        print('  target %d is outside %s range %d..%d' % (target_tick, joint, low, high),
              flush=True)
        return False
    if abs(target_tick - current) > 100:
        print('  refusing a %d-tick single step (arm controller caps at 100)'
              % abs(target_tick - current), flush=True)
        return False

    message = {
        'header': {'stamp': stamp_from(stamp), 'frame_id': ''},
        'joint_names': [joint],
        'points': [{'positions': [tick_to_rad(joint, target_tick)],
                    'velocities': [], 'accelerations': [], 'effort': [],
                    'time_from_start': {'sec': int(duration),
                                        'nanosec': int(round((duration % 1) * 1e9))}}],
    }
    before = current
    bridge.publish('/arm_controller/command', message, 'trajectory_msgs/msg/JointTrajectory')
    print('  commanded %s: %d -> %d ticks over %.1f s'
          % (joint, before, target_tick, duration), flush=True)

    time.sleep(duration + 0.8)
    after_ticks, _ = read_ticks(bridge)
    if after_ticks is None:
        print('  no feedback after the move', flush=True)
        return False
    after = after_ticks.get(joint)
    moved = abs(after - before)
    wanted = abs(target_tick - before)
    print('  measured %s: %d -> %d  (moved %d of %d ticks, error %d)'
          % (joint, before, after, moved, wanted, abs(after - target_tick)), flush=True)
    return moved >= max(2, wanted * 0.6)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--port', type=int, default=19091)
    parser.add_argument('--prepare', action='store_true',
                        help='reset the latch and authorise, then stop')
    parser.add_argument('--joint', default=None,
                        help='joint to jog, e.g. joint1')
    parser.add_argument('--delta-ticks', type=int, default=0,
                        help='signed tick offset from the current position')
    parser.add_argument('--duration', type=float, default=1.0)
    args = parser.parse_args(argv)

    if abs(args.delta_ticks) > 60:
        raise SystemExit('refusing: keep a single jog within 60 ticks')
    if args.joint and args.joint not in JOINTS:
        raise SystemExit('unknown joint %r' % args.joint)

    bridge = Rosbridge('127.0.0.1', args.port, timeout=8)
    try:
        state = status(bridge)
        print('before: estop=%s enable_arm_motion=%s health=%s feedback=%s'
              % (state.get('estop'), state.get('enable_arm_motion'),
                 state.get('health_state'), state.get('feedback_complete')), flush=True)

        if state.get('estop'):
            print('step 1: reset the boot latch', flush=True)
            if not reset(bridge):
                print('reset refused; nothing else attempted', flush=True)
                return 1
        if not status(bridge).get('enable_arm_motion'):
            print('step 2: authorise arm motion', flush=True)
            if not authorise(bridge):
                print('authorisation refused; nothing else attempted', flush=True)
                return 1

        state = status(bridge)
        print('after:  estop=%s enable_arm_motion=%s health=%s'
              % (state.get('estop'), state.get('enable_arm_motion'),
                 state.get('health_state')), flush=True)
        if not state.get('enable_arm_motion'):
            print('motion gate is still closed', flush=True)
            return 1
        if args.prepare or not args.joint or args.delta_ticks == 0:
            print('prepared; no jog requested', flush=True)
            return 0

        ticks, _ = read_ticks(bridge)
        if ticks is None:
            print('no joint feedback', flush=True)
            return 1
        target = ticks[args.joint] + args.delta_ticks
        print('\njogging %s by %+d ticks (now %d)'
              % (args.joint, args.delta_ticks, ticks[args.joint]), flush=True)
        ok = jog(bridge, args.joint, target, args.duration)
        print('\nRESULT: %s' % ('moved as commanded' if ok else 'DID NOT MOVE as commanded'),
              flush=True)
        return 0 if ok else 2
    finally:
        bridge.close()


if __name__ == '__main__':
    sys.exit(main())
