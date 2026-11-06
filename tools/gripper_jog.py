#!/usr/bin/env python3
"""Gripper travel over rosbridge (SSH exec is unavailable on this board).

Reimplements the safety logic of tools/gripper_guarded.py on the rosbridge
tunnel, because the Pi still cannot fork SSH exec sessions:

  * gate switch is explicit: reset_estop first (it forces BOTH motion gates
    off), then enable_gripper_motion=true together with enable_arm_motion=false
    -- the servo bus only carries one timed packet at a time, so the arm gate
    must be off while the gripper runs;
  * bounded increments only (<= --step ticks per message), re-reading the real
    r_joint feedback after every message;
  * three consecutive reads with zero movement abort the run -- "commanded,
    accepted, not moved" has happened on this gripper before (2026-11-01);
  * the measured r_joint range is 0..700: open end ~75 (85 = resting open),
    jaws meet ~482, and 636 is only an old software clamp that stalls the jaws.

Commands: ``--open`` / ``--close`` take a tick TARGET (absolute).
"""

import argparse
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from rosbridge_client import Rosbridge  # noqa: E402

CENTRE = 700
TICK_RAD = math.pi / 1000.0          # r_joint: 180 deg over 1000 ticks


def status(bridge, timeout=8.0, attempts=3):
    import json
    last = 'no message'
    for attempt in range(attempts):
        bridge.drain(0.3)
        message = bridge.wait_for_topic('/arm_controller/status', timeout)
        if message is None:
            last = 'timeout %d/%d' % (attempt + 1, attempts)
            continue
        try:
            return json.loads(message['data'])
        except (ValueError, TypeError, KeyError) as error:
            last = 'bad payload: %s' % error
    raise SystemExit('cannot read arm status: %s' % last)


def read_r(bridge, timeout=6.0):
    bridge.drain(0.3)
    message = bridge.wait_for_topic('/joint_states', timeout)
    if message is None:
        return None
    for name, value in zip(message.get('name', []), message.get('position', [])):
        if name == 'r_joint':
            return int(round(CENTRE + value / TICK_RAD))
    return None


def reset_estop(bridge, quiet=False):
    values = bridge.call_service('/arm_controller/reset_estop', [], timeout=20)
    if not quiet:
        print('  reset_estop -> success=%s %s' % (values.get('success'),
              str(values.get('message', ''))[:90]), flush=True)
    return bool(values.get('success'))


def set_gates(bridge, arm, gripper, timeout=15.0, quiet=False):
    values = bridge.call_service(
        '/arm_controller/set_parameters',
        [[{'name': 'enable_arm_motion', 'value': {'type': 1, 'bool_value': bool(arm)}},
          {'name': 'enable_gripper_motion', 'value': {'type': 1, 'bool_value': bool(gripper)}}]],
        timeout=timeout)
    results = values.get('results') or [{}]
    ok = all(item.get('successful') for item in results)
    reasons = '; '.join(str(item.get('reason', '')) for item in results if item.get('reason'))
    if not quiet:
        print('  gates arm=%s gripper=%s -> ok=%s %s'
              % (arm, gripper, ok, reasons[:90]), flush=True)
    return ok


def read_stamp(bridge, timeout=6.0):
    """Robot-clock stamp taken from fresh joint feedback.

    Hardcoding ``stamp: {sec: 0}`` does not get ignored -- the arm node rejects
    the whole message with "Command must carry a fresh robot-clock stamp", and
    the only visible symptom is a servo that will not move while every status
    field still reads READY.  That cost a full debugging round on 2026-11-04.
    arm_jog.py already stamps from feedback; this brings the gripper in line.
    """
    bridge.drain(0.3)
    message = bridge.wait_for_topic('/joint_states', timeout)
    if message is None:
        return None
    header = (message.get('header') or {}).get('stamp') or {}
    if 'sec' not in header:
        return None
    return {'sec': int(header['sec']), 'nanosec': int(header.get('nanosec', 0))}


def send_target(bridge, tick, duration=0.45, stamp=None):
    if stamp is None:
        stamp = read_stamp(bridge)
    if stamp is None:
        return False
    message = {
        'header': {'stamp': stamp, 'frame_id': ''},
        'joint_names': ['r_joint'],
        'points': [{'positions': [(tick - CENTRE) * TICK_RAD],
                    'velocities': [], 'accelerations': [], 'effort': [],
                    'time_from_start': {'sec': int(duration),
                                        'nanosec': int(round((duration % 1) * 1e9))}}],
    }
    bridge.publish('/gripper_controller/command', message,
                   'trajectory_msgs/msg/JointTrajectory')
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--target', type=int, required=True,
                        help='absolute r_joint tick, 0..700 (smaller = wider)')
    parser.add_argument('--step', type=int, default=20)
    parser.add_argument('--port', type=int, default=19091)
    parser.add_argument('--max-messages', type=int, default=24)
    args = parser.parse_args()

    if not 0 <= args.target <= 700:
        raise SystemExit('target must stay inside 0..700')
    if not 1 <= args.step <= 60:
        raise SystemExit('keep the increment within 1..60 ticks')

    bridge = Rosbridge('127.0.0.1', args.port, timeout=8)
    try:
        state = status(bridge)
        print('before: estop=%s arm_gate=%s gripper_gate=%s health=%s'
              % (state.get('estop'), state.get('enable_arm_motion'),
                 state.get('enable_gripper_motion'), state.get('health_state')),
              flush=True)

        if not reset_estop(bridge):
            return 1
        # a fresh reset starts a new health window; grant immediately after
        if not set_gates(bridge, arm=False, gripper=True):
            return 1

        start = read_r(bridge)
        if start is None:
            print('no r_joint feedback; refusing to command blind')
            return 1
        print('r_joint start=%d target=%d' % (start, args.target), flush=True)

        position, stalled, sent = start, 0, 0
        while position != args.target and sent < args.max_messages:
            state = status(bridge)
            if state.get('estop') or not state.get('enable_gripper_motion'):
                print('  gate lost at raw=%d; re-securing' % position, flush=True)
                if not (reset_estop(bridge) and set_gates(bridge, False, True)):
                    print('  re-secure failed; stopping at raw=%d' % position)
                    break
                continue
            nxt = position + max(-args.step, min(args.step, args.target - position))
            if not send_target(bridge, nxt):
                print('  no fresh stamp available; feedback stale, stopping')
                break
            sent += 1
            time.sleep(0.45 + 0.4)
            previous, position = position, read_r(bridge)
            if position is None:
                print('  feedback lost after msg %d' % sent, flush=True)
                continue
            temp = '?'
            print('  msg %d target=%d actual=%d'
                  % (sent, nxt, position), flush=True)
            stalled = stalled + 1 if position == previous else 0
            if stalled >= 3:
                print('  jaws stopped responding (contact or fault); stopping',
                      flush=True)
                break

        final = read_r(bridge)
        print('RESULT: start=%d final=%s target=%d messages=%d'
              % (start, final, args.target, sent), flush=True)
        return 0 if final == args.target else 2
    finally:
        bridge.close()


if __name__ == '__main__':
    sys.exit(main())
