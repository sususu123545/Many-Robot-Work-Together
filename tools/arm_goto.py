#!/usr/bin/env python3
"""Move the arm to an explicit joint target, interpolating in joint space.

Why interpolation: the four IK joints can be hundreds of ticks apart between the
current pose and a grasp pose. Sending that as one timed packet whips the arm
through a wide arc; sending it as N equal joint-space steps keeps every segment
small enough to stop safely and lets each step be verified against feedback.

Guards (this arm has bitten before):
  * joint3 outside 0..1000 latches the arm node permanently -- keep >=40 / <=960;
  * joint2 is mechanically limited to 125..875;
  * every step re-reads feedback, and a joint that moves less than 40% of the
    commanded delta aborts the run instead of continuing blind.
"""

import argparse
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from rosbridge_client import Rosbridge  # noqa: E402

TICK_RAD = math.radians(240.0) / 1000.0     # 1000 ticks span 240 degrees
SIGNS = {'joint1': 1.0, 'joint2': 1.0, 'joint3': -1.0, 'joint4': 1.0}
LIMITS = {'joint1': (0, 1000), 'joint2': (125, 875),
          'joint3': (40, 960), 'joint4': (0, 1000)}


def read_ticks(bridge, timeout=8.0):
    bridge.drain(0.3)
    message = bridge.wait_for_topic('/joint_states', timeout, newest=True)
    if message is None:
        return None
    ticks = {}
    for name, position in zip(message['name'], message['position']):
        if name in SIGNS:
            ticks[name] = int(round(500.0 + SIGNS[name] * float(position) / TICK_RAD))
    return ticks


def fresh_stamp(bridge, timeout=6.0):
    """Robot-clock stamp for the command header.

    A zero stamp is silently discarded by the arm controller -- every grasp
    attempt before 2026-11-03 evening failed with "commanded, not moved" for
    exactly this reason.
    """
    bridge.drain(0.3)
    message = bridge.wait_for_topic('/joint_states', timeout)
    if message is None:
        return {'sec': 0, 'nanosec': 0}
    return message.get('header', {}).get('stamp') or {'sec': 0, 'nanosec': 0}


def send_targets(bridge, targets, duration):
    """One timed packet moving every listed joint together."""
    names = sorted(targets)
    message = {
        'header': {'stamp': fresh_stamp(bridge), 'frame_id': ''},
        'joint_names': names,
        'points': [{'positions': [(targets[name] - 500.0) * SIGNS[name] * TICK_RAD
                                 for name in names],
                    'velocities': [], 'accelerations': [], 'effort': [],
                    'time_from_start': {'sec': int(duration),
                                        'nanosec': int(round((duration % 1) * 1e9))}}],
    }
    bridge.publish('/arm_controller/command', message,
                   'trajectory_msgs/msg/JointTrajectory')


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--j1', type=int, required=True)
    parser.add_argument('--j2', type=int, required=True)
    parser.add_argument('--j3', type=int, required=True)
    parser.add_argument('--j4', type=int, required=True)
    parser.add_argument('--steps', type=int, default=3)
    parser.add_argument('--duration', type=float, default=2.0,
                        help='seconds per interpolated segment')
    parser.add_argument('--tolerance', type=int, default=12)
    parser.add_argument('--port', type=int, default=19091)
    args = parser.parse_args()

    target = {'joint1': args.j1, 'joint2': args.j2,
              'joint3': args.j3, 'joint4': args.j4}
    for name, value in target.items():
        low, high = LIMITS[name]
        if not low <= value <= high:
            raise SystemExit('%s target %d outside safe %d..%d'
                             % (name, value, low, high))

    bridge = Rosbridge('127.0.0.1', args.port, timeout=8)
    try:
        start = read_ticks(bridge)
        if start is None:
            raise SystemExit('no joint feedback')
        print('start:  %s' % start, flush=True)
        print('target: %s' % target, flush=True)

        for step in range(1, args.steps + 1):
            fraction = step / float(args.steps)
            waypoint = {name: int(round(start[name] + fraction * (target[name] - start[name])))
                        for name in target}
            send_targets(bridge, waypoint, args.duration)
            time.sleep(args.duration + 0.8)
            actual = read_ticks(bridge)
            if actual is None:
                print('  step %d: no feedback' % step, flush=True)
                continue
            deltas = {name: actual[name] - start[name] for name in target}
            print('  step %d/%d -> %s  (delta %s)'
                  % (step, args.steps, actual, deltas), flush=True)
            expected = {name: waypoint[name] - start[name] for name in target}
            short = [name for name in target
                     if abs(expected[name]) > 20
                     and abs(deltas[name]) < 0.4 * abs(expected[name])]
            if short:
                print('  ABORT: %s barely moved' % ', '.join(short), flush=True)
                return 2

        final = read_ticks(bridge)
        print('final:  %s' % final, flush=True)
        worst = max(abs(final[name] - target[name]) for name in target)
        print('worst residual: %d ticks' % worst, flush=True)
        return 0 if worst <= args.tolerance else 1
    finally:
        bridge.close()


if __name__ == '__main__':
    sys.exit(main())
