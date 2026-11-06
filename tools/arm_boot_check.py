#!/usr/bin/env python3
"""Read the six joints and say, before anything moves, whether the arm is usable.

Why this exists
---------------
joint3 can droop past its physical limit while the robot is OFF.  When that
happens the arm node latches: every later command is refused with
``Invalid physical feedback: joint3`` and there is NO software way out -- the
operator has to reach in and push the arm back by hand.  Known occurrences:
2026-11-01, 2026-11-03, 2026-11-06.

The latch is only discoverable by asking.  Everything in the normal pipeline
either moves the arm or assumes it is usable, so the first sign of trouble used
to be a failed grasp several minutes in.  This script costs one read and a
second, and it runs before the first command.

It is strictly READ-ONLY: no gate is taken, no service is called, nothing is
published.  Safe to run on a charging robot.

Exit codes
    0   all six joints inside their physical limits
    2   joint3 is outside 0..1000 -- the arm node will refuse to move; the arm
        needs to be pushed back by hand before anything else is attempted
    1   cannot read joint feedback at all (rosbridge, tunnel, or node down)
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
import servo_health                     # noqa: E402

# Vendor tick encoding, taken from arm_jog.py.  The arm joints span 240 deg over
# 1000 ticks; the gripper spans 180 deg over 1000, so it needs its own scale --
# arm_jog.py applies the arm scale to r_joint as well, which is a latent bug
# there but harmless because it never drives the gripper.
ARM_CENTRE, ARM_RAD_PER_TICK = 500.0, math.radians(240.0) / 1000.0
GRIPPER_CENTRE, GRIPPER_RAD_PER_TICK = 700.0, math.pi / 1000.0

# name: (centre, radians per tick, sign, physical low, physical high)
JOINTS = {
    'joint1': (ARM_CENTRE, ARM_RAD_PER_TICK, 1.0, 0, 1000),
    'joint2': (ARM_CENTRE, ARM_RAD_PER_TICK, 1.0, 125, 875),
    'joint3': (ARM_CENTRE, ARM_RAD_PER_TICK, -1.0, 0, 1000),
    'joint4': (ARM_CENTRE, ARM_RAD_PER_TICK, 1.0, 0, 1000),
    'joint5': (ARM_CENTRE, ARM_RAD_PER_TICK, 1.0, 0, 1000),
    'r_joint': (GRIPPER_CENTRE, GRIPPER_RAD_PER_TICK, 1.0, 0, 700),
}

# The grasp pipeline's own comfortable window for joint3.  Inside the physical
# limit but outside this, the arm moves but has little travel left downward.
COMFORT_JOINT3 = (40, 950)

ORDER = ('joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'r_joint')


def read_ticks(bridge, timeout=8.0):
    """All six joints as integer ticks, or None if feedback is unavailable."""
    bridge.drain(0.4)
    message = bridge.wait_for_topic('/joint_states', timeout, newest=True)
    if message is None:
        return None
    ticks = {}
    for name, position in zip(message.get('name', []), message.get('position', [])):
        if name not in JOINTS:
            continue
        centre, scale, sign, _, _ = JOINTS[name]
        ticks[name] = int(round(centre + sign * float(position) / scale))
    return ticks


def read_status(bridge, timeout=6.0):
    """Arm status, or {} -- a missing status must not stop a joint check."""
    bridge.drain(0.3)
    message = bridge.wait_for_topic('/arm_controller/status', timeout)
    if message is None:
        return {}
    try:
        return json.loads(message['data'])
    except (KeyError, ValueError, TypeError):
        return {}


def inspect(ticks):
    """Return (verdict, code, lines).  Pure function: no I/O, easy to test."""
    lines, joint3_bad = [], False
    for name in ORDER:
        if name not in ticks:
            lines.append('  %-8s  (no feedback)' % name)
            continue
        value = ticks[name]
        _, _, _, low, high = JOINTS[name]
        inside = low <= value <= high
        note = ''
        if name == 'joint3' and not inside:
            joint3_bad = True
            note = '  <-- OUT OF RANGE: the arm node will refuse to move'
        elif name == 'joint3' and not (COMFORT_JOINT3[0] <= value <= COMFORT_JOINT3[1]):
            note = '  (inside the limit, outside the comfortable %d..%d)' % COMFORT_JOINT3
        lines.append('  %-8s  %4d   %-14s %s'
                     % (name, value, '%d..%d' % (low, high),
                        'ok' if inside else 'OUT OF RANGE') + note)
    if joint3_bad:
        return 'UNUSABLE', 2, lines
    return 'READY', 0, lines


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--port', type=int, default=19091)
    parser.add_argument('--tries', type=int, default=3,
                        help='joint-state reads before giving up')
    args = parser.parse_args(argv)

    bridge = Rosbridge('127.0.0.1', args.port, timeout=8)
    try:
        ticks = None
        for attempt in range(args.tries):
            ticks = read_ticks(bridge)
            if ticks is not None:
                break
            print('  joint feedback not available (attempt %d/%d)'
                  % (attempt + 1, args.tries), flush=True)
            time.sleep(1.0)
        if ticks is None:
            print('RESULT: CANNOT READ -- joint_states is silent', flush=True)
            print('  the arm node or the rosbridge tunnel is down; nothing was moved')
            return 1

        status = read_status(bridge)
        print('arm: estop=%s health=%s fault=%s'
              % (status.get('estop'), status.get('health_state'),
                 status.get('last_fault')), flush=True)
        # Temperature and fault history come from servo_health: the controller
        # only polls temperatures for servos 1-4, so a hot servo 5 is invisible
        # here except through last_fault.  It happened on 2026-11-06.
        health = servo_health.read_health(status)
        for line in servo_health.describe(health):
            print('  ' + line, flush=True)
        print('', flush=True)
        verdict, code, lines = inspect(ticks)
        for line in lines:
            print(line, flush=True)
        print('', flush=True)
        if code == 2:
            print('RESULT: UNUSABLE -- joint3 is outside 0..1000.', flush=True)
            print('  The arm node has latched and will refuse every command.')
            print('  This cannot be fixed from software: push joint3 back inside')
            print('  0..1000 by hand, then run this script again.')
            print('  Do NOT send motion commands first -- they will only fail.')
        else:
            tight = [n for n in ORDER if n in ticks
                     and not (JOINTS[n][3] <= ticks[n] <= JOINTS[n][4])]
            print('RESULT: READY -- all six joints are inside their limits.')
            if tight:
                print('  (but out of comfort: %s)' % ', '.join(tight))
        return code
    finally:
        bridge.close()


if __name__ == '__main__':
    sys.exit(main())
