#!/usr/bin/env python3
"""Sweep the arm to find the ball instead of demanding it be placed in view.

Why this exists
---------------
Every run so far started with "put the ball in front of the camera", which is a
mildly absurd requirement for a robot: the camera is on the wrist and the base
joint can turn, so the machine is perfectly capable of looking around.  This
sweeps j1 across a sector, checking each step, and reports where the ball ended
up relative to arm_base.

How the sweep covers ground
---------------------------
At search_home the camera sees roughly 60 degrees horizontally and 0.22-0.88 m of
floor.  Turning j1 by 150 ticks moves the view about 36 degrees either way, so
centre plus both extremes spans roughly 130 degrees -- most of what is in front
of the car.  The order starts at the centre and works outwards, because a ball
someone just rolled towards the robot is usually already nearby.

What it will NOT find
---------------------
Anything beside or behind the car.  Closing that gap needs a chassis rotation
(angular.z on /cmd_vel), which this tool deliberately leaves alone: turning the
body while the arm is extended has its own hazards and deserves its own
supervision.

Usage
    python3 tools/find_ball.py               # scan, report, leave the arm where it found it
    python3 tools/find_ball.py --span 200    # wider sweep
    python3 tools/find_ball.py --return-home # go back to the start angle when done
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

import grasp_sequence as gs           # noqa: E402
from rosbridge_client import Rosbridge  # noqa: E402


def log(message):
    print(message, flush=True)


def look(bridge):
    """One measurement: (ticks, located, reason)."""
    ticks = gs.read_ticks(bridge)
    if ticks is None:
        return None, None, 'no joint feedback'
    located, reason = gs.ball_in_base(ticks)
    if located is None:
        return ticks, None, reason
    return ticks, located, ''


def trustworthy(located):
    """Reject readings that are obviously not a ball sitting on the floor.

    The colour mask happily reports the red plastic basket, red tape on a bag and
    a 236 mm tall "ball" floating 118 mm up.  A real ball is round, fully inside
    the frame and about 35 mm across.
    """
    height = located['object_height_mm']
    row = located['pixel'][1]
    # Accept distant balls too.  The height estimate degrades with range -- a real
    # 35 mm ball at 0.5 m reads 26.7 mm -- so a 30-40 mm window rejects exactly the
    # sightings a sweep is most likely to make.  20-45 still excludes the red
    # basket at 236 mm.
    return 20.0 <= height <= 45.0 and row < 462


def scan(bridge, span, step, settle):
    """Walk j1 outwards from its current angle, returning the first good sighting."""
    ticks, located, reason = look(bridge)
    if ticks is None:
        return None, None, reason
    home = ticks['joint1']
    log('    starting at j1=%d, sweeping +/-%d ticks in %d-tick steps'
        % (home, span, step))

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
            return None, None, 'lost joint feedback mid-sweep'
        delta = target - current['joint1']
        if abs(delta) >= gs.MIN_JOG_TICKS:
            gs.jog('joint1', max(-55, min(55, delta)), duration=1.6)
            time.sleep(settle)
        ticks, located, reason = look(bridge)
        if ticks is None:
            return None, None, 'lost joint feedback mid-sweep'
        if located is None:
            log('    j1=%+4d  nothing' % offset)
            continue
        if trustworthy(located):
            log('    j1=%+4d  FOUND  y=%.3f m  h=%.1f mm  v=%.0f'
                % (offset, located['ball'][1], located['object_height_mm'],
                   located['pixel'][1]))
            return ticks, located, ''
        log('    j1=%+4d  something red but not a ball (h=%.0f mm, v=%.0f)'
            % (offset, located['object_height_mm'], located['pixel'][1]))

    return ticks, None, 'swept %.0f degrees and found nothing' % (span * 0.24)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--span', type=int, default=150,
                        help='how far either side of centre to sweep (ticks, 150 ~ 36 deg)')
    parser.add_argument('--step', type=int, default=30)
    parser.add_argument('--settle', type=float, default=0.7,
                        help='seconds to wait after each move for the image to catch up')
    parser.add_argument('--return-home', action='store_true',
                        help='put j1 back where it started when the sweep finishes')
    parser.add_argument('--ensure-gate', action='store_true',
                        help='authorise arm motion first (needed if nothing has moved yet)')
    args = parser.parse_args()

    bridge = gs.bind_bridge(Rosbridge('127.0.0.1', 19091, timeout=12))
    try:
        if args.ensure_gate:
            log('authorising arm motion')
            if not gs.arm_gate(bridge):
                log('cannot authorise the arm; aborting')
                return 2

        start_ticks = gs.read_ticks(bridge)
        home = start_ticks['joint1'] if start_ticks else None

        log('[1] checking straight ahead first')
        ticks, located, reason = look(bridge)
        if located is not None and trustworthy(located):
            log('    already in view: y=%.3f m, %.1f mm tall'
                % (located['ball'][1], located['object_height_mm']))
            return 0

        log('[2] sweeping')
        ticks, located, reason = scan(bridge, args.span, args.step, args.settle)
        if located is None:
            log('    %s' % reason)
            log('    the ball is probably beside or behind the car -- that needs a')
            log('    chassis rotation, which this tool does not attempt')
            if args.return_home and home is not None and ticks is not None:
                gs.jog('joint1', max(-55, min(55, home - ticks['joint1'])), duration=1.6)
            return 2

        ball = located['ball']
        log('')
        log('ball at [%.4f, %.4f, %.4f]  (y %.3f m ahead, %.0f mm above the floor)'
            % (ball[0], ball[1], ball[2], ball[1], (ball[2] + 0.094) * 1000))
        if ball[1] > 0.26:
            log('note: it is beyond 0.26 m, so approach_ball.py is needed before a grasp')
        if args.return_home and home is not None and ticks is not None:
            gs.jog('joint1', max(-55, min(55, home - ticks['joint1'])), duration=1.6)
        return 0
    finally:
        bridge.close()


if __name__ == '__main__':
    sys.exit(main())
