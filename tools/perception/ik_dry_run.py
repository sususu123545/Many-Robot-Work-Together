#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Dry-run the arm IK for a grasp target. Computes only; never moves anything.

WHY THIS EXISTS: before authorising the first real motion it must be provable
that (a) the kinematics round-trips the pose the arm is actually standing in,
(b) the requested target has a bounded solution at some wrist pitch, and
(c) the servo deltas each joint would be asked to make are small enough to be
sane. Getting any of those wrong on a joint that has already latched out of
range once is not recoverable in software.

Frames: `arm_base` published by arm_camera_tf IS the vendor arm-base frame that
arm_kinematics uses (base_T_camera = base_T_tool @ tool_T_camera, with
base_T_tool from forward_tool_pose_raw), so /grasp/target_base can be fed
straight in without a frame change.

Usage:
    python tools/perception/ik_dry_run.py --x 0.002 --y 0.285 --z -0.043 \
        --raw joint1=481,joint2=684,joint3=919,joint4=193
"""
import argparse
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]
                       / 'ros2_ws' / 'src' / 'armpi_ros2'))

from armpi_ros2.arm_kinematics import (  # noqa: E402
    forward_tool_pose_raw, inverse_kinematics, inverse_kinematics_all)

# Servo target keys from _raw_servo_targets -> the physical joint that reports
# that axis in /arm_controller/status. Easy to get backwards: servo 3 is joint4.
SERVO_TO_JOINT = {3: 'joint4', 4: 'joint3', 5: 'joint2', 6: 'joint1'}
LIMITS = {'joint1': (0, 1000), 'joint2': (125, 875),
          'joint3': (0, 1000), 'joint4': (0, 1000)}


def parse_raw(text):
    raw = {}
    for item in text.split(','):
        name, _, value = item.partition('=')
        raw[name.strip()] = float(value)
    return raw


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--x', type=float, required=True)
    parser.add_argument('--y', type=float, required=True)
    parser.add_argument('--z', type=float, required=True)
    parser.add_argument('--raw', required=True,
                        help='joint1=..,joint2=..,joint3=..,joint4=..')
    parser.add_argument('--pitch', type=float, action='append', default=None,
                        help='wrist pitch to try, degrees; repeatable')
    args = parser.parse_args()

    current = parse_raw(args.raw)
    pose = forward_tool_pose_raw(current)
    angles = pose['angles_deg']
    tip = pose['translation_m']
    pitch_now = angles['theta3'] + angles['theta4'] + angles['theta5']
    print('current tip  = (%.5f, %.5f, %.5f) m  pitch=%.2f deg'
          % (tip[0], tip[1], tip[2], pitch_now))
    print('current ticks= j1=%d j2=%d j3=%d j4=%d'
          % (current['joint1'], current['joint2'],
             current['joint3'], current['joint4']))

    # (a) Round trip. If the solver cannot reproduce the pose we are standing
    # in, the frame or pitch convention is wrong and nothing below is credible.
    print('\nROUND TRIP')
    try:
        again = inverse_kinematics(tip[0], tip[1], tip[2], pitch_now)
    except ValueError as error:
        print('  FAILED: %s  <-- solver disagrees with forward kinematics, '
              'do NOT command motion' % error)
        return 1
    back = again['servo_targets']
    offsets = {'joint1': back[6] - current['joint1'],
               'joint2': back[5] - current['joint2'],
               'joint3': back[4] - current['joint3'],
               'joint4': back[3] - current['joint4']}
    print('  ticks  j1=%d j2=%d j3=%d j4=%d' % (back[6], back[5], back[4], back[3]))
    print('  offset j1=%+d j2=%+d j3=%+d j4=%+d ticks'
          % (offsets['joint1'], offsets['joint2'],
             offsets['joint3'], offsets['joint4']))
    worst = max(abs(v) for v in offsets.values())
    if worst > 20:
        print('  WARNING: round trip is off by %d ticks; the vendor model does '
              'not describe this arm exactly' % worst)

    target_distance = math.dist(tip, (args.x, args.y, args.z))
    print('\nTARGET (%.5f, %.5f, %.5f) m ; %.1f mm from the current tip'
          % (args.x, args.y, args.z, target_distance * 1000))

    # (b)+(c) Candidate solutions, EVERY branch, sorted by travel. The vendor's
    # preferred branch is chosen for teleoperation, not for "whichever is
    # nearest where the arm already is", and on this arm the two branches differ
    # by hundreds of ticks -- a difference worth seeing before anything moves.
    pitches = args.pitch or [-60, -75, -90, -105, -120, -135, -150, -165, -180]
    rows = []
    for pitch in pitches:
        try:
            branches = inverse_kinematics_all(args.x, args.y, args.z, pitch)
        except ValueError as error:
            rows.append((None, pitch, None, str(error)))
            continue
        for branch in branches:
            servo = branch['servo_targets']
            deltas = {}
            out_of_range = []
            for key, name in SERVO_TO_JOINT.items():
                deltas[name] = servo[key] - current[name]
                low, high = LIMITS[name]
                if not low <= servo[key] <= high:
                    out_of_range.append(name)
            travel = max(abs(value) for value in deltas.values())
            rows.append((travel, pitch, (branch, servo, deltas, out_of_range),
                         None))

    rows.sort(key=lambda row: (row[0] is None, row[0] if row[0] is not None else 0))
    print('\n%6s %5s | %-19s | %-19s | %s'
          % ('pitch', 'elbow', 'theta3/4/5/6 deg', 'servo j4/j3/j2/j1',
             'delta j1/j2/j3/j4  (max|d|)'))
    print('-' * 108)
    accepted = 0
    for travel, pitch, payload, error in rows:
        if payload is None:
            print('%6.1f %5s | no solution: %s' % (pitch, '-', error))
            continue
        branch, servo, deltas, out_of_range = payload
        if not out_of_range:
            accepted += 1
        print('%6.1f %5s | %6.1f %6.1f %6.1f %6.1f | %4d %4d %4d %4d | '
              '%+5d %+5d %+5d %+5d  (%3d)%s'
              % (pitch, branch['elbow'], branch['theta3'], branch['theta4'],
                 branch['theta5'], branch['theta6'],
                 servo[3], servo[4], servo[5], servo[6],
                 deltas['joint1'], deltas['joint2'],
                 deltas['joint3'], deltas['joint4'], travel,
                 ('  OUT OF LIMITS: ' + ','.join(sorted(out_of_range))
                  if out_of_range else '')))
    print('\nbranches inside servo limits: %d' % accepted)
    return 0 if accepted else 2


if __name__ == '__main__':
    sys.exit(main())
