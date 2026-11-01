#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Turn an IK result into a job pose file that tools/arm_pose_guarded.py accepts.

WHY: arm_pose_guarded pipes car_setup/ros2/arm_startup_pose_ros.py, which reads
its targets and its limits out of one JSON document. Hand-editing that document
for every experiment is how a wrong hard limit or a stale step size gets in. This
generator copies hard_limits_raw and the step/duration settings verbatim from the
recorded startup pose, then only replaces the joint targets, and refuses to write
anything that the dry run would flag.

It never moves the arm. Getting the numbers wrong here is recoverable; getting
them wrong while the arm is travelling is not.

Usage:
    python tools/perception/make_move_job.py --x 0.0106 --y 0.1634 --z 0.0840 \
        --pitch -130.08 --current-raw joint1=481,joint2=684,joint3=919,joint4=193 \
        --name reach_forward_30mm --out tools/perception/poses/reach_forward_30mm.json
"""
import argparse
import json
import sys
from datetime import datetime, timezone
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / 'ros2_ws' / 'src' / 'armpi_ros2'))

from armpi_ros2.arm_kinematics import inverse_kinematics_all  # noqa: E402

# Servo target index -> the joint name that reports it in /arm_controller/status.
SERVO_TO_JOINT = {3: 'joint4', 4: 'joint3', 5: 'joint2', 6: 'joint1'}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--x', type=float, required=True)
    parser.add_argument('--y', type=float, required=True)
    parser.add_argument('--z', type=float, required=True)
    parser.add_argument('--pitch', type=float, required=True)
    parser.add_argument('--current-raw', required=True,
                        help='joint1=..,joint2=..,joint3=..,joint4=..,joint5=..')
    parser.add_argument('--name', required=True)
    parser.add_argument('--out', required=True)
    parser.add_argument('--base-pose',
                        default=str(REPO / 'car_setup' / 'ros2'
                                    / 'arm_startup_pose.json'))
    parser.add_argument('--keep-joint5', type=int, default=None,
                        help='wrist roll to hold; defaults to the measured value')
    parser.add_argument('--max-travel', type=int, default=120,
                        help='largest single-joint travel allowed for this job')
    args = parser.parse_args()

    current = {}
    for item in args.current_raw.split(','):
        name, _, value = item.partition('=')
        current[name.strip()] = int(value)

    base = json.loads(Path(args.base_pose).read_text(encoding='utf-8'))
    branches = inverse_kinematics_all(args.x, args.y, args.z, args.pitch)
    branch = branches[0]
    servo = branch['servo_targets']

    targets = {}
    for key, name in SERVO_TO_JOINT.items():
        targets[name] = int(servo[key])
    # The wrist roll is NOT an IK output. It must stay where the hand-eye
    # calibration was taken (joint5_raw_reference) or the arm_base->camera
    # transform silently describes a different camera pose.
    targets['joint5'] = (args.keep_joint5 if args.keep_joint5 is not None
                         else current.get('joint5', 499))
    ordered = {name: targets[name]
               for name in ('joint1', 'joint2', 'joint3', 'joint4', 'joint5')}

    print('branch: elbow=%s theta3/4/5/6 = %.1f %.1f %.1f %.1f'
          % (branch['elbow'], branch['theta3'], branch['theta4'],
             branch['theta5'], branch['theta6']))
    problems = []
    for name, value in ordered.items():
        low, high = base['hard_limits_raw'][name]
        delta = value - current[name]
        margin = min(value - low, high - value)
        print('  %-7s target=%4d delta=%+5d  limits=%d..%d  margin=%d'
              % (name, value, delta, low, high, margin))
        if not low <= value <= high:
            problems.append('%s target %d outside %d..%d' % (name, value, low, high))
        if abs(delta) > args.max_travel:
            problems.append('%s travel %+d exceeds --max-travel %d'
                            % (name, delta, args.max_travel))
        if name == 'joint3':
            # joint3 is the axis that has latched the whole arm before. An
            # absolute threshold is the wrong test: the arm currently sits at
            # 919, so a move to 901 is an IMPROVEMENT even though 901 looks
            # "close" to 1000. Refuse only moves that lose margin, or that
            # drive inside 50 ticks of a limit that is not already that close.
            margin_now = min(current[name] - low, high - current[name])
            if margin < min(margin_now, 50):
                problems.append(
                    'joint3 target %d loses ground: margin %d now -> %d after '
                    '(it is the axis that has latched the arm before)'
                    % (value, margin_now, margin))

    if problems:
        for problem in problems:
            print('REFUSED: ' + problem, file=sys.stderr)
        return 2

    startup = dict(base['startup'])
    startup['target_raw'] = ordered
    startup['target_equals_measured'] = False
    startup['decided_by_user'] = False
    startup['ik_job'] = {
        'target_m': [args.x, args.y, args.z],
        'pitch_deg': args.pitch,
        'elbow': branch['elbow'],
        'generated_by': 'tools/perception/make_move_job.py',
    }
    job = {
        'name': args.name,
        'recorded_at': datetime.now(timezone.utc).isoformat(timespec='seconds'),
        'source': 'IK job generated for a guarded travel; not a measured pose',
        'raw': ordered,
        'hard_limits_raw': base['hard_limits_raw'],
        'startup': startup,
        'provenance_warning': (
            'raw is an IK target, NOT a measured pose. Do not copy it into '
            'arm_startup_pose.json.'),
    }
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(job, indent=2) + '\n', encoding='utf-8')
    print('wrote %s' % out)
    return 0


if __name__ == '__main__':
    sys.exit(main())
