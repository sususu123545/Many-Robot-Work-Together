#!/usr/bin/env python3
"""Measure how far the gripper fingertips hang BELOW the vendor IK tool point.

Why this number matters
-----------------------
A grasp succeeds when the fingertips straddle the ball, i.e. when

    fingertip_z  ==  ball centre z

Writing ``fingertip_z = tool_z - L`` (L = drop of the tips below the tool point)
and ``ball_z = floor_z + R`` for a ball resting on the floor, the IK target
offset becomes simply:

    dz  =  tool_z - ball_z  =  L

So ``dz`` is not a free parameter -- it IS the finger drop.  Every grasp attempt
so far guessed it, and both guesses failed in opposite directions on 2026-11-03:

    dz = 0.030  -> tips pressed on the floor, jaws could not open (r_joint 367)
    dz = 0.069  -> jaws closed 7 cm above the ball ("抓夹太高了")

The image cannot supply L: the camera and the fingers are bolted to the same
wrist, so the fingers sit at a fixed pixel in every frame and two poses are
degenerate for triangulation (verified 2026-11-03 -- the reconstructed point
w wandered by 280 mm between pose pairs).

What works: touch the floor.  Drive joint2 down and the moment the tips reach
the ground the shoulder can no longer rotate, so the measured tick stops
tracking the command.  ``L = tool_z_at_contact - floor_z``.

Two things that would silently corrupt the reading, both handled here:

* the servo dead-band is ~7 ticks, so a 6-tick command moves NOTHING in free
  space and would look exactly like contact.  Steps stay well above it (20
  coarse / 8 fine with a two-strike confirmation).
* a refused command (gate closed, out of range) also reports "did not move".
  Only a line containing ``measured <joint>: ... moved N of M`` counts as a real
  motion reading; anything else is retried, never taken as contact.

    python3 tools/measure_finger_drop.py
    python3 tools/measure_finger_drop.py --coarse 20 --fine 8 --repeat 2
"""

import argparse
import json
import os
import re
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(REPO, 'ros2_ws', 'src', 'armpi_ros2', 'armpi_ros2'))

import arm_poses                                             # noqa: E402
from arm_kinematics import forward_tool_pose_raw             # noqa: E402
from rosbridge_client import Rosbridge                       # noqa: E402

FLOOR_Z = -0.094          # arm_base height of the floor, measured earlier
JOINT2_MIN = 200          # never descend past this: the shoulder would jam
MEASURED = re.compile(r'measured (joint\d): (\d+) -> (\d+)\s+\(moved (\d+) of (\d+)')


def log(message):
    print(message, flush=True)


def jog_measured(joint, delta, duration=1.6):
    """Jog one joint; return (moved, wanted, note) or (None, None, why-not)."""
    result = subprocess.run(
        [sys.executable, os.path.join(HERE, 'arm_jog.py'), '--joint', joint,
         '--delta-ticks', str(int(delta)), '--duration', str(duration)],
        capture_output=True, text=True, timeout=180, cwd=REPO)
    stdout = result.stdout or ''
    match = MEASURED.search(stdout)
    if match is None:
        # Refused, timed out or the feedback never came back: not a stall.
        tail = (stdout.strip().splitlines() or ['no output'])[-1]
        return None, None, tail[:110]
    return int(match.group(4)), int(match.group(5)), ''


def tool_z(ticks):
    return forward_tool_pose_raw(ticks)['translation_m'][2]


def descend(bridge, coarse, fine, max_steps, verbose):
    """Lower joint2 until the shoulder stops tracking; return the contact record."""
    ticks = arm_poses.read_ticks(bridge)
    if ticks is None:
        raise SystemExit('no joint feedback')
    log('  start joint2=%d  tool_z=%+.4f m  (floor %+.4f)' %
        (ticks['joint2'], tool_z(ticks), FLOOR_Z))
    history = []
    strikes = 0
    for index in range(max_steps):
        before = dict(ticks)
        z_now = tool_z(before)
        step = coarse if z_now > FLOOR_Z + 0.10 else fine
        room = before['joint2'] - JOINT2_MIN
        if room < 4:
            return {'contact': False, 'reason': 'joint2 reached the safety floor',
                    'ticks': before, 'history': history}
        delta = -min(step, room)

        moved, wanted, why = jog_measured('joint2', delta)
        if moved is None:
            log('    command refused (%s) -> waiting 8 s' % why)
            time.sleep(8)
            ticks = arm_poses.read_ticks(bridge) or before
            continue

        fresh = arm_poses.read_ticks(bridge)
        if fresh:
            ticks.update(fresh)
        history.append({'joint2': ticks['joint2'], 'asked': wanted, 'moved': moved,
                        'tool_z': round(tool_z(ticks), 4)})
        if verbose:
            log('    step %2d  joint2 %d -> %d (asked %d, moved %d)  tool_z %+.4f'
                % (index + 1, before['joint2'], ticks['joint2'], wanted, moved,
                   tool_z(ticks)))

        # Free space costs the dead-band (~7 ticks); a blocked shoulder costs all
        # of it.  Half the command separates the two with room to spare.
        if moved < wanted * 0.5:
            strikes += 1
            if strikes >= 2:
                z = tool_z(ticks)
                return {'contact': True, 'ticks': ticks, 'tool_z': round(z, 4),
                        'drop_m': round(z - FLOOR_Z, 4), 'history': history}
            log('    step looked blocked, confirming once more')
        else:
            strikes = 0
        time.sleep(0.3)
    return {'contact': False, 'reason': 'no contact within %d steps' % max_steps,
            'ticks': ticks, 'history': history}


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--coarse', type=int, default=20, help='ticks while far from the floor')
    parser.add_argument('--fine', type=int, default=8, help='ticks for the final approach')
    parser.add_argument('--max-steps', type=int, default=40)
    parser.add_argument('--repeat', type=int, default=2, help='descents to compare')
    parser.add_argument('--backs', type=int, default=14, help='ticks to raise between descents')
    parser.add_argument('--verbose', action='store_true', default=True)
    args = parser.parse_args()

    report = {'floor_z': FLOOR_Z, 'descents': []}
    bridge = Rosbridge('127.0.0.1', 19091, timeout=8)
    try:
        log('[1/2] reading the current pose')
        ticks = arm_poses.read_ticks(bridge)
        if ticks is None:
            raise SystemExit('no joint feedback at all')
        log('      %s' % ticks)

        for round_index in range(args.repeat):
            log('[2/2] descent %d/%d  (coarse %d / fine %d ticks)'
                % (round_index + 1, args.repeat, args.coarse, args.fine))
            record = descend(bridge, args.coarse, args.fine, args.max_steps,
                             args.verbose)
            report['descents'].append(record)
            if not record.get('contact'):
                log('  -> no contact: %s' % record.get('reason'))
                break
            log('  -> CONTACT at joint2=%d  tool_z=%+.4f m  L=%.4f m (%.1f mm)'
                % (record['ticks']['joint2'], record['tool_z'],
                   record['drop_m'], record['drop_m'] * 1000))
            log('     releasing the pressure')
            jog_measured('joint2', args.backs)
            time.sleep(1)
            back = arm_poses.read_ticks(bridge)
            if back:
                log('     now joint2=%d tool_z=%+.4f'
                    % (back['joint2'], tool_z(back)))
            time.sleep(0.6)
    finally:
        bridge.close()

    drops = [entry['drop_m'] for entry in report['descents'] if entry.get('contact')]
    touches = [entry['tool_z'] for entry in report['descents'] if entry.get('contact')]
    if drops:
        average = sum(drops) / len(drops)
        report['finger_drop_m'] = round(average, 4)
        report['dz_grasp'] = report['finger_drop_m']
        report['tool_z_floor_touch'] = round(sum(touches) / len(touches), 4)
        report['spread_mm'] = (round((max(drops) - min(drops)) * 1000, 1)
                               if len(drops) > 1 else None)
        suspicious = not 0.015 <= average <= 0.090
        report['ok'] = not suspicious
        report['detail'] = ('fingertips hang %.1f mm below the tool point -> '
                            'grasp IK offset dz = %.3f m' % (average * 1000, average))
        if suspicious:
            report['detail'] += ('  ⚠ outside the plausible 15..90 mm band: '
                                 'something other than the tips touched down first')
    else:
        report['ok'] = False
        report['detail'] = 'never touched the floor'
    print('RESULT ' + json.dumps(report, ensure_ascii=False), flush=True)
    return 0 if report.get('ok') else 2


if __name__ == '__main__':
    sys.exit(main())
