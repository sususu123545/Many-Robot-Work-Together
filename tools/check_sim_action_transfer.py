"""Check whether a Gazebo-trained policy's joint range is safe on the real arm.

The simulator shares the ArmPi URDF joint convention, so radians convert to
real servo ticks with spec.radians_to_tick().  That makes the *action* side
look almost free.  It is not, for one joint:

spec.joint_limits() reports the servo window (0..1000 for joint3), but crossing
1000 latches the arm node in software with no recovery short of physically
moving the joint back.  So the real safety boundary is tighter than the
declared one, and a policy whose range only just fits inside 0..1000 is still
dangerous.

Both ends of every span are checked, because several joints have a negative
sign in spec.ARM_PI_JOINTS and their low radian value maps to the HIGH tick.

Usage:
    python3 tools/check_sim_action_transfer.py --stats <dataset>/meta/stats.json
    python3 tools/check_sim_action_transfer.py --stats ... --out report.json
"""
import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'ros2_ws/src/vla_controller'))
from vla_controller import spec  # noqa: E402

JOINTS = list(spec.ARM_PI_JOINTS)

#: Hard software limits that are *not* in spec because they are not servo travel
#: limits but consequences of crossing them.  joint3 above 1000 latches the arm
#: node permanently (see the project's joint3 notes); the pre-shutdown ceiling
#: of 900 is the figure that keeps a power-down droop from crossing 1000.
LATCH_LIMITS = {
    'joint3': {'hard_max': 1000, 'pre_shutdown_max': 900,
               'why': 'crossing 1000 latches the arm node with no software recovery'},
}

#: Below this many ticks of margin, a policy is treated as unsafe to execute.
MIN_SAFE_MARGIN_TICKS = 30


def tick_span(name, low_radians, high_radians):
    """Real servo ticks for a radian interval, always returned low..high."""
    _, first = spec.radians_to_tick(name, low_radians)
    _, second = spec.radians_to_tick(name, high_radians)
    return (first, second) if first <= second else (second, first)


def check(joint, low_radians, high_radians):
    low_tick, high_tick = tick_span(joint, low_radians, high_radians)
    window_low, window_high = spec.joint_limits(joint)

    # Margin at both ends: the span may sit anywhere inside the window.
    margin = min(low_tick - window_low, window_high - high_tick)
    reasons = []

    latch = LATCH_LIMITS.get(joint)
    if latch:
        latch_margin = latch['pre_shutdown_max'] - high_tick
        reasons.append({
            'latch_hard_max': latch['hard_max'],
            'pre_shutdown_max': latch['pre_shutdown_max'],
            'margin_to_pre_shutdown': latch_margin,
            'why': latch['why'],
        })
        margin = min(margin, latch_margin)

    return {
        'joint': joint,
        'radians': [round(low_radians, 4), round(high_radians, 4)],
        'ticks': [low_tick, high_tick],
        'servo_window': [window_low, window_high],
        'margin_ticks': margin,
        'safe': margin >= MIN_SAFE_MARGIN_TICKS,
        'extra_limits': reasons,
    }


def load_ranges(path):
    stats = json.loads(path.read_text())
    action = stats.get('action') or stats
    low, high = action.get('min'), action.get('max')
    if not low or not high or len(low) != len(JOINTS):
        raise SystemExit('stats file has no usable action min/max')
    return {joint: (low[i], high[i]) for i, joint in enumerate(JOINTS)}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--stats', type=Path, required=True,
                        help='LeRobot meta/stats.json, or a JSON with action min/max')
    parser.add_argument('--out', type=Path)
    arguments = parser.parse_args(argv)

    ranges = load_ranges(arguments.stats)
    rows = [check(joint, *ranges[joint]) for joint in JOINTS]
    blocked = [r for r in rows if not r['safe']]

    print('%-8s %10s %12s %12s %8s %6s'
          % ('joint', 'radians', 'ticks', 'servo window', 'margin', 'ok'))
    print('-' * 66)
    for row in rows:
        print('%-8s %10s %12s %12s %8d %6s'
              % (row['joint'],
                 '%.3f..%.3f' % tuple(row['radians']),
                 '%d..%d' % tuple(row['ticks']),
                 '%d..%d' % tuple(row['servo_window']),
                 row['margin_ticks'],
                 'yes' if row['safe'] else 'NO'))
        for extra in row['extra_limits']:
            print('         latch risk: high tick %d vs pre-shutdown ceiling %d '
                  '(margin %d); %s'
                  % (row['ticks'][1], extra['pre_shutdown_max'],
                     extra['margin_to_pre_shutdown'], extra['why']))

    print()
    if blocked:
        print('UNSAFE: %s' % ', '.join(r['joint'] for r in blocked))
    else:
        print('all joints keep at least %d ticks of margin' % MIN_SAFE_MARGIN_TICKS)

    report = {
        'schema': 'sim_action_transfer_check_v1',
        'stats': str(arguments.stats),
        'min_safe_margin_ticks': MIN_SAFE_MARGIN_TICKS,
        'joints': rows,
        'unsafe_joints': [r['joint'] for r in blocked],
        'safe_to_execute': not blocked,
        'scope': ('servo-travel check only. It says nothing about whether the '
                  'policy suits the real camera, task or cadence.'),
    }
    if arguments.out:
        arguments.out.parent.mkdir(parents=True, exist_ok=True)
        arguments.out.write_text(json.dumps(report, indent=2) + '\n')
        print('wrote %s' % arguments.out)
    return 0 if not blocked else 1


if __name__ == '__main__':
    raise SystemExit(main())
