"""Audit SmolVLA output against the three candidate SO100 source domains.

Two questions, one tool, because they share the same statistics files:

  domains  Which source-statistics domain reverse-normalizes the whole action
           chunk into physically possible values?  A domain can look fine on
           the first three steps and still breach limits deeper in the chunk,
           so every step is checked.

  state    How far does the ArmPi observation sit from that domain's state
           distribution?  Reverse-normalizing cleanly proves only that the
           OUTPUT side is self-consistent; if the INPUT is dozens of sigma
           outside the pretrained distribution the model is extrapolating and
           the chunk has no meaning for this robot.

Neither subcommand executes anything or claims an adapter exists.  The
statistics describe the source dataset, not ArmPi, so a passing audit means
"numerically plausible in SO100 coordinates", never "valid ArmPi command".
"""
import argparse
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
STATS_DIR = ROOT / 'tools' / 'config' / 'smolvla_source_stats'

# SO100 channel order, and the ArmPi joint each one is being compared to.
# The mapping itself is unverified; it is here so the report is readable.
CHANNELS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex',
            'wrist_flex', 'wrist_roll', 'gripper']
ARMPI_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'r_joint']

# Physical possibility bounds, in each channel's own unit.  Joint angles are
# degrees; the gripper is an opening percentage.
JOINT_LIMIT_DEG = 180.0
GRIPPER_LIMIT_PCT = (0.0, 100.0)


def available_domains():
    return sorted(p.stem for p in STATS_DIR.glob('*.json'))


def load_statistics(domain, feature):
    path = STATS_DIR / (domain + '.json')
    if not path.is_file():
        raise SystemExit('unknown domain %r; available: %s'
                         % (domain, ', '.join(available_domains())))
    payload = json.loads(path.read_text())
    statistics = payload.get('statistics', {}).get(feature)
    if not statistics:
        raise SystemExit('%s has no %s statistics' % (domain, feature))
    mean, std = statistics.get('mean'), statistics.get('std')
    if not mean or not std or len(mean) != 6 or len(std) != 6:
        raise SystemExit('%s %s statistics are not six-dimensional' % (domain, feature))
    if any(s <= 0 for s in std):
        raise SystemExit('%s %s has a non-positive std' % (domain, feature))
    return mean, std


def load_chunk(path):
    """Return the [steps][6] chunk stored by the probe."""
    payload = json.loads(path.read_text())
    batch = payload.get('model_normalized')
    if batch is None:
        raise SystemExit('%s has no model_normalized array' % path)
    if len(batch) != 1 or not batch[0]:
        raise SystemExit('%s must hold exactly one nonempty chunk' % path)
    chunk = batch[0]
    for row in chunk:
        if len(row) != 6 or not all(type(v) in (int, float) and math.isfinite(v) for v in row):
            raise SystemExit('%s has a malformed action row' % path)
    return chunk


def limits_for(channel):
    if channel == 'gripper':
        return GRIPPER_LIMIT_PCT
    return -JOINT_LIMIT_DEG, JOINT_LIMIT_DEG


def audit_domain(domain, chunk):
    mean, std = load_statistics(domain, 'action')
    rows = [[z[i] * std[i] + mean[i] for i in range(6)] for z in chunk]

    channels, breaches = [], []
    for index, channel in enumerate(CHANNELS):
        values = [row[index] for row in rows]
        low_limit, high_limit = limits_for(channel)
        outside = sum(1 for v in values if v < low_limit or v > high_limit)
        jumps = [abs(values[k + 1] - values[k]) for k in range(len(values) - 1)]
        channels.append({
            'channel': channel,
            'armpi_joint': ARMPI_JOINTS[index],
            'first': round(values[0], 3),
            'last': round(values[-1], 3),
            'min': round(min(values), 3),
            'max': round(max(values), 3),
            'net_change': round(values[-1] - values[0], 3),
            'max_step_change': round(max(jumps), 3) if jumps else 0.0,
            'outside_limit': outside,
        })
        if outside:
            breaches.append({
                'channel': channel,
                'steps_outside': outside,
                'observed_range': [round(min(values), 3), round(max(values), 3)],
                'allowed_range': [low_limit, high_limit],
            })
    return {
        'domain': domain,
        'steps': len(chunk),
        'channels': channels,
        'breaches': breaches,
        'steps_outside_total': sum(c['outside_limit'] for c in channels),
        'max_step_change': max(c['max_step_change'] for c in channels),
    }


def audit_state(domain, observation):
    if observation.get('state_unit') != 'rad':
        raise SystemExit('observation.state_unit must be "rad" for this audit')
    state = observation.get('state')
    if not isinstance(state, list) or len(state) != 6:
        raise SystemExit('observation.state must hold six values')
    mean, std = load_statistics(domain, 'observation.state')

    channels = []
    for index, channel in enumerate(CHANNELS):
        degrees = math.degrees(state[index])
        z = (degrees - mean[index]) / std[index]
        channels.append({
            'channel': channel,
            'armpi_joint': ARMPI_JOINTS[index],
            'input_rad': state[index],
            'input_deg': round(degrees, 3),
            'source_mean': mean[index],
            'source_std': std[index],
            'z': round(z, 3),
        })

    ranked = sorted(channels, key=lambda c: abs(c['z']), reverse=True)
    return {
        'domain': domain,
        'state_mapping': observation.get('state_mapping'),
        'embodiment_compatible': observation.get('embodiment_compatible'),
        'channels': channels,
        'channels_beyond_3_sigma': sum(1 for c in channels if abs(c['z']) > 3),
        'worst': ranked[0],
    }


def command_domains(arguments):
    chunk = load_chunk(arguments.actions)
    domains = arguments.domain or available_domains()
    results = [audit_domain(domain, chunk) for domain in domains]
    ranked = sorted(results, key=lambda r: (r['steps_outside_total'], r['max_step_change']))

    report = {
        'schema': 'smolvla_domain_audit_v1',
        'actions': str(arguments.actions),
        'steps': len(chunk),
        'results': results,
        'ranking': [r['domain'] for r in ranked],
        'selected': ranked[0]['domain'],
        'clean': ranked[0]['steps_outside_total'] == 0,
        'scope': ('output side only: passing means the chunk reverse-normalizes into '
                  'physically possible values in that source domain, not that the '
                  'values are valid ArmPi commands'),
    }

    print('%d steps x 6 channels, %d domain(s)' % (len(chunk), len(results)))
    for result in results:
        print()
        print('%-12s steps outside physical limits: %3d / %d'
              % (result['domain'], result['steps_outside_total'], 6 * len(chunk)))
        print('  %-13s %-8s %9s %9s %9s %9s %8s %6s'
              % ('channel', 'armpi', 'first', 'last', 'min', 'max', 'jump', 'bad'))
        for c in result['channels']:
            print('  %-13s %-8s %9.2f %9.2f %9.2f %9.2f %8.2f %6d'
                  % (c['channel'], c['armpi_joint'], c['first'], c['last'],
                     c['min'], c['max'], c['max_step_change'], c['outside_limit']))
        for breach in result['breaches']:
            print('    breach: %s spans %s, allowed %s'
                  % (breach['channel'], breach['observed_range'], breach['allowed_range']))

    print()
    print('ranked: %s' % ' > '.join(report['ranking']))
    print('selected: %s (clean=%s)' % (report['selected'], report['clean']))
    print('scope: %s' % report['scope'])
    write(arguments.out, report)
    return 0


def command_state(arguments):
    observation = json.loads(arguments.observation.read_text())
    domains = arguments.domain or ['so100-blue']
    results = [audit_state(domain, observation) for domain in domains]

    report = {
        'schema': 'smolvla_state_gap_v1',
        'observation': str(arguments.observation),
        'results': results,
        'scope': ('the input side is independent of the output audit: a clean domain '
                  'reverse-normalization says nothing about whether the model has '
                  'ever seen this robot state'),
    }

    for result in results:
        print('domain %s  state_mapping=%s  embodiment_compatible=%s'
              % (result['domain'], result['state_mapping'], result['embodiment_compatible']))
        print('  %-13s %-8s %10s %10s %10s %9s'
              % ('channel', 'armpi', 'input deg', 'src mean', 'src std', 'z'))
        for c in result['channels']:
            flag = '  <== outside distribution' if abs(c['z']) > 3 else ''
            print('  %-13s %-8s %10.2f %10.2f %10.2f %9.2f%s'
                  % (c['channel'], c['armpi_joint'], c['input_deg'],
                     c['source_mean'], c['source_std'], c['z'], flag))
        worst = result['worst']
        print('  %d of 6 channels beyond 3 sigma; worst %s at %+.2f sigma'
              % (result['channels_beyond_3_sigma'], worst['channel'], worst['z']))
        print()

    print('scope: %s' % report['scope'])
    write(arguments.out, report)
    return 0


def write(path, payload):
    if path is None:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + '\n')
    print('wrote %s' % path)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    subparsers = parser.add_subparsers(dest='command', required=True)

    domains = subparsers.add_parser('domains', help='rank source domains over a full chunk')
    domains.add_argument('--actions', type=Path, required=True,
                         help='probe actions.json holding model_normalized')
    domains.add_argument('--domain', action='append',
                         help='limit to these domains (repeatable); default all')
    domains.add_argument('--out', type=Path)
    domains.set_defaults(handler=command_domains)

    state = subparsers.add_parser('state', help='measure how far the observation sits from a domain')
    state.add_argument('--observation', type=Path, required=True,
                       help='probe observation.json in ArmPi radians')
    state.add_argument('--domain', action='append')
    state.add_argument('--out', type=Path)
    state.set_defaults(handler=command_state)

    arguments = parser.parse_args(argv)
    return arguments.handler(arguments)


if __name__ == '__main__':
    raise SystemExit(main())
