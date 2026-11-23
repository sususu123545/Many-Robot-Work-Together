"""Make the SmolVLA joint-action chunk executable on this arm's bus.

SmolVLA was pretrained on SO100 joint positions sampled at 30 Hz, emitting a
50-step chunk that covers 1.67 s.  Two independent facts stop that chunk from
being streamed straight to the arm:

  * the arm node accepts one timed move of MIN_MOVE_DURATION_S..5.0 s, so the
    command rate cannot exceed 1/MIN_MOVE_DURATION_S = 3.33 Hz;
  * a single command may travel at most DEFAULT_MAX_STEP_TICKS ticks.

So the chunk has to be decimated, and the same 50 steps then span 15 s instead
of 1.67 s.  That factor is a *dataset* decision rather than a runtime one: the
policy learns whatever rate its demonstrations were recorded at, so recording
at 30 Hz and executing at 3.33 Hz would train the model on a cadence it can
never run.

This module holds that contract in one place.  It is pure Python: no ROS 2, no
torch, no robot, and it never commands anything.

Why not calibrate the ArmPi onto the SO100 convention instead?
    Because there is no SO100 convention to calibrate onto.  In LeRobot's
    DEGREES norm mode the reported value is

        mid = (range_min + range_max) / 2
        degrees = (raw - mid) * 360 / (resolution - 1)

    where range_min/range_max are recorded on *that* arm by dragging every
    joint through its travel during calibration, and the zero comes from
    parking the arm at mid-range.  Two SO100s calibrated at different
    mid-poses report different numbers for the same physical pose, so a
    per-joint affine map to "SO100 degrees" has neither a unique solution nor
    any ground truth to validate against.  Fine-tuning on ArmPi data, with the
    normalizer fitted to ArmPi, is the path that can actually be checked.
"""
import argparse
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'ros2_ws/src/vla_controller'))
from vla_controller import spec  # noqa: E402

#: Canonical order of the six-dimensional vector.  Taken from spec so there is
#: one definition: /joint_states, the URDF, the tick table and this agree.
JOINT_ORDER = tuple(spec.ARM_PI_JOINTS)

#: SmolVLA's own timing, read off the checkpoint config
#: (chunk_size=50, n_action_steps=50) and the SO100 datasets' 30 fps.
POLICY_HZ = 30.0
POLICY_CHUNK_STEPS = 50


def bus_command_limits():
    """The arm node's own limits, quoted rather than restated."""
    return {
        'min_duration_s': spec.MIN_MOVE_DURATION_S,
        'max_duration_s': spec.MAX_MOVE_DURATION_S,
        'max_ticks_per_command': spec.DEFAULT_MAX_STEP_TICKS,
        'max_command_hz': 1.0 / spec.MIN_MOVE_DURATION_S,
    }


def decimation(policy_hz=POLICY_HZ, chunk_steps=POLICY_CHUNK_STEPS,
               min_duration_s=None):
    """How far a policy chunk must be thinned to fit the bus.

    Returns the decimation factor and what the chunk then costs in wall time.
    A factor of 1 means the policy already runs slower than the bus allows.
    """
    if not math.isfinite(policy_hz) or policy_hz <= 0:
        raise ValueError('policy_hz must be positive and finite')
    if not isinstance(chunk_steps, int) or chunk_steps <= 0:
        raise ValueError('chunk_steps must be a positive integer')
    limit = spec.MIN_MOVE_DURATION_S if min_duration_s is None else min_duration_s
    if not math.isfinite(limit) or limit <= 0:
        raise ValueError('min_duration_s must be positive and finite')

    # A command cannot be issued more often than the bus accepts one, so the
    # execution period is max(policy period, bus floor).
    policy_period = 1.0 / policy_hz
    execution_period = max(policy_period, limit)
    factor = int(math.ceil(execution_period / policy_period - 1e-9))
    execution_hz = policy_hz / factor

    # Two different durations, easy to confuse:
    #   covered_intent_s  how much of the policy's plan the kept steps span.
    #                     Taking every 9th of 50 steps spans 45 steps = 1.5 s.
    #   wall_time_s       how long the arm needs to be sent those commands.
    # Both are ~the policy's own horizon, which is why decimating is not the
    # same as slowing the motion down 9x.  Executing all 50 steps at 0.3 s
    # each would take 15 s and would distort the speed; sending every 9th
    # target keeps the geometry and only thins the waypoints.
    kept = int(math.ceil(chunk_steps / factor))
    covered = max(0, (kept - 1) * factor) * policy_period

    return {
        'policy_hz': policy_hz,
        'policy_period_s': round(policy_period, 6),
        'chunk_steps': chunk_steps,
        'policy_chunk_duration_s': round(chunk_steps * policy_period, 6),
        'decimation_factor': factor,
        'execution_period_s': round(1.0 / execution_hz, 6),
        'execution_hz': round(execution_hz, 6),
        'decimated_steps': kept,
        'covered_intent_s': round(covered, 6),
        'wall_time_s': round(kept * (1.0 / execution_hz), 6),
        'naive_stream_duration_s': round(chunk_steps * execution_period, 6),
        'fits_one_command_period': factor == 1,
        'bus_floor_s': limit,
    }


def tick_budget(duration_s, max_ticks=None):
    """Ticks a joint may travel in one command of this duration.

    The bus caps ticks per command outright, so the answer does not grow with
    a longer duration.  The per-second rate is reported because that is what
    decides whether a demonstration cadence looks calm or violent.
    """
    cap = spec.DEFAULT_MAX_STEP_TICKS if max_ticks is None else max_ticks
    if not isinstance(cap, int) or cap <= 0:
        raise ValueError('max_ticks must be a positive integer')
    if not math.isfinite(duration_s) or duration_s <= 0:
        raise ValueError('duration_s must be positive and finite')
    return {
        'duration_s': duration_s,
        'max_ticks': cap,
        'max_degrees': round(cap / spec.TICKS_PER_DEGREE, 4),
        'max_ticks_per_second': round(cap / duration_s, 4),
        'max_degrees_per_second': round(cap / spec.TICKS_PER_DEGREE / duration_s, 4),
    }


def radians_to_ticks(radians_by_joint):
    """Map a canonical radian vector back to servo ticks, joints in order."""
    values = _vector_from_names(radians_by_joint)
    ticks = {}
    for name, value in zip(JOINT_ORDER, values):
        _, raw = spec.radians_to_tick(name, value)
        ticks[name] = raw
    return ticks


def _vector_from_names(radians_by_joint):
    if isinstance(radians_by_joint, dict):
        missing = [name for name in JOINT_ORDER if name not in radians_by_joint]
        if missing:
            raise ValueError('missing joints: %s' % ', '.join(missing))
        values = [radians_by_joint[name] for name in JOINT_ORDER]
    else:
        values = list(radians_by_joint)
    if len(values) != len(JOINT_ORDER):
        raise ValueError('expected %d joint values, got %d'
                         % (len(JOINT_ORDER), len(values)))
    if not all(type(v) in (int, float) and math.isfinite(v) for v in values):
        raise ValueError('joint values must be finite numbers')
    return values


def check_targets(targets, start, duration_s, max_ticks=None):
    """Can these joint targets be reached from ``start`` in one command?

    ``targets`` and ``start`` are canonical radian vectors.  Every joint is
    checked, because one over-travelling joint is enough to have the arm node
    reject the whole command.
    """
    cap = spec.DEFAULT_MAX_STEP_TICKS if max_ticks is None else max_ticks
    start_ticks = radians_to_ticks(_vector_from_names(start))
    target_ticks = radians_to_ticks(_vector_from_names(targets))

    per_joint, offenders = {}, []
    for name in JOINT_ORDER:
        delta = target_ticks[name] - start_ticks[name]
        per_joint[name] = {'from_tick': start_ticks[name],
                           'to_tick': target_ticks[name],
                           'delta_tick': delta,
                           'degrees': round(delta / spec.TICKS_PER_DEGREE, 4)}
        if abs(delta) > cap:
            offenders.append({'joint': name, 'delta_tick': delta, 'limit': cap})

    budget = tick_budget(duration_s, cap)
    return {
        'feasible': not offenders,
        'max_delta_tick': max(abs(v['delta_tick']) for v in per_joint.values()),
        'limit_tick': cap,
        'duration_s': duration_s,
        'ticks_per_second': round(
            max(abs(v['delta_tick']) for v in per_joint.values()) / duration_s, 4),
        'budget': budget,
        'joints': per_joint,
        'offenders': offenders,
    }


class Normalizer:
    """MEAN_STD statistics fitted to ArmPi's own recorded states.

    This is the fine-tuning path: the numbers describe the robot we actually
    have, so nothing has to be projected onto another arm's convention.
    """

    def __init__(self, mean, std, floor=1e-6):
        self.mean = [float(v) for v in mean]
        self.std = [float(v) for v in std]
        if len(self.mean) != len(JOINT_ORDER) or len(self.std) != len(JOINT_ORDER):
            raise ValueError('expected %d means and stds' % len(JOINT_ORDER))
        if not all(math.isfinite(v) for v in self.mean + self.std):
            raise ValueError('statistics must be finite')
        if any(v < 0 for v in self.std):
            raise ValueError('std must not be negative')
        self.floor = float(floor)
        if not self.floor > 0:
            raise ValueError('floor must be positive')

    @classmethod
    def fit(cls, samples):
        """Population mean/std over a list of canonical vectors.

        A joint that never moves in the recording gets std 0, which cannot be
        divided by.  The floor keeps the transform finite and the clamp report
        then tells the caller that the joint carries no information.
        """
        rows = [_vector_from_names(row) for row in samples]
        if not rows:
            raise ValueError('need at least one sample')
        count = len(rows)
        mean = [sum(row[i] for row in rows) / count for i in range(len(JOINT_ORDER))]
        variance = [sum((row[i] - mean[i]) ** 2 for row in rows) / count
                    for i in range(len(JOINT_ORDER))]
        std = [math.sqrt(v) for v in variance]
        return cls(mean, [s if s > 0 else 0.0 for s in std])

    def normalize(self, values):
        row = _vector_from_names(values)
        return [(row[i] - self.mean[i]) / max(self.std[i], self.floor)
                for i in range(len(JOINT_ORDER))]

    def denormalize(self, values):
        row = _vector_from_names(values)
        return [row[i] * max(self.std[i], self.floor) + self.mean[i]
                for i in range(len(JOINT_ORDER))]

    def clamp_report(self, values, limit=3.0):
        """Which joints sit outside ``limit`` sigma.  Zero std is reported."""
        normalized = self.normalize(values)
        outside = []
        for name, z, std in zip(JOINT_ORDER, normalized, self.std):
            if std <= self.floor:
                outside.append({'joint': name, 'z': None, 'reason': 'constant joint'})
            elif abs(z) > limit:
                outside.append({'joint': name, 'z': round(z, 3),
                                'reason': 'beyond %.1f sigma' % limit})
        return {'outside': outside, 'clean': not outside}

    def as_dict(self):
        return {'schema': 'armpi_mean_std_v1',
                'joint_order': list(JOINT_ORDER),
                'mean': self.mean, 'std': self.std, 'floor': self.floor}


def contract():
    """Everything a dataset recording decision depends on, in one payload."""
    limits = bus_command_limits()
    timing = decimation()
    budget = tick_budget(limits['min_duration_s'])
    return {
        'schema': 'armpi_smolvla_joint_contract_v1',
        'joint_order': list(JOINT_ORDER),
        'joint_ticks_per_degree': spec.TICKS_PER_DEGREE,
        'gripper_ticks_per_degree': 1000.0 / 180.0,
        'bus': limits,
        'policy': {'hz': POLICY_HZ, 'chunk_steps': POLICY_CHUNK_STEPS},
        'timing': timing,
        'per_command_budget': budget,
        'recording_fps_required': timing['execution_hz'],
        'note': ('record demonstrations at recording_fps_required, not at the '
                 'policy rate; training on 30 Hz data and executing at 3.33 Hz '
                 'teaches a cadence the bus cannot run'),
    }


def command_contract(arguments):
    payload = contract()
    timing = payload['timing']
    budget = payload['per_command_budget']

    print('joint order      %s' % ', '.join(payload['joint_order']))
    print('bus              one timed move %.2f..%.0f s, at most %d ticks per command'
          % (payload['bus']['min_duration_s'], payload['bus']['max_duration_s'],
             payload['bus']['max_ticks_per_command']))
    print('bus max rate     %.3f Hz' % payload['bus']['max_command_hz'])
    print()
    print('policy           %g Hz, %d-step chunk = %.3f s of intent'
          % (payload['policy']['hz'], payload['policy']['chunk_steps'],
             timing['policy_chunk_duration_s']))
    print('decimation       take every %d-th step -> %.3f Hz'
          % (timing['decimation_factor'], timing['execution_hz']))
    print('kept waypoints   %d, spanning %.3f s of the plan, sent over %.3f s'
          % (timing['decimated_steps'], timing['covered_intent_s'], timing['wall_time_s']))
    print('   (streaming all %d steps at %.2f s each would instead be %.1f s -- that is'
          % (timing['chunk_steps'], timing['execution_period_s'],
             timing['naive_stream_duration_s']))
    print('    slowing the motion down %dx, not thinning waypoints)'
          % timing['decimation_factor'])
    print()
    print('per command      <= %d ticks = %.2f deg, i.e. %.1f deg/s at %.2f s'
          % (budget['max_ticks'], budget['max_degrees'],
             budget['max_degrees_per_second'], budget['duration_s']))
    print()
    print('RECORD AT        %.3f fps  <-- this is the dataset decision'
          % payload['recording_fps_required'])
    print()
    print(payload['note'])
    if arguments.out:
        arguments.out.parent.mkdir(parents=True, exist_ok=True)
        arguments.out.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + '\n')
        print('wrote %s' % arguments.out)
    return 0


def command_normalizer(arguments):
    payload = json.loads(arguments.states.read_text())
    samples = payload['samples'] if isinstance(payload, dict) else payload
    normalizer = Normalizer.fit(samples)
    report = normalizer.as_dict()
    report['samples'] = len(samples)

    residual = 0.0
    for row in samples:
        round_trip = normalizer.denormalize(normalizer.normalize(row))
        residual = max(residual, max(abs(a - b) for a, b in zip(row, round_trip)))
    report['round_trip_max_error'] = residual

    print('fitted on %d samples' % len(samples))
    print('%-10s %14s %14s' % ('joint', 'mean', 'std'))
    for name, mean, std in zip(JOINT_ORDER, normalizer.mean, normalizer.std):
        note = '  <== constant, carries no signal' if std <= normalizer.floor else ''
        print('%-10s %14.6f %14.6f%s' % (name, mean, std, note))
    print('round-trip max error %.3e' % residual)
    if arguments.out:
        arguments.out.parent.mkdir(parents=True, exist_ok=True)
        arguments.out.write_text(json.dumps(report, indent=2) + '\n')
        print('wrote %s' % arguments.out)
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    subparsers = parser.add_subparsers(dest='command', required=True)

    show = subparsers.add_parser('contract', help='print the joint/cadence contract')
    show.add_argument('--out', type=Path)
    show.set_defaults(handler=command_contract)

    normalize = subparsers.add_parser('normalizer',
                                      help='fit MEAN_STD statistics to recorded states')
    normalize.add_argument('--states', type=Path, required=True,
                           help='JSON list of canonical 6-vectors, or {"samples": [...]}')
    normalize.add_argument('--out', type=Path)
    normalize.set_defaults(handler=command_normalizer)

    arguments = parser.parse_args(argv)
    return arguments.handler(arguments)


if __name__ == '__main__':
    raise SystemExit(main())
