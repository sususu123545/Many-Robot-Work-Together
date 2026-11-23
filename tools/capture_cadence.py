"""Capture timestamped six-axis feedback, and structure past console logs.

The only record kept for the 2026-11-23 grasp is a console log whose lines have
no clocks, so command cadence cannot be reconstructed from it after the fact.
This tool produces the missing data going forward:

  listen     subscribe to /joint_states and write one JSONL row per message
             (arrival wall clock, arrival monotonic clock, header stamp, and
             the six servo ticks).  Read-only: it never publishes and never
             touches the motion gates, so it is safe to run on an idle arm.

  parse-log  turn a grasp console log into structured command rows
             (sequence, joint, requested delta, target tick, moved, error).
             These carry no timestamps by construction -- that is the whole
             point of switching to `listen`.

Usage:
    python3 tools/capture_cadence.py listen --seconds 30 --out /tmp/js.jsonl
    python3 tools/capture_cadence.py parse-log --log tmp/run3.log --out /tmp/cmds.jsonl

`listen` needs the rosbridge tunnel (tools/ssh_tunnel.py --local-port 19091).
"""
import argparse
import json
import math
import re
import statistics
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / 'ros2_ws' / 'src' / 'armpi_ros2' / 'armpi_ros2'))

from rosbridge_client import Rosbridge              # noqa: E402
from servo_protocol import JOINTS as SERVO_TABLE    # noqa: E402

#: (servo id, centre tick, sign, low, high) per joint -- the car's own table.
JOINT_NAMES = list(SERVO_TABLE)
TICK_RAD = math.radians(240.0) / 1000.0

#: One servo report per 0.5 s timer in armpi_ros2/arm_node.py:99.
EXPECTED_FEEDBACK_HZ = 2.0

#: Matches "    joint2 -20 ok  (target 588, moved 19 of 20, error 1)"
COMMAND_LINE = re.compile(
    r'^\s+(?P<joint>joint\d)\s+(?P<delta>[+-]\d+)\s+(?P<status>ok|NO MOVEMENT)\s+'
    r'\(target (?P<target>-?\d+), moved (?P<moved>\d+) of (?P<wanted>\d+), '
    r'error (?P<error>\d+)\)')


def ticks_from_message(message):
    """JointState -> {joint: tick}, using the same sign/centre table as the car."""
    body = message.get('msg', message)
    names = body.get('name') or []
    positions = body.get('position') or []
    ticks = {}
    for name, position in zip(names, positions):
        if name not in SERVO_TABLE or not isinstance(position, (int, float)):
            continue
        if not math.isfinite(position):
            continue
        _, centre, sign, _, _ = SERVO_TABLE[name]
        scale = math.radians(180.0) / 1000 if name == 'r_joint' else TICK_RAD
        ticks[name] = int(round(centre + sign * position / scale))
    return ticks


def stamp_seconds(message):
    body = message.get('msg', message)
    stamp = (body.get('header') or {}).get('stamp') or {}
    seconds, nanos = stamp.get('sec'), stamp.get('nanosec')
    if not isinstance(seconds, int) or not isinstance(nanos, int):
        return None
    return seconds + nanos * 1e-9


def command_listen(arguments):
    bridge = Rosbridge(arguments.host, arguments.port, timeout=arguments.timeout)
    bridge.subscribe('/joint_states', 'sensor_msgs/msg/JointState')

    rows, arrivals, missing = [], [], set(JOINT_NAMES)
    started_wall = time.time()
    started_mono = time.monotonic()
    deadline = started_mono + arguments.seconds

    while time.monotonic() < deadline:
        try:
            raw = bridge.recv_text(timeout=1.0)
        except Exception:
            continue
        if not raw:
            continue
        try:
            message = json.loads(raw)
        except ValueError:
            continue
        if message.get('topic') != '/joint_states':
            continue

        now_mono, now_wall = time.monotonic(), time.time()
        ticks = ticks_from_message(message)
        if not ticks:
            continue
        missing -= set(ticks)
        stamp = stamp_seconds(message)

        arrivals.append(now_mono)
        rows.append({
            't_wall': round(now_wall, 6),
            't_mono': round(now_mono - started_wall, 6),
            'header_stamp': None if stamp is None else round(stamp, 6),
            'ticks': ticks,
        })

    bridge.close()

    gaps = [b - a for a, b in zip(arrivals, arrivals[1:])]
    elapsed = max(1e-9, time.monotonic() - started_mono)
    # Skew is only meaningful when every row carried a stamp; a partial set
    # would silently pair the wrong ones.
    skew = None
    if rows and all(row['header_stamp'] is not None for row in rows):
        skew = statistics.fmean(row['header_stamp'] - row['t_wall'] for row in rows)
    report = {
        'schema': 'joint_state_capture_v1',
        'topic': '/joint_states',
        'seconds_requested': arguments.seconds,
        'seconds_elapsed': round(elapsed, 3),
        'messages': len(rows),
        'observed_hz': round(len(rows) / elapsed, 3),
        'expected_hz_from_source': EXPECTED_FEEDBACK_HZ,
        'joints_seen': sorted(set(JOINT_NAMES) - missing),
        'joints_never_seen': sorted(missing),
        'gap_s': {
            'min': round(min(gaps), 4) if gaps else None,
            'median': round(statistics.median(gaps), 4) if gaps else None,
            'mean': round(statistics.fmean(gaps), 4) if gaps else None,
            'p95': round(sorted(gaps)[int(0.95 * (len(gaps) - 1))], 4) if gaps else None,
            'max': round(max(gaps), 4) if gaps else None,
        },
        'header_minus_arrival_s': None if skew is None else round(skew, 4),
        'note': ('read-only capture: nothing was published and no motion gate was touched'),
    }

    if arguments.out:
        arguments.out.parent.mkdir(parents=True, exist_ok=True)
        with arguments.out.open('w') as handle:
            for row in rows:
                handle.write(json.dumps(row) + '\n')
        report_path = arguments.out.with_suffix('.summary.json')
        report_path.write_text(json.dumps(report, indent=2) + '\n')
        print('wrote %s (%d rows)' % (arguments.out, len(rows)))
        print('wrote %s' % report_path)

    print()
    print('messages          %d over ~%.1f s' % (len(rows), arguments.seconds))
    print('observed rate     %.3f Hz  (source timer implies %.1f Hz)'
          % (report['observed_hz'], EXPECTED_FEEDBACK_HZ))
    if gaps:
        print('gap min/med/max   %.3f / %.3f / %.3f s'
              % (report['gap_s']['min'], report['gap_s']['median'], report['gap_s']['max']))
        print('gap p95           %.3f s' % report['gap_s']['p95'])
    if report['joints_never_seen']:
        print('never seen        %s' % ', '.join(report['joints_never_seen']))
    if report['header_minus_arrival_s'] is not None:
        print('header - arrival  %+.4f s (mean)' % report['header_minus_arrival_s'])
    print()
    print(report['note'])
    return 0


def command_parse_log(arguments):
    rows, sequence = [], 0
    for line in arguments.log.read_text(errors='replace').splitlines():
        match = COMMAND_LINE.match(line)
        if not match:
            continue
        sequence += 1
        fields = match.groupdict()
        rows.append({
            'seq': sequence,
            'joint': fields['joint'],
            'delta_requested': int(fields['delta']),
            'target_tick': int(fields['target']),
            'moved': int(fields['moved']),
            'wanted': int(fields['wanted']),
            'error': int(fields['error']),
            'completed': fields['status'] == 'ok',
            'timestamp': None,
        })

    by_joint = {}
    for row in rows:
        entry = by_joint.setdefault(row['joint'], {'commands': 0, 'completed': 0})
        entry['commands'] += 1
        entry['completed'] += 1 if row['completed'] else 0

    report = {
        'schema': 'grasp_command_sequence_v1',
        'source_log': str(arguments.log),
        'commands': len(rows),
        'completed_first_try': sum(1 for r in rows if r['completed']),
        'by_joint': by_joint,
        'timestamps_available': False,
        'why': ('the grasp script prints no clocks; per-command latency and cadence '
                'cannot be recovered from this log. Use the listen subcommand to '
                'capture timestamped feedback next session.'),
    }

    if arguments.out:
        arguments.out.parent.mkdir(parents=True, exist_ok=True)
        with arguments.out.open('w') as handle:
            for row in rows:
                handle.write(json.dumps(row) + '\n')
        summary = arguments.out.with_suffix('.summary.json')
        summary.write_text(json.dumps(report, indent=2) + '\n')
        print('wrote %s (%d rows)' % (arguments.out, len(rows)))
        print('wrote %s' % summary)

    print()
    print('commands            %d' % report['commands'])
    print('completed first try %d' % report['completed_first_try'])
    for joint in sorted(by_joint):
        entry = by_joint[joint]
        print('  %-7s %2d commands, %2d completed (%.0f%%)'
              % (joint, entry['commands'], entry['completed'],
                 100.0 * entry['completed'] / entry['commands']))
    print()
    print('timestamps: none. %s' % report['why'])
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    subparsers = parser.add_subparsers(dest='command', required=True)

    listen = subparsers.add_parser('listen', help='record timestamped /joint_states (read-only)')
    listen.add_argument('--seconds', type=float, default=30.0)
    listen.add_argument('--host', default='127.0.0.1')
    listen.add_argument('--port', type=int, default=19091)
    listen.add_argument('--timeout', type=float, default=8.0)
    listen.add_argument('--out', type=Path)
    listen.set_defaults(handler=command_listen)

    parse = subparsers.add_parser('parse-log', help='structure a grasp console log')
    parse.add_argument('--log', type=Path, required=True)
    parse.add_argument('--out', type=Path)
    parse.set_defaults(handler=command_parse_log)

    arguments = parser.parse_args(argv)
    if arguments.command == 'listen':
        if not isinstance(arguments.seconds, (int, float)) or arguments.seconds <= 0:
            parser.error('--seconds must be positive')
    return arguments.handler(arguments)


if __name__ == '__main__':
    raise SystemExit(main())
