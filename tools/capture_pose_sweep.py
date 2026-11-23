"""Capture the 3 x 3 wrist-pose x gripper-opening set the projection check needs.

The simulation side asked for 3 wrist angles x 3 jaw openings, each stop recorded
with three 640x480 wrist frames plus the six-axis feedback and the
``arm_base -> rear_camera`` transform taken at the same instant, image/arm and
image/TF skew inside 50 ms.

**The skew target is the hard part, and this tool does not pretend otherwise.**
``/arm_controller/status`` and ``/joint_states`` are both driven by one 0.5 s
timer (``armpi_ros2/arm_node.py:99``), so arm feedback is a 2 Hz stream, while the
compressed wrist image runs at roughly 12.7 Hz.  The closest image to any status
message is therefore up to ~79 ms away, so a single attempt lands inside 50 ms
only part of the time.  Every frame is scored, the attempt is retried a bounded
number of times, and the achieved skew is written into the record.  A group that
never reaches the target is reported as ``skew_ok: false`` instead of being
quietly accepted.

Safety
    * refuses to move without ``--yes``;
    * every stop re-secures the motion gate via ``grasp_sequence.secure_gate``,
      the same path the working grasp pipeline uses;
    * moves exactly one joint per command, never more than 100 ticks;
    * never touches the chassis, never changes joint1/2/3/5, never publishes
      a multi-joint trajectory.

Subcommands
    plan     write the 9-group plan as JSON
    run      execute the plan, capture, and pause per group for a caliper reading
    report   summarise a captured directory and name every group to re-shoot
"""
import argparse
import base64
import json
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / 'ros2_ws' / 'src' / 'armpi_ros2' / 'armpi_ros2'))

ARM_DEG_PER_TICK = 240.0 / 1000.0
WRIST_JOINT = 'joint4'
GRIPPER_JOINT = 'r_joint'

JOINT_LIMITS = {
    'joint1': (0, 1000), 'joint2': (125, 875), 'joint3': (0, 1000),
    'joint4': (0, 1000), 'joint5': (0, 1000), GRIPPER_JOINT: (0, 700),
}
MAX_STEP_TICKS = 100

#: The per-jog duration the working descent path actually uses (not the 1.5 s
#: default), and the wait it applies before reading feedback back.
DEFAULT_MOVE_S = 2.2
DEFAULT_SETTLE_S = 3.0

IMAGE_TOPIC = '/rear_camera/image_raw/compressed'
STATUS_TOPIC = '/arm_controller/status'
TF_TOPIC = '/tf'
CAMERA_FRAME = 'rear_camera'
ARM_FRAME = 'arm_base'


# --------------------------------------------------------------------------
# Pure helpers -- no ROS, unit tested
# --------------------------------------------------------------------------

def deg_to_ticks(degrees, per_tick=ARM_DEG_PER_TICK):
    return int(round(degrees / per_tick))


def ticks_to_deg(ticks, per_tick=ARM_DEG_PER_TICK):
    return ticks * per_tick


def clamp_tick(joint, tick):
    low, high = JOINT_LIMITS[joint]
    clamped = max(low, min(high, int(tick)))
    return clamped, clamped != int(tick)


def split_steps(delta, cap=MAX_STEP_TICKS):
    """Break one absolute move into single commands of at most ``cap`` ticks."""
    delta = int(delta)
    if delta == 0:
        return []
    sign = 1 if delta > 0 else -1
    remaining, steps = abs(delta), []
    while remaining > 0:
        take = min(cap, remaining)
        steps.append(sign * take)
        remaining -= take
    return steps


def build_plan(theta0_tick, span_deg=15.0, openings=(30, 40, 48)):
    delta = deg_to_ticks(span_deg)
    if delta <= 0:
        raise ValueError('span_deg must be at least one tick')
    if not openings or len(set(openings)) != len(openings):
        raise ValueError('openings must be distinct and non-empty')

    angles = []
    for label, offset in (('minus', -delta), ('centre', 0), ('plus', delta)):
        target, clamped = clamp_tick(WRIST_JOINT, int(theta0_tick) + offset)
        angles.append({'label': label, 'offset_ticks': offset,
                       'joint4_tick': target, 'clamped': clamped})

    groups, index = [], 1
    for angle in angles:
        for opening in sorted(openings, reverse=True):
            groups.append({'index': index,
                           'joint4_label': angle['label'],
                           'joint4_requested_tick': angle['joint4_tick'],
                           'joint4_clamped': angle['clamped'],
                           'opening_target_mm': opening})
            index += 1
    return {
        'schema': 'pose_sweep_plan_v1',
        'theta0_tick': int(theta0_tick),
        'span_deg_requested': span_deg,
        'span_ticks_requested': delta,
        'span_deg_planned': round(ticks_to_deg(2 * delta), 3),
        'openings_mm': list(openings),
        'wrist_joint': WRIST_JOINT,
        'gripper_joint': GRIPPER_JOINT,
        'groups': groups,
        'notes': ('opening targets are millimetres between the inner jaw faces; the matching '
                  'r_joint tick is not known in advance and is entered per group after a '
                  'caliper reading'),
    }


def nearest_by_stamp(samples, reference_ns, key='stamp_ns'):
    if not samples:
        return None, None
    best, best_skew = None, None
    for index, sample in enumerate(samples):
        stamp = sample.get(key)
        if not isinstance(stamp, (int, float)):
            continue
        skew = abs(stamp - reference_ns)
        if best_skew is None or skew < best_skew:
            best, best_skew = index, skew
    if best is None:
        return None, None
    return best, best_skew / 1e6


def judge_group(skews_ms, max_skew_ms=50.0):
    if not skews_ms:
        return {'skew_ok': False, 'worst_skew_ms': None, 'reason': 'no frames captured'}
    worst = max(skews_ms)
    return {'skew_ok': worst <= max_skew_ms,
            'worst_skew_ms': round(worst, 3),
            'max_allowed_ms': max_skew_ms,
            'reason': 'ok' if worst <= max_skew_ms else 'worst frame exceeded the target'}


def summarise(records, max_skew_ms=50.0):
    problems = []
    for record in records:
        verdict = judge_group(record.get('skews_ms') or [], max_skew_ms)
        if not verdict['skew_ok']:
            problems.append({'index': record['index'], **verdict})
        if record.get('measured_mm') is None:
            problems.append({'index': record['index'],
                             'reason': 'missing caliper reading'})
        if record.get('r_joint_tick') is None:
            problems.append({'index': record['index'],
                             'reason': 'missing commanded r_joint tick'})
    angles = sorted({r['joint4_tick'] for r in records if r.get('joint4_tick') is not None})
    gaps = [b - a for a, b in zip(angles, angles[1:])]
    return {
        'groups': len(records),
        'problems': problems,
        'distinct_joint4_ticks': angles,
        'joint4_step_ticks': gaps,
        'span_deg_achieved': round(ticks_to_deg(angles[-1] - angles[0]), 3) if len(angles) > 1 else 0.0,
        'openings_mm': sorted({r['measured_mm'] for r in records
                               if isinstance(r.get('measured_mm'), (int, float))}),
        'clean': not problems,
    }


# --------------------------------------------------------------------------
# Hardware-facing half
# --------------------------------------------------------------------------

def _parse_images(messages):
    """CompressedImage -> (stamp_ns, jpeg bytes).  rosbridge sends data as base64."""
    frames = []
    for message in messages:
        body = message.get('msg', message)
        stamp = (body.get('header') or {}).get('stamp') or {}
        seconds, nanos = stamp.get('sec'), stamp.get('nanosec')
        if not isinstance(seconds, int):
            continue
        data = body.get('data')
        if isinstance(data, str):
            try:
                data = base64.b64decode(data)
            except Exception:
                continue
        if not isinstance(data, (bytes, bytearray)) or not data:
            continue
        frames.append({'stamp_ns': seconds * 10**9 + int(nanos or 0),
                       'data': bytes(data)})
    return frames


def _parse_status(messages):
    out = []
    for message in messages:
        body = message.get('msg', message)
        payload = body.get('data') if isinstance(body, dict) else None
        try:
            parsed = json.loads(payload) if isinstance(payload, str) else None
        except ValueError:
            parsed = None
        if not isinstance(parsed, dict):
            continue
        stamp = parsed.get('arm_message_ns')
        if not isinstance(stamp, int):
            continue
        joints = {}
        for name, entry in (parsed.get('joints') or {}).items():
            if isinstance(entry, dict) and isinstance(entry.get('raw'), int):
                joints[name] = entry['raw']
        out.append({'stamp_ns': stamp, 'joints': joints,
                    'health': parsed.get('health_state'),
                    'temperature': {n: (e or {}).get('temperature')
                                    for n, e in (parsed.get('joints') or {}).items()
                                    if isinstance(e, dict)}})
    return out


def _parse_tf(messages):
    out = []
    for message in messages:
        body = message.get('msg', message)
        for transform in body.get('transforms') or []:
            if (transform.get('header') or {}).get('frame_id') != ARM_FRAME:
                continue
            if transform.get('child_frame_id') != CAMERA_FRAME:
                continue
            stamp = (transform.get('header') or {}).get('stamp') or {}
            seconds, nanos = stamp.get('sec'), stamp.get('nanosec')
            if not isinstance(seconds, int):
                continue
            t = transform.get('transform') or {}
            out.append({'stamp_ns': seconds * 10**9 + int(nanos or 0),
                        'xyz': [t.get('translation', {}).get(k) for k in 'xyz'],
                        'quaternion_xyzw': [t.get('rotation', {}).get(k)
                                            for k in ('x', 'y', 'z', 'w')]})
    return out


def _drain_bucket(bridge, seconds):
    """Read every message for ``seconds`` and bucket it by topic."""
    deadline = time.monotonic() + seconds
    bucket = {'images': [], 'status': [], 'tf': []}
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
        topic = message.get('topic')
        if topic == IMAGE_TOPIC:
            bucket['images'].append(message)
        elif topic == STATUS_TOPIC:
            bucket['status'].append(message)
        elif topic == TF_TOPIC:
            bucket['tf'].append(message)
    return bucket


def capture_group(bridge, out_dir, group, wanted_frames, max_skew_ms, attempts):
    """Grab ``wanted_frames`` frames, each scored against the nearest status and TF."""
    import grasp_sequence as gs

    images, statuses, tfs = [], [], []
    skews, saved = [], []
    for _ in range(max(1, attempts)):
        # Image first, then read the bucket: the status that lands closest in
        # time to an image is the one published right after it.
        bucket = _drain_bucket(bridge, seconds=1.6)
        images += _parse_images(bucket['images'])
        statuses += _parse_status(bucket['status'])
        tfs += _parse_tf(bucket['tf'])
        if not images or not statuses or not tfs:
            continue
        for image in images:
            if len(saved) >= wanted_frames:
                break
            status_index, status_skew = nearest_by_stamp(statuses, image['stamp_ns'])
            tf_index, tf_skew = nearest_by_stamp(tfs, image['stamp_ns'])
            if status_index is None or tf_index is None:
                continue
            worst = max(s for s in (status_skew, tf_skew) if s is not None)
            if worst > max_skew_ms:
                continue
            name = 'g%02d_f%02d.jpg' % (group['index'], len(saved) + 1)
            (out_dir / name).write_bytes(image['data'])
            saved.append({'file': name,
                          'image_stamp_ns': image['stamp_ns'],
                          'joint_raw': statuses[status_index]['joints'],
                          'tf': tfs[tf_index],
                          'image_status_skew_ms': round(status_skew, 3),
                          'image_tf_skew_ms': round(tf_skew, 3)})
            skews.append(worst)
        images, statuses, tfs = [], [], []
        if len(saved) >= wanted_frames:
            break
    return saved, skews


def command_run(arguments):
    if not arguments.yes:
        print('refusing to move. Pass --yes after the operator confirms:')
        print('  * joint4 +/- the span is clear of the floor, the ball and the cables;')
        print('  * the caliper is at hand for every group;')
        print('  * the marker board is placed so all three angles can see it.')
        return 2

    import grasp_sequence as gs
    from rosbridge_client import Rosbridge
    from gripper_jog import send_target

    plan = json.loads(arguments.plan.read_text())
    out_dir = arguments.out_dir
    out_dir.mkdir(parents=True, exist_ok=True)

    bridge = Rosbridge(arguments.bridge_host, arguments.bridge_port)
    bridge.drain(0.5)
    ticks, _ = gs.read_ticks_and_stamp(bridge)
    if ticks is None:
        print('no fresh joint feedback; is the core container running?')
        return 2
    print('current %s' % ', '.join('%s=%d' % (k, ticks[k]) for k in sorted(ticks)))
    print()

    records = []
    for group in plan['groups']:
        while True:
            print('--- group %d/%d  joint4 -> %d   opening target %d mm'
                  % (group['index'], len(plan['groups']),
                     group['joint4_requested_tick'], group['opening_target_mm']))

            gs.secure_gate(bridge, arm=True, gripper=False)
            ticks, _ = gs.read_ticks_and_stamp(bridge)
            if ticks is None:
                print('    lost feedback; skipping this group')
                break
            target = clamp_tick(WRIST_JOINT, group['joint4_requested_tick'])[0]
            for step in split_steps(target - ticks[WRIST_JOINT]):
                gs.jog(WRIST_JOINT, step, duration=arguments.move_s, bridge=bridge)

            # The tick that yields a given jaw opening is not known in advance;
            # the operator walks it here and the measured value is recorded.
            ticks, stamp = gs.read_ticks_and_stamp(bridge)
            if ticks is None:
                print('    lost feedback before the gripper move')
                break
            current = ticks[GRIPPER_JOINT]
            answer = input('    r_joint tick for %d mm [current %d]: '
                           % (group['opening_target_mm'], current)).strip()
            gripper_tick = clamp_tick(GRIPPER_JOINT, int(answer) if answer else current)[0]
            gs.secure_gate(bridge, arm=False, gripper=True)
            ticks, stamp = gs.read_ticks_and_stamp(bridge)
            if ticks is None or stamp is None:
                print('    lost feedback before the gripper command')
                break
            if gripper_tick != ticks[GRIPPER_JOINT]:
                send_target(bridge, gripper_tick, duration=arguments.gripper_s, stamp=stamp)
            time.sleep(arguments.settle_s)

            frames, skews = capture_group(bridge, out_dir, group,
                                          arguments.frames, arguments.max_skew_ms,
                                          arguments.attempts)
            verdict = judge_group(skews, arguments.max_skew_ms)
            print('    frames %d/%d   worst skew %s ms   %s'
                  % (len(frames), arguments.frames, verdict['worst_skew_ms'],
                     'ok' if verdict['skew_ok'] else 'MISSED TARGET'))

            raw = input('    caliper reading between the jaw faces (mm): ').strip()
            measured = float(raw) if raw else None
            if measured is not None:
                delta = abs(measured - group['opening_target_mm'])
                if delta > arguments.mm_tolerance:
                    print('    measured %.1f mm is %.1f mm off the %d mm target '
                          '(tolerance %.1f)' % (measured, delta,
                                                group['opening_target_mm'],
                                                arguments.mm_tolerance))
                    again = input('    redo this group with another tick? [y/N]: ').strip()
                    if again.lower() == 'y':
                        continue

            ticks, _ = gs.read_ticks_and_stamp(bridge)
            record = {'index': group['index'],
                      'joint4_label': group['joint4_label'],
                      'joint4_tick': (ticks or {}).get(WRIST_JOINT),
                      'opening_target_mm': group['opening_target_mm'],
                      'measured_mm': measured,
                      'r_joint_tick': (ticks or {}).get(GRIPPER_JOINT),
                      'frames': frames,
                      'skews_ms': skews,
                      **verdict}
            (out_dir / ('group_%02d.json' % group['index'])).write_text(
                json.dumps(record, indent=2, ensure_ascii=False) + '\n')
            records.append(record)
            break

    result = summarise(records, arguments.max_skew_ms)
    (out_dir / 'sweep_summary.json').write_text(
        json.dumps(result, indent=2, ensure_ascii=False) + '\n')
    print()
    print('span achieved %.1f deg over joint4 %s'
          % (result['span_deg_achieved'], result['distinct_joint4_ticks']))
    print('openings measured %s mm' % result['openings_mm'])
    if result['clean']:
        print('all groups met the skew target and carry a reading')
    else:
        print('needs re-shoot:')
        for problem in result['problems']:
            print('  group %s: %s' % (problem.get('index'), problem.get('reason')))
    return 0 if result['clean'] else 1


def command_plan(arguments):
    plan = build_plan(arguments.theta0_tick, arguments.span_deg,
                      tuple(int(v) for v in arguments.openings.split(',')))
    print('theta0 = %d tick, planned span %.1f deg (%d ticks each side)'
          % (plan['theta0_tick'], plan['span_deg_planned'],
             plan['span_ticks_requested']))
    print()
    print('%-5s %-8s %-9s %-11s %s' % ('idx', 'angle', 'joint4', 'target mm', 'note'))
    for group in plan['groups']:
        print('%-5d %-8s %-9d %-11s %s'
              % (group['index'], group['joint4_label'],
                 group['joint4_requested_tick'], group['opening_target_mm'],
                 'clamped' if group['joint4_clamped'] else ''))
    print()
    print('r_joint ticks are NOT in the plan: measure them with calipers per group.')
    if arguments.out:
        arguments.out.parent.mkdir(parents=True, exist_ok=True)
        arguments.out.write_text(json.dumps(plan, indent=2, ensure_ascii=False) + '\n')
        print('wrote %s' % arguments.out)
    return 0


def command_report(arguments):
    records = [json.loads(p.read_text())
               for p in sorted(arguments.dir.glob('group_*.json'))]
    if not records:
        print('no group_*.json under %s' % arguments.dir)
        return 1
    result = summarise(records, arguments.max_skew_ms)
    print('groups            %d' % result['groups'])
    print('joint4 values     %s' % result['distinct_joint4_ticks'])
    print('step ticks        %s' % result['joint4_step_ticks'])
    print('span achieved     %.1f deg' % result['span_deg_achieved'])
    print('openings (mm)     %s' % result['openings_mm'])
    print()
    if result['clean']:
        print('all groups met the %.0f ms target and carry a caliper reading'
              % arguments.max_skew_ms)
    else:
        print('needs a re-shoot:')
        for problem in result['problems']:
            print('  group %s: %s' % (problem.get('index'), problem.get('reason')))
    if arguments.out:
        arguments.out.write_text(json.dumps(result, indent=2, ensure_ascii=False) + '\n')
        print('wrote %s' % arguments.out)
    return 0 if result['clean'] else 1


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = parser.add_subparsers(dest='command', required=True)

    plan = sub.add_parser('plan', help='write the 3x3 plan')
    plan.add_argument('--theta0-tick', type=int, required=True,
                      help='current joint4 tick at the already-cleared pose')
    plan.add_argument('--span-deg', type=float, default=15.0)
    plan.add_argument('--openings', default='30,40,48')
    plan.add_argument('--out', type=Path)
    plan.set_defaults(handler=command_plan)

    run = sub.add_parser('run', help='execute the plan on the robot (needs --yes)')
    run.add_argument('--plan', type=Path, required=True)
    run.add_argument('--out-dir', type=Path, required=True)
    run.add_argument('--yes', action='store_true')
    run.add_argument('--bridge-host', default='127.0.0.1')
    run.add_argument('--bridge-port', type=int, default=19091)
    run.add_argument('--frames', type=int, default=3)
    run.add_argument('--max-skew-ms', type=float, default=50.0)
    run.add_argument('--attempts', type=int, default=6)
    run.add_argument('--move-s', type=float, default=DEFAULT_MOVE_S)
    run.add_argument('--gripper-s', type=float, default=0.45)
    run.add_argument('--settle-s', type=float, default=DEFAULT_SETTLE_S)
    run.add_argument('--mm-tolerance', type=float, default=2.0,
                     help='redo a group when the caliper reading misses by more than this')
    run.set_defaults(handler=command_run)

    report = sub.add_parser('report', help='summarise a captured directory')
    report.add_argument('--dir', type=Path, required=True)
    report.add_argument('--max-skew-ms', type=float, default=50.0)
    report.add_argument('--out', type=Path)
    report.set_defaults(handler=command_report)

    arguments = parser.parse_args(argv)
    return arguments.handler(arguments)


if __name__ == '__main__':
    raise SystemExit(main())
