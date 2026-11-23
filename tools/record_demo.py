"""Record a real grasp into paired (image, joints, commanded target) samples.

Why this exists
    ``grasp_sequence.py`` grabs frames and reads joints but keeps neither: the
    only artefact of a run is a console log.  That means a successful real grasp
    currently teaches nothing, and the real-robot VLA has no data at all.

    This recorder runs alongside a grasp as a passive listener.  It publishes
    nothing, touches no motion gate, and opens its own rosbridge connection so it
    cannot contend with the control loop's own reads (a single connection that
    both subscribes and publishes is known to starve the loop here).

Why 2 Hz and not 3.33
    ``/joint_states`` and ``/arm_controller/status`` both come from one 0.5 s
    timer (``armpi_ros2/arm_node.py:99``), so arm feedback *is* a 2 Hz stream.
    Recording faster would invent a cadence the robot can never run in closed
    loop, and training on it would teach the policy speeds it cannot achieve.
    Images are stored at their native ~12.7 Hz and paired to states afterwards.

What a sample is
    ``observation.state``  six servo ticks as read at that moment
    ``action``             the last accepted command the arm node reports
                           (``/arm_controller/status`` carries it as last_command)
    ``image``              the wrist frame nearest in time to that state

    Note the action is the arm node's *last accepted* command, not the intent of
    whoever issued it.  That is the honest label for what the arm was actually
    told to do, and the converter keeps it that way.

Subcommands
    run      listen and write a recording directory
    verify   check a recording for coverage, gaps and pairing rate

Usage
    python3 tools/record_demo.py run --out-dir recordings/demo_01 --seconds 420
    python3 tools/record_demo.py verify --dir recordings/demo_01
"""
import argparse
import hashlib
import json
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from capture_cadence import ticks_from_message                # noqa: E402
from capture_pose_sweep import (_parse_images, _parse_status,  # noqa: E402
                                _parse_tf, nearest_by_stamp)

JOINT_NAMES = ('joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'r_joint')
IMAGE_TOPIC = '/rear_camera/image_raw/compressed'
STATUS_TOPIC = '/arm_controller/status'
JOINT_TOPIC = '/joint_states'
TF_TOPIC = '/tf'

#: Arm feedback arrives on a 0.5 s timer; anything far above this is an artefact.
EXPECTED_STATE_HZ = 2.0


def _stamp_of(message):
    body = message.get('msg', message)
    stamp = (body.get('header') or {}).get('stamp') or {}
    seconds, nanos = stamp.get('sec'), stamp.get('nanosec')
    if not isinstance(seconds, int):
        return None
    return seconds * 10**9 + int(nanos or 0)


def check_pairing(states, images, max_skew_ms=50.0):
    """How many states can be matched to an image inside the skew budget."""
    if not states or not images:
        return {'matched': 0, 'rate': 0.0, 'worst_skew_ms': None}
    skews = []
    for state in states:
        _, skew = nearest_by_stamp(images, state['stamp_ns'])
        if skew is not None:
            skews.append(skew)
    matched = sum(1 for s in skews if s <= max_skew_ms)
    return {
        'matched': matched,
        'rate': round(matched / len(states), 4),
        'worst_skew_ms': round(max(skews), 3) if skews else None,
        'median_skew_ms': round(sorted(skews)[len(skews) // 2], 3) if skews else None,
        'budget_ms': max_skew_ms,
    }


def summarise_recording(states, images, max_skew_ms=50.0):
    complete = [s for s in states if len(s.get('ticks') or {}) == len(JOINT_NAMES)]
    with_action = [s for s in complete if s.get('last_command')]
    spans = [s['t_mono'] for s in states]
    gaps = [b - a for a, b in zip(spans, spans[1:])]
    return {
        'state_samples': len(states),
        'states_with_all_six_joints': len(complete),
        'states_with_commanded_target': len(with_action),
        'images': len(images),
        'duration_s': round(spans[-1] - spans[0], 2) if len(spans) > 1 else 0.0,
        # Rate from the intervals, not the sample count: four samples spanning
        # 1.5 s is a 2 Hz stream, not 2.7 Hz.
        'state_hz': round((len(states) - 1) / (spans[-1] - spans[0]), 3)
        if len(spans) > 1 and spans[-1] > spans[0] else None,
        'expected_state_hz': EXPECTED_STATE_HZ,
        'state_gap_s': {'median': round(sorted(gaps)[len(gaps) // 2], 3) if gaps else None,
                        'max': round(max(gaps), 3) if gaps else None},
        'pairing': check_pairing(complete, images, max_skew_ms),
    }


def assess(summary, min_pairing_rate=0.9, max_gap_s=2.0):
    """List everything that stops a recording from being a usable demonstration.

    Kept separate from the CLI so the rules can be tested without a recording
    directory, and so the same verdict can be reused by a future converter.
    """
    problems = []
    if not summary['states_with_all_six_joints']:
        problems.append('no state sample carried all six joints')
    if not summary['states_with_commanded_target']:
        problems.append('no state carried a commanded target '
                        '(is /arm_controller/status publishing?)')
    if summary['images'] < 2:
        problems.append('too few images')
    pairing = summary['pairing']
    if pairing['rate'] < min_pairing_rate:
        problems.append('pairing rate %.1f%% below the %.0f%% floor'
                        % (pairing['rate'] * 100, min_pairing_rate * 100))
    gap = summary['state_gap_s']['max']
    if gap is not None and gap > max_gap_s:
        problems.append('state stream stalled for %.1f s' % gap)
    return problems


def command_run(arguments):
    from rosbridge_client import Rosbridge

    out_dir = arguments.out_dir
    (out_dir / 'images').mkdir(parents=True, exist_ok=True)
    states_path = out_dir / 'arm_states.jsonl'
    index_path = out_dir / 'images' / 'index.jsonl'
    meta_path = out_dir / 'meta.json'

    try:
        bridge = Rosbridge(arguments.bridge_host, arguments.bridge_port)
    except Exception as error:
        print('cannot reach rosbridge at %s:%d' % (arguments.bridge_host,
                                                   arguments.bridge_port))
        print('  %s' % error)
        print('  usual cause: the tunnel is not up.  Start it with')
        print('  python tools/ssh_tunnel.py --host 100.102.57.19 --local-port 19091 --remote-port 9091')
        return 2
    for topic, kind in ((JOINT_TOPIC, 'sensor_msgs/msg/JointState'),
                        (STATUS_TOPIC, 'std_msgs/msg/String'),
                        (IMAGE_TOPIC, 'sensor_msgs/msg/CompressedImage'),
                        (TF_TOPIC, 'tf2_msgs/msg/TFMessage')):
        bridge.subscribe(topic, kind)

    states, images = [], []
    latest_tf = None
    joint_state_messages = 0
    status_messages = 0
    started_wall, started_mono = time.time(), time.monotonic()
    deadline = started_mono + arguments.seconds
    last_report = started_mono
    image_seq = 0
    last_image_mono = -1e9

    print('recording into %s for up to %.0f s  (Ctrl-C to stop early)'
          % (out_dir, arguments.seconds))
    print('passive listener: publishes nothing, touches no gate, own connection')
    if arguments.task:
        print('task string: %r' % arguments.task)

    states_handle = states_path.open('w')
    index_handle = index_path.open('w')
    try:
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
            now_mono, now_wall = time.monotonic(), time.time()

            if topic == TF_TOPIC:
                for transform in _parse_tf([message]):
                    latest_tf = transform

            elif topic == JOINT_TOPIC:
                # Only counted, not stored: the status message already carries
                # the same six ticks plus the command, and storing both would
                # double the apparent sampling rate.
                joint_state_messages += 1

            elif topic == STATUS_TOPIC:
                for parsed in _parse_status([message]):
                    status_messages += 1
                    row = {'t_mono': round(now_mono - started_mono, 4),
                           't_wall': round(now_wall, 4),
                           'stamp_ns': parsed['stamp_ns'],
                           'ticks': parsed['joints'],
                           'last_command': parsed.get('last_command'),
                           'temperature': parsed.get('temperature'),
                           'tf_arm_base_to_rear_camera': latest_tf}
                    states.append(row)
                    states_handle.write(json.dumps(row) + '\n')

            elif topic == IMAGE_TOPIC:
                if now_mono - last_image_mono < arguments.min_image_interval:
                    continue
                for frame in _parse_images([message]):
                    image_seq += 1
                    name = '%06d.jpg' % image_seq
                    (out_dir / 'images' / name).write_bytes(frame['data'])
                    entry = {'file': name, 'stamp_ns': frame['stamp_ns'],
                             't_mono': round(now_mono - started_mono, 4),
                             'bytes': len(frame['data']),
                             'sha256': hashlib.sha256(frame['data']).hexdigest()}
                    images.append(entry)
                    index_handle.write(json.dumps(entry) + '\n')
                    last_image_mono = now_mono

            if now_mono - last_report >= arguments.report_every:
                last_report = now_mono
                print('  t=%5.1f s  states %4d  images %4d'
                      % (now_mono - started_mono, len(states), len(images)), flush=True)
    except KeyboardInterrupt:
        print('  stopped by operator')
    finally:
        states_handle.close()
        index_handle.close()

    result = summarise_recording(states, images, arguments.max_skew_ms)
    meta = {
        'schema': 'armpi_demo_recording_v1',
        'started_wall': round(started_wall, 3),
        'finished_wall': round(time.time(), 3),
        'seconds_requested': arguments.seconds,
        'task': arguments.task,
        'joint_state_messages': joint_state_messages,
        'status_messages': status_messages,
        'passive': True,
        'published_anything': False,
        'touched_motion_gate': False,
        'topics': [JOINT_TOPIC, STATUS_TOPIC, IMAGE_TOPIC],
        'note': ('action labels are the arm node last accepted command, not the '
                 'caller intent; pair images to states by stamp in the converter'),
        'summary': result,
    }
    meta_path.write_text(json.dumps(meta, indent=2, ensure_ascii=False) + '\n')

    append_task = out_dir / 'task.txt'
    if arguments.task:
        append_task.write_text(arguments.task.rstrip() + '\n')

    print()
    print('duration          %.1f s' % result['duration_s'])
    print('state samples     %d  (%.2f Hz, source timer implies %.1f)'
          % (result['state_samples'], result['state_hz'] or 0.0, EXPECTED_STATE_HZ))
    print('with 6 joints     %d' % result['states_with_all_six_joints'])
    print('with target       %d' % result['states_with_commanded_target'])
    print('images            %d' % result['images'])
    pairing = result['pairing']
    print('pairing           %d states within %.0f ms (%.1f%%), median skew %s ms'
          % (pairing['matched'], pairing['budget_ms'], pairing['rate'] * 100,
             pairing['median_skew_ms']))
    print()
    if status_messages == 0:
        print()
        print('WARNING: no /arm_controller/status message arrived, so no sample')
        print('carries a commanded target. The dataset would have no action labels.')
    print('wrote %s' % meta_path)
    return 0 if result['states_with_all_six_joints'] else 1


def command_verify(arguments):
    meta_path = arguments.dir / 'meta.json'
    states_path = arguments.dir / 'arm_states.jsonl'
    index_path = arguments.dir / 'images' / 'index.jsonl'
    for path in (states_path, index_path):
        if not path.is_file():
            print('missing %s' % path)
            return 1

    states = [json.loads(line) for line in states_path.read_text().splitlines() if line.strip()]
    images = [json.loads(line) for line in index_path.read_text().splitlines() if line.strip()]
    result = summarise_recording(states, images, arguments.max_skew_ms)

    print('recording         %s' % arguments.dir)
    if meta_path.is_file():
        meta = json.loads(meta_path.read_text())
        print('task              %r' % meta.get('task'))
        print('passive           %s (published anything: %s)'
              % (meta.get('passive'), meta.get('published_anything')))
    print()
    print('duration          %.1f s' % result['duration_s'])
    print('state samples     %d  (%.2f Hz)' % (result['state_samples'],
                                              result['state_hz'] or 0.0))
    print('  six joints      %d' % result['states_with_all_six_joints'])
    print('  has target      %d' % result['states_with_commanded_target'])
    print('images            %d' % result['images'])
    print('state gap         median %s s, max %s s'
          % (result['state_gap_s']['median'], result['state_gap_s']['max']))
    pairing = result['pairing']
    print('pairing           %d / %d states within %.0f ms (%.1f%%)'
          % (pairing['matched'], result['states_with_all_six_joints'],
             pairing['budget_ms'], pairing['rate'] * 100))
    print('                  median skew %s ms, worst %s ms'
          % (pairing['median_skew_ms'], pairing['worst_skew_ms']))

    problems = assess(result, arguments.min_pairing_rate, arguments.max_gap_s)

    print()
    if problems:
        print('NOT USABLE AS A DEMO:')
        for problem in problems:
            print('  - %s' % problem)
        return 1
    print('looks usable: six joints and a target on every sample, images paired in budget')
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = parser.add_subparsers(dest='command', required=True)

    run = sub.add_parser('run', help='listen and record')
    run.add_argument('--out-dir', type=Path, required=True)
    run.add_argument('--seconds', type=float, default=420.0,
                     help='upper bound; the successful grasp took about 5-6 minutes')
    run.add_argument('--task', default=None,
                     help='language instruction stored with the episode')
    run.add_argument('--bridge-host', default='127.0.0.1')
    run.add_argument('--bridge-port', type=int, default=19091)
    run.add_argument('--max-skew-ms', type=float, default=50.0)
    run.add_argument('--min-image-interval', type=float, default=0.0,
                     help='throttle image storage; 0 keeps the native ~12.7 Hz')
    run.add_argument('--report-every', type=float, default=30.0)
    run.set_defaults(handler=command_run)

    verify = sub.add_parser('verify', help='check a recording')
    verify.add_argument('--dir', type=Path, required=True)
    verify.add_argument('--max-skew-ms', type=float, default=50.0)
    verify.add_argument('--min-pairing-rate', type=float, default=0.9)
    verify.add_argument('--max-gap-s', type=float, default=2.0)
    verify.set_defaults(handler=command_verify)

    arguments = parser.parse_args(argv)
    if arguments.command == 'run' and arguments.seconds <= 0:
        parser.error('--seconds must be positive')
    return arguments.handler(arguments)


if __name__ == '__main__':
    raise SystemExit(main())
