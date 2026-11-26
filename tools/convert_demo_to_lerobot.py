"""Turn `tools/record_demo.py` recordings into a LeRobot v2 dataset.

What the recordings hold, and why this script exists
----------------------------------------------------
`record_demo.py` writes a raw capture per episode: `arm_states.jsonl` (six-joint
ticks, a carried `last_command`, temperatures, TF), `images/*.jpg` plus
`images/index.jsonl`.  That layout is deliberately close to the wire format --
good for auditing, unreadable for `lerobot-train`, which only speaks the
LeRobot dataset layout (`meta/info.json` + per-episode parquet + an image
directory, with a task string and normalisation statistics).

This script is the translator, and it makes four decisions that the training
pipeline cannot make for itself:

1. **Rate.** The bus publishes `/joint_states` -- and therefore the recorded
   ticks -- at 2.0 Hz, set by `arm_node.py`'s 0.5 s poll timer.  That is the
   native rate of this demonstration, so `fps` is 2.0.  Resampling to SmolVLA's
   own 30 Hz would mean inventing samples between real ones; the execution side
   has to run at 2 Hz instead, which the command bus allows (0.3-5 s per
   command).

2. **Unit.** The wire carries servo ticks (0-1000); SmolVLA's `observation.state`
   wants joint radians.  Both state and action go through the one definition in
   `servo_protocol.to_radians`, so they share a space and a single normaliser.

3. **Action.** The demo controller jogs one joint at a time, so every
   `last_command` in the log has exactly one target.  The six-dimensional action
   is therefore the *carried-forward* last commanded tick per joint: hold the
   others, move one.  That is what the controller actually did, which is what
   imitation learning has to reproduce.

4. **Trim.** Each recording is ~9 minutes; the grasp itself is two to three.
   Frames outside the motion window are dead weight that skew the statistics
   towards the idle pose, so the episode is cut to first arm motion -> last
   motion, with a short margin.

Images are paired to states by nearest timestamp inside a budget (`--max-skew-ms`,
default 150).  A state with no image inside the budget is dropped, never filled
in -- the recording's own rule, kept here.

Usage:
    python tools/convert_demo_to_lerobot.py --out datasets/lerobot_armpi_grasp_v1 \\
        recordings/demo2_20261126_e11 recordings/demo2_20261126_e12b ...

    python tools/convert_demo_to_lerobot.py --verify datasets/lerobot_armpi_grasp_v1
"""

from __future__ import annotations

import argparse
import json
import math
import shutil
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'ros2_ws' / 'src' / 'armpi_ros2' / 'armpi_ros2'))
# pyarrow is vendored: this machine's pip is blocked by an application-control
# policy (pyexpat DLL), so the wheel was unpacked by hand.  Keep that path
# ahead of site-packages so the script works without a pip install.
sys.path.insert(0, str(ROOT / 'vendor' / 'site'))

import servo_protocol as sp  # noqa: E402

#: Canonical order of the six-dimensional vector.  joint5 then the gripper:
#: that is the order `spec.ARM_PI_JOINTS` declares and the order the servo table
#: uses, so state, action and the normaliser all agree.
JOINT_ORDER = ('joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'r_joint')

CODEBASE_VERSION = 'v2.0'
ROBOT_TYPE = 'armpi_pro'
CAMERA_KEY = 'observation.images.camera1'


def log(message: str) -> None:
    print(message, flush=True)


def read_jsonl(path: Path) -> list[dict]:
    rows = []
    with path.open(encoding='utf-8') as handle:
        for line in handle:
            line = line.strip()
            if line:
                rows.append(json.loads(line))
    return rows


def commanded_stream(states: list[dict]) -> list[dict]:
    """Carry each joint's last commanded tick forward through the episode.

    Commands are single-joint jogs, so a raw `last_command` only ever names one
    joint.  A six-dimensional action has to name all six, and the honest reading
    is "the target each joint is currently chasing": the most recent value ever
    commanded for it, or -- for a joint nobody has touched yet -- where it sits.
    """
    carried: dict[str, int] = {}
    stream = []
    for state in states:
        command = state.get('last_command') or {}
        names = command.get('joint_names') or []
        targets = command.get('targets') or []
        for index, name in enumerate(names):
            if index < len(targets):
                carried[name] = int(targets[index][1])
        ticks = state.get('ticks') or {}
        merged = {}
        for joint in JOINT_ORDER:
            if joint in carried:
                merged[joint] = carried[joint]
            elif joint in ticks:
                merged[joint] = int(ticks[joint])
        stream.append(merged)
    return stream


def motion_window(states: list[dict], commanded: list[dict],
                  margin_s: float, min_arm_delta: int = 8) -> tuple[int, int]:
    """Index range [start, end] that contains the grasp, not the idle tail.

    Start: the first sample where an *arm* joint (not the gripper) is commanded
    more than `min_arm_delta` ticks away from where it is.  Opening the jaw at
    the top of the recording is not motion worth training on.

    End: the last sample where any joint actually moved, plus `margin_s`.  The
    arm sits still for minutes after the lift; those frames would otherwise
    dominate the normalisation statistics.
    """
    ticks_by_sample = [state.get('ticks') or {} for state in states]

    start = 0
    for index, (ticks, target) in enumerate(zip(ticks_by_sample, commanded)):
        moved = [
            abs(target.get(joint, ticks.get(joint, 0)) - ticks.get(joint, 0))
            for joint in JOINT_ORDER if joint != 'r_joint'
        ]
        if moved and max(moved) >= min_arm_delta:
            start = max(0, index - 1)
            break

    end = len(states) - 1
    for index in range(len(states) - 1, 0, -1):
        before, after = ticks_by_sample[index - 1], ticks_by_sample[index]
        # 6 ticks, not 2: the servos idle inside a ~12-tick dead band, so a
        # threshold of 2 finds "motion" all the way to the end of the recording
        # and the trim keeps ~400 s of standing still (measured on the first
        # pass: 9 episodes, spans 324-471 s, i.e. nothing was cut).  A real
        # descent step moves 10-20 ticks; 6 separates travel from settling.
        if any(abs(after.get(joint, 0) - before.get(joint, 0)) >= 6
               for joint in JOINT_ORDER):
            end = index
            break

    dt = (states[1]['t_mono'] - states[0]['t_mono']) if len(states) > 1 else 0.5
    pad = max(1, int(round(margin_s / max(dt, 1e-3))))

    # Cap the tail at the grasp itself.  "Last motion" is not enough: after the
    # lift the operator pulls the ball out and the arm is nudged, which reads as
    # motion minutes later and drags idle frames back in (measured 2026-11-25:
    # e01b "moves" at t=400-408 s, long after the grasp, keeping 383 s of a
    # 419 s recording).  The jaw's most-closed sample IS the grasp moment -- the
    # one unambiguous event -- so the episode ends 40 s after it, which covers
    # the two lift jogs and the camera check.
    # NOTE the sense of the gripper scale: 128 is wide open, ~330 is closed on
    # the ball.  LARGER is more closed.  (Taking min() here first picked the
    # open pose at t=29 s and cut every episode down to 23-50 s.)
    peak = max(ticks_by_sample[i].get('r_joint', 0) for i in range(len(states)))
    close_index = next((i for i in range(len(states))
                        if ticks_by_sample[i].get('r_joint', 0) >= peak - 6),
                       len(states) - 1)
    hold_frames = int(round(40.0 / max(dt, 1e-3)))
    end = min(end, close_index + hold_frames, len(states) - 1)
    return max(0, start - pad), end


def pair_images(states: list[dict], images: list[dict], max_skew_s: float):
    """Nearest image per state, within budget; unpaired states are dropped."""
    if not images:
        return []
    times = [image['t_mono'] for image in images]
    order = sorted(range(len(images)), key=lambda i: times[i])
    sorted_times = [times[i] for i in order]

    import bisect

    pairs = []
    for state in states:
        position = bisect.bisect_left(sorted_times, state['t_mono'])
        best = None
        for candidate in (position - 1, position):
            if 0 <= candidate < len(sorted_times):
                skew = abs(sorted_times[candidate] - state['t_mono'])
                if best is None or skew < best[0]:
                    best = (skew, images[order[candidate]])
        if best is not None and best[0] <= max_skew_s:
            pairs.append({'state': state, 'image': best[1], 'skew_s': best[0]})
    return pairs


def radians_vector(ticks: dict) -> list[float]:
    return [float(sp.to_radians(joint, ticks[joint])) for joint in JOINT_ORDER]


def convert_episode(source: Path, fps: float, max_skew_s: float,
                    margin_s: float) -> dict:
    state_path = source / 'arm_states.jsonl'
    image_dir = source / 'images'
    if not state_path.is_file():
        raise SystemExit('%s: no arm_states.jsonl' % source)

    states = read_jsonl(state_path)
    # A state without all six joints cannot be a training frame: the arm node
    # drops the gripper sample whenever an arm read fails, so those rows carry
    # five joints.  Dropping them is honest; filling the gap would invent a
    # gripper angle.  Measured 2026-11-25/26: a handful per episode.
    complete = [s for s in states
                if all(j in (s.get('ticks') or {}) for j in JOINT_ORDER)]
    dropped_incomplete = len(states) - len(complete)
    states = complete
    if len(states) < 10:
        raise SystemExit('%s: only %d states, nothing to convert'
                         % (source, len(states)))
    images = read_jsonl(image_dir / 'index.jsonl') if (image_dir / 'index.jsonl').is_file() else []

    commanded = commanded_stream(states)
    start, end = motion_window(states, commanded, margin_s)
    kept = list(range(start, end + 1))

    windows = [{'t': states[i]['t_mono'],
                'state': radians_vector(states[i]['ticks']),
                'action': radians_vector(commanded[i]),
                'image_meta': None} for i in kept]

    pairs = pair_images([{**w, 't_mono': w['t']} for w in windows], images,
                        max_skew_s)
    # Build the map unconditionally.  The earlier version only did this when
    # something had been dropped, so a fully paired episode reached the writer
    # with no 'image' key at all (KeyError on the first copy).
    by_time = {round(p['state']['t_mono'], 6): p['image'] for p in pairs}
    windows = [w for w in windows if round(w['t'], 6) in by_time]
    for window in windows:
        window['image'] = by_time[round(window['t'], 6)]

    if not windows:
        raise SystemExit('%s: no state/image pair inside %.0f ms'
                         % (source, max_skew_s * 1000))

    first = windows[0]['t']
    for window in windows:
        window['timestamp'] = window['t'] - first

    return {
        'source': source,
        'image_dir': image_dir,
        'windows': windows,
        'dropped_states': len(kept) - len(windows),
        'dropped_incomplete': dropped_incomplete,
        'trim': (states[start]['t_mono'], states[end]['t_mono']),
    }


def write_dataset(out: Path, episodes: list[dict], task: str, fps: float,
                  image_shape: tuple[int, int], val_count: int) -> dict:
    import pyarrow as pa
    import pyarrow.parquet as pq

    if out.exists():
        shutil.rmtree(out)
    (out / 'meta').mkdir(parents=True)
    (out / 'data' / 'chunk-000').mkdir(parents=True)

    height, width = image_shape

    for episode_index, episode in enumerate(episodes):
        windows = episode['windows']
        frame_dir = (out / 'images' / CAMERA_KEY / 'chunk-000'
                     / ('episode_%06d' % episode_index))
        frame_dir.mkdir(parents=True)

        states, actions, stamps, frame_paths = [], [], [], []
        for frame_index, window in enumerate(windows):
            name = 'frame_%06d.jpg' % frame_index
            shutil.copyfile(episode['image_dir'] / window['image']['file'],
                            frame_dir / name)
            frame_paths.append('%s/chunk-000/episode_%06d/%s'
                               % ('images/' + CAMERA_KEY, episode_index, name))
            states.append(window['state'])
            actions.append(window['action'])
            stamps.append(window['timestamp'])

        table = pa.table({
            'observation.state': pa.array(states, type=pa.list_(pa.float32(), 6)),
            'action': pa.array(actions, type=pa.list_(pa.float32(), 6)),
            CAMERA_KEY: pa.array(frame_paths, type=pa.string()),
            'timestamp': pa.array(stamps, type=pa.float32()),
            'frame_index': pa.array(list(range(len(windows))), type=pa.int64()),
            'episode_index': pa.array([episode_index] * len(windows), type=pa.int64()),
            'index': pa.array(list(range(len(windows))), type=pa.int64()),
            'task_index': pa.array([0] * len(windows), type=pa.int64()),
        })
        pq.write_table(table, out / 'data' / 'chunk-000'
                       / ('episode_%06d.parquet' % episode_index))

    total_frames = sum(len(e['windows']) for e in episodes)

    # Splits: hold the last `val_count` episodes out.  Same idea as the sim
    # pilot split -- a contiguous tail keeps the validation set out of the
    # frames the training set has just seen.
    val_count = max(0, min(val_count, len(episodes) - 1))
    train_count = len(episodes) - val_count
    splits = {'train': '0:%d' % train_count}
    if val_count:
        splits['val'] = '%d:%d' % (train_count, len(episodes))

    info = {
        'codebase_version': CODEBASE_VERSION,
        'robot_type': ROBOT_TYPE,
        'total_episodes': len(episodes),
        'total_frames': total_frames,
        'total_tasks': 1,
        'total_videos': 0,
        'total_chunks': 1,
        'chunks_size': 1000,
        'fps': fps,
        'splits': splits,
        'data_path': 'data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet',
        'video_path': None,
        'features': {
            'observation.state': {'dtype': 'float32', 'shape': [6],
                                  'names': list(JOINT_ORDER)},
            'action': {'dtype': 'float32', 'shape': [6],
                       'names': list(JOINT_ORDER)},
            CAMERA_KEY: {'dtype': 'image', 'shape': [height, width, 3],
                         'names': ['height', 'width', 'channels']},
            'timestamp': {'dtype': 'float32', 'shape': [1], 'names': None},
            'frame_index': {'dtype': 'int64', 'shape': [1], 'names': None},
            'episode_index': {'dtype': 'int64', 'shape': [1], 'names': None},
            'index': {'dtype': 'int64', 'shape': [1], 'names': None},
            'task_index': {'dtype': 'int64', 'shape': [1], 'names': None},
        },
    }
    (out / 'meta' / 'info.json').write_text(
        json.dumps(info, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')

    (out / 'meta' / 'tasks.jsonl').write_text(
        json.dumps({'task_index': 0, 'task': task}, ensure_ascii=False) + '\n',
        encoding='utf-8')

    with (out / 'meta' / 'episodes.jsonl').open('w', encoding='utf-8') as handle:
        for episode_index, episode in enumerate(episodes):
            handle.write(json.dumps({
                'episode_index': episode_index,
                'tasks': [task],
                'length': len(episode['windows']),
            }, ensure_ascii=False) + '\n')

    # Statistics, computed here rather than by lerobot so the numbers describe
    # THIS arm.  The SO100 statistics are not transferable: measured 2026-11-23,
    # feeding ArmPi radians against the so100-blue normaliser put 4 of 6 channels
    # beyond 12 sigma (gripper -22.6 sigma).
    all_states = [w['state'] for e in episodes for w in e['windows']]
    all_actions = [w['action'] for e in episodes for w in e['windows']]
    stats = {
        'observation.state': _stats(all_states),
        'action': _stats(all_actions),
    }
    (out / 'meta' / 'stats.json').write_text(
        json.dumps(stats, indent=2) + '\n', encoding='utf-8')

    return {
        'episodes': len(episodes),
        'frames': total_frames,
        'splits': splits,
        'stats': stats,
    }


def _stats(vectors: list[list[float]]) -> dict:
    import numpy as np

    array = np.asarray(vectors, dtype=np.float64)
    return {
        'mean': [round(float(v), 6) for v in array.mean(axis=0)],
        'std': [round(float(v), 6) for v in array.std(axis=0)],
        'min': [round(float(v), 6) for v in array.min(axis=0)],
        'max': [round(float(v), 6) for v in array.max(axis=0)],
    }


def verify(out: Path) -> int:
    import pyarrow.parquet as pq

    info_path = out / 'meta' / 'info.json'
    if not info_path.is_file():
        log('no info.json under %s' % out)
        return 2
    info = json.loads(info_path.read_text(encoding='utf-8'))
    log('数据集: %s' % out)
    log('  robot_type=%s  fps=%.2f  总帧数 %d  episodes %d  切分 %s'
        % (info['robot_type'], info['fps'], info['total_frames'],
           info['total_episodes'], info['splits']))
    log('  特征: %s' % ', '.join(info['features']))

    tasks = read_jsonl(out / 'meta' / 'tasks.jsonl')
    log('  任务: %s' % [t['task'] for t in tasks])

    total = 0
    for episode in read_jsonl(out / 'meta' / 'episodes.jsonl'):
        index = episode['episode_index']
        path = out / 'data' / 'chunk-000' / ('episode_%06d.parquet' % index)
        table = pq.read_table(path)
        frames = table.num_rows
        total += frames
        first = table.column('observation.state')[0].as_py()
        images = table.column(CAMERA_KEY)
        sample = out / images[0].as_py()
        log('  ep %02d: %4d 帧  state[0]=[%s]  图像存在=%s'
            % (index, frames, ' '.join('%+.3f' % v for v in first),
               sample.is_file()))
    log('  合计 %d 帧（info 声明 %d）%s'
        % (total, info['total_frames'],
           '' if total == info['total_frames'] else '  ← 不一致'))

    stats = json.loads((out / 'meta' / 'stats.json').read_text(encoding='utf-8'))
    log('  状态均值: %s' % ' '.join('%+.3f' % v for v in stats['observation.state']['mean']))
    log('  动作均值: %s' % ' '.join('%+.3f' % v for v in stats['action']['mean']))
    return 0 if total == info['total_frames'] else 1


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('sources', nargs='*', type=Path,
                        help='record_demo.py output directories, in order')
    parser.add_argument('--out', type=Path, help='dataset directory to create')
    parser.add_argument('--task', default='Pick up the red ball.',
                        help='language instruction attached to every episode')
    parser.add_argument('--fps', type=float, default=2.0,
                        help='dataset frame rate; 2.0 is the state stream rate')
    parser.add_argument('--max-skew-ms', type=float, default=150.0,
                        help='largest state/image time difference to accept')
    parser.add_argument('--margin-s', type=float, default=0.5,
                        help='keep this much either side of the motion window')
    parser.add_argument('--val', type=int, default=2,
                        help='hold the last N episodes out as validation')
    parser.add_argument('--verify', type=Path, metavar='DIR',
                        help='only inspect an existing dataset')
    args = parser.parse_args(argv)

    if args.verify:
        return verify(args.verify)

    if not args.sources or not args.out:
        parser.error('give at least one recording directory and --out')

    episodes = []
    for source in args.sources:
        episode = convert_episode(source, args.fps, args.max_skew_ms / 1000.0,
                                  args.margin_s)
        windows = episode['windows']
        span = windows[-1]['timestamp']
        log('%s: 保留 %d 帧（丢弃 %d 帧六轴不全、%d 帧无图像）  窗口 %.1f->%.1f s，跨度 %.0f s'
            % (source.name, len(windows), episode['dropped_incomplete'],
               episode['dropped_states'], episode['trim'][0],
               episode['trim'][1], span))
        episodes.append(episode)

    # Every camera in this batch is the same sensor, so the shape comes from the
    # first frame rather than a constant that could drift from the hardware.
    from PIL import Image
    sample = Image.open(episodes[0]['image_dir'] / episodes[0]['windows'][0]['image']['file'])
    width, height = sample.size

    summary = write_dataset(args.out, episodes, args.task, args.fps,
                            (height, width), args.val)
    log('')
    log('写入 %s：%d 条 episode / %d 帧 / 切分 %s'
        % (args.out, summary['episodes'], summary['frames'], summary['splits']))
    return verify(args.out)


if __name__ == '__main__':
    sys.exit(main())
