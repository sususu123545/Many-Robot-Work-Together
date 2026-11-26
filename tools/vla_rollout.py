"""Drive the arm from a VLA policy, one validated action at a time.

Why this exists
---------------
The friend-side fine-tune has been trained on `datasets/lerobot_armpi_grasp_v1`
-- 6-dimensional absolute joint radians at 2 Hz.  The rest of this repository's
VLA machinery (`vla_controller.safety`, `chunk.py`, `chunk_executor_node.py`)
was written for a *different* action space: 7-dimensional TCP deltas in
`arm_base` (`armpi_pro_tcp_delta_v1`).  Handing a 6-dim joint vector to that
executor would be rejected on `action_space_id` -- correctly, it is a different
space -- so this script carries its own gate for the joint space rather than
pretending the two are the same.  The rules are the ones that already exist for
motion, restated in joint terms:

* one command is at most `spec.DEFAULT_MAX_STEP_TICKS` (100) ticks per joint;
  exceeding it scales the whole action down uniformly (never per-component,
  which bends a straight approach into a curved miss);
* a target outside the joint's mechanical range is rejected outright;
* `control_dt` must be inside the bus window (0.3..5.0 s);
* motion permission is never granted here -- it is switched on by an operator
  before this script runs, and Ctrl+C stops commanding immediately.

Three modes, deliberately escalating:

    --shadow         predict, log, command nothing.  The only way to find out
                     whether the policy's shoulder/elbow channels are noise
                     (offline they were WORSE than copying the current state)
                     before the arm is allowed to follow them.
    --single-step    execute exactly the first action of the first chunk, after
                     clamping it to +/-20 ticks.  Confirms direction and scale.
    --closed-loop    full loop: chunk in, execute at control_dt, re-plan.

Usage:
    python tools/vla_rollout.py --endpoint http://100.x.y.z:8000/act --shadow
    python tools/vla_rollout.py --endpoint ... --single-step --allow-motion
    python tools/vla_rollout.py --endpoint ... --closed-loop --allow-motion --seconds 180
"""

from __future__ import annotations

import argparse
import base64
import json
import math
import signal
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
sys.path.insert(0, str(ROOT / 'ros2_ws' / 'src' / 'armpi_ros2' / 'armpi_ros2'))
sys.path.insert(0, str(ROOT / 'ros2_ws' / 'src' / 'vla_controller'))

import grasp_sequence as gs          # noqa: E402  (proven command plumbing)
import servo_protocol as sp          # noqa: E402  (the one tick<->radian rule)

JOINT_ORDER = ('joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'r_joint')

#: Mechanical range each joint may be commanded into.  Taken from
#: `grasp_sequence.JOG_SAFE` (the vendor node's own jog envelope) rather than
#: invented here.
SAFE_RANGE = {
    'joint1': (0, 1000),
    'joint2': (125, 875),
    'joint3': (0, 1000),
    'joint4': (0, 1000),
    'joint5': (0, 1000),
    'r_joint': (0, 700),
}

MAX_STEP_TICKS = 100          # spec.DEFAULT_MAX_STEP_TICKS
CONTROL_DT_RANGE = (0.3, 5.0)
VALID_KINDS = ('physical_radians',)

_stop = False


def log(message: str) -> None:
    print(message, flush=True)


def on_signal(signum, frame):        # noqa: ARG001 - signature is fixed
    global _stop
    _stop = True
    log('\n收到中断——停止下发（当前这条动作会走完）')


def radians_to_ticks(values) -> dict:
    """Inverse of `servo_protocol.to_radians` for the six-joint vector.

    Per-joint scale, not one global constant: `tick_radians()` gives the arm
    joints 240 deg / 1000 ticks and the gripper 180 deg / 1000 ticks.  Using the
    arm constant for `r_joint` shrinks every gripper move to 75% of its
    requested travel (measured here: tick 128 round-tripped to 271).
    """
    ticks = {}
    for joint, value in zip(JOINT_ORDER, values):
        _, centre, sign, _, _ = sp.JOINTS[joint]
        ticks[joint] = int(round(centre + (float(value) / (sign * sp.tick_radians(joint)))))
    return ticks


def grab_snapshot(url: str, timeout: float = 8.0) -> bytes:
    with urllib.request.urlopen(url, timeout=timeout) as response:
        return response.read()


def build_state(ticks: dict) -> list:
    return [float(sp.to_radians(joint, ticks[joint])) for joint in JOINT_ORDER]


def call_policy(endpoint: str, observation: dict, timeout: float) -> dict:
    body = json.dumps(observation).encode('utf-8')
    request = urllib.request.Request(
        endpoint, data=body, method='POST',
        headers={'Content-Type': 'application/json'})
    with urllib.request.urlopen(request, timeout=timeout) as response:
        return json.loads(response.read().decode('utf-8'))


def validate_response(payload: dict) -> dict:
    """Check the block before anything touches the arm."""
    actions = payload.get('actions')
    if not isinstance(actions, list) or not actions:
        raise ValueError('response has no actions')
    kind = payload.get('output_kind')
    if kind not in VALID_KINDS:
        # A normalised block needs the training statistics; the policy server is
        # required to de-normalise on its side.  Executing raw normalised values
        # would send the arm to a few radians outside its range.
        raise ValueError('output_kind %r is not one of %s' % (kind, VALID_KINDS))
    control_dt = payload.get('control_dt')
    if not isinstance(control_dt, (int, float)):
        raise ValueError('control_dt missing')
    low, high = CONTROL_DT_RANGE
    if not low <= float(control_dt) <= high:
        raise ValueError('control_dt %.3f outside %.1f..%.1f s'
                         % (control_dt, low, high))
    clean = []
    for index, action in enumerate(actions):
        if not isinstance(action, (list, tuple)) or len(action) != len(JOINT_ORDER):
            raise ValueError('action %d is not %d numbers' % (index, len(JOINT_ORDER)))
        values = tuple(float(v) for v in action)
        if not all(math.isfinite(v) for v in values):
            raise ValueError('action %d contains non-finite values' % index)
        clean.append(values)
    return {'actions': clean, 'control_dt': float(control_dt),
            'validity_s': float(payload.get('validity_s') or 0.0),
            'policy_id': str(payload.get('policy_id') or 'remote'),
            'output_kind': kind}


def gate(action_radians, current_ticks: dict) -> tuple[dict, list]:
    """Joint-space admission: range check, then uniform magnitude scaling.

    Returns (targets_ticks, notes).  An empty targets dict means "reject".
    """
    targets = radians_to_ticks(action_radians)
    notes = []
    for joint in JOINT_ORDER:
        low, high = SAFE_RANGE[joint]
        if not low <= targets[joint] <= high:
            return {}, ['%s target %d outside %d..%d'
                        % (joint, targets[joint], low, high)]

    deltas = {joint: targets[joint] - current_ticks[joint] for joint in JOINT_ORDER}
    worst = max((abs(v) for v in deltas.values()), default=0)
    if worst > MAX_STEP_TICKS:
        scale = MAX_STEP_TICKS / float(worst)
        # Uniform scaling, on purpose: clamping components independently keeps
        # every number legal while bending the path (see safety.py's note).
        targets = {joint: int(round(current_ticks[joint] + deltas[joint] * scale))
                   for joint in JOINT_ORDER}
        notes.append('scaled %.2f (worst %d ticks)' % (scale, worst))
    return targets, notes


def send_targets(bridge, current_ticks: dict, targets: dict, duration: float) -> bool:
    """One command per joint group, using the command formats this repo proved.

    Two different radian scales live in this stack and mixing them silently
    shortens the gripper's travel to 75%: the arm joints are commanded as
    `sign * (tick - 500) * pi/750`, the gripper as `(tick - 700) * pi/1000`.
    """
    arm_joints = [j for j in JOINT_ORDER if j != 'r_joint']
    changed = [j for j in arm_joints
               if abs(targets[j] - current_ticks[j]) >= 2]
    if changed:
        message = {
            'header': {'stamp': {'sec': 0, 'nanosec': 0}, 'frame_id': ''},
            'joint_names': changed,
            'points': [{
                'positions': [gs.SIGNS[j] * (targets[j] - 500.0) * gs.TICK_RAD
                              for j in changed],
                'velocities': [], 'accelerations': [], 'effort': [],
                'time_from_start': {'sec': int(duration),
                                    'nanosec': int(round((duration % 1) * 1e9))},
            }],
        }
        bridge.publish('/arm_controller/command', message,
                       'trajectory_msgs/msg/JointTrajectory')

    if abs(targets['r_joint'] - current_ticks['r_joint']) >= 2:
        gs.gripper_send(bridge, targets['r_joint'], duration=max(duration, 0.3))
    return True


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--endpoint', required=True,
                        help='policy server URL, e.g. http://100.x.y.z:8000/act')
    parser.add_argument('--task', default='Pick up the red ball.')
    parser.add_argument('--rosbridge', default='127.0.0.1:19091')
    parser.add_argument('--snapshot',
                        default='http://127.0.0.1:19099/snapshot?topic=/rear_camera/image_raw&type=ros_compressed')
    parser.add_argument('--control-dt', type=float, default=0.5,
                        help='seconds per action (2 Hz); the policy may override')
    parser.add_argument('--timeout', type=float, default=2.0,
                        help='policy request timeout, seconds')
    parser.add_argument('--seconds', type=float, default=120.0,
                        help='how long to run the loop')
    parser.add_argument('--shadow', action='store_true',
                        help='predict and log only; do not command the arm')
    parser.add_argument('--single-step', action='store_true',
                        help='execute one clamped action, then stop')
    parser.add_argument('--closed-loop', action='store_true',
                        help='execute every action of every chunk')
    parser.add_argument('--allow-motion', action='store_true',
                        help='required for anything but --shadow')
    parser.add_argument('--out', type=Path, default=None,
                        help='JSONL log path (default tmp/vla_rollout_<ts>.jsonl)')
    args = parser.parse_args(argv)

    modes = [args.shadow, args.single_step, args.closed_loop]
    if sum(modes) != 1:
        parser.error('choose exactly one of --shadow / --single-step / --closed-loop')
    if not args.shadow and not args.allow_motion:
        parser.error('--single-step / --closed-loop move the arm; add --allow-motion')

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    host, _, port = args.rosbridge.partition(':')
    bridge = gs.Rosbridge(host, int(port or 19091))
    bridge.drain(0.5)

    if not args.shadow:
        if not gs.secure_gate(bridge, arm=True, gripper=True):
            log('无法授权运动闸门——检查 estop 与健康状态后重试')
            bridge.close()
            return 2
        log('运动闸门已授权（操作者显式传入 --allow-motion）')

    out = args.out or Path('tmp/vla_rollout_%s.jsonl' % time.strftime('%Y%m%d_%H%M%S'))
    out.parent.mkdir(parents=True, exist_ok=True)
    log('日志: %s' % out)

    started = time.monotonic()
    deadline = started + args.seconds
    stats = {'chunks': 0, 'actions': 0, 'rejected': 0, 'scaled': 0,
             'steps_executed': 0, 'late_chunks': 0}
    close_values = []
    single_step_done = False

    with out.open('w', encoding='utf-8') as handle:
        while time.monotonic() < deadline and not _stop:
            loop_start = time.monotonic()
            ticks, _ = gs.read_ticks_and_stamp(bridge)
            if not ticks or any(j not in ticks for j in JOINT_ORDER):
                log('读数不完整，跳过这一拍')
                time.sleep(0.2)
                continue

            state = build_state(ticks)
            request_start = time.monotonic()
            try:
                image = grab_snapshot(args.snapshot)
            except Exception as error:      # noqa: BLE001 - report, keep looping
                log('取图失败: %s' % error)
                time.sleep(0.2)
                continue
            observation = {
                'task': args.task,
                'stamp': time.time(),
                'state': state,
                'image_format': 'jpeg',
                'image_b64': base64.b64encode(image).decode('ascii'),
            }
            try:
                payload = call_policy(args.endpoint, observation, args.timeout)
                block = validate_response(payload)
            except (urllib.error.URLError, ValueError, json.JSONDecodeError) as error:
                log('策略请求/响应无效: %s' % error)
                stats['rejected'] += 1
                time.sleep(0.2)
                continue
            policy_ms = (time.monotonic() - request_start) * 1000
            stats['chunks'] += 1
            if policy_ms > args.control_dt * 1000:
                stats['late_chunks'] += 1

            control_dt = block['control_dt']
            log('[%5.1fs] %s  %d 条动作  %.0f ms%s'
                % (time.monotonic() - started, block['policy_id'],
                   len(block['actions']), policy_ms,
                   '  ← 超时预算' if policy_ms > control_dt * 1000 else ''))

            for index, action in enumerate(block['actions']):
                if _stop:
                    break
                if args.shadow:
                    current = gs.read_ticks_and_stamp(bridge)[0] or ticks
                    targets, notes = gate(action, current)
                    stats['actions'] += 1
                    if not targets:
                        stats['rejected'] += 1
                    elif notes:
                        stats['scaled'] += 1
                    record = {
                        't': round(time.monotonic() - started, 3),
                        'mode': 'shadow',
                        'state': state,
                        'action': list(action),
                        'targets': targets,
                        'notes': notes,
                        'policy_id': block['policy_id'],
                        'policy_ms': round(policy_ms, 1),
                        'index': index,
                    }
                    handle.write(json.dumps(record) + '\n')
                    handle.flush()
                    delta = ({j: targets[j] - current[j] for j in JOINT_ORDER}
                             if targets else {})
                    if delta:
                        log('    #%d 目标 tick 变化 %s'
                            % (index, ' '.join('%s%+d' % (j[:3], delta[j])
                                               for j in JOINT_ORDER)))
                    time.sleep(control_dt)
                    continue

                current = gs.read_ticks_and_stamp(bridge)[0]
                if not current:
                    break
                if args.single_step:
                    # Clamp hard: the first real motion should be a nudge, not a
                    # demonstration of faith in a model that has never driven
                    # hardware.
                    targets = {}
                    for joint in JOINT_ORDER:
                        want = radians_to_ticks(action)[joint]
                        delta = max(-20, min(20, want - current[joint]))
                        targets[joint] = current[joint] + delta
                    targets, notes = gate(
                        [sp.to_radians(j, targets[j]) for j in JOINT_ORDER], current)
                else:
                    targets, notes = gate(action, current)

                delta = {j: targets.get(j, current[j]) - current[j] for j in JOINT_ORDER}
                record = {
                    't': round(time.monotonic() - started, 3),
                    'mode': 'single-step' if args.single_step else 'closed-loop',
                    'state': state,
                    'action': list(action),
                    'targets': targets,
                    'delta_ticks': delta,
                    'notes': notes,
                    'policy_id': block['policy_id'],
                    'policy_ms': round(policy_ms, 1),
                    'index': index,
                }
                if not targets:
                    stats['rejected'] += 1
                    record['rejected'] = True
                    handle.write(json.dumps(record) + '\n')
                    handle.flush()
                    log('    拒收（超出安全范围）')
                    continue
                if notes:
                    stats['scaled'] += 1
                send_targets(bridge, current, targets, control_dt)
                stats['steps_executed'] += 1
                for joint in ('joint2', 'joint3', 'joint4'):
                    if abs(delta[joint]) >= 8:
                        log('    #%d %s %+d tick' % (index, joint, delta[joint]))
                handle.write(json.dumps(record) + '\n')
                handle.flush()
                if args.single_step:
                    single_step_done = True
                    break
                time.sleep(control_dt)
                after, _ = gs.read_ticks_and_stamp(bridge)
                if after and 'r_joint' in after:
                    close_values.append(after['r_joint'])
            if single_step_done or _stop:
                break

            # Re-plan cadence: one chunk per control_dt*len(actions) window is
            # the friend-side requirement (chunk 10 at 2 Hz = 5 s).  Sleep out
            # the remainder so the loop does not free-run.
            elapsed = time.monotonic() - loop_start
            if args.shadow:
                continue
            planned = control_dt * len(block['actions'])
            if elapsed < planned:
                time.sleep(min(planned - elapsed, 1.0))

    bridge.close()

    log('')
    log('=== 汇总 ===')
    log('模式: %s' % ('shadow' if args.shadow else
                     ('single-step' if args.single_step else 'closed-loop')))
    log('收到动作块 %d，动作 %d，执行 %d 步，拒收 %d，缩放 %d，超时块 %d'
        % (stats['chunks'], stats['actions'] + stats['steps_executed'],
           stats['steps_executed'], stats['rejected'], stats['scaled'],
           stats['late_chunks']))
    if close_values:
        deepest = min(close_values)
        log('夹爪观测到的最闭合值: %d（≤350 才算真咬合，≥371 会滑脱）' % deepest)
    if args.shadow:
        log('影子模式：臂未动。检查日志里 joint2/joint3 的抖动——离线数据显示')
        log('这两维比"照抄当前状态"还差，若日志里抖动幅度与轨迹量级相当，')
        log('就不要用 --closed-loop 直接跑，先做混合控制。')
    return 0


if __name__ == '__main__':
    sys.exit(main())
