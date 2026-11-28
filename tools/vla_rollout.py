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
import http.client
import io
import json
import math
import signal
import sys
import time
import urllib.parse
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
sys.path.insert(0, str(ROOT / 'ros2_ws' / 'src' / 'armpi_ros2' / 'armpi_ros2'))
sys.path.insert(0, str(ROOT / 'ros2_ws' / 'src' / 'vla_controller'))

import grasp_sequence as gs          # noqa: E402  (proven command plumbing)
import servo_protocol as sp          # noqa: E402  (the one tick<->radian rule)

JOINT_ORDER = ('joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'r_joint')

#: Range each joint may be commanded into: the PROVEN jog envelope from
#: `grasp_sequence.SAFE`, not the raw servo table.
#:
#: This distinction cost a latched arm on 2026-11-28.  The servo table says
#: joint3 is legal to 1000, so the gate happily allowed a policy that kept
#: pushing it up; the arm reached 1028, the node logged "Invalid physical
#: feedback: joint3: raw=1028, allowed_raw=0..1000" and latched, after which NO
#: software command is accepted and only a service restart (whose pose_restore
#: drives the bus directly, bypassing the node) brings it back.
#: The jog envelope stops at 950 precisely to keep that margin -- the difference
#: between 950 and 1000 is the whole remaining safety budget.
SAFE_RANGE = {
    'joint1': (0, 1000),
    'joint2': (150, 860),
    'joint3': (40, 950),
    'joint4': (0, 980),
    'joint5': (0, 1000),
    'r_joint': (0, 700),
}

#: Warn when a target lands this close to a joint's limit -- an autonomous loop
#: walking towards a ceiling should say so before it gets there.
LIMIT_WARNING_MARGIN = 25

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


def read_state(bridge, timeout: float = 8.0,
               drain: bool = True) -> tuple[dict | None, dict | None]:
    """All six joints from /joint_states, in servo ticks.

    Deliberately NOT `grasp_sequence.read_ticks_and_stamp`: that helper filters to
    `SIGNS` (joint1..joint4) because the jog path only commands those, so the
    gripper and joint5 come back missing and every observation would be rejected
    (measured 2026-11-26: 100+ samples dropped, "缺 ['joint5','r_joint']").  The
    policy's sixth input IS the gripper, so it has to be here.
    """
    if drain:
        # Only the first read per cycle needs the flush; doing it per action
        # costs 0.3 s each and turned a 5 s chunk into 11 s (2026-11-26).
        bridge.drain(0.3)
    message = bridge.wait_for_topic('/joint_states', timeout, newest=True)
    if message is None:
        return None, None
    header = (message.get('header') or {}).get('stamp') or {}
    stamp = ({'sec': int(header['sec']), 'nanosec': int(header.get('nanosec', 0))}
             if 'sec' in header else None)
    ticks = {}
    for name, position in zip(message.get('name') or [], message.get('position') or []):
        if name in JOINT_ORDER:
            _, centre, sign, _, _ = sp.JOINTS[name]
            ticks[name] = int(round(centre + sign * float(position) / sp.tick_radians(name)))
    missing = [j for j in JOINT_ORDER if j not in ticks]
    if missing:
        return None, stamp
    return ticks, stamp


def grab_snapshot(url: str, timeout: float = 8.0) -> bytes:
    import urllib.request
    with urllib.request.urlopen(url, timeout=timeout) as response:
        return response.read()


def recompress(jpeg: bytes, quality: int) -> bytes:
    """Re-encode the wrist frame smaller before it crosses the network.

    The camera hands us ~83 KB at 640x480.  Over a Tailscale *relay* (the path
    measured 356-526 ms round trip on 2026-11-26, direct connection not
    established, ping jitter to 225 ms) that is the dominant cost, not the
    policy's 180 ms of inference.  Quality 70 gives 23 KB with the resolution
    the policy was trained on intact; dropping to 320x240 would be 8 KB but
    would hand the model a different image than the dataset contains.
    """
    if quality >= 95:
        return jpeg
    from PIL import Image
    image = Image.open(io.BytesIO(jpeg)).convert('RGB')
    buffer = io.BytesIO()
    image.save(buffer, 'JPEG', quality=quality, optimize=True)
    return buffer.getvalue()


def build_state(ticks: dict) -> list:
    return [float(sp.to_radians(joint, ticks[joint])) for joint in JOINT_ORDER]


class PolicyClient:
    """POST /act over ONE kept-alive connection.

    A fresh TCP+TLS-less handshake per request is fine on a LAN and ruinous over
    a relayed tailnet path -- the friend measured 356-526 ms round trip with
    direct connection unavailable, and a good part of that is connection setup
    rather than inference.  Reusing the socket removes it.  On any transport
    error the connection is dropped and rebuilt once, then the call fails.
    """

    def __init__(self, endpoint: str, timeout: float):
        parsed = urllib.parse.urlsplit(endpoint)
        if not parsed.scheme or not parsed.hostname:
            raise ValueError('endpoint must look like http://host:port/path')
        self.https = parsed.scheme == 'https'
        self.host = parsed.hostname
        self.port = parsed.port or (443 if self.https else 80)
        self.path = parsed.path or '/'
        self.timeout = timeout
        self._connection = None

    def _connect(self):
        factory = http.client.HTTPSConnection if self.https else http.client.HTTPConnection
        self._connection = factory(self.host, self.port, timeout=self.timeout)
        return self._connection

    def call(self, observation: dict) -> dict:
        body = json.dumps(observation).encode('utf-8')
        headers = {'Content-Type': 'application/json',
                   'Content-Length': str(len(body)),
                   'Connection': 'keep-alive'}
        last_error = None
        for attempt in range(2):
            connection = self._connection or self._connect()
            try:
                connection.request('POST', self.path, body=body, headers=headers)
                response = connection.getresponse()
                raw = response.read()
                if response.status != 200:
                    raise IOError('HTTP %d: %s' % (response.status, raw[:200]))
                return json.loads(raw.decode('utf-8'))
            except Exception as error:      # noqa: BLE001 - retry once, then raise
                last_error = error
                try:
                    connection.close()
                except Exception:           # noqa: BLE001
                    pass
                self._connection = None
        raise IOError('policy call failed: %s' % last_error)

    def close(self):
        if self._connection is not None:
            try:
                self._connection.close()
            except Exception:               # noqa: BLE001
                pass
            self._connection = None


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

    for joint in JOINT_ORDER:
        low, high = SAFE_RANGE[joint]
        if targets[joint] > high - LIMIT_WARNING_MARGIN:
            notes.append('%s near its upper limit (%d, cap %d)'
                         % (joint, targets[joint], high))
        elif targets[joint] < low + LIMIT_WARNING_MARGIN:
            notes.append('%s near its lower limit (%d, floor %d)'
                         % (joint, targets[joint], low))

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


def send_targets(bridge, current_ticks: dict, targets: dict, duration: float,
                 stamp: dict | None) -> bool:
    """One command per joint group, in the formats this stack actually accepts.

    Two failure modes were measured here on 2026-11-28 and both are silent from
    the caller's side -- the arm simply does not move:

    * `Command must carry a fresh robot-clock stamp`.  The header stamp must be
      the robot's clock taken from the /joint_states message just read, not
      zero and not the PC's clock.
    * `Invalid duration or preceding point still executing`.  A 0.5 s move must
      be sent as sec=0 nanosec=500000000; packing it as `int(duration)` with a
      zero nanosecond field asks for a zero-length move.

    Two different radian scales live in this stack as well, and mixing them
    shortens the gripper's travel to 75%: arm joints are `sign*(tick-500)*pi/750`,
    the gripper is `(tick-700)*pi/1000`.
    """
    header = {'stamp': stamp or {'sec': 0, 'nanosec': 0}, 'frame_id': ''}
    time_from_start = {'sec': int(duration),
                       'nanosec': int(round((duration % 1) * 1e9))}

    arm_joints = [j for j in JOINT_ORDER if j != 'r_joint']
    changed = [j for j in arm_joints
               if abs(targets[j] - current_ticks[j]) >= 2]
    if changed:
        message = {
            'header': header,
            'joint_names': changed,
            'points': [{
                'positions': [gs.SIGNS[j] * (targets[j] - 500.0) * gs.TICK_RAD
                              for j in changed],
                'velocities': [], 'accelerations': [], 'effort': [],
                'time_from_start': time_from_start,
            }],
        }
        bridge.publish('/arm_controller/command', message,
                       'trajectory_msgs/msg/JointTrajectory')

    if abs(targets['r_joint'] - current_ticks['r_joint']) >= 2:
        # Same message shape, but the gripper's own scale and topic.  Written
        # out rather than reused from grasp_sequence because `gripper_send`
        # hardcodes nanosec=0 and therefore cannot express a 0.5 s move.
        bridge.publish('/gripper_controller/command', {
            'header': header,
            'joint_names': ['r_joint'],
            'points': [{
                'positions': [(targets['r_joint'] - 700) * math.pi / 1000.0],
                'velocities': [], 'accelerations': [], 'effort': [],
                'time_from_start': time_from_start,
            }],
        }, 'trajectory_msgs/msg/JointTrajectory')
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
    parser.add_argument('--image-quality', type=int, default=70,
                        help='re-encode the wrist frame at this JPEG quality '
                             '(70 -> ~23 KB; 95 disables re-encoding)')
    parser.add_argument('--no-warmup', action='store_true',
                        help='skip the throwaway first request (it pays the '
                             'server-side model warm-up)')
    parser.add_argument('--execute-first', type=int, default=0,
                        help='execute only the first N actions of each chunk and '
                             're-plan (0 = the whole chunk).  A policy with a '
                             'small systematic per-channel bias drifts because the '
                             'loop integrates it: measured 2026-11-28, joint3 was '
                             'biased +6.6 ticks/step with 100%% sign consistency, '
                             'which compounds to +30-40 ticks per minute.  A short '
                             'open-loop window caps that accumulation.')
    parser.add_argument('--stride', type=int, default=1,
                        help='execute every Nth action of a chunk and stretch the '
                             'command duration to N*control_dt.  The per-step servo '
                             'dead band is ~12 ticks and the policy asks for 9-20, '
                             'so at stride 1 half of every correction is eaten and '
                             'the loop oscillates (measured 2026-11-28).')
    parser.add_argument('--warmup', type=int, default=5,
                        help='max warm-up requests; stops early once two in a '
                             'row come in under the control period')
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
    # Pre-register the command topics.  rosbridge treats the FIRST publish to an
    # unknown topic as "create the registration" and drops that message.  The
    # symptom is nasty because nothing anywhere reports it: measured 2026-11-28,
    # two single-step runs announced a command, the arm never moved, and the arm
    # node logged no rejection at all -- the message simply never arrived.
    bridge.advertise('/arm_controller/command', 'trajectory_msgs/msg/JointTrajectory')
    bridge.advertise('/gripper_controller/command', 'trajectory_msgs/msg/JointTrajectory')
    time.sleep(2.5)
    log('命令话题已预注册（等待 2.5 s 生效）')

    if not args.shadow:
        if not gs.secure_gate(bridge, arm=True, gripper=True):
            log('无法授权运动闸门——检查 estop 与健康状态后重试')
            bridge.close()
            return 2
        log('运动闸门已授权（操作者显式传入 --allow-motion）')
    else:
        # Shadow mode must not grant motion, but it still needs to READ the arm:
        # after a reboot the health sits in RECOVERED_AWAIT_RESET and the node
        # withholds /joint_states until the latch is cleared, so every sample
        # came back incomplete and the loop spun on "读数不完整" (measured
        # 2026-11-26).  Clearing the latch does not authorise anything.
        if gs.reset_estop(bridge, quiet=True):
            log('已清急停闩锁（仅恢复状态流，未授权运动）')
        else:
            log('清闩锁失败——状态流可能不完整')

    out = args.out or Path('tmp/vla_rollout_%s.jsonl' % time.strftime('%Y%m%d_%H%M%S'))
    out.parent.mkdir(parents=True, exist_ok=True)
    log('日志: %s' % out)

    client = PolicyClient(args.endpoint, args.timeout)
    log('策略端点: %s' % args.endpoint)

    started = time.monotonic()
    deadline = started + args.seconds
    stats = {'chunks': 0, 'actions': 0, 'rejected': 0, 'scaled': 0,
             'steps_executed': 0, 'late_chunks': 0, 'bytes': []}
    latencies = []

    if not args.no_warmup:
        # The first request pays for connection setup AND the server's own warm
        # up: measured 2026-11-26, 5228 ms on the first call, 1.3-2.0 s for the
        # next few, 400 ms once settled.  Recording those numbers as "latency"
        # would condemn a working link, so warm up until two consecutive calls
        # come in under the control period (or give up after --warmup tries).
        ticks, _ = read_state(bridge)
        if ticks and all(j in ticks for j in JOINT_ORDER):
            budget_ms = args.control_dt * 1000
            fast = 0
            for attempt in range(1, args.warmup + 1):
                try:
                    warm = {
                        'task': args.task, 'stamp': time.time(),
                        'state': build_state(ticks), 'image_format': 'jpeg',
                        'image_b64': base64.b64encode(
                            recompress(grab_snapshot(args.snapshot),
                                       args.image_quality)).decode('ascii'),
                    }
                    warm_start = time.monotonic()
                    client.call(warm)
                    elapsed = (time.monotonic() - warm_start) * 1000
                    log('预热 %d/%d: %.0f ms%s' % (attempt, args.warmup, elapsed,
                                                  '' if elapsed <= budget_ms else ' (超出预算)'))
                    fast = fast + 1 if elapsed <= budget_ms else 0
                    if fast >= 2:
                        log('预热完成（连续两次进入预算）')
                        break
                except Exception as error:  # noqa: BLE001 - warm-up is best effort
                    log('预热失败（不影响后续）: %s' % error)
                    break
    close_values = []
    single_step_done = False

    with out.open('w', encoding='utf-8') as handle:
        while time.monotonic() < deadline and not _stop:
            loop_start = time.monotonic()
            ticks, stamp = read_state(bridge)
            if not ticks or any(j not in ticks for j in JOINT_ORDER):
                stats['incomplete'] = stats.get('incomplete', 0) + 1
                if stats['incomplete'] in (1, 10) or stats['incomplete'] % 50 == 0:
                    missing = ([j for j in JOINT_ORDER if j not in (ticks or {})]
                               if ticks else '整帧')
                    log('读数不完整（第 %d 次，缺 %s）'
                        % (stats['incomplete'], missing))
                time.sleep(0.2)
                continue

            state = build_state(ticks)
            snapshot_start = time.monotonic()
            try:
                image = grab_snapshot(args.snapshot)
            except Exception as error:      # noqa: BLE001 - report, keep looping
                log('取图失败: %s' % error)
                time.sleep(0.2)
                continue
            compact = recompress(image, args.image_quality)
            stats['bytes'].append(len(compact))
            observation = {
                'task': args.task,
                'stamp': time.time(),
                'state': state,
                'image_format': 'jpeg',
                'image_b64': base64.b64encode(compact).decode('ascii'),
            }
            request_start = time.monotonic()
            snapshot_ms = (request_start - snapshot_start) * 1000
            try:
                payload = client.call(observation)
                block = validate_response(payload)
            except (urllib.error.URLError, ValueError, json.JSONDecodeError) as error:
                log('策略请求/响应无效: %s' % error)
                stats['rejected'] += 1
                time.sleep(0.2)
                continue
            policy_ms = (time.monotonic() - request_start) * 1000
            latencies.append(policy_ms)
            stats.setdefault('snapshot_ms', []).append(snapshot_ms)
            stats['chunks'] += 1
            if policy_ms > args.control_dt * 1000:
                stats['late_chunks'] += 1

            control_dt = block['control_dt']
            log('[%5.1fs] %s  %d 条动作  取图 %.0f + 策略 %.0f ms%s'
                % (time.monotonic() - started, block['policy_id'],
                   len(block['actions']), snapshot_ms, policy_ms,
                   '  ← 策略超预算' if policy_ms > control_dt * 1000 else ''))

            stride = max(1, args.stride)
            command_dt = control_dt * stride
            for index, action in enumerate(block['actions']):
                if _stop:
                    break
                if index % stride:
                    continue
                if args.execute_first and index >= args.execute_first:
                    break
                if args.shadow:
                    current = read_state(bridge, drain=False)[0] or ticks
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

                # Order matters: re-securing the gate takes two service round
                # trips (~1-2 s), which is longer than the arm node's stamp
                # window (-0.1..+1 s).  Read the stamp AFTER the gate, or the
                # command is rejected with "fresh robot-clock stamp" while every
                # status field still says READY (measured 2026-11-28: two
                # attempts, arm never moved).
                if not gs.secure_gate(bridge, arm=True, gripper=True):
                    log('    闸门续期失败，跳过这一步')
                    stats['rejected'] += 1
                    continue
                current, current_stamp = read_state(bridge)
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
                send_targets(bridge, current, targets, command_dt,
                             current_stamp or stamp)
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
                after, _ = read_state(bridge)
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

    client.close()
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
    if latencies:
        ordered = sorted(latencies)
        middle = ordered[len(ordered) // 2]
        high = ordered[min(len(ordered) - 1, int(len(ordered) * 0.95))]
        log('往返延迟: 中位 %.0f ms  p95 %.0f ms  最大 %.0f ms（预算 %.0f ms）'
            % (middle, high, ordered[-1], args.control_dt * 1000))
    if stats['bytes']:
        sizes = sorted(stats['bytes'])
        log('图像体积: 中位 %.0f KB（q=%d）' % (sizes[len(sizes) // 2] / 1024.0,
                                             args.image_quality))
    if args.shadow:
        log('影子模式：臂未动。检查日志里 joint2/joint3 的抖动——离线数据显示')
        log('这两维比"照抄当前状态"还差，若日志里抖动幅度与轨迹量级相当，')
        log('就不要用 --closed-loop 直接跑，先做混合控制。')
    return 0


if __name__ == '__main__':
    sys.exit(main())
