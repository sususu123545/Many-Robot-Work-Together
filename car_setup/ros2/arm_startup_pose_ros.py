"""Inner guarded pose travel. Runs INSIDE the native ROS 2 container via stdin.

Single source of truth: both tools/arm_pose_guarded.py (ad-hoc, from the PC) and
/opt/armpi-ros2/arm_startup_pose.py (boot service) pipe this exact file into
`docker exec -i <container> bash -lc 'source ... && python3 -'`.

Configuration arrives as environment variables exported by the shell that runs
this file (stdin is this file, so a bare `X=1` line would parse as an assignment):

  JOINTS        comma separated joint names to command, e.g. joint1,...,joint5
  POSE_<name>   raw tick target per joint, must be inside the measured limits
  STEP          1..100 ticks per joint per message (node enforces 100 too)
  DURATION      seconds per message, 0.3..5
  GATE_TRIES    bounded reset attempts; each latch costs about 10.5 s recovery
  MESSAGE_CAP   hard ceiling on published messages
  MAX_TRAVEL    refuse to start if any joint must travel further than this

It never opens the serial port: the arm node keeps /dev/rrc and remains the only
reader. It only calls documented services and publishes to the documented topic.
"""
import json
import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from rcl_interfaces.srv import SetParametersAtomically
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

LIMITS = {'joint1': (0, 1000), 'joint2': (125, 875), 'joint3': (0, 1000),
          'joint4': (0, 1000), 'joint5': (0, 1000), 'r_joint': (0, 700)}
CENTRE = {'joint1': 500, 'joint2': 500, 'joint3': 500, 'joint4': 500,
          'joint5': 500, 'r_joint': 700}
# Mirrors servo_protocol.JOINTS[2]. joint3 is the only joint that is INVERTED, so
# dropping this sign sends it to the mirrored position: the node then decodes the
# target as 100 instead of 900 and rejects the message with "Step too large".
SIGN = {'joint1': 1, 'joint2': 1, 'joint3': -1, 'joint4': 1, 'joint5': 1, 'r_joint': 1}
ALL = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'r_joint']

NAMES = [n for n in os.environ['JOINTS'].split(',') if n]
if not NAMES or any(n not in CENTRE for n in NAMES):
    raise SystemExit('JOINTS must name known joints')
TARGETS = {n: int(os.environ['POSE_' + n]) for n in NAMES}
STEP = int(os.environ.get('STEP', '100'))
DURATION = float(os.environ.get('DURATION', '0.6'))
GATE_TRIES = int(os.environ.get('GATE_TRIES', '6'))
MESSAGE_CAP = int(os.environ.get('MESSAGE_CAP', '60'))
MAX_TRAVEL = int(os.environ.get('MAX_TRAVEL', '400'))
# A servo stops once it is inside its own positioning deadband, so demanding an
# exact tick match guarantees a false "stalled" verdict. 12 ticks is about 2.9
# degrees on an arm joint and 2.2 degrees on the gripper.
TOLERANCE = int(os.environ.get('TOLERANCE', '12'))

if not 1 <= STEP <= 100:
    raise SystemExit('STEP must stay inside the enforced 1..100 bound')
if not 0.3 <= DURATION <= 5:
    raise SystemExit('DURATION must stay inside the enforced 0.3..5 bound')
for name, raw in TARGETS.items():
    low, high = LIMITS[name]
    if not low <= raw <= high:
        raise SystemExit('Target outside measured limits: %s=%d not in %d..%d'
                         % (name, raw, low, high))
print('CONFIG joints=%s targets=%s step=%d duration=%.2f gate_tries=%d max_travel=%d tolerance=%d'
      % (NAMES, json.dumps(TARGETS), STEP, DURATION, GATE_TRIES, MAX_TRAVEL, TOLERANCE),
      flush=True)


def ticks(name):
    return 3.141592653589793 / 1000.0 if name == 'r_joint' else 4.188790204786391 / 1000.0


def to_radians(name, raw):
    """Exactly servo_protocol.to_radians: sign must be applied."""
    return SIGN[name] * (raw - CENTRE[name]) * ticks(name)


def to_raw(name, radians):
    """Exactly servo_protocol.to_raw, used only to prove the round trip."""
    return round(CENTRE[name] + SIGN[name] * radians / ticks(name))


rclpy.init()
node = Node('arm_startup_pose')
state = {}


def on_status(message):
    try:
        state['value'] = json.loads(message.data)
        state['received_at'] = time.monotonic()
    except ValueError:
        pass


node.create_subscription(String, '/arm_controller/status', on_status, 10)
snapshot = lambda: state.get('value') or {}
actual = lambda name: ((snapshot().get('joints') or {}).get(name) or {}).get('raw')


def fresh_feedback(view):
    if (not view.get('feedback_complete') or not view.get('feedback_real')
            or time.monotonic() - state.get('received_at', 0) > 1.0):
        return False
    ages = view.get('feedback_age_seconds') or {}
    joints = view.get('joints') or {}
    return (set(joints) == set(ALL) and set(ages) == set(ALL)
            and all(0 <= ages[n] < 1.5 and LIMITS[n][0] <= joints[n]['raw'] <= LIMITS[n][1]
                    and 0 <= joints[n]['temperature'] < 65 for n in ALL))


def reject_fault(view):
    if 'manual_estop' in view.get('stop_reasons', []):
        raise SystemExit('REFUSED manual emergency; startup cannot clear it')
    if view.get('current_read_error'):
        raise SystemExit('REFUSED current feedback error: ' + view['current_read_error'])


def spin_for(seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)


def wait_until(predicate, timeout, label, quiet=False):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
        reject_fault(snapshot())
        if predicate(snapshot()):
            return True
    if not quiet:
        print('  timeout waiting for ' + label, flush=True)
    return False


def call(client, request, timeout=15.0):
    if not client.wait_for_service(timeout_sec=8):
        raise SystemExit('Service unavailable: ' + client.srv_name)
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    return future.result()


reset_client = node.create_client(Trigger, '/arm_controller/reset_estop')
param_client = node.create_client(SetParametersAtomically,
                                  '/arm_controller/set_parameters_atomically')


def reset_and_authorize():
    view = snapshot()
    print('  health=%s healthy_seconds=%s reset_eligible=%s estop=%s fault=%s'
          % (view.get('health_state'), view.get('healthy_seconds'),
             view.get('reset_eligible'), view.get('estop'),
             (view.get('fault') or 'NONE')[:70]), flush=True)
    if not wait_until(lambda v: fresh_feedback(v) and v.get('reset_eligible') and not v.get('poll_pending'),
                      45, 'reset_eligible'):
        return False
    result = call(reset_client, Trigger.Request())
    if not result or not result.success:
        print('  reset refused: ' + (result.message if result else 'NO_RESPONSE'), flush=True)
        return False
    request = SetParametersAtomically.Request()
    request.parameters = [
        Parameter(name='enable_arm_motion',
                  value=ParameterValue(type=ParameterType.PARAMETER_BOOL,
                                       bool_value=any(n != 'r_joint' for n in NAMES))),
        Parameter(name='enable_gripper_motion',
                  value=ParameterValue(type=ParameterType.PARAMETER_BOOL,
                                       bool_value='r_joint' in NAMES)),
    ]
    result = call(param_client, request)
    if not result or not result.result.successful:
        print('  authorize refused: '
              + (result.result.reason if result else 'NO_RESPONSE'), flush=True)
        return False
    return wait_until(lambda v: not v.get('estop') and v.get('enable_arm_motion') is True,
                      5, 'motion gate')


try:
    # 1. A complete, fresh whole-arm read before anything moves.
    if not wait_until(fresh_feedback, 30, 'feedback'):
        raise SystemExit('REFUSED no complete arm feedback')
    print('START actual=' + json.dumps({n: actual(n) for n in ALL}), flush=True)
    before = {n: actual(n) for n in NAMES}
    if any(v is None for v in before.values()):
        raise SystemExit('REFUSED missing joint reading for ' + NAMES[0])
    travel = {n: TARGETS[n] - before[n] for n in NAMES}
    print('START travel=' + json.dumps(travel), flush=True)

    # 2. Refuse a sweep that is too long to be an intentional boot pose.
    worst = max(abs(v) for v in travel.values())
    if worst > MAX_TRAVEL:
        raise SystemExit('REFUSED largest travel %d ticks exceeds max_travel %d; '
                         'the arm is probably not where a boot pose expects it'
                         % (worst, MAX_TRAVEL))

    # 3. Secure the gate. Every latch costs about 10.5 s, and the resettable
    #    window is only a few seconds wide, so a single try usually loses.
    granted = False
    for attempt in range(1, GATE_TRIES + 1):
        print('GATE %d/%d' % (attempt, GATE_TRIES), flush=True)
        if reset_and_authorize():
            granted = True
            break
    print('STEP gate=%s arm_gate=%s gripper_gate=%s estop=%s'
          % (granted, snapshot().get('enable_arm_motion'),
             snapshot().get('enable_gripper_motion'), snapshot().get('estop')), flush=True)
    if not granted:
        raise SystemExit('REFUSED could not secure the motion gate')

    # 4. Bounded simultaneous travel.
    publisher = node.create_publisher(JointTrajectory, '/arm_controller/command', 1)
    end = time.monotonic() + 10
    while publisher.get_subscription_count() < 1 and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    print('STEP subscribers=%d' % publisher.get_subscription_count(), flush=True)
    if publisher.get_subscription_count() < 1:
        raise SystemExit('REFUSED no subscriber on /arm_controller/command')

    published, stalled, seconds = 0, 0, 0.0
    while published < MESSAGE_CAP:
        reject_fault(snapshot())
        if not fresh_feedback(snapshot()):
            raise SystemExit('REFUSED stale feedback during travel; no cached target')
        position = {n: actual(n) for n in NAMES}
        if any(v is None for v in position.values()):
            spin_for(0.5)
            continue
        remaining = {n: TARGETS[n] - position[n] for n in NAMES}
        # Stop as soon as every joint is inside tolerance: this is the normal end
        # of travel, because the servos settle in their deadband, not on the tick.
        if all(abs(v) <= TOLERANCE for v in remaining.values()):
            print('  all joints within %d ticks; stopping' % TOLERANCE, flush=True)
            break
        if snapshot().get('estop') or not snapshot().get('enable_arm_motion'):
            raise SystemExit('REFUSED latch interrupted travel; no automatic reset/retry')
        following = {n: position[n] + max(-STEP, min(STEP, remaining[n])) for n in NAMES}
        angles = [to_radians(n, following[n]) for n in NAMES]
        # Round-trip guard. joint3 is inverted, and a missing sign silently sends
        # the joint to ~100 instead of ~900, which the node answers with
        # "Step too large" and no motion. Prove the decode before publishing.
        mismatch = {n: to_raw(n, a) for n, a in zip(NAMES, angles)
                    if to_raw(n, a) != following[n]}
        if mismatch:
            raise SystemExit('REFUSED angle/raw round trip mismatch: ' + json.dumps(mismatch))
        message = JointTrajectory()
        message.joint_names = list(NAMES)
        message.header.stamp = node.get_clock().now().to_msg()
        point = JointTrajectoryPoint()
        point.positions = angles
        point.time_from_start = Duration(sec=int(DURATION),
                                         nanosec=int((DURATION % 1) * 1e9))
        message.points = [point]
        publisher.publish(message)
        published += 1
        spin_for(DURATION + 0.5)
        reject_fault(snapshot())
        if not fresh_feedback(snapshot()):
            raise SystemExit('REFUSED stale feedback after command')
        seconds += DURATION + 0.5
        after = {n: actual(n) for n in NAMES}
        print('  msg %d commanded=%s actual=%s estop=%s'
              % (published, json.dumps(following), json.dumps(after),
                 snapshot().get('estop')), flush=True)
        stalled = 0 if any(after[n] != position[n] for n in NAMES) else stalled + 1
        if stalled >= 3:
            print('  no joint moved for three messages; abandoning the rest', flush=True)
            break

    final = {n: actual(n) for n in NAMES}
    deviation = {n: TARGETS[n] - final[n] for n in NAMES}
    within = {n: abs(deviation[n]) <= TOLERANCE for n in NAMES}
    reached = all(within.values())
    exact = all(deviation[n] == 0 for n in NAMES)
    print('RESULT published=%d seconds=%.1f' % (published, seconds), flush=True)
    print('RESULT actual=' + json.dumps({n: actual(n) for n in ALL}), flush=True)
    print('RESULT deviation=' + json.dumps(deviation), flush=True)
    print('RESULT within_tolerance=%s exact=%s tolerance=%d'
          % (json.dumps(within), exact, TOLERANCE), flush=True)
    print('RESULT reached=%s estop=%s fault_count=%s'
          % (reached, snapshot().get('estop'), snapshot().get('fault_count')), flush=True)
    print('RESULT_JSON=' + json.dumps({
        'joints': NAMES, 'targets': TARGETS, 'final': final, 'deviation': deviation,
        'within_tolerance': within, 'tolerance': TOLERANCE, 'exact': exact,
        'reached': reached, 'messages': published, 'seconds': round(seconds, 1),
        'estop': snapshot().get('estop'), 'fault_count': snapshot().get('fault_count'),
        'health_state': snapshot().get('health_state')}), flush=True)
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if reached else 3)
except SystemExit as stop:
    raise
finally:
    try:
        node.destroy_node()
    except Exception:
        pass
