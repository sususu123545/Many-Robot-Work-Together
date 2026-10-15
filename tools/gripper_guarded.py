"""Guarded gripper-only travel on the native ROS 2 arm node: close or open.

Direction is set by --target alone (measured r_joint range 0..700, open end
about 75, jaws meet at about 482). Both directions use the same guards.

Scope, deliberately narrow:
  * never takes serial ownership - the arm node keeps /dev/rrc and does all reads
  * never authorizes enable_arm_motion; joints 1-5 receive no new target
  * asks the node for a real reset, then an explicitly separate gripper gate
  * one bounded increment per message, never more than max_step_ticks

Why this retries: the arm node latches a fault roughly every 30 s on a
temperature-read timeout (command 0x09 only; position reads never fail), and
`reset_eligible` is only true for a few seconds of each recovery cycle. A
single attempt is therefore expected to lose the race. Retries are bounded and
re-read the real gripper position each time, so a latch can never cause a
blind continuation.

This does NOT prove the servo holds its own weight. A reset emits one stop
packet for all six servos, but the fault handler already emits one on every
latch, so a reset adds no new class of command - the area must still be clear.
"""
import argparse
import json
import shlex
import sys

import paramiko

ROS2_SETUP = 'source /opt/ros/jazzy/setup.bash && export ROS_DOMAIN_ID=42 && '

WATCH = r'''
import json, os, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from rcl_interfaces.srv import SetParametersAtomically
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

TARGET = int(os.environ['TARGET'])
MAXSTEP = int(os.environ.get('MAXSTEP', '100'))
DURATION = float(os.environ.get('DURATION', '0.45'))
GATE_TRIES = int(os.environ.get('GATE_TRIES', '6'))
MESSAGE_CAP = int(os.environ.get('MESSAGE_CAP', '24'))
CENTRE, TICK = 700, 3.141592653589793 / 1000.0   # r_joint: 180 deg / 1000 ticks
if not 0 <= TARGET <= 700:
    raise SystemExit('Target must stay inside the measured 0..700 r_joint range')
if not 1 <= MAXSTEP <= 100:
    raise SystemExit('Step must stay inside the enforced 1..100 tick bound')

rclpy.init()
node = Node('close_gripper_guarded')
state = {}


def on_status(message):
    try:
        state['value'] = json.loads(message.data)
        state['at'] = time.monotonic()
    except ValueError:
        pass


node.create_subscription(String, '/arm_controller/status', on_status, 10)
snapshot = lambda: state.get('value') or {}


def spin_for(seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)


def wait_until(predicate, timeout, label):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate(snapshot()):
            return True
    print('  timeout waiting for ' + label)
    return False


def joint(name):
    return (snapshot().get('joints') or {}).get(name) or {}


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
    """Secure the gripper gate inside one short recovery window.

    Returns True only when the node reports gate on, arm gate off, estop clear.
    """
    first = snapshot()
    print('  waiting: health=%s healthy_seconds=%s reset_eligible=%s estop=%s'
          % (first.get('health_state'), first.get('healthy_seconds'),
             first.get('reset_eligible'), first.get('estop')))
    if not wait_until(lambda v: v.get('reset_eligible') and not v.get('poll_pending'),
                      40, 'reset_eligible'):
        return False
    result = call(reset_client, Trigger.Request())
    if not result or not result.success:
        print('  reset refused: ' + (result.message if result else 'NO_RESPONSE'))
        return False
    print('  reset accepted: ' + result.message)
    request = SetParametersAtomically.Request()
    request.parameters = [
        Parameter(name='enable_arm_motion',
                  value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=False)),
        Parameter(name='enable_gripper_motion',
                  value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=True)),
    ]
    result = call(param_client, request)
    if not result or not result.result.successful:
        print('  authorize refused: ' + (result.result.reason if result else 'NO_RESPONSE'))
        return False
    return wait_until(lambda v: v.get('enable_gripper_motion') is True
                      and v.get('enable_arm_motion') is False and not v.get('estop'),
                      5, 'gripper gate visible')


# --- 1. never move before one complete, fresh, in-range whole-arm read -----
if not wait_until(lambda v: v.get('feedback_complete') and v.get('joints'), 25, 'feedback'):
    raise SystemExit('No complete arm feedback; refusing to touch the gripper')
view = snapshot()
print('STEP1 positions=' + json.dumps({k: v['raw'] for k, v in view['joints'].items()}))
print('STEP1 temperatures=' + json.dumps({k: v['temperature'] for k, v in view['joints'].items()}))
print('STEP1 faults=%s last_fault=%s'
      % (view.get('fault_count'), (view.get('last_fault') or {}).get('message')))

# --- 2. secure the gate, bounded retries, each fault costs ~10.5 s ---------
granted = False
for attempt in range(1, GATE_TRIES + 1):
    print('GATE %d/%d' % (attempt, GATE_TRIES))
    if reset_and_authorize():
        granted = True
        break
print('STEP2 gate=%s enable_arm_motion=%s enable_gripper_motion=%s estop=%s'
      % (granted, snapshot().get('enable_arm_motion'),
         snapshot().get('enable_gripper_motion'), snapshot().get('estop')))
if not granted:
    raise SystemExit('Could not secure the gripper gate within the attempt budget')

# --- 3. bounded increments, re-securing the gate if a latch interrupts -----
publisher = node.create_publisher(JointTrajectory, '/gripper_controller/command', 1)
end = time.monotonic() + 8
while publisher.get_subscription_count() < 1 and time.monotonic() < end:
    rclpy.spin_once(node, timeout_sec=0.05)
print('STEP3 subscribers=%d' % publisher.get_subscription_count())
if publisher.get_subscription_count() < 1:
    raise SystemExit('No subscriber on /gripper_controller/command')

position = int(joint('r_joint')['raw'])
print('STEP3 start_raw=%d target_raw=%d' % (position, TARGET))
published, stalled, trace = 0, 0, []
while position != TARGET and published < MESSAGE_CAP:
    if snapshot().get('estop') or not snapshot().get('enable_gripper_motion'):
        print('  latch interrupted at raw=%d; re-securing the gate' % position)
        if not reset_and_authorize():
            print('  re-secure failed; gripper left at raw=%d' % position)
            break
        continue
    following = position + max(-MAXSTEP, min(MAXSTEP, TARGET - position))
    message = JointTrajectory()
    message.joint_names = ['r_joint']
    message.header.stamp = node.get_clock().now().to_msg()
    point = JointTrajectoryPoint()
    point.positions = [(following - CENTRE) * TICK]
    point.time_from_start = Duration(sec=0, nanosec=int(DURATION * 1e9))
    message.points = [point]
    publisher.publish(message)
    published += 1
    spin_for(DURATION + 0.4)
    previous, position = position, int(joint('r_joint')['raw'])
    temp = joint('r_joint').get('temperature')
    trace.append(position)
    print('  msg %d target=%d actual=%d temp=%s estop=%s'
          % (published, following, position, temp, snapshot().get('estop')))
    stalled = stalled + 1 if position == previous else 0
    if stalled >= 3:
        print('  gripper stopped responding; abandoning the remaining travel')
        break

final = snapshot()
print('RESULT published=%d trace=%s final_raw=%d final_temp=%s estop=%s'
      % (published, trace, int(joint('r_joint')['raw']), joint('r_joint').get('temperature'),
         final.get('estop')))
print('RESULT reached=%s enable_arm_motion=%s enable_gripper_motion=%s fault_count=%s'
      % (int(joint('r_joint')['raw']) == TARGET, final.get('enable_arm_motion'),
         final.get('enable_gripper_motion'), final.get('fault_count')))
print('RESULT positions=' + json.dumps({k: v['raw'] for k, v in final['joints'].items()}))
node.destroy_node()
rclpy.shutdown()
'''


def run(client, container, source, env):
    # Environment must be exported by the shell, not pasted into stdin: stdin is
    # fed to `python3 -`, where a bare `TARGET=500` would parse as an assignment
    # and os.environ would stay empty.
    setup = ROS2_SETUP + ''.join('export %s=%s && ' % (key, value)
                                for key, value in env.items()) + 'python3 - '
    command = ('sudo -n docker exec -i ' + shlex.quote(container) + ' bash -lc ' +
               shlex.quote(setup))
    stdin, out, err = client.exec_command(command, timeout=420)
    stdin.write(source)
    stdin.flush()
    stdin.channel.shutdown_write()
    text, errors = out.read().decode('utf-8', 'replace'), err.read().decode('utf-8', 'replace')
    if text:
        print(text, flush=True)
    if out.channel.recv_exit_status() != 0:
        raise RuntimeError(errors or text)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', required=True)
    parser.add_argument('--user', default='pi')
    parser.add_argument('--password', default='raspberrypi')
    parser.add_argument('--target', type=int, default=500,
                        help='Target in raw ticks. Measured: open end about 75 '
                             '(85 is the observed resting open position), jaws '
                             'meet at about 482. 500 is close to the old ROS 1 '
                             'close value; 636 is only an old software clamp and '
                             'stalls the jaws')
    parser.add_argument('--gate-tries', type=int, default=6)
    parser.add_argument('--confirm-gripper-area-clear', action='store_true', required=True)
    parser.add_argument('--plan-only', action='store_true')
    args = parser.parse_args()
    if not 0 <= args.target <= 700:
        parser.error('Target must stay inside the measured 0..700 r_joint range')
    if not 1 <= args.gate_tries <= 12:
        parser.error('Keep the gate attempt budget between 1 and 12')

    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(args.host, username=args.user, password=args.password,
                   timeout=10, auth_timeout=10, allow_agent=False, look_for_keys=False)
    try:
        container = json.loads(
            client.exec_command('cat /run/armpi-ros2/container.json', timeout=10)[1]
            .read().decode())['name']
        if args.plan_only:
            print(json.dumps({
                'container': container, 'target_raw': args.target,
                'sequence': ('wait feedback -> reset_estop -> enable_gripper_motion=true '
                             '(enable_arm_motion stays false) -> increments of <=100 ticks, '
                             're-read position after every message'),
                'gate_tries': args.gate_tries, 'no_command_sent': True}, indent=2))
            return
        run(client, container, WATCH, {'TARGET': args.target,
                                       'GATE_TRIES': args.gate_tries})
    finally:
        client.close()


if __name__ == '__main__':
    sys.stdout.reconfigure(encoding='utf-8')
    main()
