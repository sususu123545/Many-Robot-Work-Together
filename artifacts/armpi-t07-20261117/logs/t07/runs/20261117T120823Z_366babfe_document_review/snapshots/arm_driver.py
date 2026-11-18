"""Arm driver: one action per invocation, all gates checked from live status.

Actions
  status                      print a compact health snapshot
  reset                       clear the estop latch and authorise arm motion
  grip <raw>                  authorise gripper motion, then move r_joint
  move <json> <duration>      send one JointTrajectory point (raw ticks)
"""
import json
import math
import sys
import time

import rclpy
from rcl_interfaces.msg import Parameter as ParamMsg, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

CENTRE = {'joint1': 500, 'joint2': 500, 'joint3': 500, 'joint4': 500,
          'joint5': 500, 'r_joint': 700}
SIGN = {'joint1': 1, 'joint2': 1, 'joint3': -1, 'joint4': 1, 'joint5': 1, 'r_joint': 1}
ARM4 = ('joint1', 'joint2', 'joint3', 'joint4')


def tick_rad(name):
    return math.radians(180.0 if name == 'r_joint' else 240.0) / 1000.0


def to_rad(name, raw):
    return SIGN[name] * (raw - CENTRE[name]) * tick_rad(name)


class Driver(Node):
    def __init__(self):
        super().__init__('arm_driver')
        self.status = {}
        self.traj = self.create_publisher(JointTrajectory, '/arm_controller/command', 10)
        self.create_subscription(String, '/arm_controller/status',
                                 lambda m: self.status.update(json.loads(m.data)), 10)

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_status(self, seconds=8.0):
        end = time.monotonic() + seconds
        while time.monotonic() < end and not self.status:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.status

    def set_param(self, name, value):
        client = self.create_client(SetParameters, '/arm_controller/set_parameters')
        if not client.wait_for_service(timeout_sec=5.0):
            return {'ok': False, 'reason': 'no set_parameters service'}
        request = SetParameters.Request()
        request.parameters = [ParamMsg(name=name,
                                       value=ParameterValue(type=ParameterType.PARAMETER_BOOL,
                                                            bool_value=bool(value)))]
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=8.0)
        if not future.done() or future.result() is None:
            return {'ok': False, 'reason': 'set_parameters timeout'}
        result = future.result().results[0]
        return {'ok': result.successful, 'reason': result.reason or 'ok'}

    def report(self):
        s = self.status
        joints = s.get('joints', {})
        return {'estop': s.get('estop'), 'health': s.get('health_state'),
                'fault': s.get('fault'),
                'arm_on': s.get('enable_arm_motion'),
                'grip_on': s.get('enable_gripper_motion'),
                'gripper': s.get('gripper_feedback_state'),
                'raw': {n: joints.get(n, {}).get('raw') for n in CENTRE},
                'temp': {n: joints.get(n, {}).get('temperature') for n in CENTRE},
                'last': (s.get('last_command_result') or {}).get('reason')}


def main():
    action = sys.argv[1] if len(sys.argv) > 1 else 'status'
    rclpy.init()
    node = Driver()
    out = {'action': action}
    try:
        node.wait_status()
        if action == 'status':
            out.update(node.report())
        elif action == 'reset':
            client = node.create_client(Trigger, '/arm_controller/reset_estop')
            if not client.wait_for_service(timeout_sec=8.0):
                out['error'] = 'no reset service'
            else:
                deadline = time.monotonic() + 20.0
                while time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=0.2)
                    if node.status.get('reset_eligible') and not node.status.get('poll_pending'):
                        break
                future = client.call_async(Trigger.Request())
                rclpy.spin_until_future_complete(node, future, timeout_sec=15.0)
                response = future.result()
                out['reset'] = {'ok': getattr(response, 'success', None),
                                'message': getattr(response, 'message', None)}
                node.spin_for(1.0)
                out['arm_gate'] = node.set_param('enable_arm_motion', True)
                node.spin_for(1.0)
                out.update(node.report())
        elif action == 'grip':
            raw = int(sys.argv[2])
            out['grip_gate'] = node.set_param('enable_gripper_motion', True)
            node.spin_for(0.8)
            out['command'] = move(node, {'r_joint': raw}, 1.5)
            node.spin_for(0.6)
            out.update(node.report())
        elif action == 'move4':
            values = [int(v) for v in sys.argv[2:6]]
            targets = dict(zip(ARM4, values))
            targets['joint5'] = int(sys.argv[7]) if len(sys.argv) > 7 else 495
            duration = float(sys.argv[6]) if len(sys.argv) > 6 else 2.0
            out['command'] = move(node, targets, duration)
            node.spin_for(duration + 0.6)
            out.update(node.report())
        else:
            out['error'] = 'unknown action'
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print(json.dumps(out, sort_keys=True))


def move(node, targets, duration):
    """Publish one immediate point; the controller validates everything itself."""
    names = list(targets)
    if any(n not in CENTRE for n in names):
        return {'ok': False, 'reason': 'unknown joint name'}
    msg = JointTrajectory()
    msg.joint_names = names
    point = JointTrajectoryPoint()
    point.positions = [to_rad(n, targets[n]) for n in names]
    point.time_from_start.sec = int(duration)
    point.time_from_start.nanosec = int(round((duration - int(duration)) * 1e9))
    msg.points = [point]
    msg.header.stamp = node.get_clock().now().to_msg()
    node.traj.publish(msg)
    return {'ok': True, 'sent': targets, 'duration': duration}


if __name__ == '__main__':
    main()
