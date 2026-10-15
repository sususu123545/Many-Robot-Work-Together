"""Native ROS2 arm/gripper, true feedback, no automatic initial pose."""
import json
import math
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Bool, Empty, String
from std_srvs.srv import Trigger
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.parameter import Parameter

from .servo_protocol import JOINTS, ServoBoard, to_radians, validated_targets
from .arm_health import ArmHealth, motion_allowed, complete_feedback


class TemperatureFault(OSError):
    pass


class ArmNode(Node):
    def __init__(self, **kwargs):
        super().__init__('arm_controller', **kwargs)
        for name, value in {'dry_run': True, 'enable_arm_motion': False,
                            'enable_gripper_motion': False, 'recovery_seconds': 10.0,
                            'serial_port': '/dev/rrc', 'baudrate': 1000000,
                            'max_step_ticks': 100, 'max_temperature': 65}.items():
            self.declare_parameter(name, value, ParameterDescriptor(
                read_only=name not in ('enable_arm_motion', 'enable_gripper_motion')))
        p = lambda k: self.get_parameter(k).value
        self.dry = p('dry_run')
        self.enabled = p('enable_arm_motion')
        self.gripper_enabled = p('enable_gripper_motion')
        self.step, self.temp_max = p('max_step_ticks'), p('max_temperature')
        if not 1 <= self.step <= 100 or not 1 <= self.temp_max <= 65:
            raise ValueError('Invalid arm safety bounds')
        self.board = None if self.dry else ServoBoard(p('serial_port'), p('baudrate'))
        self.health = ArmHealth(p('recovery_seconds'), self.temp_max)
        self.busy_until = 0.0
        self.stop_generation = 0
        self.command_not_before = self.get_clock().now().nanoseconds * 1e-9
        self.state_lock = threading.RLock()
        self.last_health_state = None
        if self.board:
            try:
                for name, (ident, _, _, low, high) in JOINTS.items():
                    if self.board.limits(ident) != [low, high]:
                        raise ValueError('Hardware limit differs from captured calibration: ' + name)
            except Exception:
                self.board.close()
                raise
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.status_pub = self.create_publisher(String, '/arm_controller/status', 10)
        commands = MutuallyExclusiveCallbackGroup()
        polling = MutuallyExclusiveCallbackGroup()
        self.create_subscription(JointTrajectory, '/arm_controller/command', self.on_command, 1, callback_group=commands)
        self.create_subscription(JointTrajectory, '/gripper_controller/command', self.on_gripper, 1, callback_group=commands)
        self.create_subscription(Empty, '/arm_controller/stop', self.on_stop, 1)
        self.create_subscription(Bool, '/safety/estop', self.on_estop, 1)
        self.create_service(Trigger, '/arm_controller/reset_estop', self.reset)
        self.create_timer(.5, self.poll, callback_group=polling)
        self.add_on_set_parameters_callback(self.validate_permissions)
        self.add_post_set_parameters_callback(self.apply_permissions)
        self.get_logger().info('True feedback; no startup pose/torque/offset commands')

    def validate_permissions(self, parameters):
        with self.state_lock:
            for parameter in parameters:
                if parameter.name not in ('enable_arm_motion', 'enable_gripper_motion'):
                    return SetParametersResult(successful=False, reason='Safety configuration is read-only')
                if parameter.type_ != Parameter.Type.BOOL:
                    return SetParametersResult(successful=False, reason='Motion permission must be bool')
                if parameter.value and (self.dry or self.health.estop or self.health.pending
                                         or not self.health.fresh(time.monotonic())
                                         or (self.board and self.board.diagnostics()['receiver_error'])):
                    return SetParametersResult(successful=False, reason='No healthy, explicitly reset physical feedback')
        return SetParametersResult(successful=True)

    def apply_permissions(self, _parameters):
        # This callback runs only AFTER the parameter update has been accepted.
        with self.state_lock:
            arm = self.get_parameter('enable_arm_motion').value
            gripper = self.get_parameter('enable_gripper_motion').value
            disabling = (self.enabled and not arm) or (self.gripper_enabled and not gripper)
            self.enabled, self.gripper_enabled = arm, gripper
            if disabling:
                self.on_stop(None)

    def on_gripper(self, msg):
        if list(msg.joint_names) != ['r_joint']:
            self.reject('Gripper expects only r_joint')
            return
        self.on_command(msg)

    def reject(self, error):
        self.get_logger().warning(str(error))

    def on_command(self, msg):
        try:
            with self.state_lock:
                generation = self.stop_generation
                names = list(msg.joint_names)
                if (self.dry or self.health.estop or not self.health.fresh(time.monotonic())
                        or (self.board and self.board.diagnostics()['receiver_error'])
                        or not motion_allowed(names, self.enabled, self.gripper_enabled)):
                    raise ValueError('Arm motion is disabled, faulted or emergency-latched')
                stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                now = self.get_clock().now().nanoseconds * 1e-9
                if stamp <= self.command_not_before or not -.1 <= now - stamp <= 1:
                    raise ValueError('Command must carry a fresh robot-clock stamp')
                if msg.header.frame_id or len(msg.points) != 1:
                    raise ValueError('Only one immediate point without frame override is supported')
                point = msg.points[0]
                if not names or len(set(names)) != len(names) or len(point.positions) != len(names):
                    raise ValueError('Invalid joint/position dimensions')
                if point.velocities or point.accelerations or point.effort:
                    raise ValueError('Velocity/acceleration/effort trajectories are not implemented')
                duration = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
                if not .3 <= duration <= 5 or time.monotonic() < self.busy_until:
                    raise ValueError('Invalid duration or preceding point still executing')
                targets = validated_targets(names, point.positions, self.health.samples,
                                            time.monotonic(), self.step, self.temp_max)
                # Check emergency again under the serial transaction lock.
                with self.board.lock:
                    if self.health.estop or generation != self.stop_generation:
                        raise ValueError('Emergency occurred during validation')
                    self.board.move(duration, targets)
                self.busy_until = time.monotonic() + duration
        except ValueError as error:
            self.reject(error)
        except (OSError, TimeoutError) as error:
            with self.state_lock:
                self.health.fail(error, time.monotonic(), 'command_io')
                self.on_stop(None)

    def on_stop(self, _msg):
        with self.state_lock:
            self.stop_generation += 1
            self.command_not_before = self.get_clock().now().nanoseconds * 1e-9
            if self.board:
                try:
                    self.board.stop()
                except (OSError, TimeoutError) as error:
                    self.health.fail(error, time.monotonic(), 'stop_io')
                    self.get_logger().error('Stop write failed: ' + str(error))
            self.busy_until = 0.0

    def on_estop(self, msg):
        if msg.data:
            with self.state_lock:
                self.health.manual_stop()
                self.on_stop(None)

    def reset(self, _request, response):
        with self.state_lock:
            now = time.monotonic()
            ready = (not self.dry and self.board is not None and self.health.can_reset(now)
                     and now >= self.busy_until and not self.board.diagnostics()['receiver_error'])
            generation = self.health.generation
            if ready:
                # Reset NEVER grants movement. Revoke both gates and invalidate old messages.
                result = self.set_parameters_atomically([
                    Parameter('enable_arm_motion', value=False),
                    Parameter('enable_gripper_motion', value=False)])
                ready = result.successful
            if ready:
                self.on_stop(None)
                ready = self.health.acknowledge(time.monotonic(), generation,
                                               stop_succeeded=not self.board.diagnostics()['receiver_error'])
            response.success = ready
            response.message = ('Reset acknowledged; motion gates OFF; await NEW authorization and command'
                                if ready else 'Need 10 seconds of complete healthy feedback, no pending poll and a successful stop; keep emergency')
        return response

    def poll(self):
        readings = {}
        if self.board:
            with self.state_lock:
                self.health.begin_cycle()
            try:
                for name, (ident, _, _, low, high) in JOINTS.items():
                    raw = self.board.position(ident)
                    position_time = time.monotonic()
                    temperature = self.board.temperature(ident)
                    temperature_time = time.monotonic()
                    if not low <= raw <= high or not 0 <= temperature <= 100:
                        raise OSError('Invalid physical feedback: ' + name)
                    if temperature >= self.temp_max:
                        raise TemperatureFault(f'Servo {ident} is {temperature} C; temperature protection')
                    readings[name] = {'raw': raw, 'temperature': temperature,
                                      'time': min(position_time, temperature_time),
                                      'position_time': position_time, 'temperature_time': temperature_time}
                with self.state_lock:
                    if not complete_feedback(readings, time.monotonic(), self.temp_max):
                        raise OSError('Incomplete, stale or invalid whole-arm feedback')
                    self.health.success(readings, time.monotonic())
                msg = JointState()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.name = list(readings)
                msg.position = [to_radians(n, readings[n]['raw']) for n in msg.name]
                self.joint_pub.publish(msg)
            except (OSError, TimeoutError) as error:
                with self.state_lock:
                    self.health.fail(error, time.monotonic(),
                                     'overtemperature' if isinstance(error, TemperatureFault) else 'communication')
                    self.on_stop(None)
                self.get_logger().error('Arm feedback fault (latched): ' + str(error))
        stamp = self.get_clock().now().to_msg()
        with self.state_lock:
            now = time.monotonic()
            status = {'dry_run': self.dry, 'enable_arm_motion': self.enabled,
                      'enable_gripper_motion': self.gripper_enabled,
                      'estop': self.health.estop, 'fault': self.health.fault,
                      'stamp': {'sec': stamp.sec, 'nanosec': stamp.nanosec},
                      'feedback_real': not self.dry and self.health.fresh(now),
                      'max_step_ticks': self.step, 'joints': self.health.samples,
                      **self.health.snapshot(now),
                      'feedback_age_seconds': {k: round(now-v['time'], 3) for k, v in self.health.samples.items()},
                      'transport': self.board.diagnostics() if self.board else None}
            if status['health_state'] != self.last_health_state:
                self.last_health_state = status['health_state']
                self.get_logger().info('Arm health: ' + self.last_health_state)
        self.status_pub.publish(String(data=json.dumps(status)))


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = MultiThreadedExecutor(num_threads=3)
    try:
        node = ArmNode()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        if node:
            # No torque unload or forced pose, even when shutting down.
            node.on_stop(None)
            if node.board:
                node.board.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
