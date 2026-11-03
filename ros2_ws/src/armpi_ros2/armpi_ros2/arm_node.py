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

ARM_JOINTS = tuple(name for name in JOINTS if name != 'r_joint')
GRIPPER_JOINT = 'r_joint'


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
        # The five arm joints form the startup homing safety group.  The
        # gripper is monitored separately so a bad gripper encoder cannot
        # suppress otherwise valid arm feedback or block the saved arm pose.
        self.health = ArmHealth(p('recovery_seconds'), self.temp_max,
                                required_names=ARM_JOINTS)
        self.gripper_read_error = None
        self.gripper_read_error_count = 0
        self.gripper_command_not_before = self.get_clock().now().nanoseconds * 1e-9
        self.busy_until = 0.0
        self.stop_generation = 0
        # A single missed servo reply must not lock the arm behind a human reset:
        # see ArmHealth.transient_fail. Only a run of this many consecutive
        # complete-cycle failures is treated as a real communication fault, and
        # an overtemperature trips immediately regardless of the count.
        self.consecutive_failures = 0
        self.failure_threshold = 3
        self.command_not_before = self.get_clock().now().nanoseconds * 1e-9
        self.state_lock = threading.RLock()
        # Commands wait for one complete poll without blocking stop/estop callbacks.
        self.feedback_cycle_lock = threading.Lock()
        self.last_command_result = None
        self.command_sequence = 0
        self.gripper_calibration = {'hardware_limits': list(JOINTS[GRIPPER_JOINT][3:]),
                                    'offset': None, 'offset_error': None}
        self.last_health_state = None
        if self.board:
            try:
                for name, (ident, _, _, low, high) in JOINTS.items():
                    if self.board.limits(ident) != [low, high]:
                        raise ValueError('Hardware limit differs from captured calibration: ' + name)
            except Exception:
                self.board.close()
                raise
            try:
                self.gripper_calibration['offset'] = self.board.offset(JOINTS[GRIPPER_JOINT][0])
            except (OSError, TimeoutError, ValueError) as error:
                # Diagnostic-only read must not suppress the five arm joints.
                self.gripper_calibration['offset_error'] = str(error)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.status_pub = self.create_publisher(String, '/arm_controller/status', 10)
        self.command_result_pub = self.create_publisher(String, '/arm_controller/command_result', 10)
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
                if parameter.value:
                    now = time.monotonic()
                    arm_ok = (not self.dry and not self.health.estop
                              and self.health.current_error is None and self.health.fresh(now))
                    gripper_ok = (arm_ok and self.gripper_read_error is None
                                  and complete_feedback(
                                      self.health.samples, now, self.temp_max,
                                      required_names=(GRIPPER_JOINT,)))
                    group_ok = arm_ok if parameter.name == 'enable_arm_motion' else gripper_ok
                    if (not group_ok or (self.board
                            and self.board.diagnostics()['receiver_error'])):
                        return SetParametersResult(
                            successful=False,
                            reason='No healthy, explicitly reset feedback for requested joint group')
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
            self.command_result(msg, False, 'Gripper expects only r_joint')
            return
        self.on_command(msg)

    def reject(self, error):
        self.get_logger().warning(str(error))

    def command_result(self, msg, accepted, reason, targets=None):
        with self.state_lock:
            self.command_sequence += 1
            result = {'sequence': self.command_sequence, 'accepted': accepted,
                      'reason': str(reason), 'joint_names': list(msg.joint_names),
                      'stamp': {'sec': msg.header.stamp.sec,
                                'nanosec': msg.header.stamp.nanosec},
                      'targets': targets or []}
            self.last_command_result = result
        self.command_result_pub.publish(String(data=json.dumps(result)))
        if not accepted:
            self.reject(reason)

    def on_command(self, msg):
        with self.state_lock:
            generation = self.stop_generation
        if not self.feedback_cycle_lock.acquire(timeout=.8):
            self.command_result(msg, False, 'Feedback cycle wait timed out; no command sent')
            return
        try:
            self._execute_command(msg, generation)
        finally:
            self.feedback_cycle_lock.release()

    def _execute_command(self, msg, generation):
        try:
            with self.state_lock:
                names = list(msg.joint_names)
                now_monotonic = time.monotonic()
                gripper_requested = GRIPPER_JOINT in names
                if generation != self.stop_generation:
                    raise ValueError('Stop occurred while command waited; no command sent')
                if self.dry or self.board is None:
                    raise ValueError('Hardware motion is disabled in dry run')
                if self.health.estop:
                    raise ValueError('Arm safety latch is active')
                if self.health.pending or not self.health.fresh(now_monotonic):
                    raise ValueError('No completed fresh five-joint feedback cycle')
                if gripper_requested and (self.gripper_read_error is not None or not complete_feedback(
                        self.health.samples, now_monotonic, self.temp_max,
                        required_names=(GRIPPER_JOINT,))):
                    raise ValueError('Gripper feedback invalid: ' + str(self.gripper_read_error))
                if self.board.diagnostics()['receiver_error']:
                    raise ValueError('Serial receiver has failed')
                if not motion_allowed(names, self.enabled, self.gripper_enabled):
                    raise ValueError('Requested joints are unknown or their motion permission is OFF')
                stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                now = self.get_clock().now().nanoseconds * 1e-9
                if stamp <= self.command_not_before or not -.1 <= now - stamp <= 1:
                    raise ValueError('Command must carry a fresh robot-clock stamp')
                if gripper_requested and stamp <= self.gripper_command_not_before:
                    raise ValueError('Gripper command predates the latest gripper feedback stop')
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
                self.command_result(msg, True, 'Serial move written; physical arrival is not yet verified', targets)
        except ValueError as error:
            self.command_result(msg, False, error)
        except (OSError, TimeoutError) as error:
            with self.state_lock:
                self.health.fail(error, time.monotonic(), 'command_io')
                self.on_stop(None)
            self.command_result(msg, False, 'Serial command failed: ' + str(error))

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
            response.message = ('Arm reset acknowledged; motion gates OFF; await NEW authorization and command'
                                if ready else 'Need 10 seconds of fresh valid five-joint arm feedback, no pending poll and a successful stop; keep emergency')
            if ready and (self.gripper_read_error is not None
                          or not complete_feedback(
                              self.health.samples, time.monotonic(), self.temp_max,
                              required_names=(GRIPPER_JOINT,))):
                response.message = ('Arm reset acknowledged; arm motion remains OFF until separately authorized; '
                                    'gripper remains unavailable because its feedback is invalid')
        return response

    def handle_feedback_error(self, error):
        """Overtemperature latches at once; a communication glitch needs a run of them."""
        now = time.monotonic()
        fatal = isinstance(error, TemperatureFault)
        with self.state_lock:
            self.consecutive_failures += 1
            if fatal or self.consecutive_failures >= self.failure_threshold:
                self.health.fail(error, now,
                                 'overtemperature' if fatal else 'communication')
                self.on_stop(None)
                self.consecutive_failures = 0
                self.get_logger().error('Arm feedback fault (latched): ' + str(error))
                return
            self.health.transient_fail(error, now)
            self.on_stop(None)
            self.get_logger().warning(
                'Transient arm feedback error (%d/%d): %s'
                % (self.consecutive_failures, self.failure_threshold, error))

    def read_joint_feedback(self, name):
        ident, _, _, low, high = JOINTS[name]
        raw = self.board.position(ident)
        position_time = time.monotonic()
        temperature = self.board.temperature(ident)
        temperature_time = time.monotonic()
        if not low <= raw <= high or not 0 <= temperature <= 100:
            raise OSError(f'Invalid physical feedback: {name}: raw={raw}, '
                          f'temperature={temperature}, allowed_raw={low}..{high}')
        if temperature >= self.temp_max:
            raise TemperatureFault(
                f'Servo {ident} is {temperature} C; temperature protection')
        return {'raw': raw, 'temperature': temperature,
                'time': min(position_time, temperature_time),
                'position_time': position_time, 'temperature_time': temperature_time}

    def handle_gripper_feedback_error(self, error):
        """Lock only the gripper when its own encoder/temperature is invalid."""
        with self.state_lock:
            self.gripper_read_error = str(error)
            self.gripper_read_error_count += 1
            self.gripper_command_not_before = self.get_clock().now().nanoseconds * 1e-9
            if self.board:
                try:
                    # A bad gripper sample must not stop the five arm joints.
                    self.board.stop([JOINTS[GRIPPER_JOINT][0]])
                except (OSError, TimeoutError, ValueError) as stop_error:
                    # If even the targeted stop cannot be sent, latch the arm
                    # safety state too; bus control is no longer trustworthy.
                    self.health.fail('Gripper stop write failed: ' + str(stop_error),
                                     time.monotonic(), 'stop_io')
                    self.on_stop(None)

    def poll(self):
        with self.feedback_cycle_lock:
            self._poll()

    def _poll(self):
        if self.board:
            with self.state_lock:
                self.health.begin_cycle()
            arm_readings = {}
            arm_error = None
            for name in ARM_JOINTS:
                try:
                    arm_readings[name] = self.read_joint_feedback(name)
                except (OSError, TimeoutError) as error:
                    arm_error = error
                    break

            gripper_reading = None
            gripper_error = None
            try:
                gripper_reading = self.read_joint_feedback(GRIPPER_JOINT)
            except (OSError, TimeoutError) as error:
                gripper_error = error

            with self.state_lock:
                if arm_error:
                    self.handle_feedback_error(arm_error)
                    self.health.samples.pop(GRIPPER_JOINT, None)
                elif not complete_feedback(arm_readings, time.monotonic(), self.temp_max,
                                           required_names=ARM_JOINTS):
                    self.handle_feedback_error(
                        OSError('Incomplete, stale or invalid five-joint arm feedback'))
                    self.health.samples.pop(GRIPPER_JOINT, None)
                else:
                    self.health.success(arm_readings, time.monotonic())
                    self.consecutive_failures = 0
                    if gripper_error is None:
                        self.health.samples[GRIPPER_JOINT] = gripper_reading
                        self.gripper_read_error = None
                    else:
                        self.health.samples.pop(GRIPPER_JOINT, None)

                if gripper_error is not None:
                    self.handle_gripper_feedback_error(gripper_error)

                now = time.monotonic()
                all_fresh = (self.health.fresh(now)
                             and complete_feedback(
                                 self.health.samples, now, self.temp_max,
                                 required_names=tuple(JOINTS)))
                if self.health.fresh(now):
                    msg = JointState()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.name = list(ARM_JOINTS) + ([GRIPPER_JOINT] if all_fresh else [])
                    msg.position = [to_radians(n, self.health.samples[n]['raw'])
                                    for n in msg.name]
                    self.joint_pub.publish(msg)

        stamp = self.get_clock().now().to_msg()
        with self.state_lock:
            now = time.monotonic()
            arm_fresh = self.health.fresh(now)
            gripper_fresh = complete_feedback(
                self.health.samples, now, self.temp_max,
                required_names=(GRIPPER_JOINT,))
            all_fresh = (arm_fresh and complete_feedback(
                self.health.samples, now, self.temp_max,
                required_names=tuple(JOINTS)))
            arm_ages = {n: round(now - self.health.samples[n]['time'], 3)
                        for n in ARM_JOINTS if n in self.health.samples}
            gripper_ages = ({GRIPPER_JOINT: round(
                now - self.health.samples[GRIPPER_JOINT]['time'], 3)}
                if GRIPPER_JOINT in self.health.samples else {})
            arm_snapshot = self.health.snapshot(now)
            status = {'dry_run': self.dry, 'enable_arm_motion': self.enabled,
                      'enable_gripper_motion': self.gripper_enabled,
                      'estop': self.health.estop, 'fault': self.health.fault,
                      'stamp': {'sec': stamp.sec, 'nanosec': stamp.nanosec},
                      **arm_snapshot,
                      # Legacy whole-system fields stay strict: these are true
                      # only when all six servos return valid physical feedback.
                      'feedback_real': not self.dry and all_fresh,
                      'feedback_complete': all_fresh,
                      'arm_stop_reasons': list(arm_snapshot['stop_reasons']),
                      'arm_feedback_real': not self.dry and arm_fresh,
                      'arm_feedback_complete': arm_fresh,
                      'arm_joints': {n: self.health.samples[n] for n in ARM_JOINTS
                                     if n in self.health.samples},
                      'arm_feedback_age_seconds': arm_ages,
                      'gripper_feedback_real': not self.dry and gripper_fresh,
                      'gripper_feedback_complete': gripper_fresh,
                      'gripper_feedback_state': ('READ_ERROR' if self.gripper_read_error
                          else 'READY' if gripper_fresh else 'NO_FRESH_FEEDBACK'),
                      'gripper_read_error': self.gripper_read_error,
                      'gripper_calibration': self.gripper_calibration,
                      'gripper_fault': self.gripper_read_error,
                      'gripper_read_error_count': self.gripper_read_error_count,
                      'gripper_feedback_age_seconds': gripper_ages,
                      'gripper_motion_ready': (arm_fresh and gripper_fresh
                                               and not self.health.estop
                                               and not self.gripper_read_error),
                      'max_step_ticks': self.step, 'joints': self.health.samples,
                      'last_command_result': self.last_command_result,
                      'feedback_age_seconds': {k: round(now-v['time'], 3)
                                               for k, v in self.health.samples.items()},
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
