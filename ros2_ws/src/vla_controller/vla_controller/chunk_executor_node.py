"""Execute a predicted action chunk on the arm, one gated action at a time.

This node is the ROS 2 shell around the pure-Python layers. It owns no safety
policy of its own: the health latch belongs to ``armpi_ros2``'s arm controller,
the admission rules belong to :mod:`vla_controller.safety`, and the geometry
belongs to :mod:`vla_controller.adapter`. Everything here is plumbing between
those three and the topics they speak.

Deliberate non-features, each a decision rather than an omission:

* It never grants motion permission. ``enable_arm_motion`` stays off until an
  operator turns it on through the existing console. A controller that can
  authorise itself is not a safety layer.
* It never drives the gripper unless explicitly enabled. The gripper on this arm
  has a measured mechanical fault (usable travel compressed to roughly 298..379
  ticks, no motion on the opening direction); commanding it from a policy while
  that is unresolved would burn a servo to prove a point.
* It runs at the bus cadence, not at the 20 Hz in the design document. One
  command here is a timed servo move of at least 0.3 s, so 20 Hz would have to
  discard five of every six predictions. ``control_dt`` is validated against the
  bus range, so the mismatch cannot be configured away silently.

Threading: a MultiThreadedExecutor is used on purpose. The action callback waits
for the chunk to be consumed while the control timer does the work, so a
single-threaded executor would either deadlock or stop servicing
``/safety/estop`` mid-motion. Shared state is guarded by ``_lock``.
"""

import json
import math
import threading

import rclpy
from builtin_interfaces.msg import Duration as DurationMsg
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from vla_controller import spec
from vla_controller.adapter import AdapterError, TcpDeltaAdapter
from vla_controller.chassis import ChassisGuard
from vla_controller.chunk import ChunkError, ChunkExecutor
from vla_controller.safety import Context, Limits, SafetyFilter, Workspace
from vla_interfaces.action import ExecuteActionChunk
from vla_interfaces.msg import VlaAction, VlaSafetyVerdict


def _seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _scale_ladder(min_scale, steps=5):
    """Descending uniform scales from 1.0 down to ``min_scale`` inclusive."""
    if steps < 2:
        return (1.0,)
    ratio = float(min_scale) ** (1.0 / (steps - 1))
    ladder = [1.0]
    for _ in range(steps - 1):
        ladder.append(ladder[-1] * ratio)
    ladder[-1] = float(min_scale)
    return tuple(round(value, 6) for value in ladder)


def _ticks_from_joint_state(msg):
    """JointState radians -> servo ticks. Returns ({joint: tick}, stamp).

    An out-of-range value is not clamped. Clamping would turn "the encoder is
    lying" into "the arm is where you think it is", and the joint3 latch on this
    robot is exactly that failure: a reading outside the mechanical range is the
    evidence, not noise to be tidied away.
    """
    raw = {}
    for name, radians in zip(msg.name, msg.position):
        if name not in spec.ARM_PI_JOINTS:
            continue
        if not math.isfinite(radians):
            return {}, 0.0
        _, centre, sign, low, high = spec.ARM_PI_JOINTS[name]
        tick = round(centre + sign * radians / spec.RADIANS_PER_TICK)
        if not low <= tick <= high:
            return {}, 0.0
        raw[name] = int(tick)
    return raw, _seconds(msg.header.stamp)


class ChunkExecutorNode(Node):
    def __init__(self):
        super().__init__('vla_chunk_executor')

        defaults = {
            'control_dt': 0.35,
            'execute_steps': 4,
            'max_step_m': 0.010,
            'max_step_rad': math.radians(5.0),
            'max_gripper_step': 0.35,
            'max_observation_age_s': 0.75,
            'max_chunk_age_s': 1.5,
            'min_scale': 0.25,
            'max_step_ticks': 60,
            'max_residual_m': 0.002,
            'max_yaw_deg_for_rotation': 30.0,
            'allow_projection': False,
            'enable_gripper_output': False,
            'joint_state_timeout_s': 1.5,
            'arm_status_timeout_s': 2.0,
            'workspace_x_min': -0.22, 'workspace_x_max': 0.22,
            'workspace_y_min': 0.03, 'workspace_y_max': 0.33,
            'workspace_z_min': 0.03, 'workspace_z_max': 0.34,
            'workspace_max_radius': 0.34, 'workspace_min_radius': 0.05,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        p = lambda name: self.get_parameter(name)

        limits = Limits(
            max_step_m=p('max_step_m').value,
            max_step_rad=p('max_step_rad').value,
            control_dt=p('control_dt').value,
            max_gripper_step=p('max_gripper_step').value,
            max_observation_age_s=p('max_observation_age_s').value,
            max_chunk_age_s=p('max_chunk_age_s').value,
            min_scale=p('min_scale').value,
            workspace=Workspace(
                x=(p('workspace_x_min').value, p('workspace_x_max').value),
                y=(p('workspace_y_min').value, p('workspace_y_max').value),
                z=(p('workspace_z_min').value, p('workspace_z_max').value),
                max_radius=p('workspace_max_radius').value,
                min_radius=p('workspace_min_radius').value),
        )
        self.limits = limits
        self.filter = SafetyFilter(limits, allow_projection=p('allow_projection').value)
        self.adapter = TcpDeltaAdapter(
            max_step_ticks=p('max_step_ticks').value,
            scales=_scale_ladder(limits.min_scale),
            max_residual_m=p('max_residual_m').value,
            max_yaw_deg_for_rotation=p('max_yaw_deg_for_rotation').value)
        self.chunks = ChunkExecutor(execute_steps=p('execute_steps').value)
        self.guard = ChassisGuard()
        self.gripper_output = bool(p('enable_gripper_output').value)
        if self.gripper_output:
            # Not implemented, and pretending otherwise would be worse than
            # refusing: the gripper path needs interleaving with the arm bus
            # (one move in flight at a time) plus a working gripper, and this
            # unit has neither yet. See docs/VLA架构落地-Round1.md.
            raise ValueError(
                'enable_gripper_output is not implemented on this build. The arm '
                'controller accepts one timed bus move at a time, so the gripper '
                'must be interleaved with arm steps, not sent alongside them.')
        self.joint_state_timeout = float(p('joint_state_timeout_s').value)
        self.arm_status_timeout = float(p('arm_status_timeout_s').value)

        self._lock = threading.RLock()
        self.raw = {}
        self.raw_stamp = 0.0
        self.arm_status = {}
        self.arm_status_stamp = 0.0
        self.estop_received = False
        self.last_command_accepted = None
        self.last_command_reason = ''
        self.awaiting_result = False
        self.command_failure = ''
        self.busy_until = 0.0
        self.goal_handle = None
        self.observation_stamp = 0.0
        self.executed = 0
        self.rejected = 0
        self.reject_codes = []
        self.min_scale = 1.0

        self.command_pub = self.create_publisher(JointTrajectory, '/arm_controller/command', 1)
        self.gripper_pub = self.create_publisher(JointTrajectory, '/gripper_controller/command', 1)
        self.verdict_pub = self.create_publisher(VlaSafetyVerdict, '/vla/safety_verdict', 10)
        self.status_pub = self.create_publisher(String, '/vla/executor_status', 10)

        self.create_subscription(JointState, '/joint_states', self.on_joint_state, 10)
        self.create_subscription(String, '/arm_controller/status', self.on_arm_status, 10)
        self.create_subscription(String, '/arm_controller/command_result',
                                 self.on_command_result, 10)
        self.create_subscription(Bool, '/safety/estop', self.on_estop, 10)
        # Best-effort subscriptions accept both best-effort and reliable
        # publishers, which matters because /cmd_vel comes from whatever teleop
        # happens to be running.
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, qos_profile_sensor_data)

        self._action_server = ActionServer(
            self, ExecuteActionChunk, '/vla/execute_chunk',
            execute_callback=self.on_execute,
            goal_callback=self.on_goal,
            cancel_callback=lambda _goal: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup())

        self.create_timer(0.05, self.on_tick)
        self.create_timer(1.0, self.publish_status)
        self.get_logger().info(
            'VLA chunk executor ready: %d tick budget, control_dt %.2f s, '
            'gripper output %s'
            % (self.adapter.max_step_ticks, self.limits.control_dt,
               'ENABLED' if self.gripper_output else 'disabled'))
        self.get_logger().info('Limits: %s' % json.dumps(self.filter.describe()))
        self.get_logger().info('Adapter: %s' % json.dumps(self.adapter.describe()))

    # -- inputs ------------------------------------------------------------

    def on_joint_state(self, msg):
        raw, stamp = _ticks_from_joint_state(msg)
        if not raw:
            return
        with self._lock:
            self.raw = raw
            self.raw_stamp = stamp if stamp > 0.0 else self.now()

    def on_arm_status(self, msg):
        try:
            payload = json.loads(msg.data)
        except (TypeError, ValueError):
            payload = {'parse_error': msg.data[:200]}
        with self._lock:
            self.arm_status = payload
            self.arm_status_stamp = self.now()

    def on_command_result(self, msg):
        try:
            payload = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        with self._lock:
            self.last_command_accepted = bool(payload.get('accepted'))
            self.last_command_reason = str(payload.get('reason', ''))
            if self.awaiting_result:
                self.awaiting_result = False
                if not self.last_command_accepted:
                    self.command_failure = self.last_command_reason or 'command refused'

    def on_estop(self, msg):
        if msg.data:
            with self._lock:
                self.estop_received = True

    def on_odom(self, msg):
        twist = msg.twist.twist
        self.guard.observe_odom(twist.linear.x, twist.linear.y, twist.angular.z, self.now())

    def on_cmd_vel(self, msg):
        magnitude = abs(msg.linear.x) + abs(msg.linear.y) + abs(msg.angular.z)
        self.guard.observe_cmd_vel(self.now(), magnitude)

    # -- derived state -----------------------------------------------------

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def arm_ready(self):
        """(ready, detail) derived only from what the arm controller publishes."""
        if self.estop_received:
            return False, '/safety/estop is latched'
        if not self.arm_status:
            return False, 'no /arm_controller/status yet'
        if self.now() - self.arm_status_stamp > self.arm_status_timeout:
            return False, 'arm status is stale'
        status = self.arm_status
        if status.get('dry_run'):
            return False, 'arm controller is in dry_run'
        if status.get('estop'):
            return False, 'arm safety latch is active'
        if not status.get('enable_arm_motion'):
            return False, 'arm motion permission is OFF'
        if not status.get('feedback_complete', True):
            return False, 'arm feedback is incomplete'
        return True, ''

    def pose_ready(self):
        if not self.raw:
            return False, 'no /joint_states yet'
        if self.now() - self.raw_stamp > self.joint_state_timeout:
            return False, 'joint states are stale'
        missing = [name for name in spec.POSE_JOINTS if name not in self.raw]
        if missing:
            return False, 'joint states missing ' + ', '.join(missing)
        return True, ''

    # -- action server -----------------------------------------------------

    def on_goal(self, goal_request):
        chunk = goal_request.chunk
        if not chunk.actions:
            self.get_logger().error('Rejecting goal: empty chunk')
            return GoalResponse.REJECT
        if chunk.action_frame != spec.ACTION_FRAME:
            self.get_logger().error('Rejecting goal: frame %r is not %r'
                                    % (chunk.action_frame, spec.ACTION_FRAME))
            return GoalResponse.REJECT
        if chunk.action_space_id != spec.ACTION_SPACE_ID:
            self.get_logger().error('Rejecting goal: action space %r is not %r'
                                    % (chunk.action_space_id, spec.ACTION_SPACE_ID))
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def on_execute(self, goal_handle):
        goal = goal_handle.request
        chunk = goal.chunk
        requested = len(chunk.actions)
        try:
            with self._lock:
                self.chunks.load(
                    [(a.dx, a.dy, a.dz, a.drx, a.dry, a.drz, a.gripper)
                     for a in chunk.actions],
                    control_dt=chunk.control_dt, validity_s=_seconds(chunk.validity),
                    stamp=_seconds(chunk.header.stamp), policy_id=chunk.policy_id,
                    action_space_id=chunk.action_space_id, frame=chunk.action_frame)
                self.goal_handle = goal_handle
                self.observation_stamp = _seconds(goal.observation.header.stamp)
                self.executed = 0
                self.rejected = 0
                self.reject_codes = []
                self.min_scale = 1.0
                self.command_failure = ''
        except ChunkError as error:
            goal_handle.abort()
            return self.build_result(False, 'chunk_rejected', str(error),
                                     requested, 0, 0)
        self.get_logger().info('Chunk accepted: %d actions, dt %.2f s, policy %r'
                               % (requested, chunk.control_dt, chunk.policy_id))

        rate = self.create_rate(20)
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                with self._lock:
                    self.chunks.clear()
                    self.goal_handle = None
                return self.build_result(False, 'canceled', 'client cancelled',
                                         requested, self.executed, self.rejected)
            with self._lock:
                state = self.chunks.state(self.now())
                aborted = self.chunks.aborted_reason
            if state in ('idle', 'done', 'expired', 'aborted'):
                break
            rate.sleep()

        with self._lock:
            self.goal_handle = None
            executed, rejected = self.executed, self.rejected
            expired = self.chunks.state(self.now()) == 'expired'
        goal_handle.succeed()
        if aborted:
            return self.build_result(False, 'aborted', aborted,
                                     requested, executed, rejected)
        if expired:
            return self.build_result(False, 'expired', 'chunk validity window elapsed',
                                     requested, executed, rejected)
        return self.build_result(True, 'done', '', requested, executed, rejected)

    def build_result(self, success, code, reason, requested, executed, rejected):
        response = ExecuteActionChunk.Result()
        response.success = bool(success)
        response.reason_code = code
        response.reason = reason
        response.requested = int(requested)
        response.executed = int(executed)
        response.rejected = int(rejected)
        response.min_scale_applied = float(self.min_scale)
        response.reject_codes = list(dict.fromkeys(self.reject_codes))
        return response

    # -- control loop ------------------------------------------------------

    def on_tick(self):
        with self._lock:
            if self.goal_handle is None or not self.chunks.loaded:
                return
            if self.command_failure:
                reason = self.command_failure
                self.reject_codes.append('command')
                self.rejected += 1
                self.chunks.abort('command refused: ' + reason)
                self.get_logger().error('Aborting chunk: arm refused the command: %s' % reason)
                return
            now = self.now()
            if now < self.busy_until:
                return
            action = self.chunks.next_action(now)
            if action is None:
                return
            chunk_stamp = self.chunks.stamp
            chunk_validity = self.chunks.validity_s

        ready, detail = self.arm_ready()
        pose_ready, pose_detail = self.pose_ready()
        if not pose_ready:
            ready, detail = False, pose_detail
        parked, base_detail = self.guard.parked(now)

        context = Context(
            now=now,
            observation_stamp=self.observation_stamp,
            chunk_stamp=chunk_stamp,
            chunk_validity_s=chunk_validity,
            tcp_position=self.current_tcp_position(),
            gripper_opening=self.current_gripper_opening(),
            frame=spec.ACTION_FRAME,
            action_space_id=spec.ACTION_SPACE_ID,
            arm_ready=ready, arm_detail=detail,
            base_parked=parked, base_detail=base_detail,
            latched=self.estop_received,
        )

        verdict = self.filter.check(action, context)
        self.publish_verdict(verdict)
        if not verdict.accepted:
            self.reject_codes.append(verdict.code)
            self.rejected += 1
            with self._lock:
                self.chunks.abort('%s: %s' % (verdict.code, verdict.reason))
            self.get_logger().warning('Action refused (%s): %s'
                                      % (verdict.code, verdict.reason))
            return

        try:
            solution = self.adapter.solve(self.raw, verdict.action)
        except AdapterError as error:
            self.reject_codes.append('adapter')
            self.rejected += 1
            with self._lock:
                self.chunks.abort('adapter: %s' % error)
            self.get_logger().warning('Action refused by adapter: %s' % error)
            return

        self.send_command(solution, verdict.scale)
        with self._lock:
            self.min_scale = min(self.min_scale, verdict.scale * solution.scale)
            self.chunks.commit()
            self.executed += 1
            self.busy_until = now + self.limits.control_dt
        self.publish_feedback()

    def current_tcp_position(self):
        try:
            return self.adapter.pose_from_raw(self.raw).position
        except AdapterError:
            return (0.0, 0.0, 0.0)

    def current_gripper_opening(self):
        tick = self.raw.get(spec.GRIPPER_JOINT)
        if tick is None:
            return 0.0
        return spec.gripper_tick_to_opening(tick)

    def send_command(self, solution, filter_scale):
        """Publish one timed arm move (and the gripper, only when enabled)."""
        duration = self.limits.control_dt
        radians = self.adapter.radian_targets(solution)
        names = [name for name in spec.POSE_JOINTS if name in radians]
        message = JointTrajectory()
        message.header.stamp = self.get_clock().now().to_msg()
        message.joint_names = names
        point = JointTrajectoryPoint()
        point.positions = [radians[name] for name in names]
        whole = int(duration)
        point.time_from_start = DurationMsg(
            sec=whole, nanosec=int(round((duration - whole) * 1e9)))
        message.points = [point]
        with self._lock:
            self.awaiting_result = True
        self.command_pub.publish(message)

        self.get_logger().info(
            'step %d/%d filter %.2f geom %.2f %s residual %.2f mm -> %s'
            % (self.chunks.index + 1, min(self.chunks.execute_steps, self.chunks.total),
               filter_scale, solution.scale, solution.branch,
               solution.residual_m * 1000.0, solution.targets))

    # -- publishing --------------------------------------------------------

    def publish_verdict(self, verdict):
        message = VlaSafetyVerdict()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = spec.ACTION_FRAME
        message.accepted = verdict.accepted
        message.reason_code = verdict.code
        message.reason = verdict.reason
        message.scale_applied = verdict.scale
        message.dropped_components = list(verdict.dropped)
        message.action_out = VlaAction(**spec.action_from_vector(verdict.action))
        message.latched = bool(self.estop_received)
        message.policy_id = self.chunks.policy_id
        self.verdict_pub.publish(message)

    def publish_feedback(self):
        with self._lock:
            if self.goal_handle is None:
                return
            feedback = ExecuteActionChunk.Feedback()
            feedback.index = self.chunks.index
            feedback.total = min(self.chunks.execute_steps, self.chunks.total)
            feedback.state = self.chunks.state(self.now())
            handle = self.goal_handle
        if handle is not None and handle.is_active:
            handle.publish_feedback(feedback)

    def publish_status(self):
        ready, detail = self.arm_ready()
        pose_ready, pose_detail = self.pose_ready()
        parked, base_detail = self.guard.parked(self.now())
        with self._lock:
            payload = {
                'arm_ready': ready, 'arm_detail': detail,
                'pose_ready': pose_ready, 'pose_detail': pose_detail,
                'base_parked': parked, 'base_detail': base_detail,
                'chunk_state': self.chunks.state(self.now()),
                'chunk_index': self.chunks.index,
                'executed': self.executed, 'rejected': self.rejected,
                'last_command_accepted': self.last_command_accepted,
                'last_command_reason': self.last_command_reason,
                'gripper_output': self.gripper_output,
                'estop': self.estop_received,
            }
        self.status_pub.publish(String(data=json.dumps(payload)))


def main(argv=None):
    rclpy.init(args=argv)
    node = ChunkExecutorNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
