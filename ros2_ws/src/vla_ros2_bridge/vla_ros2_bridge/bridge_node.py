"""Close the VLA loop: observation -> policy -> ``/vla/execute_chunk`` goal.

This is the client that did not exist.  ``vla_controller`` starts an
ActionServer on ``/vla/execute_chunk``; without this node no goal is ever sent,
so a natural-language instruction cannot move the robot at all.

Safety posture, matching the rest of this project:

* ``one_shot`` defaults to **true**.  After one chunk completes the node stops
  sending goals.  An autonomous loop that keeps re-planning while the arm is
  unattended is not something to enable by default on a machine with no
  collision sensing.
* motion permission is never granted here; the executor requires the operator
  to enable it from the console.
* a remote (normalised-output) policy without a statistics file is refused at
  startup, not at the moment a goal would otherwise be sent.

Everything that decides anything lives in the pure-Python modules.  This node
only converts messages and reports results.
"""

import math

from builtin_interfaces.msg import Duration, Time
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Header
from vla_interfaces.action import ExecuteActionChunk
from vla_interfaces.msg import VlaAction, VlaActionChunk, VlaObservation

from vla_controller import spec
from vla_controller.normalization import NormalizationError, NormalizationStats
from vla_controller.safety import Limits
from vla_ros2_bridge.bridge import BridgeError, BridgePlanner
from vla_ros2_bridge.observation import seconds
from vla_ros2_bridge.policy import PolicyError, RemotePolicy, ScriptedPolicy


def time_from_seconds(value):
    value = float(value)
    whole = math.floor(value)
    return Time(sec=int(whole), nanosec=int(round((value - whole) * 1e9)))


def duration_from_seconds(value):
    value = float(value)
    whole = math.floor(value)
    return Duration(sec=int(whole), nanosec=int(round((value - whole) * 1e9)))


def observation_to_dict(message):
    """VlaObservation -> the plain mapping the pure-Python layer consumes."""
    return {
        'header': {'stamp': seconds(message.header.stamp),
                   'frame_id': message.header.frame_id},
        'instruction': str(message.instruction),
        'robot_config_id': str(message.robot_config_id),
        'tcp_position_m': (float(message.tcp_position_m.x),
                           float(message.tcp_position_m.y),
                           float(message.tcp_position_m.z)),
        'tcp_orientation': (float(message.tcp_orientation.x),
                            float(message.tcp_orientation.y),
                            float(message.tcp_orientation.z),
                            float(message.tcp_orientation.w)),
        'joint_names': list(message.joint_names),
        'joint_positions_rad': [float(v) for v in message.joint_positions_rad],
        'gripper_opening': float(message.gripper_opening),
        'wrist_image_stamp': seconds(message.wrist_image_stamp),
        'wrist_image': {'format': str(message.wrist_image.format),
                        'data': bytes(message.wrist_image.data)},
        'front_image_stamp': seconds(message.front_image_stamp),
        'front_image': {'format': str(message.front_image.format),
                        'data': bytes(message.front_image.data)},
    }


class BridgeNode(Node):
    """One observation in, one action goal out (unless one_shot is disabled)."""

    def __init__(self):
        super().__init__('vla_bridge_node')
        self.declare_parameter('observation_topic', '/vla/observation')
        self.declare_parameter('action_name', '/vla/execute_chunk')
        self.declare_parameter('policy_kind', 'scripted')
        self.declare_parameter('endpoint', 'http://127.0.0.1:8080/vla')
        self.declare_parameter('timeout_s', 2.0)
        self.declare_parameter('stats_file', '')
        self.declare_parameter('stats_file_required_for_remote', True)
        self.declare_parameter('goal_position', [0.0, 0.20, 0.12])
        self.declare_parameter('step_m', 0.010)
        self.declare_parameter('horizon', 16)
        self.declare_parameter('control_dt', 0.35)
        self.declare_parameter('validity_s', 1.0)
        self.declare_parameter('execute_steps', 4)
        self.declare_parameter('max_observation_age_s', 0.75)
        self.declare_parameter('max_chunk_age_s', 1.5)
        self.declare_parameter('allow_projection', False)
        self.declare_parameter('one_shot', True)
        self.declare_parameter('wait_for_server_s', 5.0)

        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.one_shot = bool(p('one_shot'))
        self.busy = False
        self.finished = False
        self.goals_sent = 0

        self.stats = self._load_stats(str(p('stats_file') or ''))
        self.policy = self._make_policy(p)
        self.limits = Limits(
            control_dt=float(p('control_dt')),
            max_observation_age_s=float(p('max_observation_age_s')),
            max_chunk_age_s=float(p('max_chunk_age_s')))
        if self.policy_kind == 'remote' and self.stats is None \
                and bool(p('stats_file_required_for_remote')):
            raise RuntimeError(
                'remote policies emit normalised actions; set stats_file to the '
                'statistics of the dataset that trained the model. Refusing to '
                'guess a scale.')
        self.planner = BridgePlanner(
            limits=self.limits, stats=self.stats,
            execute_steps=int(p('execute_steps')),
            allow_projection=bool(p('allow_projection')))

        self.client = ActionClient(self, ExecuteActionChunk, str(p('action_name')))
        self.server_ready = self.client.wait_for_server(
            timeout_sec=float(p('wait_for_server_s')))
        if not self.server_ready:
            self.get_logger().warn(
                'Executor action server not up at startup; will retry on each '
                'observation (no goal is sent until it appears)')
        self.create_subscription(VlaObservation, str(p('observation_topic')),
                                 self.on_observation, 10)
        self.get_logger().info(
            'VLA bridge ready: policy=%s one_shot=%s stats=%r'
            % (self.policy_kind, self.one_shot, repr(self.stats)))

    # -- construction ------------------------------------------------------

    def _load_stats(self, path):
        if not path:
            return None
        try:
            stats = NormalizationStats.load(path)
            stats.check_space(spec.ACTION_SPACE_ID)
            return stats
        except (OSError, ValueError, NormalizationError) as error:
            raise RuntimeError('Cannot use normalisation stats %r: %s'
                               % (path, error))

    def _make_policy(self, p):
        self.policy_kind = str(p('policy_kind'))
        if self.policy_kind == 'scripted':
            return ScriptedPolicy(
                goal_position=tuple(float(v) for v in p('goal_position')),
                step_m=float(p('step_m')), horizon=int(p('horizon')),
                control_dt=float(p('control_dt')),
                validity_s=float(p('validity_s')))
        if self.policy_kind == 'remote':
            return RemotePolicy(str(p('endpoint')),
                                timeout_s=float(p('timeout_s')))
        raise ValueError('Unknown policy_kind %r (use scripted or remote)'
                         % self.policy_kind)

    # -- main path ---------------------------------------------------------

    def on_observation(self, message):
        if self.busy or self.finished:
            return
        if not self.server_ready:
            # A zero timeout turns this into a pure "is it there yet" check, so
            # a stream of observations never blocks the executor thread.
            self.server_ready = self.client.wait_for_server(timeout_sec=0.0)
        if not self.server_ready:
            self.get_logger().warn('Executor action server is not up yet')
            return
        observation = observation_to_dict(message)
        now = self.get_clock().now().nanoseconds / 1e9
        try:
            output = self.policy.predict(observation)
            goal_spec = self.planner.plan(observation, output, now)
        except (PolicyError, BridgeError) as error:
            self.get_logger().warn('Not sending a goal: %s' % error)
            return
        goal = self.to_goal(goal_spec, observation, message)
        self.busy = True
        self.goals_sent += 1
        self.get_logger().info(
            'Sending goal %d: %d actions from %s'
            % (self.goals_sent, goal_spec.action_count, output.policy_id))
        future = self.client.send_goal_async(
            goal, feedback_callback=self.on_feedback)
        future.add_done_callback(self.on_goal_response)

    def to_goal(self, spec_, _observation, original_message):
        """ChunkSpec + the original VlaObservation -> ExecuteActionChunk.Goal."""
        chunk_data = spec_.chunk
        header = Header()
        header.stamp = time_from_seconds(chunk_data['header']['stamp'])
        header.frame_id = chunk_data['header']['frame_id']

        chunk = VlaActionChunk()
        chunk.header = header
        chunk.policy_id = str(chunk_data['policy_id'])
        chunk.action_space_id = str(chunk_data['action_space_id'])
        chunk.action_frame = str(chunk_data['action_frame'])
        chunk.control_dt = float(chunk_data['control_dt'])
        chunk.validity = duration_from_seconds(chunk_data['validity'])
        chunk.actions = [VlaAction(**{key: float(value) for key, value
                                      in action.items()})
                         for action in chunk_data['actions']]
        chunk.realizable_mask = [float(v) for v in chunk_data['realizable_mask']]

        goal = ExecuteActionChunk.Goal()
        goal.goal_id = '%s-%d' % (chunk.policy_id, self.goals_sent)
        # Forward the ORIGINAL message, not a re-serialised dictionary, so the
        # executor sees exactly the observation the policy was shown.
        goal.observation = original_message
        goal.chunk = chunk
        return goal

    # -- results -----------------------------------------------------------

    def on_feedback(self, feedback_message):
        feedback = feedback_message.feedback
        self.get_logger().info('feedback %d/%d state=%s'
                               % (feedback.index, feedback.total, feedback.state))

    def on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Goal rejected by executor')
            self.busy = False
            return
        result_future = handle.get_result_async()
        result_future.add_done_callback(self.on_result)

    def on_result(self, future):
        result = future.result().result
        self.get_logger().info(
            'Chunk result: success=%s code=%s requested=%d executed=%d '
            'rejected=%d min_scale=%.2f'
            % (result.success, result.reason_code, result.requested,
               result.executed, result.rejected, result.min_scale_applied))
        self.busy = False
        if self.one_shot:
            self.finished = True
            self.get_logger().info(
                'one_shot is true; no further goals will be sent. Restart the '
                'node (or set one_shot:=false) to continue.')


def main(args=None):
    import rclpy

    rclpy.init(args=args)
    node = BridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
