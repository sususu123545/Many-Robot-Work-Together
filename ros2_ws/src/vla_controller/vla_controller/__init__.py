"""VLA action control for the ArmPi Pro arm.

Two layers, deliberately kept apart:

* :mod:`vla_controller.spec`, :mod:`normalization`, :mod:`safety`, :mod:`chunk`,
  :mod:`chassis`, :mod:`adapter` are pure Python. No ``rclpy``, no ROS types, no
  robot required. That is what makes them testable on a workstation and
  reviewable without a timeline.
* :mod:`vla_controller.chunk_executor_node` is the thin ROS 2 shell that feeds
  the layers above from real topics and writes the resulting command.
"""

from .adapter import AdapterError, Pose, Solution, TcpDeltaAdapter
from .chassis import ChassisGuard
from .chunk import ChunkError, ChunkExecutor
from .normalization import NormalizationError, NormalizationStats
from .safety import Code, Context, Limits, SafetyFilter, Verdict, Workspace
from .spec import (ACTION_COMPONENTS, ACTION_DIM, ACTION_FRAME, ACTION_SPACE_ID,
                   REALIZABLE_MASK, ROBOT_CONFIG_ID, action_from_vector,
                   action_to_vector, gripper_opening_to_tick,
                   gripper_tick_to_opening, zero_action)

__all__ = [
    'ACTION_COMPONENTS', 'ACTION_DIM', 'ACTION_FRAME', 'ACTION_SPACE_ID',
    'REALIZABLE_MASK', 'ROBOT_CONFIG_ID',
    'AdapterError', 'ChassisGuard', 'ChunkError', 'ChunkExecutor', 'Code',
    'Context', 'Limits', 'NormalizationError', 'NormalizationStats', 'Pose',
    'SafetyFilter', 'Solution', 'TcpDeltaAdapter', 'Verdict', 'Workspace',
    'action_from_vector', 'action_to_vector', 'gripper_opening_to_tick',
    'gripper_tick_to_opening', 'zero_action',
]
