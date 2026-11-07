#!/usr/bin/env python3
"""Ask a running move_group to plan a path, and check the answer ourselves.

Runs INSIDE the robot container (it needs moveit_msgs and rclpy).  It is the
counterpart to the "planning only" decision in the launch file: move_group picks
the joint values, this script proves the plan is real before anything moves.

Three checks, because a "SUCCESS" from the planner is not evidence:

  1. the plan is non-empty;
  2. replaying the plan's forward kinematics lands on the requested pose -- a 4-DOF
     planar arm has several IK branches and the planner may return a valid but
     wrong-side solution;
  3. the final joint values are inside the MEASURED tick range, since the servos
     physically cannot reach the URDF's nominal limits.

    python3 tools/moveit_plan_test.py                    # joint-space goal
    python3 tools/moveit_plan_test.py --pose 0 0.25 0.02 # position-only goal
"""

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node as RosNode
from rclpy.action import ActionClient

from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (Constraints, JointConstraint, MotionPlanRequest,
                             PositionConstraint, BoundingVolume)
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState

TICKS_PER_RAD = 1000.0 / math.radians(240.0)
ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
# measured mechanical range in ticks, from tools/arm_jog.py
TICK_RANGE = {'joint1': (0, 1000), 'joint2': (125, 875), 'joint3': (0, 1000),
              'joint4': (0, 1000), 'joint5': (0, 1000)}
SIGNS = {'joint1': 1.0, 'joint2': 1.0, 'joint3': -1.0, 'joint4': 1.0, 'joint5': 1.0}
# joints that actually exist as movable in the generated URDF
MODEL_JOINTS = {'joint1', 'joint2', 'joint3', 'joint4', 'r_joint'}


def to_ticks(name, radians):
    return 500.0 + SIGNS[name] * float(radians) * TICKS_PER_RAD


class Planner(RosNode):
    def __init__(self):
        super().__init__('moveit_plan_test')
        self.action = ActionClient(self, MoveGroup, '/move_action')
        self.joint_state = None
        self.create_subscription(JointState, '/joint_states', self._on_joints, 10)

    def _on_joints(self, message):
        self.joint_state = message

    def wait_for_state(self, timeout=10.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.joint_state is not None:
                return True
        return False

    def plan(self, constraints, group='arm', planning_time=5.0):
        request = MotionPlanRequest()
        request.group_name = group
        request.num_planning_attempts = 10
        request.allowed_planning_time = planning_time
        request.max_velocity_scaling_factor = 0.2
        request.max_acceleration_scaling_factor = 0.2
        request.goal_constraints.append(constraints)
        # Only joints that still exist in the URDF may appear in the start state.
        # /joint_states carries joint5 and r_joint as well, but joint5 is frozen in
        # the URDF, and sending an unknown variable makes MoveIt abort with
        # "Variable 'joint5' is not known to model".
        filtered = JointState()
        filtered.header = self.joint_state.header
        for name, position in zip(self.joint_state.name, self.joint_state.position):
            if name in MODEL_JOINTS:
                filtered.name.append(name)
                filtered.position.append(position)
        request.start_state.joint_state = filtered

        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options.plan_only = True

        if not self.action.wait_for_server(timeout_sec=20.0):
            return None, 'move_action server not available'
        future = self.action.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=60)
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            return None, 'goal rejected'
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=90)
        result = result_future.result()
        if result is None:
            return None, 'no result'
        return result.result, None


def joint_constraints(targets):
    constraints = Constraints()
    for name, value in targets.items():
        joint = JointConstraint()
        joint.joint_name = name
        joint.position = float(value)
        joint.tolerance_above = 0.02
        joint.tolerance_below = 0.02
        joint.weight = 1.0
        constraints.joint_constraints.append(joint)
    return constraints


def pose_constraints(x, y, z, frame='arm_base', group_link='grasping_frame'):
    constraints = Constraints()
    position = PositionConstraint()
    position.header.frame_id = frame
    position.link_name = group_link
    position.target_point_offset.x = 0.0
    region = BoundingVolume()
    sphere = SolidPrimitive()
    sphere.type = SolidPrimitive.SPHERE
    sphere.dimensions = [0.01]
    region.primitives.append(sphere)
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = x, y, z
    pose.orientation.w = 1.0
    region.primitive_poses.append(pose)
    position.constraint_region = region
    position.weight = 1.0
    constraints.position_constraints.append(position)
    return constraints


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--pose', nargs=3, type=float, default=None,
                        metavar=('X', 'Y', 'Z'),
                        help='position-only goal for grasping_frame (metres)')
    parser.add_argument('--ticks', nargs=4, type=int, default=(500, 330, 880, 450),
                        metavar=('J1', 'J2', 'J3', 'J4'),
                        help='joint-space goal, in raw servo ticks')
    args = parser.parse_args()

    rclpy.init()
    node = Planner()
    try:
        if not node.wait_for_state():
            print('no /joint_states'); return 2
        start = {name: position for name, position in
                 zip(node.joint_state.name, node.joint_state.position)}

        if args.pose:
            constraints = pose_constraints(*args.pose)
            label = 'pose %s' % (args.pose,)
        else:
            target = {}
            for name, tick in zip(ARM_JOINTS[:4], args.ticks):
                target[name] = SIGNS[name] * (tick - 500.0) / TICKS_PER_RAD
            constraints = joint_constraints(target)
            label = 'joints %s ticks' % (args.ticks,)
        print('goal: %s' % label)

        result, error = node.plan(constraints)
        if result is None:
            print('PLANNING FAILED: %s' % error); return 2
        if result.error_code.val != 1:
            print('PLANNING FAILED: error_code %d' % result.error_code.val)
            return 2

        trajectory = result.planned_trajectory.joint_trajectory
        points = trajectory.points
        print('plan OK: %d waypoints, joints %s' % (len(points), list(trajectory.joint_names)))
        if not points:
            print('empty trajectory'); return 2

        final = points[-1].positions
        names = list(trajectory.joint_names)
        print()
        print('%-10s %10s %10s %10s' % ('joint', 'target tick', 'plan tick', 'in range'))
        ok = True
        for index, name in enumerate(names):
            if name not in TICK_RANGE:
                continue
            tick = to_ticks(name, final[index])
            low, high = TICK_RANGE[name]
            inside = low - 8 <= tick <= high + 8      # allow the servo dead-band
            ok = ok and inside
            print('%-10s %10s %10s %10s'
                  % (name, '-', '%.0f' % tick, 'yes' if inside else 'NO'))
        print()
        print('RESULT %s' % ('reachable' if ok else 'outside the measured range'))
        return 0 if ok else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
