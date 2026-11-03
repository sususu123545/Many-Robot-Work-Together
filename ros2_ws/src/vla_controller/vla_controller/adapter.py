"""Turn a canonical TCP delta into servo targets this arm can actually reach.

This is the "model-specific adapter" the design document insists on, and it is
also where the platform's limits stop being abstract.

The arm is not a general 6-DOF manipulator with a Cartesian controller. It has a
vendor solver that takes ``(x, y, z, pitch)`` and returns four servo ticks; the
wrist-roll joint is outside that model and must be held still. Consequences that
a naive adapter would hide:

* roll about the tool axis cannot be commanded at all;
* yaw is not independent -- the solver picks joint1 to aim at the requested
  point, so it changes as a side effect of moving in x/y;
* one command may move a servo by at most ``max_step_ticks``, which is a *joint*
  limit, not a Cartesian one. Whether a Cartesian step fits therefore depends on
  the pose, and the only honest way to honour it is to shrink the whole step
  until it does.

Shrinking is done by a single uniform factor and the result is verified by
forward kinematics before it is returned. Clamping joints individually would
produce a target that satisfies every limit and is not on the requested path.
"""

import math

from .spec import (ACTION_DIM, ARM_PI_JOINTS, DEFAULT_MAX_STEP_TICKS,
                   POSE_JOINTS, WRIST_ROLL_JOINT, RADIANS_PER_TICK)


class AdapterError(ValueError):
    """The delta cannot be realised on this arm."""


def _load_kinematics(module):
    if module is not None:
        return module
    try:
        from armpi_ros2 import arm_kinematics
    except ImportError as error:  # pragma: no cover - depends on install layout
        raise AdapterError(
            'armpi_ros2.arm_kinematics is unavailable; it is pure Python and only '
            'needs to be on sys.path (%s)' % error)
    return arm_kinematics


class Pose:
    """TCP pose in the arm-base frame, vendor convention."""

    __slots__ = ('x', 'y', 'z', 'pitch_deg', 'yaw_deg', 'rotation')

    def __init__(self, x, y, z, pitch_deg, yaw_deg, rotation):
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)
        self.pitch_deg = float(pitch_deg)
        self.yaw_deg = float(yaw_deg)
        self.rotation = rotation

    @property
    def position(self):
        return (self.x, self.y, self.z)

    def __repr__(self):
        return ('Pose(x=%.4f, y=%.4f, z=%.4f, pitch=%.2f deg, yaw=%.2f deg)'
                % (self.x, self.y, self.z, self.pitch_deg, self.yaw_deg))


class Solution:
    """A realised step: servo targets plus the evidence that they are right."""

    __slots__ = ('targets', 'scale', 'branch', 'residual_m', 'pose', 'target',
                 'pitch_deg')

    def __init__(self, targets, scale, branch, residual_m, pose, target, pitch_deg):
        self.targets = dict(targets)
        self.scale = float(scale)
        self.branch = str(branch)
        self.residual_m = float(residual_m)
        self.pose = pose
        self.target = tuple(float(value) for value in target)
        self.pitch_deg = float(pitch_deg)

    def __repr__(self):
        return ('Solution(scale=%.3f, branch=%s, residual=%.4f mm, targets=%s)'
                % (self.scale, self.branch, self.residual_m * 1000.0,
                   self.targets))


class TcpDeltaAdapter:
    """Vendor-solver-backed realisation of the 7-component action."""

    def __init__(self, kinematics=None, max_step_ticks=DEFAULT_MAX_STEP_TICKS,
                 scales=(1.0, 0.75, 0.5, 0.375, 0.25),
                 max_residual_m=0.002,
                 max_yaw_deg_for_rotation=30.0,
                 wrist_roll_hold_tick=None):
        if not isinstance(max_step_ticks, int) or not 1 <= max_step_ticks <= 100:
            raise ValueError('max_step_ticks must be an integer within 1..100')
        ordered = tuple(sorted((float(value) for value in scales), reverse=True))
        if not ordered or ordered[0] > 1.0 or ordered[-1] <= 0.0:
            raise ValueError('scales must be descending values within (0, 1]')
        self.kinematics = _load_kinematics(kinematics)
        self.max_step_ticks = max_step_ticks
        self.scales = ordered
        self.max_residual_m = float(max_residual_m)
        self.max_yaw_deg_for_rotation = float(max_yaw_deg_for_rotation)
        self.wrist_roll_hold_tick = wrist_roll_hold_tick

    # -- state -------------------------------------------------------------

    def pose_from_raw(self, raw):
        """Servo ticks for joint1..joint4 -> Pose. Raises on out-of-range input."""
        missing = [name for name in POSE_JOINTS if name not in raw]
        if missing:
            raise AdapterError('Missing joint feedback: ' + ', '.join(missing))
        try:
            fk = self.kinematics.forward_tool_pose_raw(raw)
        except ValueError as error:
            raise AdapterError('Pose is not computable from current feedback: %s' % error)
        angles = fk['angles_deg']
        pitch_deg = angles['theta3'] + angles['theta4'] + angles['theta5']
        translation = fk['translation_m']
        return Pose(translation[0], translation[1], translation[2],
                    pitch_deg, angles['theta6'], fk['rotation_matrix'])

    def wrist_roll_target(self, raw):
        """Servo tick for joint5: held at its measured value unless configured."""
        if self.wrist_roll_hold_tick is not None:
            return int(self.wrist_roll_hold_tick)
        if WRIST_ROLL_JOINT in raw:
            return int(round(float(raw[WRIST_ROLL_JOINT])))
        return None

    # -- realisation -------------------------------------------------------

    def solve(self, raw, action):
        """Realise a 7-component action from the current joint feedback.

        Returns a :class:`Solution`, or raises :class:`AdapterError` when no
        scale in the configured ladder produces a legal step. ``action`` must
        already have passed the safety filter: the gripper component is ignored
        here, and rotation components this arm cannot realise are expected to be
        zero.
        """
        values = tuple(float(value) for value in action)
        if len(values) != ACTION_DIM:
            raise AdapterError('Action has %d components, expected %d'
                               % (len(values), ACTION_DIM))
        dx, dy, dz, drx, dry, drz = values[:6]
        if abs(dry) > 1e-9 or abs(drz) > 1e-9:
            raise AdapterError('Roll/yaw deltas reached the adapter; they are not '
                               'realisable and must be projected or rejected earlier')

        pose = self.pose_from_raw(raw)
        if abs(drx) > 1e-9 and abs(pose.yaw_deg) > self.max_yaw_deg_for_rotation:
            raise AdapterError(
                'Pitch delta at yaw %.1f deg exceeds the %.1f deg model limit; the '
                'vendor solver treats pitch as a rotation about the arm lateral '
                'axis, which is only a pitch in the world frame near zero yaw'
                % (pose.yaw_deg, self.max_yaw_deg_for_rotation))

        current = {name: float(raw[name]) for name in POSE_JOINTS}
        last_error = 'no scale produced a legal step'
        for scale in self.scales:
            target = (pose.x + scale * dx, pose.y + scale * dy, pose.z + scale * dz)
            pitch_deg = pose.pitch_deg + math.degrees(scale * drx)
            try:
                branches = self.kinematics.inverse_kinematics_all(
                    target[0], target[1], target[2], pitch_deg)
            except ValueError as error:
                if not last_error:
                    last_error = 'scale %.3f: %s' % (scale, error)
                continue

            best = None
            for branch in branches:
                targets = self._branch_targets(branch)
                step = max(abs(targets[name] - current[name]) for name in POSE_JOINTS)
                residual = self._residual(raw, targets, target)
                if step > self.max_step_ticks or residual > self.max_residual_m:
                    continue
                key = (step, residual)
                if best is None or key < best[0]:
                    best = (key, targets, branch, residual)
            if best is None:
                if not last_error:
                    last_error = ('scale %.3f: IK found %d branch(es) but none fit the '
                                  '%d-tick step / %.1f mm residual budget'
                                  % (scale, len(branches), self.max_step_ticks,
                                     self.max_residual_m * 1000.0))
                continue
            _key, targets, branch, residual = best
            return Solution(targets, scale, branch.get('elbow', '?'), residual,
                            pose, target, pitch_deg)
        raise AdapterError('No realisable step from pose %r: %s' % (pose, last_error))

    # -- internals ---------------------------------------------------------

    def _branch_targets(self, branch):
        """Vendor servo ids 3/4/5/6 -> ROS joint names joint4/joint3/joint2/joint1."""
        servo = branch['servo_targets']
        return {'joint4': int(servo[3]), 'joint3': int(servo[4]),
                'joint2': int(servo[5]), 'joint1': int(servo[6])}

    def _residual(self, raw, targets, target):
        """Forward-kinematics check of the returned targets, in metres."""
        try:
            probe = {name: float(raw[name]) for name in POSE_JOINTS}
            probe.update(targets)
            fk = self.kinematics.forward_tool_pose_raw(probe)
        except ValueError:
            return float('inf')
        translation = fk['translation_m']
        return math.sqrt(sum((a - b) ** 2 for a, b in zip(translation, target)))

    # -- reporting ---------------------------------------------------------

    def radian_targets(self, solution):
        """Solution -> {joint: radians} as carried on /joint_commands."""
        result = {}
        for name, tick in solution.targets.items():
            _, centre, sign, _, _ = ARM_PI_JOINTS[name]
            result[name] = sign * (tick - centre) * RADIANS_PER_TICK
        return result

    def tick_targets(self, solution):
        """Solution -> {joint: servo tick}, including the held wrist roll."""
        result = dict(solution.targets)
        return result

    def describe(self):
        return {'max_step_ticks': self.max_step_ticks, 'scales': list(self.scales),
                'max_residual_m': self.max_residual_m,
                'max_yaw_deg_for_rotation': self.max_yaw_deg_for_rotation,
                'wrist_roll_hold_tick': self.wrist_roll_hold_tick,
                'held_joint': WRIST_ROLL_JOINT}
