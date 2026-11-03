"""Canonical action and joint specification for the VLA control path.

Every number here is either a physical measurement from this robot or a vendor
constant that another module in this repository already relies on. Nothing is
imported from a paper, and nothing is guessed: a wrong value in this file turns
into a real arm moving to a real place.

The joint table is deliberately duplicated from ``armpi_ros2.servo_protocol`` so
that this package can be imported, unit-tested and reasoned about on a machine
without ROS 2 installed. ``test/test_spec.py`` asserts the two tables agree
whenever ``armpi_ros2`` happens to be importable, so the copy cannot drift
silently.
"""

import math

# --------------------------------------------------------------------------
# Action space
# --------------------------------------------------------------------------

#: Order of the seven components, fixed for the whole project. Changing this is
#: a breaking change to every recorded dataset.
ACTION_COMPONENTS = ('dx', 'dy', 'dz', 'drx', 'dry', 'drz', 'gripper')
ACTION_DIM = len(ACTION_COMPONENTS)

#: Translation is metres, rotation is a radian-scaled rotation vector,
#: gripper is a dimensionless opening in [0, 1].
POSITION_UNIT = 'm'
ROTATION_UNIT = 'rad'

#: Frame the deltas are expressed in. The ArmPi Pro publishes no static
#: transform from ``base_link`` to ``arm_base``, so anything that says
#: "arm_base_link" is talking about a frame that does not exist on this robot.
ACTION_FRAME = 'arm_base'

#: Version tag written into every chunk and every normalisation file. Bump it
#: whenever the meaning of a component changes, not when a model changes.
ACTION_SPACE_ID = 'armpi_pro_tcp_delta_v1'

#: Which physical configuration produced an observation.
ROBOT_CONFIG_ID = 'armpi_pro_v1'

#: Components this arm can actually realise.
#:
#: The vendor solver is a 3-link planar reach plus a base yaw: it takes
#: (x, y, z, pitch). Pitch is a rotation about the lateral x axis, so ``drx`` is
#: realisable. Roll about the tool axis lives on joint5, which the pose model
#: does not include and which must therefore be held fixed, so ``dry`` is not.
#: Yaw is not an independent degree of freedom either: joint1 is chosen by the
#: solver to aim at the requested point, so ``drz`` cannot be commanded without
#: also moving in x/y.
#:
#: A consumer that silently ignores these two components produces a policy that
#: looks like it is learning rotations and is in fact learning nothing. Hence
#: the explicit mask and the explicit reject.
REALIZABLE_MASK = (1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 1.0)

#: Names of the components that are not realisable, for verdict reporting.
NON_REALIZABLE_COMPONENTS = tuple(
    name for name, flag in zip(ACTION_COMPONENTS, REALIZABLE_MASK) if flag == 0.0)


# --------------------------------------------------------------------------
# Joints
# --------------------------------------------------------------------------

#: name -> (servo id, centre tick, sign, low tick, high tick).
#: Copied from armpi_ros2.servo_protocol.JOINTS; see the module docstring.
ARM_PI_JOINTS = {
    'joint1': (6, 500, 1, 0, 1000),
    'joint2': (5, 500, 1, 125, 875),
    'joint3': (4, 500, -1, 0, 1000),
    'joint4': (3, 500, 1, 0, 1000),
    'joint5': (2, 500, 1, 0, 1000),
    'r_joint': (1, 700, 1, 0, 700),
}

#: Joints the vendor pose model consumes. joint5 is excluded on purpose.
POSE_JOINTS = ('joint1', 'joint2', 'joint3', 'joint4')

#: The wrist-roll joint, held fixed while the pose model is in use.
WRIST_ROLL_JOINT = 'joint5'

GRIPPER_JOINT = 'r_joint'

#: One servo tick is 240/1000 degrees.
TICKS_PER_DEGREE = 1000.0 / 240.0
RADIANS_PER_TICK = math.radians(1.0 / TICKS_PER_DEGREE)

#: Servo-tick travel the arm node accepts in a single command.
#: Mirrors the ``max_step_ticks`` parameter of armpi_ros2's arm_controller.
DEFAULT_MAX_STEP_TICKS = 100

#: A single bus command must declare this much time, and no more.
#: Mirrors ``movement_payload`` in armpi_ros2.servo_protocol.
MIN_MOVE_DURATION_S = 0.3
MAX_MOVE_DURATION_S = 5.0


def tick_to_radians(name, raw):
    """Servo tick -> the radian value carried on /joint_states."""
    _, centre, sign, _, _ = ARM_PI_JOINTS[name]
    return sign * (raw - centre) * RADIANS_PER_TICK


def radians_to_tick(name, radians):
    """Radian /joint_states value -> servo tick, range checked."""
    if name not in ARM_PI_JOINTS:
        raise ValueError('Unknown joint: ' + str(name))
    if not math.isfinite(radians):
        raise ValueError('Non-finite angle for ' + name)
    ident, centre, sign, low, high = ARM_PI_JOINTS[name]
    raw = round(centre + sign * radians / RADIANS_PER_TICK)
    if not low <= raw <= high:
        raise ValueError('Target exceeds hardware limits for %s: %d not in %d..%d'
                         % (name, raw, low, high))
    return ident, raw


def joint_limits(name):
    """Inclusive servo-tick limits for a joint."""
    _, _, _, low, high = ARM_PI_JOINTS[name]
    return low, high


# --------------------------------------------------------------------------
# Gripper
# --------------------------------------------------------------------------

#: Measured on this robot: fully open is ~85 ticks, jaws meeting is ~482.
#: Recorded in tools/gripper_guarded.py and confirmed on the bench. Smaller
#: ticks mean wider jaws; 636 was an old software clamp that jammed the jaws.
GRIPPER_TICK_OPEN = 85
GRIPPER_TICK_CLOSED = 482


def gripper_tick_to_opening(raw):
    """Servo tick -> normalised opening in [0, 1]; 1 is fully open."""
    if not math.isfinite(raw):
        raise ValueError('Non-finite gripper position')
    span = GRIPPER_TICK_CLOSED - GRIPPER_TICK_OPEN
    return min(1.0, max(0.0, (GRIPPER_TICK_CLOSED - float(raw)) / span))


def gripper_opening_to_tick(opening):
    """Normalised opening in [0, 1] -> servo tick, range checked."""
    if not math.isfinite(opening) or not 0.0 <= opening <= 1.0:
        raise ValueError('Gripper opening must be within [0, 1]')
    span = GRIPPER_TICK_CLOSED - GRIPPER_TICK_OPEN
    return round(GRIPPER_TICK_CLOSED - float(opening) * span)


# --------------------------------------------------------------------------
# Vector <-> message helpers
# --------------------------------------------------------------------------

def action_to_vector(action):
    """Any object with the seven named attributes -> a tuple in canonical order."""
    return tuple(float(getattr(action, name)) for name in ACTION_COMPONENTS)


def action_from_vector(vector):
    """Canonical 7-tuple -> ``{component: value}`` mapping.

    Callers that build a ``VlaAction`` message do ``VlaAction(**mapping)`` so the
    field order can never drift from ``ACTION_COMPONENTS``.
    """
    if len(vector) != ACTION_DIM:
        raise ValueError('Expected %d components, got %d'
                         % (ACTION_DIM, len(vector)))
    values = tuple(float(value) for value in vector)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('Action contains non-finite components')
    return dict(zip(ACTION_COMPONENTS, values))


def zero_action(gripper_opening=0.0):
    """A hold-still action.

    Translation and rotation are zero. The gripper component is an absolute
    target, not a delta, so "hold still" means "keep the current opening", which
    the caller has to supply.
    """
    if not math.isfinite(gripper_opening) or not 0.0 <= gripper_opening <= 1.0:
        raise ValueError('Gripper opening must be within [0, 1]')
    return tuple(0.0 for _ in ACTION_COMPONENTS[:-1]) + (float(gripper_opening),)


# --------------------------------------------------------------------------
# Chassis (phase I: parked)
# --------------------------------------------------------------------------

#: The chassis in use is the existing four-wheel mecanum base, not the tracked
#: platform in the design document. It speaks REP-103 through /cmd_vel:
#: x forward, y left, z counter-clockwise, from armpi_ros2's chassis_node.
#:
#: Phase I of the design keeps the base still while the arm works, because the
#: action frame is ``arm_base`` and the vendor pose model knows nothing about
#: where the base is. Any base motion during chunk execution therefore turns
#: every predicted delta into a lie, which is why ``ChassisGuard`` below treats
#: a moving base as a hard reject rather than a warning.
CHASSIS_CONFIG_ID = 'armpi_pro_mecanum_v1'
CHASSIS_CMD_TOPIC = '/cmd_vel'
CHASSIS_ODOM_TOPIC = '/odom'

#: Wheel geometry measured on this base (2026-10-10), mirrored from
#: armpi_ros2.motion.Geometry. Only the derived quantity is used here.
WHEEL_SEPARATION_M = 0.110 + 0.0975

#: Envelope already enforced by armpi_ros2.motion.MotionLease. Repeated rather
#: than imported so this package stays importable without ROS 2.
CHASSIS_MAX_LINEAR_MPS = 0.08
CHASSIS_MAX_ANGULAR_RPS = 0.20

#: A base is "parked" when its measured motion stays under these for the whole
#: settling window. Values sit below the mecanum base's noise floor: the wheels
#: slip on smooth floor, so a quiet reading is the only evidence available.
PARKED_LINEAR_MPS = 0.004
PARKED_ANGULAR_RPS = 0.012

#: How long the base must have been quiet, and how long since the last /cmd_vel,
#: before an arm action may execute. A fresh command means a live navigator that
#: has not yet produced visible motion.
PARKED_HOLD_S = 0.6
CMD_VEL_QUIET_S = 0.5
