"""Build a ``VlaObservation`` from heterogeneous robot feedback, without ROS 2.

A VLA observation is not "whatever the node happens to have cached".  It is a
claim that *at one instant* the joints were here, the tool was there and the
wrist camera saw this.  Mixing a fresh image with joints from half a second ago
produces an observation that is internally consistent and physically false --
and a policy trained on such pairs learns a timing artefact instead of a task.

So this module is mostly checks:

* every required joint is present, finite and inside its measured limits;
* the joint frame and the wrist image are within ``max_skew_s`` of each other;
* the instruction is carried verbatim (no templating, no rewriting);
* a missing scene camera is represented as an EMPTY image, which the message
  contract distinguishes from a camera that died;
* the TCP pose comes from the same joint ticks, via forward kinematics, never
  from a separate cached pose.

The output is a plain dictionary whose keys are exactly the fields of
``vla_interfaces/msg/VlaObservation.msg``.  The ROS 2 node is a one-line
``VlaObservation(**observation.as_msg_kwargs())``; no decision lives there.
"""

import math

from vla_controller import spec


def seconds(stamp):
    """A ROS 2 ``Time``-like object -> float seconds.

    Duck-typed on purpose: both ROS 2 shells and the offline stub tests can use
    it, and the pure-Python layer never has to import ``builtin_interfaces``.
    Keeping the nanoseconds matters -- a 0.75 s freshness window is the same
    order of magnitude as the rounding error from truncating to whole seconds.
    """
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class ObservationError(ValueError):
    """The available feedback does not describe one coherent instant."""

    def __init__(self, code, reason):
        self.code = str(code)
        super().__init__('%s: %s' % (self.code, reason))


class Code:
    """Stable machine-readable rejection codes, matching safety.Code's style."""
    OK = 'ok'
    MISSING_JOINT = 'missing_joint'
    JOINT_RANGE = 'joint_out_of_range'
    NONFINITE = 'nonfinite'
    SKEW = 'sensor_skew'
    IMAGE = 'wrist_image_missing'
    INSTRUCTION = 'instruction_missing'
    POSE = 'pose_unavailable'
    STAMP = 'stamp_invalid'


class SensorSnapshot:
    """One set of readings offered as a candidate observation.

    ``joint_ticks`` maps joint name -> servo tick (the physical feedback, not
    the radian values on ``/joint_states``).  ``wrist_image`` is the compressed
    bytes; an empty value is a refusal, not "no camera", because this arm does
    have a wrist camera and a dead one must not look like a dark frame.
    """

    __slots__ = ('joint_ticks', 'joint_stamp', 'wrist_image',
                 'wrist_image_format', 'wrist_stamp', 'instruction')

    def __init__(self, joint_ticks, joint_stamp, wrist_image,
                 wrist_image_format, wrist_stamp, instruction):
        self.joint_ticks = dict(joint_ticks)
        self.joint_stamp = float(joint_stamp)
        self.wrist_image = bytes(wrist_image) if wrist_image else b''
        self.wrist_image_format = str(wrist_image_format or 'jpeg')
        self.wrist_stamp = float(wrist_stamp)
        self.instruction = str(instruction)


class Observation:
    """A validated observation, ready to be turned into a ROS 2 message."""

    __slots__ = ('data', 'stamp')

    def __init__(self, data, stamp):
        self.data = dict(data)
        self.stamp = float(stamp)

    def as_msg_kwargs(self):
        """Field names match VlaObservation.msg exactly.

        ``header``, ``tcp_position_m``, ``tcp_orientation``, ``wrist_image`` and
        ``front_image`` are returned as small dictionaries here; the ROS 2 shell
        expands them into the generated message types.
        """
        return dict(self.data)

    def __getitem__(self, key):
        return self.data[key]

    def __repr__(self):
        return ('Observation(stamp=%.3f, instruction=%r, tcp=%s, joints=%d)'
                % (self.stamp, self.data['instruction'],
                   self.data['tcp_position_m'], len(self.data['joint_names'])))


def quaternion_from_matrix(rotation):
    """3x3 rotation matrix -> normalised ``(x, y, z, w)``.

    Pure Python because ``armpi_ros2.arm_kinematics`` deliberately returns plain
    tuples and has no numpy dependency.  A matrix that is not finite, or whose
    diagonal is degenerate, is rejected rather than quietly producing an
    identity quaternion: the quaternion goes into a recorded dataset, where a
    confident wrong number is worse than no number.
    """
    try:
        m = [[float(rotation[row][column]) for column in range(3)]
             for row in range(3)]
    except (TypeError, ValueError, IndexError, KeyError) as error:
        raise ObservationError(Code.POSE, 'rotation matrix is not 3x3: %s' % error)
    if not all(math.isfinite(value) for row in m for value in row):
        raise ObservationError(Code.NONFINITE, 'rotation matrix has non-finite entries')
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * scale
        qx = (m[2][1] - m[1][2]) / scale
        qy = (m[0][2] - m[2][0]) / scale
        qz = (m[1][0] - m[0][1]) / scale
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        scale = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        qw = (m[2][1] - m[1][2]) / scale
        qx = 0.25 * scale
        qy = (m[0][1] + m[1][0]) / scale
        qz = (m[0][2] + m[2][0]) / scale
    elif m[1][1] > m[2][2]:
        scale = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        qw = (m[0][2] - m[2][0]) / scale
        qx = (m[0][1] + m[1][0]) / scale
        qy = 0.25 * scale
        qz = (m[1][2] + m[2][1]) / scale
    else:
        scale = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
        qw = (m[1][0] - m[0][1]) / scale
        qx = (m[0][2] + m[2][0]) / scale
        qy = (m[1][2] + m[2][1]) / scale
        qz = 0.25 * scale
    if scale <= 0.0:
        raise ObservationError(Code.POSE, 'rotation matrix is degenerate')
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if not math.isfinite(norm) or norm < 1e-6:
        raise ObservationError(Code.POSE, 'quaternion is not normalisable')
    return (qx / norm, qy / norm, qz / norm, qw / norm)


def default_pose_provider(joint_ticks, kinematics=None):
    """Forward kinematics -> ``{'position': (x,y,z), 'quaternion': (x,y,z,w)}``.

    Imported lazily so that the pure-Python tests can inject a fake provider
    without ``armpi_ros2`` on ``sys.path``.  ``kinematics`` is injectable for
    exactly that reason.
    """
    if kinematics is None:
        try:
            from armpi_ros2 import arm_kinematics as kinematics
        except ImportError as error:
            raise ObservationError(
                Code.POSE,
                'armpi_ros2.arm_kinematics is pure Python but not on sys.path (%s)'
                % error)
    try:
        fk = kinematics.forward_tool_pose_raw(joint_ticks)
    except (ValueError, KeyError, TypeError) as error:
        raise ObservationError(Code.POSE,
                               'forward kinematics rejected the ticks: %s' % error)
    position = tuple(float(value) for value in fk['translation_m'])
    quaternion = quaternion_from_matrix(fk['rotation_matrix'])
    return {'position': position, 'quaternion': quaternion}


class ObservationBuilder:
    """Validate a :class:`SensorSnapshot` and emit one observation."""

    #: Joints that must all be present.  ``joint5`` is held fixed by the pose
    #: model but still belongs in the observation: a policy has to see the whole
    #: measured state, including the axis it is not allowed to move.
    REQUIRED_JOINTS = tuple(spec.POSE_JOINTS) + (spec.WRIST_ROLL_JOINT,
                                                 spec.GRIPPER_JOINT)

    def __init__(self, pose_provider=None, max_skew_s=0.75,
                 frame_id=spec.ACTION_FRAME, require_wrist_image=True):
        if not math.isfinite(max_skew_s) or max_skew_s <= 0:
            raise ValueError('max_skew_s must be finite and positive')
        self.pose_provider = pose_provider or default_pose_provider
        self.max_skew_s = float(max_skew_s)
        self.frame_id = str(frame_id)
        self.require_wrist_image = bool(require_wrist_image)

    def build(self, snapshot):
        """Return an :class:`Observation`, or raise :class:`ObservationError`."""
        ticks = snapshot.joint_ticks
        missing = [name for name in self.REQUIRED_JOINTS if name not in ticks]
        if missing:
            raise ObservationError(Code.MISSING_JOINT,
                                   'missing %s' % ', '.join(missing))

        for name in self.REQUIRED_JOINTS:
            value = ticks[name]
            if not isinstance(value, (int, float)) or not math.isfinite(float(value)):
                raise ObservationError(Code.NONFINITE,
                                       'joint %s reading is not finite' % name)
            low, high = spec.joint_limits(name)
            if not low <= float(value) <= high:
                raise ObservationError(
                    Code.JOINT_RANGE,
                    'joint %s = %s outside %d..%d (encoder may be lying)'
                    % (name, value, low, high))

        for label, stamp in (('joint', snapshot.joint_stamp),
                             ('wrist image', snapshot.wrist_stamp)):
            if not math.isfinite(stamp) or stamp <= 0:
                raise ObservationError(Code.STAMP,
                                       '%s stamp is missing or invalid' % label)
        skew = abs(snapshot.joint_stamp - snapshot.wrist_stamp)
        if skew > self.max_skew_s:
            raise ObservationError(
                Code.SKEW,
                'joint frame and wrist image are %.3f s apart (limit %.3f s)'
                % (skew, self.max_skew_s))

        if self.require_wrist_image and not snapshot.wrist_image:
            raise ObservationError(Code.IMAGE,
                                   'wrist camera produced no data; treating a dead '
                                   'camera as a dark frame would corrupt the dataset')
        if not snapshot.instruction:
            raise ObservationError(Code.INSTRUCTION,
                                   'instruction is empty; never fabricate one')

        try:
            pose = self.pose_provider(dict(ticks))
            position = tuple(float(value) for value in pose['position'])
            quaternion = tuple(float(value) for value in pose['quaternion'])
        except ObservationError:
            raise
        except (KeyError, TypeError, ValueError) as error:
            raise ObservationError(Code.POSE,
                                   'pose provider returned unusable data: %s' % error)
        if len(position) != 3 or len(quaternion) != 4:
            raise ObservationError(Code.POSE, 'pose provider returned wrong dimensions')
        if not all(math.isfinite(value) for value in (*position, *quaternion)):
            raise ObservationError(Code.NONFINITE, 'TCP pose contains non-finite values')

        names = list(self.REQUIRED_JOINTS)
        radians = [spec.tick_to_radians(name, float(ticks[name])) for name in names]
        gripper_tick = float(ticks[spec.GRIPPER_JOINT])
        data = {
            'header': {'stamp': snapshot.joint_stamp, 'frame_id': self.frame_id},
            'instruction': snapshot.instruction,
            'robot_config_id': spec.ROBOT_CONFIG_ID,
            'tcp_position_m': position,
            'tcp_orientation': quaternion,
            'joint_names': names,
            'joint_positions_rad': radians,
            'gripper_opening': spec.gripper_tick_to_opening(gripper_tick),
            'wrist_image_stamp': snapshot.wrist_stamp,
            'wrist_image': {'format': snapshot.wrist_image_format,
                            'data': snapshot.wrist_image},
            # No scene camera exists on this platform.  Empty data is the
            # contract's explicit "absent" value; a policy must not read it as
            # "the camera went dark".
            'front_image_stamp': 0.0,
            'front_image': {'format': '', 'data': b''},
        }
        return Observation(data, snapshot.joint_stamp)
