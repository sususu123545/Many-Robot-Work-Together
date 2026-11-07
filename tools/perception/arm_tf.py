#!/usr/bin/env python3
"""arm_base / rear_camera transforms for the TF tree -- pure geometry.

Why this module exists
----------------------
The car publishes only three frames: `odom -> base_link` (mecanum_odom) plus the
two latched statics `base_link -> base_laser` and `base_link -> imu_link`.  There
is no `robot_state_publisher` on the car, so neither `arm_base` nor `rear_camera`
exists in TF at all -- every PC-side tool that wants `base_link -> rear_camera`
(Nav2 docking, MoveIt, verify_tf_tree) fails to look it up.

`rear_camera` cannot be a static transform.  The camera is bolted to the wrist:
it moves with joint1 (base yaw) and joint2 (shoulder) -- measured 2026-11-05,
joint1 +20 ticks moved a ball 30 px.  So it has to be recomputed from joint
feedback, which is what arm_tf_publisher.py does at ~10 Hz.

Which camera offset is right
---------------------------
Two numbers exist for the camera's position on the wrist and they disagree:

  robot_description/urdf/armpi_pro.urdf  camera_joint = (0, -0.042169, 0.046483)
                                         relative to link4
  hand-eye (rear_camera_hand_eye.json)   tool_T_camera = (0.00225, -0.04794,
                                         -0.09497), i.e. (0.00225, -0.04794,
                                         0.06834) relative to link4 once the
                                         0.16331 m tool offset is added

z differs by 21.8 mm.  The hand-eye number wins: it comes from 8 checkerboard
views with 2.46 mm translation RMS and 0.52 deg rotation RMS, and the pipeline
built on it put a real 35 mm ball within 1.5 mm of where the arm reached for it.
The URDF value has no recorded provenance.  test_arm_tf.py pins the difference
so a future edit to either side fails loudly instead of silently shifting the
camera by 2 cm.

Frames and conventions
----------------------
`arm_kinematics` (x lateral, y forward, z up) is the vendor frame.  It matches
the URDF's `arm_base` orientation, so no basis change is needed -- the hand-eye
translation drops straight in.
"""
import json
import math

BASE_LINK_TO_ARM_BASE_M = (0.0, 0.062, 0.094)

# link4 -> camera, derived from tool_T_camera plus the tool offset along link4's
# z axis.  TOOL_LINK_M is imported lazily by callers so this module stays
# importable without the ROS 2 tree on sys.path.
LINK4_Z_TO_TOOL_M = 0.16331


def quaternion_from_matrix(rotation):
    """xyzw quaternion from a 3x3 row-major rotation matrix."""
    m00, m01, m02 = rotation[0]
    m10, m11, m12 = rotation[1]
    m20, m21, m22 = rotation[2]
    trace = m00 + m11 + m22
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m21 - m12) / s
        y = (m02 - m20) / s
        z = (m10 - m01) / s
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        w = (m21 - m12) / s
        x = 0.25 * s
        y = (m01 + m10) / s
        z = (m02 + m20) / s
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        w = (m02 - m20) / s
        x = (m01 + m10) / s
        y = 0.25 * s
        z = (m12 + m21) / s
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        w = (m10 - m01) / s
        x = (m02 + m20) / s
        y = (m12 + m21) / s
        z = 0.25 * s
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    return (x / norm, y / norm, z / norm, w / norm)


def matrix_from_quaternion(x, y, z, w):
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return ((1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)),
            (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)),
            (2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)))


def mat_mul(a, b):
    return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3))
                 for i in range(3))


def mat_vec(a, v):
    return tuple(sum(a[i][k] * v[k] for k in range(3)) for i in range(3))


def load_hand_eye(path):
    """Return (rotation_tool_camera, translation_tool_camera, raw_json)."""
    with open(path, 'r', encoding='utf-8') as handle:
        data = json.load(handle)
    block = data['tool_T_camera']
    quaternion = block['quaternion_xyzw']
    return (matrix_from_quaternion(*quaternion),
            tuple(float(v) for v in block['translation_m']),
            data)


def camera_pose_in_arm_base(raw, hand_eye, forward_tool_pose_raw):
    """arm_base -> rear_camera as (translation_m, quaternion_xyzw).

    `raw` is the four-joint servo tick dict, `hand_eye` is what load_hand_eye
    returns, `forward_tool_pose_raw` is injected so this module does not import
    the ROS 2 tree itself.
    """
    rotation_tool_camera, translation_tool_camera, _ = hand_eye
    tool = forward_tool_pose_raw(raw)
    rotation_base_tool = tool['rotation_matrix']
    translation_base_tool = tool['translation_m']
    translation = tuple(
        sum(rotation_base_tool[i][k] * translation_tool_camera[k] for k in range(3))
        + translation_base_tool[i] for i in range(3))
    rotation = mat_mul(rotation_base_tool, rotation_tool_camera)
    return translation, quaternion_from_matrix(rotation)


def camera_offset_in_link4(hand_eye):
    """Where the hand-eye says the camera sits relative to link4, in metres."""
    rotation_tool_camera, translation_tool_camera, _ = hand_eye
    forward = mat_vec(rotation_tool_camera, (0.0, 0.0, LINK4_Z_TO_TOOL_M))
    return tuple(forward[i] + translation_tool_camera[i] for i in range(3))


# The URDF's own camera_joint value, kept here so a test can assert the two
# sources stay distinguishable instead of one silently drifting into the other.
URDF_CAMERA_JOINT_M = (0.0, -0.042169, 0.046483)
