"""Pure-Python ArmPi Pro planar arm kinematics.

The link dimensions and servo-angle mapping match the checked-in Hiwonder
``ik_transform.py`` wrapper. The proprietary Python extension is deliberately
not imported, so this module works with the ROS 2 Jazzy Python runtime.

Coordinates use the vendor convention: x is lateral, y is forward, z is up;
pitch is in degrees. This module only computes geometry and servo ticks. It
never sends commands to the robot.
"""
import math


BASE_HEIGHT_M = 0.064605
SHOULDER_LINK_M = 0.10048
ELBOW_LINK_M = 0.094714
TOOL_LINK_M = 0.16331
TICKS_PER_DEGREE = 1000.0 / 240.0


def _finite(values, label):
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError(label + ' must contain only finite numbers')


def forward_kinematics(theta3_deg, theta4_deg, theta5_deg, theta6_deg):
    """Return vendor-frame (x, y, z) in meters for four IK angles in degrees."""
    _finite((theta3_deg, theta4_deg, theta5_deg, theta6_deg), 'Angles')
    q1 = math.radians(theta5_deg)
    q2 = q1 + math.radians(theta4_deg)
    q3 = q2 + math.radians(theta3_deg)
    radial = -(SHOULDER_LINK_M * math.sin(q1)
               + ELBOW_LINK_M * math.sin(q2)
               + TOOL_LINK_M * math.sin(q3))
    z = (BASE_HEIGHT_M + SHOULDER_LINK_M * math.cos(q1)
         + ELBOW_LINK_M * math.cos(q2)
         + TOOL_LINK_M * math.cos(q3))
    yaw = math.radians(theta6_deg)
    x = -radial * math.sin(yaw)
    y = radial * math.cos(yaw)
    return x, y, z


def forward_tool_pose_raw(raw):
    """Return the stock four-axis IK tool pose from physical servo ticks.

    ``joint1`` is base yaw, ``joint2`` shoulder, ``joint3`` elbow,
    ``joint4`` wrist pitch. The ROS ``joint5`` wrist-roll axis is intentionally
    not included: callers must hold it fixed while using this pose model. The
    returned origin is the vendor IK tool point in the arm-base frame (not the
    mobile chassis ``base_link`` frame).
    """
    names = ('joint1', 'joint2', 'joint3', 'joint4')
    if any(name not in raw for name in names):
        raise ValueError('Missing arm joint raw feedback')
    values = {name: float(raw[name]) for name in names}
    _finite(values.values(), 'Servo readings')
    bounds = {'joint1': (0.0, 1000.0), 'joint2': (125.0, 875.0),
              'joint3': (0.0, 1000.0), 'joint4': (0.0, 1000.0)}
    if any(not bounds[name][0] <= value <= bounds[name][1]
           for name, value in values.items()):
        raise ValueError('Servo reading is outside its measured range')

    tick_deg = 240.0 / 1000.0
    theta6 = (values['joint1'] - 500.0) * tick_deg
    theta5 = (values['joint2'] - 500.0) * tick_deg
    theta4 = -(values['joint3'] - 500.0) * tick_deg
    theta3 = (values['joint4'] - 500.0) * tick_deg
    pitch = math.radians(theta3 + theta4 + theta5)
    yaw = math.radians(theta6)
    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    rotation = ((cy, -sy * cp, sy * sp),
                (sy, cy * cp, -cy * sp),
                (0.0, sp, cp))
    return {'translation_m': forward_kinematics(theta3, theta4, theta5, theta6),
            'rotation_matrix': rotation,
            'angles_deg': {'theta3': theta3, 'theta4': theta4,
                           'theta5': theta5, 'theta6': theta6}}


def _raw_servo_targets(theta3_deg, theta4_deg, theta5_deg, theta6_deg):
    """Match the stock ArmIK.transformAngelAdaptArm pulse conversion."""
    _finite((theta3_deg, theta4_deg, theta5_deg, theta6_deg), 'Angles')
    servo3 = round(theta3_deg * TICKS_PER_DEGREE + 500.0)
    servo4 = round(-theta4_deg * TICKS_PER_DEGREE + 500.0)
    servo5 = round(theta5_deg * TICKS_PER_DEGREE + 500.0)
    # The vendor wrapper quantizes base yaw to a whole degree before scaling.
    servo6 = round(round(120.0 + theta6_deg) * TICKS_PER_DEGREE)
    targets = {3: int(servo3), 4: int(servo4), 5: int(servo5), 6: int(servo6)}
    if any(not 0 <= value <= 1000 for value in targets.values()):
        raise ValueError('IK solution exceeds the vendor servo range')
    return targets


def inverse_kinematics(x, y, z, pitch_deg):
    """Solve the stock 3-link arm at a requested wrist pitch.

    The negative elbow branch is preferred, matching the ArmPi Pro vendor
    solver for its normal reachable workspace. If it exceeds stock servo
    bounds, the alternate branch is considered. Returns joint angles and raw
    servo targets, or raises ValueError when no bounded solution exists.

    Use inverse_kinematics_all when the caller cares WHICH branch it gets: the
    preferred branch is the vendor's choice for teleoperation, not necessarily
    the one that is nearest the pose the arm is already standing in, and for a
    guarded motion the shorter travel is the safer instruction.
    """
    return inverse_kinematics_all(x, y, z, pitch_deg)[0]


def inverse_kinematics_all(x, y, z, pitch_deg):
    """Return every servo-bounded IK branch, vendor-preferred branch first.

    Each entry is the same dict inverse_kinematics returns. There are at most
    two branches (the elbow up/down pair); a branch disappears when its servo
    targets fall outside the stock range.
    """
    _finite((x, y, z, pitch_deg), 'Target')
    x, y, z, pitch_deg = map(float, (x, y, z, pitch_deg))
    radius = math.hypot(x, y)
    alpha = math.radians(pitch_deg)

    # Remove the terminal tool link. Its radial and vertical components in the
    # vendor convention are (-L4*sin(alpha), L4*cos(alpha)).
    wrist_r = radius + TOOL_LINK_M * math.sin(alpha)
    wrist_z = z - BASE_HEIGHT_M - TOOL_LINK_M * math.cos(alpha)
    distance_sq = wrist_r * wrist_r + wrist_z * wrist_z
    cos_elbow = ((distance_sq - SHOULDER_LINK_M**2 - ELBOW_LINK_M**2)
                 / (2.0 * SHOULDER_LINK_M * ELBOW_LINK_M))
    if cos_elbow < -1.0 - 1e-10 or cos_elbow > 1.0 + 1e-10:
        raise ValueError('Target is outside the arm workspace')
    cos_elbow = min(1.0, max(-1.0, cos_elbow))
    elbow_magnitude = math.acos(cos_elbow)
    yaw_deg = -math.degrees(math.atan2(x, y)) if radius > 1e-12 else 0.0

    solutions = []
    # Vendor IK selects the negative elbow solution first.
    for theta4 in (-elbow_magnitude, elbow_magnitude):
        shoulder_absolute = (math.atan2(wrist_z, wrist_r)
                             - math.atan2(ELBOW_LINK_M * math.sin(theta4),
                                          SHOULDER_LINK_M
                                          + ELBOW_LINK_M * math.cos(theta4)))
        theta5 = shoulder_absolute - math.pi / 2.0
        theta3 = alpha - theta5 - theta4
        angles = (math.degrees(theta3), math.degrees(theta4),
                  math.degrees(theta5), yaw_deg)
        try:
            raw_targets = _raw_servo_targets(*angles)
        except ValueError:
            continue
        solutions.append({'theta3': angles[0], 'theta4': angles[1],
                          'theta5': angles[2], 'theta6': angles[3],
                          'servo_targets': raw_targets,
                          'requested_pitch_deg': pitch_deg,
                          'elbow': 'down' if theta4 < 0 else 'up'})
    if not solutions:
        raise ValueError('No IK branch fits the stock servo limits')
    return solutions
