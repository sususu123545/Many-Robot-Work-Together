import math

import pytest

from armpi_ros2.arm_kinematics import (
    forward_kinematics, forward_tool_pose_raw, inverse_kinematics)


@pytest.mark.parametrize('target,pitch,expected', [
    ((0.0, 0.15, 0.03), -165.0, (-89.65, -66.13, -9.22, 0.0)),
    ((0.0, 0.12, 0.15), -120.0, (-95.94, -60.76, 36.70, 0.0)),
    ((0.0, 0.15, -0.08), -180.0, (-56.46, -78.54, -45.0, 0.0)),
    ((0.04, 0.15, 0.03), -165.0, (-90.31, -62.26, -12.43, -14.93)),
    ((0.0, 0.20, 0.20), -90.0, (-29.12, -88.16, 27.28, 0.0)),
])
def test_matches_isolated_vendor_ik_samples(target, pitch, expected):
    solution = inverse_kinematics(*target, pitch)
    actual = tuple(solution[name] for name in
                   ('theta3', 'theta4', 'theta5', 'theta6'))
    assert actual == pytest.approx(expected, abs=0.02)
    assert forward_kinematics(*actual) == pytest.approx(target, abs=2e-5)


def test_servo_ticks_match_stock_wrapper_conversion():
    solution = inverse_kinematics(0.04, 0.15, 0.03, -165.0)
    assert solution['servo_targets'] == {3: 124, 4: 759, 5: 448, 6: 438}


def test_rejects_unreachable_and_nonfinite_targets():
    with pytest.raises(ValueError, match='outside the arm workspace'):
        inverse_kinematics(0.0, 1.0, 1.0, -90.0)
    with pytest.raises(ValueError, match='finite'):
        inverse_kinematics(math.nan, 0.1, 0.1, -90.0)


def test_forward_tool_pose_matches_zero_raw_reference():
    pose = forward_tool_pose_raw({
        'joint1': 500, 'joint2': 500, 'joint3': 500, 'joint4': 500,
        'joint5': 777})
    assert pose['translation_m'] == pytest.approx((0.0, 0.0, 0.423109), abs=1e-9)
    for actual, expected in zip(pose['rotation_matrix'],
                                ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0),
                                 (0.0, 0.0, 1.0))):
        assert actual == pytest.approx(expected, abs=1e-12)


def test_forward_tool_pose_rejects_incomplete_or_invalid_feedback():
    with pytest.raises(ValueError, match='Missing'):
        forward_tool_pose_raw({'joint1': 500})
    with pytest.raises(ValueError, match='outside'):
        forward_tool_pose_raw({'joint1': 500, 'joint2': 876,
                               'joint3': 500, 'joint4': 500})
