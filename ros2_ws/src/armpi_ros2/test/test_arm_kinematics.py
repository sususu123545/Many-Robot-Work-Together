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


def test_all_branches_keeps_the_vendor_preferred_solution_first():
    """inverse_kinematics must stay a plain "give me the vendor branch" call."""
    from armpi_ros2.arm_kinematics import inverse_kinematics_all
    target, pitch = (0.0, 0.12, 0.15), -120.0
    branches = inverse_kinematics_all(*target, pitch)
    preferred = inverse_kinematics(*target, pitch)
    assert branches
    assert branches[0]['servo_targets'] == preferred['servo_targets']
    assert branches[0]['elbow'] == 'down'


def test_all_branches_exposes_the_alternate_elbow_when_it_is_bounded():
    """At the real grasp target both branches exist, and they are far apart.

    This is why the dry run needs the full list: the vendor-preferred branch can
    be the FARTHER one, and picking it means asking a joint for hundreds of ticks
    of travel it does not need.
    """
    from armpi_ros2.arm_kinematics import inverse_kinematics_all
    branches = inverse_kinematics_all(0.00245, 0.28509, -0.04327, -135.0)
    assert len(branches) == 2
    assert {branch['elbow'] for branch in branches} == {'down', 'up'}
    for branch in branches:
        # The label must describe the sign the solver actually used, otherwise a
        # planner reading it would mis-reason about which way the elbow folds.
        expected = 'down' if branch['theta4'] < 0 else 'up'
        assert branch['elbow'] == expected
    down, up = branches
    assert abs(down['servo_targets'][5] - up['servo_targets'][5]) > 200


def test_all_branches_raises_when_no_branch_fits():
    from armpi_ros2.arm_kinematics import inverse_kinematics_all
    with pytest.raises(ValueError, match='outside the arm workspace'):
        inverse_kinematics_all(0.0, 0.5, 0.0, -90.0)
