import cv2
import numpy as np
import pytest

from armpi_ros2.arm_kinematics import forward_tool_pose_raw
from armpi_ros2.hand_eye_calibration import (
    _transform, _rotation_to_quaternion, joint_state_to_raw, solve_hand_eye)
from sensor_msgs.msg import JointState


def test_joint_state_round_trip_to_physical_servo_ticks():
    msg = JointState()
    msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'r_joint']
    msg.position = [
        (482 - 500) * np.deg2rad(0.24),
        (681 - 500) * np.deg2rad(0.24),
        -(917 - 500) * np.deg2rad(0.24),
        (191 - 500) * np.deg2rad(0.24),
        (499 - 500) * np.deg2rad(0.24),
        -np.pi / 2,
    ]
    assert joint_state_to_raw(msg) == {
        'joint1': 482, 'joint2': 681, 'joint3': 917,
        'joint4': 191, 'joint5': 499}


def test_hand_eye_solver_recovers_synthetic_fixed_camera_extrinsic():
    raw_poses = [
        (440, 650, 360, 200), (480, 620, 400, 240),
        (520, 560, 430, 270), (560, 600, 480, 200),
        (420, 700, 520, 250), (540, 520, 370, 320),
        (460, 680, 460, 160), (500, 580, 500, 300),
        (580, 540, 420, 220), (430, 630, 540, 310),
    ]
    tool_T_camera_rotation, _ = cv2.Rodrigues(
        np.array([0.25, -0.18, 0.12], dtype=np.float64))
    tool_T_camera = _transform(tool_T_camera_rotation,
                               (0.035, -0.021, 0.044))
    board_rotation, _ = cv2.Rodrigues(
        np.array([-0.1, 0.08, 0.2], dtype=np.float64))
    base_T_board = _transform(board_rotation, (0.12, 0.28, 0.04))
    samples = []
    for joint1, joint2, joint3, joint4 in raw_poses:
        raw = {'joint1': joint1, 'joint2': joint2, 'joint3': joint3,
               'joint4': joint4, 'joint5': 499}
        pose = forward_tool_pose_raw(raw)
        base_T_tool = _transform(pose['rotation_matrix'], pose['translation_m'])
        camera_T_board = np.linalg.inv(base_T_tool @ tool_T_camera) @ base_T_board
        samples.append({
            'base_T_tool': base_T_tool.tolist(),
            'camera_T_board': camera_T_board.tolist(),
            'angles_deg': pose['angles_deg'],
            'reprojection_rms_px': 0.5,
        })

    result = solve_hand_eye(samples)
    translation = result['tool_T_camera']['translation_m']
    quaternion = result['tool_T_camera']['quaternion_xyzw']
    recovered_rotation = np.array([
        [1-2*(quaternion[1]**2+quaternion[2]**2),
         2*(quaternion[0]*quaternion[1]-quaternion[2]*quaternion[3]),
         2*(quaternion[0]*quaternion[2]+quaternion[1]*quaternion[3])],
        [2*(quaternion[0]*quaternion[1]+quaternion[2]*quaternion[3]),
         1-2*(quaternion[0]**2+quaternion[2]**2),
         2*(quaternion[1]*quaternion[2]-quaternion[0]*quaternion[3])],
        [2*(quaternion[0]*quaternion[2]-quaternion[1]*quaternion[3]),
         2*(quaternion[1]*quaternion[2]+quaternion[0]*quaternion[3]),
         1-2*(quaternion[0]**2+quaternion[1]**2)],
    ])
    assert translation == pytest.approx(tool_T_camera[:3, 3], abs=1e-6)
    assert recovered_rotation == pytest.approx(tool_T_camera[:3, :3], abs=1e-6)
    assert result['quality']['accepted'] is True


def test_hand_eye_solver_requires_rotational_pose_diversity():
    raw = {'joint1': 500, 'joint2': 600, 'joint3': 450,
           'joint4': 300, 'joint5': 499}
    pose = forward_tool_pose_raw(raw)
    transform = _transform(pose['rotation_matrix'], pose['translation_m']).tolist()
    sample = {'base_T_tool': transform, 'camera_T_board': transform,
              'angles_deg': pose['angles_deg'], 'reprojection_rms_px': 0.5}
    with pytest.raises(ValueError, match='at least 8'):
        solve_hand_eye([sample] * 7)


@pytest.mark.parametrize('vector', [
    (0.0, 0.0, 0.0), (np.pi, 0.0, 0.0), (0.0, np.pi, 0.0),
    (0.1, -0.2, 0.3),
])
def test_rotation_to_quaternion_is_normalized(vector):
    rotation, _ = cv2.Rodrigues(np.asarray(vector, dtype=np.float64))
    quaternion = _rotation_to_quaternion(rotation)
    assert np.linalg.norm(quaternion) == pytest.approx(1.0, abs=1e-12)
