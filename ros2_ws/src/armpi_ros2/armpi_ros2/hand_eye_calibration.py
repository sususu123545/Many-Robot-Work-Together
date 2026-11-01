"""Read-only eye-in-hand calibration for the ArmPi Pro rear camera.

The checkerboard must remain fixed while the arm moves. Captured arm poses are
computed in the vendor IK arm-base frame; the fifth wrist axis must remain at
one fixed raw position throughout collection. This module never publishes any
motion, gripper, chassis, reset, or safety command.
"""
import json
import math
import os
import time
from collections import deque

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_srvs.srv import Trigger
from std_msgs.msg import String

from armpi_ros2.arm_kinematics import forward_tool_pose_raw
from armpi_ros2.red_ball_detector import image_msg_to_bgr


TICK_RAD = math.radians(240.0) / 1000.0
JOINTS = ('joint1', 'joint2', 'joint3', 'joint4', 'joint5')
SIGNS = {'joint1': 1.0, 'joint2': 1.0, 'joint3': -1.0,
         'joint4': 1.0, 'joint5': 1.0}
CENTRES = {name: 500.0 for name in JOINTS}
LIMITS = {'joint1': (0, 1000), 'joint2': (125, 875),
          'joint3': (0, 1000), 'joint4': (0, 1000), 'joint5': (0, 1000)}


def joint_state_to_raw(message):
    """Convert a ROS JointState from the native arm node back to raw ticks."""
    values = dict(zip(message.name, message.position))
    if any(name not in values for name in JOINTS):
        raise ValueError('JointState is missing one or more arm joints')
    result = {}
    for name in JOINTS:
        angle = float(values[name])
        if not math.isfinite(angle):
            raise ValueError('JointState contains a non-finite angle')
        raw = round(CENTRES[name] + SIGNS[name] * angle / TICK_RAD)
        low, high = LIMITS[name]
        if not low <= raw <= high:
            raise ValueError('%s feedback is outside its measured limits' % name)
        result[name] = int(raw)
    return result


def _transform(rotation, translation):
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    result[:3, 3] = np.asarray(translation, dtype=np.float64).reshape(3)
    return result


def checkerboard_pose(image_bgr, camera_info, pattern=(9, 6), square_m=0.025):
    """Return T_camera_board and reprojection RMS for the calibrated lens."""
    width, height = int(camera_info.width), int(camera_info.height)
    if image_bgr.shape[:2] != (height, width):
        raise ValueError('Image and CameraInfo dimensions differ')
    if len(pattern) != 2 or min(pattern) < 3 or square_m <= 0:
        raise ValueError('Invalid checkerboard geometry')
    k = np.asarray(camera_info.k, dtype=np.float64).reshape(3, 3)
    d = np.asarray(camera_info.d, dtype=np.float64)
    model = camera_info.distortion_model.lower()
    if model == 'equidistant':
        if d.size != 4:
            raise ValueError('Equidistant CameraInfo must contain four coefficients')
        rectified = cv2.fisheye.undistortImage(
            image_bgr, k, d.reshape(4, 1), Knew=k)
        def rectify_points(points):
            return cv2.fisheye.undistortPoints(
                np.asarray(points, dtype=np.float64).reshape(-1, 1, 2),
                k, d.reshape(4, 1), P=k).reshape(-1, 2)
    elif model in ('plumb_bob', 'rational_polynomial'):
        rectified = cv2.undistort(image_bgr, k, d)
        def rectify_points(points):
            return cv2.undistortPoints(
                np.asarray(points, dtype=np.float64).reshape(-1, 1, 2),
                k, d, P=k).reshape(-1, 2)
    else:
        raise ValueError('Unsupported checkerboard camera model: ' + model)

    # With this wide-angle camera, image resampling may erase small squares.
    # Try half resolution first, then full resolution, on rectified and raw
    # images. Raw corners are mapped through the measured lens model before PnP.
    gray_images = (
        (cv2.cvtColor(rectified, cv2.COLOR_BGR2GRAY), False),
        (cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY), True),
    )
    image_points = None
    legacy_flags = (cv2.CALIB_CB_ADAPTIVE_THRESH
                    | cv2.CALIB_CB_NORMALIZE_IMAGE)
    for gray, is_raw in gray_images:
        for scale in (0.5, 1.0):
            working = (cv2.resize(gray, None, fx=scale, fy=scale,
                                  interpolation=cv2.INTER_AREA)
                       if scale != 1.0 else gray)
            found, corners = False, None
            if hasattr(cv2, 'findChessboardCornersSB'):
                flags = cv2.CALIB_CB_NORMALIZE_IMAGE
                found, corners = cv2.findChessboardCornersSB(
                    working, tuple(pattern), flags)
            if not found:
                found, corners = cv2.findChessboardCorners(
                    working, tuple(pattern), legacy_flags)
                if found and corners is not None:
                    corners = cv2.cornerSubPix(
                        working, corners, (5, 5), (-1, -1),
                        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                         30, 0.01))
            if found and corners is not None:
                image_points = (np.asarray(corners, dtype=np.float64)
                                .reshape(-1, 2) / scale)
                if is_raw:
                    image_points = rectify_points(image_points)
                break
        if image_points is not None:
            break
    if image_points is None:
        raise ValueError('checkerboard_not_found_in_raw_or_rectified_image')

    cols, rows = map(int, pattern)
    object_points = np.zeros((rows * cols, 3), dtype=np.float64)
    object_points[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_m
    success, rvec, tvec = cv2.solvePnP(
        object_points, image_points, k, np.zeros(5, dtype=np.float64),
        flags=cv2.SOLVEPNP_ITERATIVE)
    if not success or float(tvec[2]) <= 0:
        raise ValueError('checkerboard_pose_solve_failed')
    projected, _ = cv2.projectPoints(
        object_points, rvec, tvec, k, np.zeros(5, dtype=np.float64))
    residual = projected.reshape(-1, 2) - image_points
    rms = float(np.sqrt(np.mean(np.sum(residual * residual, axis=1))))
    if not math.isfinite(rms) or rms > 3.0:
        raise ValueError('checkerboard_reprojection_rms_too_high: %.3f px' % rms)
    rotation, _ = cv2.Rodrigues(rvec)
    return _transform(rotation, tvec), rms


def _rotation_distance_deg(left, right):
    relative = left[:3, :3].T @ right[:3, :3]
    cosine = float(np.clip((np.trace(relative) - 1.0) * 0.5, -1.0, 1.0))
    return math.degrees(math.acos(cosine))


def _average_rotation(rotations):
    u, _, vt = np.linalg.svd(np.sum(rotations, axis=0))
    result = u @ vt
    if np.linalg.det(result) < 0:
        u[:, -1] *= -1
        result = u @ vt
    return result


def _rotation_to_quaternion(rotation):
    """Convert a proper 3x3 rotation matrix to xyzw quaternion."""
    m = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    trace = float(np.trace(m))
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2.0
        q = ((m[2, 1] - m[1, 2]) / s,
             (m[0, 2] - m[2, 0]) / s,
             (m[1, 0] - m[0, 1]) / s,
             0.25 * s)
    else:
        i = int(np.argmax(np.diag(m)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = math.sqrt(max(0.0, 1.0 + m[i, i] - m[j, j] - m[k, k])) * 2.0
        if s < 1e-12:
            raise ValueError('Degenerate rotation matrix')
        qv = [0.0, 0.0, 0.0]
        qv[i] = 0.25 * s
        qv[j] = (m[j, i] + m[i, j]) / s
        qv[k] = (m[k, i] + m[i, k]) / s
        q = (qv[0], qv[1], qv[2], (m[k, j] - m[j, k]) / s)
    norm = math.sqrt(sum(value * value for value in q))
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError('Invalid rotation quaternion')
    return [float(value / norm) for value in q]


def _consistent_checkerboard_poses(samples, tool_poses, pattern, square_m):
    """Resolve the 180-degree corner-order ambiguity across adjacent samples.

    A symmetric chessboard detector may return the grid corners in either
    direction. For a fixed board and rigid eye-in-hand camera, the angle between
    consecutive camera-to-board rotations must match the measured tool rotation
    angle. Use that invariant to choose the direct or 180-degree-reversed grid.
    """
    cols, rows = map(int, pattern)
    board_flip = _transform(
        np.diag((-1.0, -1.0, 1.0)),
        ((cols - 1) * float(square_m), (rows - 1) * float(square_m), 0.0))
    poses = [np.asarray(sample['camera_T_board'], dtype=np.float64).reshape(4, 4)
             for sample in samples]
    flipped_sample_indices = []
    for index in range(1, len(poses)):
        direct = poses[index]
        reversed_order = direct @ board_flip
        expected_delta = _rotation_distance_deg(tool_poses[index - 1],
                                                 tool_poses[index])
        direct_error = abs(_rotation_distance_deg(poses[index - 1], direct)
                           - expected_delta)
        reversed_error = abs(_rotation_distance_deg(poses[index - 1],
                                                     reversed_order)
                             - expected_delta)
        if abs(direct_error - reversed_error) < 0.5:
            raise ValueError('Checkerboard corner order is ambiguous between samples %d and %d'
                             % (index, index + 1))
        if reversed_error < direct_error:
            poses[index] = reversed_order
            flipped_sample_indices.append(index + 1)
    return poses, flipped_sample_indices


def solve_hand_eye(samples, pattern=(9, 6), square_m=0.025):
    """Solve tool_T_camera and report fixed-board consistency residuals."""
    if len(samples) < 8:
        raise ValueError('Need at least 8 distinct stationary checkerboard views')
    tool_poses = [np.asarray(sample['base_T_tool'], dtype=np.float64).reshape(4, 4)
                  for sample in samples]
    board_poses, flipped_sample_indices = _consistent_checkerboard_poses(
        samples, tool_poses, pattern, square_m)
    if (max(sample['angles_deg']['theta6'] for sample in samples)
            - min(sample['angles_deg']['theta6'] for sample in samples) < 8.0):
        raise ValueError('Need at least 8 degrees of base-yaw variation')
    pitches = [sum(sample['angles_deg'][name]
                   for name in ('theta3', 'theta4', 'theta5')) for sample in samples]
    if max(pitches) - min(pitches) < 8.0:
        raise ValueError('Need at least 8 degrees of arm-pitch variation')
    if max(_rotation_distance_deg(tool_poses[0], pose) for pose in tool_poses) < 10.0:
        raise ValueError('Arm rotation diversity is too small')

    rotation_camera_tool, translation_camera_tool = cv2.calibrateHandEye(
        [pose[:3, :3] for pose in tool_poses],
        [pose[:3, 3].reshape(3, 1) for pose in tool_poses],
        [pose[:3, :3] for pose in board_poses],
        [pose[:3, 3].reshape(3, 1) for pose in board_poses],
        method=cv2.CALIB_HAND_EYE_TSAI)
    tool_T_camera = _transform(rotation_camera_tool, translation_camera_tool)
    fixed_board_poses = [tool @ tool_T_camera @ camera_board
                         for tool, camera_board in zip(tool_poses, board_poses)]
    mean_translation = np.mean([pose[:3, 3] for pose in fixed_board_poses], axis=0)
    mean_rotation = _average_rotation([pose[:3, :3] for pose in fixed_board_poses])
    translation_errors = [float(np.linalg.norm(pose[:3, 3] - mean_translation))
                          for pose in fixed_board_poses]
    rotation_errors = [float(_rotation_distance_deg(
        _transform(mean_rotation, (0, 0, 0)), pose)) for pose in fixed_board_poses]
    translation_rms = math.sqrt(float(np.mean(np.square(translation_errors))))
    rotation_rms = math.sqrt(float(np.mean(np.square(rotation_errors))))
    accepted = (translation_rms <= 0.015 and max(translation_errors) <= 0.03
                and rotation_rms <= 5.0 and max(rotation_errors) <= 10.0)
    result = {
        'tool_T_camera': {
            'translation_m': [float(value) for value in tool_T_camera[:3, 3]],
            'quaternion_xyzw': _rotation_to_quaternion(tool_T_camera[:3, :3]),
        },
        'fixed_board_in_arm_base_mean_m': [float(value) for value in mean_translation],
        'quality': {
            'accepted': bool(accepted), 'sample_count': len(samples),
            'translation_rms_m': translation_rms,
            'translation_max_m': max(translation_errors),
            'rotation_rms_deg': rotation_rms,
            'rotation_max_deg': max(rotation_errors),
            'checkerboard_order_flip_sample_indices': flipped_sample_indices,
            'max_checkerboard_reprojection_rms_px': max(
                sample['reprojection_rms_px'] for sample in samples),
        },
    }
    if not accepted:
        raise ValueError('hand-eye residual too high: ' + json.dumps(result['quality']))
    return result


class HandEyeCalibration(Node):
    """Capture/solve services for a fixed board; never commands robot motion."""

    def __init__(self):
        super().__init__('arm_camera_hand_eye_calibration')
        self.declare_parameter('image_topic', '/rear_camera/image_raw')
        self.declare_parameter('camera_info_topic', '/rear_camera/camera_info')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('capture_service', '/grasp/calibration/capture')
        self.declare_parameter('solve_service', '/grasp/calibration/solve')
        self.declare_parameter('status_topic', '/grasp/calibration/status')
        self.declare_parameter('pattern_columns', 9)
        self.declare_parameter('pattern_rows', 6)
        self.declare_parameter('square_size_m', 0.025)
        self.declare_parameter('output_path', '/tmp/rear_camera_hand_eye.json')
        self.declare_parameter('samples_path', '')
        self.declare_parameter('resume_samples', False)
        self.declare_parameter('max_joint_image_skew_sec', 0.25)
        self.declare_parameter('max_candidate_age_sec', 2.5)
        self.declare_parameter('detection_rate_hz', 0.5)
        detection_rate = float(self.get_parameter('detection_rate_hz').value)
        if not math.isfinite(detection_rate) or detection_rate <= 0:
            raise ValueError('detection_rate_hz must be positive and finite')
        self.detection_period = 1.0 / detection_rate
        self.camera_info = None
        self.joints = deque(maxlen=100)
        self.samples = []
        self.latest_candidate = None
        self.joint5_reference = None
        if bool(self.get_parameter('resume_samples').value):
            self.load_samples()
        else:
            self.save_samples()
        self.last_detection_at = 0.0
        self.input_counts = {'camera_info': 0, 'joint_states': 0, 'images': 0}
        self.status_pub = self.create_publisher(
            String, self.get_parameter('status_topic').value, 10)
        self.create_subscription(CameraInfo,
                                 self.get_parameter('camera_info_topic').value,
                                 self.on_camera_info, 5)
        self.create_subscription(JointState,
                                 self.get_parameter('joint_states_topic').value,
                                 self.on_joint_state, 10)
        self.create_subscription(Image, self.get_parameter('image_topic').value,
                                 self.on_image, 2)
        self.create_service(Trigger,
                            self.get_parameter('capture_service').value,
                            self.capture)
        self.create_service(Trigger,
                            self.get_parameter('solve_service').value,
                            self.solve)
        self.create_timer(5.0, self.report_input_counts)
        self.get_logger().info(
            'Hand-eye calibration ready; fixed board required; no motion publishers')

    def report_input_counts(self):
        self.get_logger().info(
            'Input callbacks: camera_info=%d joint_states=%d images=%d' % (
                self.input_counts['camera_info'], self.input_counts['joint_states'],
                self.input_counts['images']))

    def samples_checkpoint_path(self):
        configured = str(self.get_parameter('samples_path').value).strip()
        if configured:
            return os.path.expanduser(configured)
        output = os.path.expanduser(str(self.get_parameter('output_path').value))
        root, _extension = os.path.splitext(output)
        return root + '.samples.json'

    def save_samples(self):
        path = self.samples_checkpoint_path()
        directory = os.path.dirname(path) or '.'
        os.makedirs(directory, exist_ok=True)
        payload = {
            'schema': 'armpi_ros2_hand_eye_samples_v1',
            'samples': [{key: value for key, value in sample.items()
                         if key != 'received_at'} for sample in self.samples],
        }
        temporary = path + '.tmp'
        with open(temporary, 'w', encoding='utf-8') as stream:
            json.dump(payload, stream, indent=2, allow_nan=False)
            stream.write('\n')
        os.replace(temporary, path)

    def load_samples(self):
        path = self.samples_checkpoint_path()
        if not os.path.isfile(path):
            self.save_samples()
            return
        with open(path, 'r', encoding='utf-8') as stream:
            payload = json.load(stream)
        if payload.get('schema') != 'armpi_ros2_hand_eye_samples_v1':
            raise ValueError('Unsupported hand-eye sample checkpoint schema')
        samples = payload.get('samples')
        if not isinstance(samples, list) or len(samples) > 200:
            raise ValueError('Invalid hand-eye sample checkpoint')
        required = ('base_T_tool', 'camera_T_board', 'reprojection_rms_px',
                    'angles_deg', 'raw', 'camera_frame', 'stamp_ns',
                    'joint_image_skew_sec')
        for sample in samples:
            if not isinstance(sample, dict) or any(key not in sample for key in required):
                raise ValueError('Incomplete hand-eye sample checkpoint')
            if not all(name in sample['raw'] for name in JOINTS):
                raise ValueError('Checkpoint sample is missing arm feedback')
        self.samples = samples
        for sample in self.samples:
            sample['received_at'] = 0.0
        if self.samples:
            self.joint5_reference = int(self.samples[0]['raw']['joint5'])
        self.get_logger().info('Loaded %d hand-eye samples from %s' %
                               (len(self.samples), path))

    @staticmethod
    def stamp_ns(header):
        return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)

    def report(self, valid, reason, **extra):
        payload = {'valid': bool(valid), 'reason': str(reason),
                   'captured_samples': len(self.samples), **extra}
        message = String()
        message.data = json.dumps(payload, separators=(',', ':'), allow_nan=False)
        self.status_pub.publish(message)

    def on_camera_info(self, message):
        self.camera_info = message
        self.input_counts['camera_info'] += 1

    def on_joint_state(self, message):
        try:
            raw = joint_state_to_raw(message)
            stamp = self.stamp_ns(message.header)
            if stamp <= 0:
                return
            self.joints.append({'stamp_ns': stamp, 'raw': raw,
                                'received_at': time.monotonic()})
            self.input_counts['joint_states'] += 1
        except (ValueError, TypeError, OverflowError):
            return

    def on_image(self, message):
        self.input_counts['images'] += 1
        now = time.monotonic()
        if now - self.last_detection_at < self.detection_period:
            return
        self.last_detection_at = now
        if self.camera_info is None:
            self.report(False, 'waiting_for_camera_info')
            return
        if not self.joints:
            self.report(False, 'waiting_for_joint_states')
            return
        try:
            if (message.header.frame_id != self.camera_info.header.frame_id
                    or not message.header.frame_id):
                raise ValueError('camera_frame_id_mismatch')
            image_stamp = self.stamp_ns(message.header)
            nearest = min(self.joints,
                          key=lambda sample: abs(sample['stamp_ns'] - image_stamp))
            skew = abs(nearest['stamp_ns'] - image_stamp) * 1e-9
            if skew > float(self.get_parameter('max_joint_image_skew_sec').value):
                raise ValueError('joint_image_timestamp_skew: %.3f s' % skew)
            if time.monotonic() - nearest['received_at'] > 0.5:
                raise ValueError('stale_joint_feedback')
            self.report(False, 'processing_checkerboard',
                        joint_image_skew_sec=round(skew, 3))
            frame = image_msg_to_bgr(message)
            pattern = (int(self.get_parameter('pattern_columns').value),
                       int(self.get_parameter('pattern_rows').value))
            board_pose, reprojection_rms = checkerboard_pose(
                frame, self.camera_info, pattern,
                float(self.get_parameter('square_size_m').value))
            arm_pose = forward_tool_pose_raw(nearest['raw'])
            base_T_tool = _transform(arm_pose['rotation_matrix'],
                                     arm_pose['translation_m'])
            self.latest_candidate = {
                'base_T_tool': base_T_tool.tolist(),
                'camera_T_board': board_pose.tolist(),
                'reprojection_rms_px': reprojection_rms,
                'angles_deg': arm_pose['angles_deg'],
                'raw': dict(nearest['raw']),
                'camera_frame': message.header.frame_id,
                'stamp_ns': image_stamp,
                'received_at': time.monotonic(),
                'joint_image_skew_sec': skew,
            }
            self.report(True, 'checkerboard_visible',
                        reprojection_rms_px=round(reprojection_rms, 3),
                        joint_image_skew_sec=round(skew, 3))
        except Exception as error:
            reason = str(error)
            self.report(False, reason)

    def joint_pose_stable(self):
        recent = list(self.joints)[-4:]
        if len(recent) < 3 or time.monotonic() - recent[-1]['received_at'] > 0.5:
            return False, 'need three fresh joint states'
        for name in JOINTS:
            values = [sample['raw'][name] for sample in recent]
            if (max(values) - min(values)) * TICK_RAD > math.radians(1.0):
                return False, 'arm is moving; hold still before capture'
        return True, 'stable'

    def capture(self, _request, response):
        candidate = self.latest_candidate
        if candidate is None:
            response.success = False
            response.message = 'No valid synchronized checkerboard and arm pose in view'
            return response
        age = time.monotonic() - candidate['received_at']
        if age > float(self.get_parameter('max_candidate_age_sec').value):
            response.success = False
            response.message = 'Checkerboard observation is stale; keep it in view and retry'
            return response
        stable, reason = self.joint_pose_stable()
        if not stable:
            response.success = False
            response.message = reason
            return response
        current_raw = self.joints[-1]['raw']
        if any(abs(candidate['raw'][name] - current_raw[name]) > 3
               for name in JOINTS):
            response.success = False
            response.message = 'Latest valid checkerboard observation belongs to a different arm pose'
            return response
        raw5 = candidate['raw']['joint5']
        if self.joint5_reference is None:
            self.joint5_reference = raw5
        elif abs(raw5 - self.joint5_reference) > 8:
            response.success = False
            response.message = 'joint5 changed; restore its original wrist-roll position'
            return response
        if self.samples:
            previous = np.asarray(self.samples[-1]['base_T_tool'], dtype=np.float64)
            current = np.asarray(candidate['base_T_tool'], dtype=np.float64)
            translation_delta = float(np.linalg.norm(previous[:3, 3] - current[:3, 3]))
            rotation_delta = _rotation_distance_deg(previous, current)
            if translation_delta < 0.01 and rotation_delta < 4.0:
                response.success = False
                response.message = 'Pose too similar to last capture; vary yaw and arm pitch'
                return response
        self.samples.append(candidate)
        try:
            self.save_samples()
        except (OSError, TypeError, ValueError) as error:
            self.samples.pop()
            response.success = False
            response.message = 'Sample checkpoint failed: %s' % error
            self.report(False, 'sample_checkpoint_failed', error=str(error))
            return response
        self.report(True, 'pose_captured', pose_index=len(self.samples),
                    wrist_roll_raw=raw5,
                    reprojection_rms_px=round(candidate['reprojection_rms_px'], 3))
        response.success = True
        response.message = ('Captured pose %d; keep the checkerboard fixed and vary arm yaw/pitch'
                            % len(self.samples))
        return response

    def solve(self, _request, response):
        try:
            pattern = (int(self.get_parameter('pattern_columns').value),
                       int(self.get_parameter('pattern_rows').value))
            square_m = float(self.get_parameter('square_size_m').value)
            result = solve_hand_eye(self.samples, pattern, square_m)
            result.update({
                'schema': 'armpi_ros2_hand_eye_v1',
                'arm_base_frame': 'arm_base',
                'tool_frame': 'ik_tool',
                'camera_frame': (self.samples[-1]['camera_frame'] if self.samples else ''),
                'joint5_raw_reference': self.joint5_reference,
                'board_pattern_inner_corners': [
                    int(self.get_parameter('pattern_columns').value),
                    int(self.get_parameter('pattern_rows').value)],
                'board_square_size_m': float(self.get_parameter('square_size_m').value),
            })
            path = os.path.expanduser(str(self.get_parameter('output_path').value))
            temporary = path + '.tmp'
            with open(temporary, 'w', encoding='utf-8') as stream:
                json.dump(result, stream, indent=2, allow_nan=False)
                stream.write('\n')
            os.replace(temporary, path)
            self.report(True, 'hand_eye_saved', output_path=path,
                        quality=result['quality'])
            response.success = True
            response.message = ('Hand-eye accepted: translation RMS %.1f mm, rotation RMS %.2f deg; saved %s'
                                % (1000.0 * result['quality']['translation_rms_m'],
                                   result['quality']['rotation_rms_deg'], path))
        except Exception as error:
            self.report(False, str(error))
            response.success = False
            response.message = str(error)
        return response


def main(args=None):
    # A duplicate calibration node performs expensive checkerboard detection
    # on the same camera stream and can starve the video/robot processes.
    # flock is released by the kernel on normal exit, crash, or SIGKILL.
    import fcntl
    import sys
    lock = open('/tmp/armpi_hand_eye_calibration.lock', 'w', encoding='utf-8')
    try:
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        print('Another hand-eye calibration node is already running', file=sys.stderr)
        lock.close()
        return
    rclpy.init(args=args)
    node = HandEyeCalibration()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        lock.close()


if __name__ == '__main__':
    main()
