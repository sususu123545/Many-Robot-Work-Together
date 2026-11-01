import math

import cv2
import numpy as np
import pytest
from sensor_msgs.msg import CameraInfo, Image

from armpi_ros2.red_ball_detector import (
    find_red_sphere, fuse_recent_detections, image_msg_to_bgr,
    sphere_center_in_camera)


def _calibrated_camera():
    msg = CameraInfo()
    msg.width = 640
    msg.height = 480
    msg.distortion_model = 'equidistant'
    msg.k = [541.286233208, 0.0, 308.972422909,
             0.0, 536.831652047, 220.80791832,
             0.0, 0.0, 1.0]
    msg.d = [-0.173265335147, 0.276973718043,
             -1.97373104458, 3.56886883326]
    return msg


def test_finds_single_synthetic_red_ball():
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.circle(image, (284, 260), 30, (0, 0, 255), -1)
    detection, count = find_red_sphere(image)
    assert count == 1
    assert detection is not None
    assert abs(detection['center_x'] - 284) <= 1
    assert abs(detection['center_y'] - 260) <= 1
    assert 29 <= detection['radius_px'] <= 31


def test_rejects_two_similarly_sized_red_targets():
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.circle(image, (200, 240), 30, (0, 0, 255), -1)
    cv2.circle(image, (420, 240), 29, (0, 0, 255), -1)
    detection, count = find_red_sphere(image)
    assert count == 2
    assert detection is None


def test_uses_fisheye_calibration_and_known_ball_diameter():
    point, angular_radius = sphere_center_in_camera(
        (308.972422909, 220.80791832), 30.0, _calibrated_camera(), 0.035)
    assert angular_radius > 0
    assert math.isclose(point[0], 0.0, abs_tol=1e-5)
    assert math.isclose(point[1], 0.0, abs_tol=1e-5)
    assert 0.28 < point[2] < 0.38


def test_rejects_invalid_radius():
    with pytest.raises(ValueError):
        sphere_center_in_camera((320, 240), 0.0, _calibrated_camera(), 0.035)


def test_decodes_rgb8_and_ignores_row_padding():
    msg = Image()
    msg.width = 2
    msg.height = 2
    msg.encoding = 'rgb8'
    msg.step = 8
    msg.data = bytes((255, 0, 0, 0, 255, 0, 9, 9,
                      0, 0, 255, 255, 255, 255, 8, 8))
    image = image_msg_to_bgr(msg)
    assert image.shape == (2, 2, 3)
    assert tuple(image[0, 0]) == (0, 0, 255)
    assert tuple(image[0, 1]) == (0, 255, 0)
    assert tuple(image[1, 0]) == (255, 0, 0)


def test_rejects_truncated_ros_image():
    msg = Image()
    msg.width = 2
    msg.height = 2
    msg.encoding = 'rgb8'
    msg.step = 6
    msg.data = bytes(11)
    with pytest.raises(ValueError, match='shorter'):
        image_msg_to_bgr(msg)


def _detection(x, y, radius):
    return {'center_x': x, 'center_y': y, 'radius_px': radius}


def test_fusion_returns_the_median_and_reports_the_spread():
    window = [_detection(292.0, 309.0, 32.0),
              _detection(293.0, 310.0, 32.4),
              _detection(292.5, 309.5, 32.2),
              _detection(291.5, 308.5, 31.8),
              _detection(292.2, 309.2, 32.1)]
    fusion = fuse_recent_detections(window)
    assert fusion['frames'] == 5
    assert fusion['center_x'] == pytest.approx(292.2)
    assert fusion['center_y'] == pytest.approx(309.2)
    assert fusion['radius_px'] == pytest.approx(32.1)
    assert fusion['center_spread_px'] == pytest.approx(1.5)
    assert fusion['radius_fraction'] == pytest.approx(0.6 / 32.1, abs=1e-6)


def test_one_shifted_frame_cannot_drag_the_fused_target():
    """A mean would be pulled by the outlier; the median must not move.

    This is the whole point of fusing: a single frame where the blob leaks onto
    something red used to become the published grasp target outright.
    """
    nominal = 32.0
    clean = [_detection(292.0, 309.0, nominal) for _ in range(5)]
    polluted = clean[:4] + [_detection(300.0, 320.0, nominal * 1.5)]
    assert fuse_recent_detections(clean)['radius_px'] == pytest.approx(nominal)
    assert fuse_recent_detections(polluted)['radius_px'] == pytest.approx(nominal)
    assert fuse_recent_detections(polluted)['center_x'] == pytest.approx(292.0)


def test_fusion_turns_pixel_jitter_into_a_smaller_range_error():
    """End-to-end justification, in millimetres, through the real projection.

    z is recovered from the apparent radius, so the single-frame path inherits
    every pixel of radius jitter multiplied by the range. Fusing first must land
    closer to the true range than either a jittered frame or the outlier.
    """
    camera = _calibrated_camera()
    true_radius_px = 32.0
    window = [_detection(292.0, 309.0, true_radius_px + delta)
              for delta in (-0.6, -0.3, 0.0, 0.3, 0.6)]
    true_range = sphere_center_in_camera(
        (292.0, 309.0), true_radius_px, camera, 0.035)[0][2]

    fused = fuse_recent_detections(window)
    fused_range = sphere_center_in_camera(
        (fused['center_x'], fused['center_y']), fused['radius_px'],
        camera, 0.035)[0][2]
    worst_frame_range = sphere_center_in_camera(
        (292.0, 309.0), true_radius_px + 0.6, camera, 0.035)[0][2]

    assert abs(fused_range - true_range) < 1e-3
    assert abs(worst_frame_range - true_range) > abs(fused_range - true_range)


def test_fusion_rejects_an_empty_window():
    with pytest.raises(ValueError, match='No detections'):
        fuse_recent_detections([])
