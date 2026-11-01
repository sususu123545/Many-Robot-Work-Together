import math

import cv2
import numpy as np
import pytest
from sensor_msgs.msg import CameraInfo, Image

from armpi_ros2.red_ball_detector import (
    find_red_sphere, image_msg_to_bgr, sphere_center_in_camera)


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
