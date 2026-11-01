"""ROS 2 red-ball perception and monocular range estimate for the arm camera.

This node is read-only with respect to the robot: it subscribes to camera
topics and publishes detections, an annotated image, and a camera-frame point.
It never publishes arm, gripper, chassis, reset, or safety commands.
"""
import json
import math
from collections import deque

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String


def _unit_ray(normalized_xy):
    ray = np.array([float(normalized_xy[0]), float(normalized_xy[1]), 1.0],
                   dtype=np.float64)
    norm = float(np.linalg.norm(ray))
    if not math.isfinite(norm) or norm <= 0:
        raise ValueError('Invalid undistorted camera ray')
    return ray / norm


def _undistort_pixels(points, camera_info):
    k = np.asarray(camera_info.k, dtype=np.float64).reshape(3, 3)
    d = np.asarray(camera_info.d, dtype=np.float64)
    pixels = np.asarray(points, dtype=np.float64).reshape(-1, 1, 2)
    model = camera_info.distortion_model.lower()
    if model == 'equidistant':
        if d.size != 4:
            raise ValueError('Equidistant calibration requires four distortion terms')
        normalized = cv2.fisheye.undistortPoints(pixels, k, d.reshape(4, 1))
    elif model in ('plumb_bob', 'rational_polynomial'):
        normalized = cv2.undistortPoints(pixels, k, d)
    else:
        raise ValueError('Unsupported camera distortion model: ' + model)
    return normalized.reshape(-1, 2)


def image_msg_to_bgr(message):
    """Decode a tightly or row-padded ROS Image into OpenCV BGR8."""
    width, height, step = int(message.width), int(message.height), int(message.step)
    encoding = str(message.encoding).lower()
    channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4,
                'mono8': 1, '8uc1': 1}.get(encoding)
    if width <= 0 or height <= 0 or channels is None:
        raise ValueError('Unsupported image dimensions or encoding: %s' % encoding)
    row_bytes = width * channels
    if step < row_bytes or len(message.data) < step * height:
        raise ValueError('Image data is shorter than its declared step and height')
    rows = np.frombuffer(message.data, dtype=np.uint8,
                         count=step * height).reshape(height, step)
    pixels = rows[:, :row_bytes].reshape(height, width, channels)
    if encoding == 'rgb8':
        return cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)
    if encoding == 'rgba8':
        return cv2.cvtColor(pixels, cv2.COLOR_RGBA2BGR)
    if encoding == 'bgra8':
        return cv2.cvtColor(pixels, cv2.COLOR_BGRA2BGR)
    if channels == 1:
        return cv2.cvtColor(pixels.reshape(height, width), cv2.COLOR_GRAY2BGR)
    return pixels.copy()


def sphere_center_in_camera(center_xy, radius_px, camera_info, diameter_m):
    """Estimate a known-diameter sphere center in the raw camera frame.

    Uses the calibrated raw-image distortion model and the sphere's apparent
    angular radius. Returns x-right, y-down, z-forward in meters.
    """
    if (not math.isfinite(radius_px) or radius_px <= 0
            or not math.isfinite(diameter_m) or diameter_m <= 0):
        raise ValueError('Radius and diameter must be positive and finite')
    cx, cy = map(float, center_xy)
    angles = np.linspace(0.0, 2.0 * math.pi, 16, endpoint=False)
    boundary = [(cx + radius_px * math.cos(a), cy + radius_px * math.sin(a))
                for a in angles]
    normalized = _undistort_pixels([(cx, cy), *boundary], camera_info)
    center_ray = _unit_ray(normalized[0])
    boundary_rays = np.asarray([_unit_ray(p) for p in normalized[1:]])
    dots = np.clip(boundary_rays @ center_ray, -1.0, 1.0)
    angular_radius = float(np.median(np.arccos(dots)))
    if not 1e-4 < angular_radius < math.radians(70):
        raise ValueError('Sphere angular radius is outside the supported range')
    distance = (diameter_m / 2.0) / math.sin(angular_radius)
    point = center_ray * distance
    if not np.all(np.isfinite(point)) or not 0.03 <= distance <= 2.0:
        raise ValueError('Estimated ball range is outside 0.03..2.0 m')
    return tuple(float(v) for v in point), angular_radius


def find_red_sphere(image_bgr, min_radius_px=6.0, max_radius_px=180.0,
                    min_area_px=100.0):
    """Return the largest unambiguous red, approximately circular image blob."""
    if image_bgr is None or image_bgr.ndim != 3 or image_bgr.shape[2] != 3:
        raise ValueError('Expected a three-channel BGR image')
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    low_red = cv2.inRange(hsv, np.array((0, 75, 45), np.uint8),
                          np.array((12, 255, 255), np.uint8))
    high_red = cv2.inRange(hsv, np.array((168, 75, 45), np.uint8),
                           np.array((180, 255, 255), np.uint8))
    mask = cv2.bitwise_or(low_red, high_red)
    kernel = np.ones((3, 3), dtype=np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidates = []
    for contour in contours:
        area = float(cv2.contourArea(contour))
        if area < min_area_px:
            continue
        (x, y), radius = cv2.minEnclosingCircle(contour)
        if not min_radius_px <= radius <= max_radius_px:
            continue
        perimeter = float(cv2.arcLength(contour, True))
        if perimeter <= 0:
            continue
        circularity = min(1.0, 4.0 * math.pi * area / (perimeter * perimeter))
        _, _, width, height = cv2.boundingRect(contour)
        aspect = width / float(max(height, 1))
        if circularity < 0.48 or not 0.58 <= aspect <= 1.72:
            continue
        candidates.append({'center_x': float(x), 'center_y': float(y),
                           'radius_px': float(radius), 'area_px': area,
                           'circularity': circularity,
                           'bbox': [int(v) for v in cv2.boundingRect(contour)]})
    candidates.sort(key=lambda item: item['area_px'], reverse=True)
    if not candidates:
        return None, 0
    if len(candidates) > 1 and candidates[1]['area_px'] >= 0.72 * candidates[0]['area_px']:
        return None, len(candidates)
    result = candidates[0]
    result['confidence'] = min(1.0, result['circularity'] *
                               min(1.0, result['area_px'] / 1200.0))
    return result, len(candidates)


def fuse_recent_detections(recent):
    """Median of the stable window, instead of the newest single frame.

    WHY: the range z is recovered from the apparent radius -- sin(theta) = r/d --
    so one pixel of edge jitter at r ~ 32 px moves the target by about 3 % of the
    distance, i.e. ~8 mm at 285 mm. Publishing the latest frame therefore handed
    the arm a target that wandered by roughly a centimetre between runs even
    though the ball never moved. Fusing in PIXEL space (median centre + median
    radius, then one projection) is both cheaper and more correct than averaging
    the resulting 3-D points, because the projection is nonlinear in the radius.

    Median rather than mean on purpose: a single frame where the blob leaks onto
    a nearby red object would drag a mean and is simply out-voted by a median.

    The window is safe to average only because the stability gate has already
    rejected it when the spread is too wide, so this never smooths a moving ball.
    """
    if not recent:
        raise ValueError('No detections to fuse')
    xs = [item['center_x'] for item in recent]
    ys = [item['center_y'] for item in recent]
    radii = [item['radius_px'] for item in recent]
    return {
        'frames': len(recent),
        'center_x': float(np.median(xs)),
        'center_y': float(np.median(ys)),
        'radius_px': float(np.median(radii)),
        'center_spread_px': float(max(max(xs) - min(xs), max(ys) - min(ys))),
        'radius_fraction': float((max(radii) - min(radii))
                                 / max(float(np.mean(radii)), 1.0)),
    }


class RedBallDetector(Node):
    def __init__(self):
        super().__init__('red_ball_detector')
        self.declare_parameter('image_topic', '/rear_camera/image_raw')
        self.declare_parameter('camera_info_topic', '/rear_camera/camera_info')
        self.declare_parameter('ball_diameter_m', 0.035)
        self.declare_parameter('min_radius_px', 6.0)
        self.declare_parameter('max_radius_px', 180.0)
        self.declare_parameter('stability_frames', 5)
        self.declare_parameter('stability_center_px', 7.0)
        self.declare_parameter('stability_radius_fraction', 0.18)
        self.declare_parameter('annotated_topic', '/grasp/red_ball/annotated')
        self.camera_info = None
        self.history = deque(maxlen=max(2, int(self.get_parameter('stability_frames').value)))
        self.info_sub = self.create_subscription(
            CameraInfo, self.get_parameter('camera_info_topic').value,
            self.on_camera_info, 5)
        self.image_sub = self.create_subscription(
            Image, self.get_parameter('image_topic').value, self.on_image, 2)
        self.point_pub = self.create_publisher(PointStamped, '/grasp/target_camera', 5)
        self.status_pub = self.create_publisher(String, '/grasp/red_ball/detection', 5)
        self.annotated_pub = self.create_publisher(
            Image, self.get_parameter('annotated_topic').value, 2)
        self.get_logger().info('Read-only red-ball detector ready; no robot motion outputs')

    def on_camera_info(self, msg):
        self.camera_info = msg

    def _stable(self):
        count = int(self.get_parameter('stability_frames').value)
        if count < 2 or len(self.history) < count:
            return False
        recent = list(self.history)[-count:]
        xs = [v['center_x'] for v in recent]
        ys = [v['center_y'] for v in recent]
        rs = [v['radius_px'] for v in recent]
        center_limit = float(self.get_parameter('stability_center_px').value)
        radius_fraction = float(self.get_parameter('stability_radius_fraction').value)
        return (max(xs) - min(xs) <= center_limit
                and max(ys) - min(ys) <= center_limit
                and (max(rs) - min(rs)) / max(float(np.mean(rs)), 1.0) <= radius_fraction)

    def on_image(self, msg):
        reason = 'no_red_sphere'
        detection = None
        point = None
        count = 0
        fusion = None
        try:
            frame = image_msg_to_bgr(msg)
            detection, count = find_red_sphere(
                frame, float(self.get_parameter('min_radius_px').value),
                float(self.get_parameter('max_radius_px').value))
            if detection is None:
                self.history.clear()
                reason = 'ambiguous_or_no_candidate'
            else:
                self.history.append(detection)
                if not self._stable():
                    reason = 'target_not_stable'
                elif self.camera_info is None:
                    reason = 'waiting_for_camera_info'
                elif (self.camera_info.width != msg.width
                      or self.camera_info.height != msg.height):
                    reason = 'camera_info_size_mismatch'
                elif not msg.header.frame_id:
                    reason = 'missing_camera_frame_id'
                else:
                    window = max(int(self.get_parameter('stability_frames').value), 1)
                    fusion = fuse_recent_detections(list(self.history)[-window:])
                    point, angle = sphere_center_in_camera(
                        (fusion['center_x'], fusion['center_y']),
                        fusion['radius_px'], self.camera_info,
                        float(self.get_parameter('ball_diameter_m').value))
                    pmsg = PointStamped()
                    pmsg.header = msg.header
                    pmsg.point.x, pmsg.point.y, pmsg.point.z = point
                    self.point_pub.publish(pmsg)
                    detection['angular_radius_rad'] = angle
                    reason = 'stable_and_localized'
        except Exception as error:  # keep the node alive; report the frame failure
            self.history.clear()
            reason = 'processing_error: ' + str(error)
            frame = None
        payload = {
            'stamp_sec': int(msg.header.stamp.sec),
            'stamp_nanosec': int(msg.header.stamp.nanosec),
            'frame_id': msg.header.frame_id,
            'valid': point is not None,
            'reason': reason,
            'candidate_count': count,
            'ball_diameter_m': float(self.get_parameter('ball_diameter_m').value),
            'detection': detection,
            'fusion': fusion,
            'point_camera_m': list(point) if point is not None else None,
        }
        status = String()
        status.data = json.dumps(payload, separators=(',', ':'), allow_nan=False)
        self.status_pub.publish(status)
        if frame is not None:
            if detection is not None:
                x, y, radius = (round(detection['center_x']), round(detection['center_y']),
                                round(detection['radius_px']))
                cv2.circle(frame, (x, y), radius, (0, 255, 0) if point else (0, 190, 255), 2)
                cv2.circle(frame, (x, y), 3, (255, 255, 0), -1)
                cv2.putText(frame, reason, (8, 24), cv2.FONT_HERSHEY_SIMPLEX,
                            .55, (0, 255, 0) if point else (0, 190, 255), 2)
            out = Image()
            out.height, out.width = frame.shape[:2]
            out.encoding = 'bgr8'
            out.is_bigendian = 0
            out.step = int(frame.shape[1] * 3)
            out.data = frame.tobytes()
            out.header = msg.header
            self.annotated_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = RedBallDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
