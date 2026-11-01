#!/usr/bin/env bash
# Fast end-to-end probe of the passive grasp perception chain.
# Runs INSIDE a sidecar container that shares the core's IPC namespace.
# Moves nothing: grasp_vision.launch.py is documented as "never moves joints".
#
# WHY PYTHON INSTEAD OF `ros2 topic echo`: on this Pi the ROS 2 CLI needs ~10 s
# just to start, and its message-type resolution intermittently fails with
# "Could not determine the type for the passed topic" even while the publisher
# is alive and publishing. That false negative cost a whole debug round. A
# single rclpy node subscribes to everything at once, imports the types
# directly, and prints one consolidated snapshot.
#
# arm_camera_tf REFUSES to publish arm_base->rear_camera while the arm reports
# estop / non-READY / any fault, so run arm_reset_once.sh first if the chain
# stays empty.
#
# NOTE: no `set -u`: install/setup.bash expands COLCON_TRACE before defining it.
set +u
source /opt/armpi_ros2/install/setup.bash

echo "@@CALIB_FILE@@"
ls -l /opt/armpi-calibration/rear_camera_hand_eye.json 2>&1

ros2 launch armpi_ros2 grasp_vision.launch.py > /tmp/grasp.log 2>&1 &
LAUNCH_PID=$!

python3 - <<'PY'
import json
import time

import rclpy
from geometry_msgs.msg import PointStamped
import numpy as np
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
import tf2_ros

rclpy.init()
node = Node('grasp_probe')
latest = {}
for topic in ('/grasp/target_base', '/grasp/target_camera'):
    node.create_subscription(
        PointStamped, topic,
        lambda msg, t=topic: latest.__setitem__(t, msg), 10)
for topic in ('/grasp/red_ball/detection', '/grasp/camera_tf/status',
              '/grasp/transform/status'):
    node.create_subscription(
        String, topic,
        lambda msg, t=topic: latest.__setitem__(t, msg.data), 10)

buffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(buffer, node)


def point(msg):
    p = msg.point
    return '[%.4f, %.4f, %.4f]' % (p.x, p.y, p.z)


def wait_for(topic, seconds):
    """First sample of `topic`, or None. Each topic has its own deadline so one
    missing link does not starve the rest of the report."""
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if topic in latest:
            return latest[topic]
        rclpy.spin_once(node, timeout_sec=0.1)
    return None


def ensure(topic, seconds):
    """Like wait_for, but keeps spinning: the status topics publish on state
    CHANGE, not on a fixed timer, so a stable scene can legitimately produce no
    new message for many seconds. Reporting MISSING in that window was a false
    negative -- the value was only missing because nothing had changed."""
    deadline = time.monotonic() + seconds
    while topic not in latest and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    return latest.get(topic)


base = wait_for('/grasp/target_base', 30.0)
print('@@TARGET_BASE@@')
if isinstance(base, PointStamped):
    print('frame_id=%s stamp=%d.%09d' % (
        base.header.frame_id, base.header.stamp.sec,
        base.header.stamp.nanosec))
    print('point_base=%s' % point(base))
else:
    print('MISSING (no /grasp/target_base within 30 s)')
    status = latest.get('/grasp/transform/status')
    print('transform_status=%s' % (status if status else 'MISSING'))

cam = wait_for('/grasp/target_camera', 20.0)
print('@@TARGET_CAMERA@@')
if isinstance(cam, PointStamped):
    print('frame_id=%s stamp=%d.%09d' % (
        cam.header.frame_id, cam.header.stamp.sec,
        cam.header.stamp.nanosec))
    print('point_camera=%s' % point(cam))
else:
    print('MISSING')

print('@@DETECTION@@')
print(ensure('/grasp/red_ball/detection', 15.0) or 'MISSING')
print('@@CAMERA_TF_STATUS@@')
print(ensure('/grasp/camera_tf/status', 10.0) or 'MISSING')
print('@@TRANSFORM_STATUS@@')
print(ensure('/grasp/transform/status', 10.0) or 'MISSING')

print('@@TF_ARM_CAMERA@@')
translation = None
rotation = None
try:
    tf = buffer.lookup_transform('arm_base', 'rear_camera', Time())
    translation = tf.transform.translation
    rotation = tf.transform.rotation
    print('translation=[%.4f, %.4f, %.4f]' % (
        translation.x, translation.y, translation.z))
except Exception as error:                                    # noqa: BLE001
    print('LOOKUP_FAILED: %s' % error)

# Independent geometric cross-check. `target_base` was produced from an OLDER
# arm pose than the TF looked up just now, so the two only agree if the whole
# chain is self-consistent -- a sign or frame flip would blow the residual up
# by tens of millimetres. The arm is stationary during the probe, which is what
# makes this comparison meaningful at all.
print('@@CROSS_CHECK@@')
if translation is not None and isinstance(cam, PointStamped) and isinstance(base, PointStamped):
    x, y, z, w = rotation.x, rotation.y, rotation.z, rotation.w
    rot = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    camera_point = np.array([cam.point.x, cam.point.y, cam.point.z])
    base_point = np.array([base.point.x, base.point.y, base.point.z])
    predicted = rot.dot(camera_point) + np.array(
        [translation.x, translation.y, translation.z])
    residual = np.linalg.norm(predicted - base_point)
    print('predicted_base=[%.4f, %.4f, %.4f]' % tuple(predicted))
    print('reported_base =[%.4f, %.4f, %.4f]' % tuple(base_point))
    print('residual_mm=%.2f  (same ball, different messages: detection reports a '
          '35 mm diameter, so anything above a few mm means a frame/sign error)'
          % (residual * 1000.0))
else:
    print('SKIPPED (need target_base, target_camera and the TF)')
node.destroy_node()
rclpy.shutdown()
PY

echo "@@LAUNCH_LOG@@"
kill -INT $LAUNCH_PID 2>/dev/null
sleep 3
pkill -f 'grasp_vision.launch.py' 2>/dev/null
pkill -f armpi_ros2/red_ball_detector 2>/dev/null
pkill -f grasp_target_transform 2>/dev/null
pkill -f arm_camera_transform 2>/dev/null
tail -30 /tmp/grasp.log
echo "@@DONE@@"
