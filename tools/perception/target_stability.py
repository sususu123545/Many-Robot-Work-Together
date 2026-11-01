"""Measure how repeatable the grasp target is while the ball sits still.

WHY: a grasp needs a tolerance, and a tolerance needs a number. The single-frame
detector already showed ~6 mm of drift between runs, which came from the range
being recovered out of the apparent radius. This samples /grasp/target_base and
the detector status over a window and reports the actual spread, so the gripper
approach can be sized from evidence instead of from a guess.

Runs inside a sidecar that shares the core container's IPC namespace; the
wrapper script sets that up. Prints machine-greppable @@SECTION@@ markers.
"""
import json
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from std_msgs.msg import String

SECONDS = 20.0

rclpy.init()
node = Node('target_stability')
points = []
fusion_radius_fraction = []
fusion_center_spread = []
reasons = {}
arrivals = []


def on_point(msg):
    points.append((msg.point.x, msg.point.y, msg.point.z))
    arrivals.append(time.monotonic())


def on_status(msg):
    try:
        payload = json.loads(msg.data)
    except ValueError:
        return
    reasons[payload.get('reason')] = reasons.get(payload.get('reason'), 0) + 1
    fusion = payload.get('fusion')
    if fusion:
        fusion_radius_fraction.append(fusion['radius_fraction'])
        fusion_center_spread.append(fusion['center_spread_px'])


node.create_subscription(PointStamped, '/grasp/target_base', on_point, 10)
node.create_subscription(String, '/grasp/red_ball/detection', on_status, 10)

deadline = time.monotonic() + SECONDS
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)

print('@@SAMPLES@@')
print('target_base messages=%d  detection messages=%d  window=%.0fs'
      % (len(points), sum(reasons.values()), SECONDS))
if len(arrivals) > 1:
    gaps = np.diff(arrivals)
    print('rate_hz=%.2f  max_gap_s=%.3f' % (1.0 / np.mean(gaps), np.max(gaps)))

print('@@REASONS@@')
for reason, count in sorted(reasons.items(), key=lambda kv: -kv[1]):
    print('%-28s %d' % (reason, count))

print('@@SPREAD@@')
if len(points) >= 2:
    data = np.asarray(points)
    mean = data.mean(axis=0)
    std = data.std(axis=0)
    low, high = data.min(axis=0), data.max(axis=0)
    print('mean_mm=[%.2f, %.2f, %.2f]' % tuple(mean * 1000))
    print('std_mm =[%.2f, %.2f, %.2f]' % tuple(std * 1000))
    print('range_mm=[%.2f, %.2f, %.2f]' % tuple((high - low) * 1000))
    # Worst-case distance between two accepted targets: this is the number the
    # gripper opening has to absorb.
    worst = 0.0
    for index in range(len(data)):
        delta = np.linalg.norm(data - data[index], axis=1)
        worst = max(worst, float(delta.max()))
    print('max_pairwise_mm=%.2f' % (worst * 1000))
else:
    print('NOT_ENOUGH_SAMPLES')

print('@@FUSION_WINDOW@@')
if fusion_radius_fraction:
    print('radius_fraction mean=%.4f max=%.4f  (gate allows 0.08)'
          % (float(np.mean(fusion_radius_fraction)),
             float(np.max(fusion_radius_fraction))))
    print('center_spread_px mean=%.2f max=%.2f  (gate allows 3.5)'
          % (float(np.mean(fusion_center_spread)),
             float(np.max(fusion_center_spread))))
else:
    print('NO_STABLE_FRAMES')

node.destroy_node()
rclpy.shutdown()
