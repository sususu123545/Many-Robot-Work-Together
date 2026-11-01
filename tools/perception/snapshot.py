"""Grab one frame from the wrist camera and write it as a PNG.

Used to look at the hardware when the joint feedback alone cannot explain what
is happening -- for example a gripper that moves one way and is hard-blocked the
other way. Reuses the detector's decoder because the raw image arrives as rgb8
with row padding, and a naive cv2.imdecode of the buffer produces a garbled
image that looks like a different kind of failure entirely.

Writes to the path given as argv[1]; the wrapper mounts a host directory there
because the sidecar container is removed on exit.
"""
import sys
import time

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from armpi_ros2.red_ball_detector import image_msg_to_bgr

out_path = sys.argv[1] if len(sys.argv) > 1 else '/snapshots/frame.png'
topic = sys.argv[2] if len(sys.argv) > 2 else '/rear_camera/image_raw'

rclpy.init()
node = Node('frame_snapshot')
received = []


def on_image(msg):
    received.append(msg)


node.create_subscription(Image, topic, on_image, 2)
node.get_logger().info('waiting for %s' % topic)

end = time.monotonic() + 20.0
while time.monotonic() < end and not received:
    rclpy.spin_once(node, timeout_sec=0.2)

if not received:
    print('NO_FRAME within 20 s on %s' % topic)
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(2)

msg = received[0]
frame = image_msg_to_bgr(msg)
cv2.imwrite(out_path, frame)
print('WROTE %s  %dx%d  encoding=%s  frame_id=%s'
      % (out_path, msg.width, msg.height, msg.encoding, msg.header.frame_id))
node.destroy_node()
rclpy.shutdown()
