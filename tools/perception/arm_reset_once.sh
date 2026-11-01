#!/usr/bin/env bash
# One-shot estop reset for the arm controller, from a sidecar that shares the
# core container's IPC namespace.
#
# WHY THIS IS SAFE: ArmNode.reset() revokes BOTH motion gates and invalidates
# every queued command ("Reset NEVER grants movement"), so clearing a latch can
# never make the arm move. The only thing reset buys is a fresh READY window,
# which arm_camera_tf requires before it will publish arm_base->rear_camera.
#
# The service itself refuses while a poll is in flight or feedback is stale, so
# this retries against /arm_controller/status rather than hammering the bus.
set +u
source /opt/armpi_ros2/install/setup.bash

echo "@@STATUS_BEFORE@@"
timeout 20 ros2 topic echo /arm_controller/status --once 2>&1 | head -30

# Wait for an actually resettable window: fresh feedback, reset_eligible, and
# no poll in flight. Status runs at 2 Hz, so ~30 samples is 15 s.
echo "@@RESET@@"
python3 - <<'PY'
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

rclpy.init()
node = Node('arm_reset_once')
latest = {}
node.create_subscription(String, '/arm_controller/status',
                         lambda m: latest.update(json.loads(m.data)), 10)
client = node.create_client(Trigger, '/arm_controller/reset_estop')
if not client.wait_for_service(timeout_sec=10.0):
    print('NO_SERVICE')
    raise SystemExit(3)
deadline = time.monotonic() + 20.0
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.2)
    if latest.get('reset_eligible') and not latest.get('poll_pending'):
        break
print('gate_before: estop=%s health=%s eligible=%s' % (
    latest.get('estop'), latest.get('health_state'), latest.get('reset_eligible')))
future = client.call_async(Trigger.Request())
rclpy.spin_until_future_complete(node, future, timeout_sec=15.0)
response = future.result()
print('reset_ok=%s message=%s' % (
    getattr(response, 'success', None), getattr(response, 'message', None)))
# Read one more status so the caller sees the post-reset gate, not the prompt.
# The FULL json is printed on purpose: an earlier version summarised it with a
# guessed field name and reported `arm_motion=None` forever, which hid the one
# value that actually matters here.
latest.clear()
deadline = time.monotonic() + 10.0
while time.monotonic() < deadline and not latest:
    rclpy.spin_once(node, timeout_sec=0.2)
print('status_after: %s' % json.dumps(latest, sort_keys=True))
node.destroy_node()
rclpy.shutdown()
PY
echo "@@DONE@@"
