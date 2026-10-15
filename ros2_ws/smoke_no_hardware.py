"""Run inside the isolated ROS 2 test container only. Refuse real hardware mode."""
import json
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32MultiArray, String
from tf2_msgs.msg import TFMessage


def main():
    rclpy.init()
    node = Node('migration_smoke_probe')
    counts = {'odom': 0, 'encoder': 0, 'wheel': 0}
    state, frames, wheel_outputs = [], set(), []
    def on_odom(msg):
        assert msg.header.frame_id == 'odom' and msg.child_frame_id == 'base_link'
        assert msg.pose.pose.orientation.w == 1.0
        counts['odom'] += 1
    def on_encoder(msg):
        assert list(msg.data) == [0, 0, 0, 0]
        counts['encoder'] += 1
    def on_wheel(msg):
        wheel_outputs.append(list(msg.data))
        counts['wheel'] += 1
    node.create_subscription(Odometry, '/odom', on_odom, 10)
    node.create_subscription(Int32MultiArray, '/chassis_control/encoder_counts', on_encoder, 10)
    node.create_subscription(Int32MultiArray, '/chassis_control/wheel_command', on_wheel, 10)
    node.create_subscription(String, '/chassis_control/status', lambda msg: state.append(json.loads(msg.data)), 10)
    node.create_subscription(TFMessage, '/tf_static',
        lambda msg: frames.update(t.child_frame_id for t in msg.transforms),
        QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    pub = node.create_publisher(Twist, '/cmd_vel', 1)
    deadline = time.monotonic() + 8
    try:
        while not state and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        assert state, 'No chassis status'
        assert state[-1]['dry_run'] is True and state[-1]['enable_motion'] is False, 'Not a safe no-hardware container'
        command = Twist()
        command.linear.x = .06
        pub.publish(command)
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        assert counts['odom'] >= 20 and counts['encoder'] >= 20
        assert counts['wheel'] >= 20 and all(v == [0, 0, 0, 0] for v in wheel_outputs)
        assert {'imu_link', 'base_laser'} <= frames, 'Static TF not received'
        assert state[-1]['hardware_fault'] is False
        print(json.dumps({'success': True, 'dry_run': True, 'motion_disabled': True,
                          'samples': counts, 'static_frames': sorted(frames)}, indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
