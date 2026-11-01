import math
import threading
import time
from types import SimpleNamespace

import pytest

import rclpy
from geometry_msgs.msg import PointStamped, TransformStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster

from armpi_ros2.target_transform import GraspTargetTransform, transform_xyz


def make_transform(translation=(0.0, 0.0, 0.0), rotation=(0.0, 0.0, 0.0, 1.0)):
    xyz = SimpleNamespace(x=translation[0], y=translation[1], z=translation[2])
    quat = SimpleNamespace(x=rotation[0], y=rotation[1], z=rotation[2], w=rotation[3])
    return SimpleNamespace(transform=SimpleNamespace(translation=xyz, rotation=quat))


def test_identity_transform_preserves_camera_point():
    assert transform_xyz((0.1, -0.2, 0.3), make_transform()) == pytest.approx(
        (0.1, -0.2, 0.3))


def test_transform_rotates_and_translates_point():
    half = math.sqrt(0.5)
    transform = make_transform((1.0, 2.0, 3.0), (0.0, 0.0, half, half))
    assert transform_xyz((1.0, 0.0, 0.0), transform) == pytest.approx(
        (1.0, 3.0, 3.0))


def test_rejects_nonfinite_points():
    with pytest.raises(ValueError, match='non-finite'):
        transform_xyz((math.nan, 0.0, 0.0), make_transform())


def test_rejects_invalid_quaternion():
    with pytest.raises(ValueError, match='not normalized'):
        transform_xyz((0.0, 0.0, 1.0), make_transform(rotation=(0.0, 0.0, 0.0, 0.0)))


def test_ros_tf_chain_publishes_transformed_target():
    rclpy.init()
    transform_node = GraspTargetTransform(parameter_overrides=[
        Parameter('input_topic', Parameter.Type.STRING, '/test/grasp/target_camera'),
        Parameter('output_topic', Parameter.Type.STRING, '/test/grasp/target_base'),
        Parameter('status_topic', Parameter.Type.STRING, '/test/grasp/status'),
        Parameter('target_frame', Parameter.Type.STRING, 'test_base'),
    ])
    driver = Node('grasp_target_transform_test_driver')
    static_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)
    broadcaster = StaticTransformBroadcaster(driver, qos=static_qos)
    input_pub = driver.create_publisher(PointStamped,
                                        '/test/grasp/target_camera', 5)
    received = []
    ready = threading.Event()
    driver.create_subscription(PointStamped, '/test/grasp/target_base',
                               lambda msg: (received.append(msg), ready.set()), 5)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(transform_node)
    executor.add_node(driver)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        tf = TransformStamped()
        tf.header.frame_id = 'test_base'
        tf.child_frame_id = 'test_camera'
        tf.transform.translation.x = 1.0
        tf.transform.translation.y = 2.0
        tf.transform.translation.z = 3.0
        half = math.sqrt(0.5)
        tf.transform.rotation.z = half
        tf.transform.rotation.w = half
        broadcaster.sendTransform(tf)

        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and input_pub.get_subscription_count() == 0:
            time.sleep(0.02)
        assert input_pub.get_subscription_count() > 0
        while time.monotonic() < deadline and not ready.is_set():
            target = PointStamped()
            target.header.stamp = driver.get_clock().now().to_msg()
            target.header.frame_id = 'test_camera'
            target.point.x, target.point.y, target.point.z = 1.0, 0.0, 0.0
            input_pub.publish(target)
            ready.wait(0.05)
        assert ready.is_set(), 'TF transform did not produce /target_base'
        result = received[-1]
        assert result.header.frame_id == 'test_base'
        assert (result.point.x, result.point.y, result.point.z) == pytest.approx(
            (1.0, 3.0, 3.0), abs=1e-5)
    finally:
        executor.shutdown()
        spin_thread.join(timeout=1.0)
        transform_node.destroy_node()
        driver.destroy_node()
        rclpy.shutdown()
