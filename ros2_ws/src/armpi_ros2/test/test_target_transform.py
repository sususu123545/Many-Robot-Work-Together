import json
import math
import threading
import time
from types import SimpleNamespace

import pytest

import rclpy
from geometry_msgs.msg import PointStamped, TransformStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

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


def test_newer_point_uses_newest_tf_and_refuses_genuinely_stale_tf():
    """2 Hz arm feedback vs 8 Hz camera: the image stamp is usually NEWER than
    the newest arm_base->camera TF, so an exact-time lookup needs future
    extrapolation and tf2 refuses it. The node must fall back to the newest
    transform while it is still within the lag budget, and refuse beyond it.

    Both halves share one rclpy session: repeatedly init/shutdown a DDS context
    inside a single pytest process crashes the interpreter on this image.
    """
    rclpy.init()
    node = GraspTargetTransform(parameter_overrides=[
        Parameter('input_topic', Parameter.Type.STRING, '/test2/target_camera'),
        Parameter('output_topic', Parameter.Type.STRING, '/test2/target_base'),
        Parameter('status_topic', Parameter.Type.STRING, '/test2/status'),
        Parameter('target_frame', Parameter.Type.STRING, 'test_base2'),
        Parameter('max_tf_lag_sec', Parameter.Type.DOUBLE, 0.75),
    ])
    driver = Node('grasp_target_transform_lag_driver')
    broadcaster = TransformBroadcaster(driver)
    input_pub = driver.create_publisher(PointStamped, '/test2/target_camera', 5)
    received, statuses = [], []
    driver.create_subscription(PointStamped, '/test2/target_base',
                               lambda msg: received.append(msg), 5)
    driver.create_subscription(String, '/test2/status',
                               lambda msg: statuses.append(json.loads(msg.data)), 5)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor.add_node(driver)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    def publish_case(tf_age_sec, rounds=6):
        for _ in range(rounds):
            tf = TransformStamped()
            # Stamp arithmetic goes through Duration on purpose. `stamp.sec` is
            # an int32 field: `stamp.sec -= 0.3` leaves a float in the Python
            # object and the message serializer ABORTS the interpreter
            # (SIGABRT) instead of raising, which surfaces as pytest's opaque
            # "missing_result" with no failing test named.
            tf.header.stamp = (driver.get_clock().now()
                               - Duration(seconds=tf_age_sec)).to_msg()
            tf.header.frame_id = 'test_base2'
            tf.child_frame_id = 'test_camera2'
            tf.transform.translation.x = 1.0
            tf.transform.translation.y = 2.0
            tf.transform.translation.z = 3.0
            tf.transform.rotation.w = 1.0
            broadcaster.sendTransform(tf)
            target = PointStamped()
            target.header.stamp = driver.get_clock().now().to_msg()
            target.header.frame_id = 'test_camera2'
            target.point.x, target.point.y, target.point.z = 1.0, 0.0, 0.0
            input_pub.publish(target)
            time.sleep(0.05)

    try:
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and input_pub.get_subscription_count() == 0:
            time.sleep(0.02)
        assert input_pub.get_subscription_count() > 0

        publish_case(0.3)
        assert received, 'a point newer than the newest TF must still be transformed'
        result = received[-1]
        assert result.header.frame_id == 'test_base2'
        assert (result.point.x, result.point.y, result.point.z) == pytest.approx(
            (2.0, 2.0, 3.0), abs=1e-5)
        assert any(s['reason'] == 'transformed_with_latest_tf' for s in statuses)

        # Shrinking the budget below the real lag must refuse, not guess.
        node.set_parameters([Parameter('max_tf_lag_sec', value=0.0)])
        statuses.clear()
        accepted_before = len(received)
        publish_case(0.3)
        assert len(received) == accepted_before, 'stale arm pose must not yield a target'
        assert any(s['reason'] == 'tf_lag_out_of_range' for s in statuses)

        # The opposite skew: a TF stamped slightly AFTER the image. On the car
        # this happened ~8 ms in the other direction and was wrongly rejected,
        # which silently dropped good points whenever the 2 Hz arm feedback
        # landed just after the frame. Restore a sane budget and require that
        # the NEWER-arm direction is never reported as out of range.
        node.set_parameters([Parameter('max_tf_lag_sec', value=0.75)])
        statuses.clear()
        publish_case(-0.02)
        assert received, 'a TF newer than the point must still yield a target'
        assert not any(s['reason'] == 'tf_lag_out_of_range' for s in statuses), \
            'a TF that is only slightly newer than the image must not be refused'
    finally:
        executor.shutdown()
        thread.join(timeout=1.0)
        node.destroy_node()
        driver.destroy_node()
        rclpy.shutdown()
