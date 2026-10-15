"""Native ROS2 callback tests using ONLY a fake board (never open a device).

These run inside the ARM64 ROS image. Non-ROS local Python skips this module.
"""
import json
import time
import unittest
from unittest.mock import Mock, patch
from armpi_ros2.servo_protocol import JOINTS, to_radians

try:
    import rclpy
except ImportError:
    rclpy = None

if rclpy is not None:
    from rclpy.context import Context
    from rclpy.parameter import Parameter
    from std_msgs.msg import Bool
    from std_srvs.srv import Trigger
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from armpi_ros2.arm_node import ArmNode


class FakeBoard:
    def __init__(self, *_args):
        self.lock = __import__('threading').RLock()
        self.moves = []
        self.stops = 0
        self.fail_temperature = None
        self.hot = None
        self.fail_stop = False
        self.receiver_error = None

    def limits(self, ident):return next(list(v[3:]) for v in JOINTS.values() if v[0] == ident)
    def position(self, ident):return 86 if ident == 1 else 500
    def temperature(self, ident):
        if ident == self.fail_temperature:raise TimeoutError('injected read timeout')
        return 65 if ident == self.hot else 31
    def diagnostics(self):return {'receiver_error': self.receiver_error, 'counters': {}}
    def move(self, duration, targets):self.moves.append((duration, targets))
    def stop(self):
        self.stops += 1
        if self.fail_stop:raise OSError('injected stop failure')
    def close(self):pass


@unittest.skipUnless(rclpy, 'ROS2 callback tests run in the isolated ARM64 image')
class ArmCallbackTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.context = Context()
        # Separate from robot domain 42 even during a container image build.
        rclpy.init(context=cls.context, domain_id=199)

    @classmethod
    def tearDownClass(cls):cls.context.shutdown()

    def setUp(self):
        with patch('armpi_ros2.arm_node.ServoBoard', FakeBoard):
            self.node = ArmNode(context=self.context,
                                parameter_overrides=[Parameter('dry_run', value=False)])
        self.node.status_pub = Mock()
        self.node.joint_pub = Mock()

    def tearDown(self):self.node.destroy_node()

    def healthy(self, seconds=10):
        now = time.monotonic()
        for i in range(int(seconds*2)+1):
            t = now-seconds+i*.5
            samples = {n: {'raw': self.node.board.position(v[0]), 'temperature':31, 'time':t}
                       for n,v in JOINTS.items()}
            self.node.health.success(samples, t)

    def reset(self):return self.node.reset(Trigger.Request(), Trigger.Response())

    def grant(self, **values):
        return self.node.set_parameters_atomically([Parameter(k, value=v) for k,v in values.items()])

    def command(self, name, raw):
        m = JointTrajectory();m.header.stamp = self.node.get_clock().now().to_msg()
        m.joint_names = [name];p = JointTrajectoryPoint();p.positions = [to_radians(name, raw)]
        p.time_from_start.nanosec = 600000000;m.points = [p]
        return m

    def test_startup_and_one_cycle_cannot_grant_or_reset(self):
        self.assertEqual(self.node.board.moves, [])
        self.assertFalse(self.grant(enable_gripper_motion=True).successful)
        self.healthy(0)
        self.assertFalse(self.reset().success)
        self.assertFalse(self.grant(enable_arm_motion=True).successful)

    def test_parameter_commit_updates_only_gripper_permission(self):
        self.healthy();self.assertTrue(self.reset().success)
        self.assertTrue(self.grant(enable_gripper_motion=True).successful)
        self.assertTrue(self.node.gripper_enabled);self.assertFalse(self.node.enabled)
        self.node.on_command(self.command('joint1', 505))
        self.assertEqual(self.node.board.moves, [])
        self.node.on_gripper(self.command('r_joint', 90))
        self.assertEqual(len(self.node.board.moves), 1)
        self.assertAlmostEqual(self.node.board.moves[0][0], .6)
        self.assertEqual(self.node.board.moves[0][1], [(1,90)])
        stops = self.node.board.stops
        self.assertTrue(self.grant(enable_gripper_motion=False).successful)
        self.assertFalse(self.node.gripper_enabled)
        self.assertGreater(self.node.board.stops, stops)

    def test_arm_permission_cannot_bypass_gripper_gate_on_other_topic(self):
        self.healthy();self.assertTrue(self.reset().success)
        self.assertTrue(self.grant(enable_arm_motion=True).successful)
        self.node.on_command(self.command('r_joint', 90))
        self.assertEqual(self.node.board.moves, [])

    def test_reset_revokes_both_permissions_and_does_not_move(self):
        self.healthy();self.assertTrue(self.reset().success)
        self.assertTrue(self.grant(enable_arm_motion=True, enable_gripper_motion=True).successful)
        self.assertTrue(self.reset().success)
        self.assertFalse(self.node.enabled);self.assertFalse(self.node.gripper_enabled)
        self.assertFalse(self.node.get_parameter('enable_gripper_motion').value)
        self.assertEqual(self.node.board.moves, [])

    def test_failed_stop_cannot_clear_latch(self):
        self.healthy();self.node.board.fail_stop = True
        self.assertFalse(self.reset().success)
        self.assertTrue(self.node.health.estop)
        self.assertEqual(self.node.health.last_fault['kind'], 'stop_io')

    def test_partial_poll_does_not_publish_true_feedback_or_new_joint_state(self):
        self.node.board.fail_temperature = 5
        self.node.poll()
        status = json.loads(self.node.status_pub.publish.call_args[0][0].data)
        self.assertFalse(status['feedback_real']);self.assertFalse(status['feedback_complete'])
        self.assertTrue(status['estop']);self.assertEqual(status['health_state'], 'READ_ERROR')
        self.assertEqual(status['fault_count'], 1)
        self.node.joint_pub.publish.assert_not_called()

    def test_overtemperature_immediately_stops_and_latches(self):
        self.node.board.hot = 6
        self.node.poll()
        self.assertTrue(self.node.health.estop)
        self.assertEqual(self.node.health.last_fault['kind'], 'overtemperature')
        self.assertGreater(self.node.board.stops, 0)
        self.node.joint_pub.publish.assert_not_called()

    def test_false_estop_message_cannot_clear_manual_latch(self):
        self.healthy();self.assertTrue(self.reset().success)
        self.node.on_estop(Bool(data=True));self.node.on_estop(Bool(data=False))
        self.assertTrue(self.node.health.manual_estop)

    def test_stop_rejects_buffered_old_command(self):
        self.healthy();self.assertTrue(self.reset().success)
        self.assertTrue(self.grant(enable_gripper_motion=True).successful)
        old = self.command('r_joint', 90);self.node.on_stop(None)
        self.node.on_gripper(old)
        self.assertEqual(self.node.board.moves, [])

    def test_safety_bounds_remain_read_only(self):
        self.assertFalse(self.grant(max_temperature=80).successful)
        self.assertEqual(self.node.temp_max, 65)

    def test_receiver_failure_blocks_reset_grant_and_new_command(self):
        self.healthy();self.assertTrue(self.reset().success)
        self.assertTrue(self.grant(enable_gripper_motion=True).successful)
        self.node.board.receiver_error = 'injected RX thread failure'
        self.node.on_gripper(self.command('r_joint', 90))
        self.assertEqual(self.node.board.moves, [])
        self.assertFalse(self.reset().success)
        self.assertFalse(self.grant(enable_gripper_motion=True).successful)
