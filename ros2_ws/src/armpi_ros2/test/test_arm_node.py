"""Native ROS2 callback tests using ONLY a fake board (never open a device).

These run inside the ARM64 ROS image. Non-ROS local Python skips this module.
"""
import json
import time
import threading
import unittest
from contextlib import contextmanager
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
        self.torque_enabled = Mock(return_value=False)
        self.voltage_mv = Mock(return_value=7900)

    def limits(self, ident):return next(list(v[3:]) for v in JOINTS.values() if v[0] == ident)
    def offset(self, ident):return 0
    def position(self, ident):return 86 if ident == 1 else 500
    def temperature(self, ident):
        if ident == self.fail_temperature:raise TimeoutError('injected read timeout')
        return 65 if ident == self.hot else 31
    def diagnostics(self):return {'receiver_error': self.receiver_error, 'counters': {}}
    def move(self, duration, targets):self.moves.append((duration, targets))
    def stop(self, identifiers=None):
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

    def read_elbow(self):
        return self.node.read_elbow_diagnostics(Trigger.Request(), Trigger.Response())

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

    def test_elbow_diagnostic_reports_disabled_torque_without_changing_state(self):
        response = self.read_elbow()
        self.assertTrue(response.success)
        self.assertEqual(json.loads(response.message), {'joint': 'joint3', 'servo_id': 4,
            'torque_enabled': False, 'voltage_mv': 7900, 'errors': {}})
        self.node.board.torque_enabled.assert_called_once_with(4, attempts=1)
        self.node.board.voltage_mv.assert_called_once_with(4, attempts=1)
        self.assertTrue(self.node.health.estop)
        self.assertFalse(self.node.enabled)
        self.assertFalse(self.node.gripper_enabled)
        self.assertEqual(self.node.board.stops, 0)
        self.assertEqual(self.node.board.moves, [])

    def test_elbow_diagnostic_keeps_voltage_when_torque_read_fails(self):
        self.node.board.torque_enabled.side_effect = TimeoutError('injected torque read timeout')
        response = self.read_elbow()
        self.assertFalse(response.success)
        result = json.loads(response.message)
        self.assertIsNone(result['torque_enabled'])
        self.assertEqual(result['voltage_mv'], 7900)
        self.assertEqual(result['errors'], {'torque_enabled': 'injected torque read timeout'})
        self.assertIsNone(self.node.health.current_error)
        self.assertEqual(self.node.board.stops, 0)

    def test_elbow_diagnostic_refuses_with_motion_permission_or_active_point(self):
        self.healthy()
        self.assertTrue(self.reset().success)
        for permission in ('enable_arm_motion', 'enable_gripper_motion'):
            with self.subTest(permission=permission):
                self.assertTrue(self.grant(**{permission: True}).successful)
                self.assertFalse(self.read_elbow().success)
                self.assertTrue(self.grant(**{permission: False}).successful)
        self.node.busy_until = time.monotonic() + 3
        self.assertFalse(self.read_elbow().success)
        self.node.board.torque_enabled.assert_not_called()
        self.node.board.voltage_mv.assert_not_called()

    def test_elbow_diagnostic_refuses_with_a_busy_feedback_lock(self):
        self.node.feedback_cycle_lock = Mock()
        self.node.feedback_cycle_lock.acquire.return_value = False
        self.assertFalse(self.read_elbow().success)
        self.node.feedback_cycle_lock.release.assert_not_called()
        self.node.board.torque_enabled.assert_not_called()
        self.node.board.voltage_mv.assert_not_called()

    def test_estop_does_not_wait_for_the_elbow_reply(self):
        reading = threading.Event()
        finish_read = threading.Event()
        stopped = threading.Event()

        def wait_for_reply(*_args, **_kwargs):
            reading.set()
            finish_read.wait(2)
            return True

        def request_stop():
            self.node.on_estop(Bool(data=True))
            stopped.set()

        self.node.board.torque_enabled.side_effect = wait_for_reply
        diagnostic = threading.Thread(target=self.read_elbow)
        stop = threading.Thread(target=request_stop)
        diagnostic.start()
        try:
            self.assertTrue(reading.wait(1))
            stop.start()
            self.assertTrue(stopped.wait(.75))
            self.assertTrue(self.node.health.manual_estop)
        finally:
            finish_read.set()
            diagnostic.join(2)
            if stop.ident is not None:
                stop.join(2)

    def test_partial_poll_does_not_publish_true_feedback_or_new_joint_state(self):
        self.node.board.fail_temperature = 5
        self.node.poll()
        status = json.loads(self.node.status_pub.publish.call_args[0][0].data)
        self.assertFalse(status['feedback_real']);self.assertFalse(status['feedback_complete'])
        self.assertTrue(status['estop']);self.assertEqual(status['health_state'], 'READ_ERROR')
        # `estop` here is the startup lock, not a fault: one missed reply is
        # transient and must NOT be counted as a latched fault.
        self.assertEqual(status['fault_count'], 0)
        self.node.joint_pub.publish.assert_not_called()

    def test_single_read_miss_aborts_motion_without_latching_the_estop(self):
        self.healthy();self.assertTrue(self.reset().success)
        self.assertTrue(self.grant(enable_gripper_motion=True).successful)
        self.node.board.fail_temperature = 5
        self.node.poll()
        status = json.loads(self.node.status_pub.publish.call_args[0][0].data)
        self.assertFalse(status['estop']);self.assertEqual(status['fault_count'], 0)
        self.assertEqual(status['health_state'], 'READ_ERROR')
        # Transient is not permission: the unverified cycle must still refuse.
        self.node.on_gripper(self.command('r_joint', 90))
        self.assertEqual(self.node.board.moves, [])

    def test_repeated_read_misses_latch_the_estop(self):
        self.healthy();self.assertTrue(self.reset().success)
        self.node.board.fail_temperature = 5
        for _ in range(self.node.failure_threshold):
            self.node.poll()
        status = json.loads(self.node.status_pub.publish.call_args[0][0].data)
        self.assertTrue(status['estop']);self.assertEqual(status['fault_count'], 1)
        self.assertEqual(self.node.health.last_fault['kind'], 'communication')
        self.assertFalse(self.reset().success)

    def test_a_healthy_cycle_breaks_the_failure_run(self):
        # Clear the startup lock first, otherwise `estop` is True for that reason
        # and the assertion would say nothing about the fault latch.
        self.healthy();self.assertTrue(self.reset().success)
        self.node.board.fail_temperature = 5
        self.node.poll();self.node.poll()
        self.assertEqual(self.node.consecutive_failures, 2)
        self.node.board.fail_temperature = None
        self.node.poll()
        self.assertEqual(self.node.consecutive_failures, 0)
        self.node.board.fail_temperature = 5
        self.node.poll()
        self.assertFalse(self.node.health.estop)

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

    @contextmanager
    def pending_feedback(self, callback):
        """Hold the poll transaction and observe a callback entering its wait."""
        lock = threading.Lock()
        lock.acquire()
        waiting = threading.Event()

        class ObservedLock:
            def acquire(_self, timeout):
                waiting.set()
                return lock.acquire(timeout=timeout)
            def release(_self):
                lock.release()

        self.node.feedback_cycle_lock = ObservedLock()
        self.node.health.begin_cycle()
        worker = threading.Thread(target=callback)
        worker.start()
        try:
            self.assertTrue(waiting.wait(1), 'Callback did not wait for pending feedback')
            yield
        finally:
            lock.release()
            worker.join(2)
        self.assertFalse(worker.is_alive())

    def test_reset_waits_for_poll_and_checks_committed_feedback(self):
        self.healthy()
        self.assertTrue(self.node.health.can_reset(time.monotonic()))
        responses = []
        with self.pending_feedback(lambda: responses.append(self.reset())):
            self.assertTrue(self.node.health.estop)
            self.assertEqual(self.node.board.stops, 0)
            self.healthy(0)
        self.assertTrue(responses[0].success, responses[0].message)
        self.assertFalse(self.node.health.estop)
        self.assertFalse(self.node.enabled)
        self.assertFalse(self.node.gripper_enabled)
        self.assertEqual(self.node.board.moves, [])

    def test_new_estop_during_poll_wait_cancels_reset(self):
        self.healthy()
        responses = []
        with self.pending_feedback(lambda: responses.append(self.reset())):
            self.node.on_estop(Bool(data=True))
            self.healthy(0)
        self.assertFalse(responses[0].success)
        self.assertIn('Stop or fault occurred', responses[0].message)
        self.assertTrue(self.node.health.manual_estop)
        self.assertEqual(self.node.board.moves, [])

    def test_failed_poll_during_wait_blocks_reset(self):
        self.healthy()
        responses = []
        with self.pending_feedback(lambda: responses.append(self.reset())):
            self.node.health.transient_fail('injected read timeout', time.monotonic())
        self.assertFalse(responses[0].success)
        self.assertIn('Arm feedback error: injected read timeout', responses[0].message)
        self.assertTrue(self.node.health.estop)
        self.assertEqual(self.node.board.stops, 0)
        self.assertEqual(self.node.board.moves, [])

    def test_command_waits_for_poll_and_uses_committed_feedback(self):
        self.healthy();self.assertTrue(self.reset().success)
        self.assertTrue(self.grant(enable_arm_motion=True).successful)
        command = self.command('joint2', 525)
        with self.pending_feedback(lambda: self.node.on_command(command)):
            self.assertEqual(self.node.board.moves, [])
            self.healthy(0)
        self.assertEqual(self.node.board.moves[0][1], [(5, 525)])
        self.assertTrue(self.node.last_command_result['accepted'])

    def test_stop_during_poll_wait_cancels_waiting_command(self):
        self.healthy();self.assertTrue(self.reset().success)
        self.assertTrue(self.grant(enable_arm_motion=True).successful)
        command = self.command('joint2', 525)
        with self.pending_feedback(lambda: self.node.on_command(command)):
            self.node.on_stop(None)
            self.healthy(0)
        self.assertEqual(self.node.board.moves, [])
        self.assertFalse(self.node.last_command_result['accepted'])
        self.assertIn('Stop occurred', self.node.last_command_result['reason'])

    def test_bad_gripper_does_not_block_arm_feedback_or_arm_command(self):
        self.healthy();self.assertTrue(self.reset().success)
        self.assertTrue(self.grant(enable_arm_motion=True).successful)
        position = self.node.board.position
        self.node.board.position = lambda ident: -398 if ident == 1 else position(ident)
        self.node.poll()
        status = json.loads(self.node.status_pub.publish.call_args[0][0].data)
        self.assertTrue(status['arm_feedback_complete'])
        self.assertFalse(status['gripper_feedback_complete'])
        self.assertEqual(len(self.node.joint_pub.publish.call_args[0][0].name), 5)
        self.node.on_command(self.command('joint2', 525))
        self.assertTrue(self.node.last_command_result['accepted'])
        self.node.on_gripper(self.command('r_joint', 100))
        self.assertFalse(self.node.last_command_result['accepted'])
