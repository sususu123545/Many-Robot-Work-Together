import importlib.util
from pathlib import Path
import queue
import struct
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from armpi_ros2 import servo_protocol

ROOT = Path(__file__).resolve().parents[1]


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, ROOT/path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


with patch.dict(sys.modules, {'servo_protocol': servo_protocol}):
    ab = load('ab_sdk_tests', 'tools/arm_transport_readonly_ab.py')
    with patch.dict(sys.modules, {'arm_transport_readonly_ab': ab}):
        probe = load('sdk_probe_tests', 'tools/arm_sdk_readonly_compare.py')
with patch.dict(sys.modules, {'serial': SimpleNamespace()}):
    sdk = load('stock_sdk_tests', 'armpi_pro/src/ros_robot_controller/src/ros_robot_controller/ros_robot_controller_sdk.py')


class SDKReadOnlyTests(unittest.TestCase):
    def test_sdk_list_and_native_bytes_allowed(self):
        port = Mock(); port.write.side_effect = lambda b: len(b)
        guard = probe.GuardedPort(port)
        for data in (list(servo_protocol.packet(bytes([9, 1]))), servo_protocol.packet(bytes([5, 6]))):
            self.assertEqual(guard.write(data), 7)
        self.assertEqual(len(guard.tx), 2)

    def test_guard_rejects_motion_torque_broadcast_and_bad_crc(self):
        port = Mock(); guard = probe.GuardedPort(port)
        for payload in (bytes([1, 1]), bytes([3, 1]), bytes([12, 1]), bytes([9, 254])):
            with self.assertRaises(ValueError): guard.write(servo_protocol.packet(payload))
        data = bytearray(servo_protocol.packet(bytes([9, 1]))); data[-1] ^= 1
        with self.assertRaises(ValueError): guard.write(data)
        port.write.assert_not_called()

    def test_queue_reports_only_consumed_payload(self):
        q = probe.ObservedQueue(queue.Queue(maxsize=1))
        payload = struct.pack('<BBbB', 2, 9, 0, 29)
        q.put_nowait(payload)
        self.assertEqual(q.get(timeout=.01), payload)
        self.assertEqual(probe.validate_sdk_reply(q, 0, 2, 9, [29]), 29)
        with self.assertRaises(queue.Empty): q.get(timeout=.001)
        self.assertEqual(q.timeouts, 1)

    def test_sdk_mismatched_id_command_status_or_value_is_not_success(self):
        for payload, value in ((struct.pack('<BBbB', 3, 9, 0, 29), [29]),
                               (struct.pack('<BBbB', 2, 7, 0, 29), [29]),
                               (struct.pack('<BBbB', 2, 9, 1, 29), [29]),
                               (struct.pack('<BBbB', 2, 9, 0, 29), [30])):
            q = SimpleNamespace(consumed=[(0, payload)])
            with self.assertRaises(ValueError): probe.validate_sdk_reply(q, 0, 2, 9, value)
        with self.assertRaises(TimeoutError): probe.validate_sdk_reply(SimpleNamespace(consumed=[]), 0, 2, 9, None)

    def test_sdk_stale_matching_reply_rejected(self):
        q = SimpleNamespace(consumed=[(1.0, struct.pack('<BBbB', 2, 9, 0, 29))])
        with self.assertRaises(ValueError): probe.validate_sdk_reply(q, 0, 2, 9, [29], sent=2.0)

    def test_stock_read_calls_only_temperature_and_is_bounded(self):
        board = sdk.Board.__new__(sdk.Board)
        board.servo_read_lock = threading.Lock()
        board.retry_times = 2
        board.bus_servo_queue = Mock()
        board.bus_servo_queue.get.side_effect = queue.Empty
        port = Mock(); port.write.side_effect = lambda b: len(b)
        board.port = probe.GuardedPort(port)
        self.assertIsNone(board.bus_servo_read_temp(2))
        self.assertEqual(len(board.port.tx), 3)
        self.assertTrue(all(data == servo_protocol.packet(bytes([9, 2])) for _, data in board.port.tx))

    def test_stock_read_success_validated_against_real_consumption(self):
        board = sdk.Board.__new__(sdk.Board)
        board.servo_read_lock = threading.Lock(); board.retry_times = 2
        board.port = probe.GuardedPort(Mock())
        q = probe.ObservedQueue(queue.Queue(maxsize=1)); board.bus_servo_queue = q
        q.put_nowait(struct.pack('<BBbh', 6, 5, 0, 483))
        value = board.bus_servo_read_position(6)
        self.assertEqual(probe.validate_sdk_reply(q, 0, 6, 5, value), 483)


if __name__ == '__main__': unittest.main()
