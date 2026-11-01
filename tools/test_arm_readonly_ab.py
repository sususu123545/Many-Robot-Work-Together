import importlib.util
from pathlib import Path
import unittest
from unittest.mock import patch
from unittest.mock import Mock
from armpi_ros2 import servo_protocol

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('ab',ROOT/'tools/arm_transport_readonly_ab.py')
ab=importlib.util.module_from_spec(spec)
with patch.dict('sys.modules',{'servo_protocol':servo_protocol}):spec.loader.exec_module(ab)
from armpi_ros2.servo_protocol import packet


class ReadOnlyPacketTests(unittest.TestCase):
    def test_exact_diagnostic_reads_only(self):
        for command in (5,7,9):
            for ident in range(1,7):ab.check_read_packet(packet(bytes([command,ident])))

    def test_movement_stop_torque_and_broadcast_rejected(self):
        for payload in (bytes([1,4]),bytes([3,4]),bytes([12,4]),bytes([13,4]),
                        bytes([5,254]),bytes([9,0]),bytes([5,4,0])):
            with self.subTest(payload=payload):
                with self.assertRaises(ValueError):ab.check_read_packet(packet(payload))

    def test_bad_crc_and_function_rejected(self):
        frame=bytearray(packet(bytes([5,4])));frame[-1]^=1
        with self.assertRaises(ValueError):ab.check_read_packet(frame)
        with self.assertRaises(ValueError):ab.check_read_packet(b'\x55\x55'+bytes(5))

    def test_interleaved_recovery_only_reads_real_temperature_and_position(self):
        board=Mock();board.read.side_effect=[TimeoutError(),[940],[29]]
        self.assertEqual(ab.interleaved_temperature(board,4),29)
        self.assertEqual([c.args[1] for c in board.read.call_args_list],[9,5,9])

    def test_interleaved_recovery_is_bounded_and_position_error_not_hidden(self):
        board=Mock();board.read.side_effect=[TimeoutError(),[940],TimeoutError(),[940],TimeoutError()]
        with self.assertRaises(TimeoutError):ab.interleaved_temperature(board,4)
        self.assertEqual([c.args[1] for c in board.read.call_args_list],[9,5,9,5,9])
        board=Mock();board.read.side_effect=[TimeoutError(),TimeoutError()]
        with self.assertRaises(TimeoutError):ab.interleaved_temperature(board,4)
        self.assertEqual(board.read.call_count,2)


if __name__=='__main__':unittest.main()
