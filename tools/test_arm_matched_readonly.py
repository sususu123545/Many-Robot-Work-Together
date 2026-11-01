import importlib.util
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch
from armpi_ros2 import servo_protocol

ROOT = Path(__file__).resolve().parents[1]


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, ROOT/path)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    return module


with patch.dict(sys.modules, {'servo_protocol': servo_protocol}):
    ab = load('matched_ab_tests', 'tools/arm_transport_readonly_ab.py')
    with patch.dict(sys.modules, {'arm_transport_readonly_ab': ab}):
        matched = load('matched_tests', 'tools/arm_matched_readonly.py')


class MatchedTests(unittest.TestCase):
    def test_balanced_order_and_exact_read_plans(self):
        self.assertEqual(matched.ORDER, matched.ORDER[::-1])
        self.assertEqual(matched.commands('A_pair'), (5, 9))
        self.assertEqual(matched.commands('B_voltage'), (5, 7, 9))
        self.assertEqual(matched.commands('C_wait40'), (5, 9))
        for command, fmt in matched.FORMATS.items(): self.assertEqual(servo_protocol.READ_FORMATS[command], fmt)
        with self.assertRaises(ValueError): matched.commands('unknown')

    def test_wait_only_changes_c_never_sleeps_negative(self):
        self.assertAlmostEqual(matched.temperature_wait('C_wait40', 10.0, 10.003), .037)
        self.assertEqual(matched.temperature_wait('C_wait40', 10.0, 10.2), 0)
        self.assertEqual(matched.temperature_wait('A_pair', 10.0, 10.003), 0)
        self.assertEqual(matched.temperature_wait('B_voltage', 10.0, 10.003), 0)

    def test_trace_rejects_motion_torque_and_broadcast_before_write(self):
        p = Mock(); wrapped = matched.TraceReadPort(p)
        for command, ident in ((1, 1), (3, 1), (12, 1), (9, 254)):
            with self.assertRaises(ValueError): wrapped.write(servo_protocol.packet(bytes([command, ident])))
        p.write.assert_not_called(); self.assertEqual(wrapped.tx, [])

    def test_trace_records_exact_read_attempt(self):
        p = Mock(); p.write.return_value = 7; wrapped = matched.TraceReadPort(p)
        self.assertEqual(wrapped.write(servo_protocol.packet(bytes([7, 4]))), 7)
        self.assertEqual(wrapped.tx[0]['servo'], 4)
        self.assertEqual(wrapped.tx[0]['command'], 7)

    def test_summary_separates_retry_from_failed_query(self):
        rows = [{'command': 9, 'servo': 1, 'ok': True, 'attempts': 3},
                {'command': 9, 'servo': 2, 'ok': False, 'attempts': 3},
                {'command': 5, 'servo': 1, 'ok': True, 'attempts': 1}]
        result = matched.summarize(rows)
        self.assertEqual(result['9']['queries'], 2)
        self.assertEqual(result['9']['retried_queries'], 2)
        self.assertEqual(result['9']['failed'], 1)
        self.assertEqual(result['9']['failure_ids'], {'2': 1})
        self.assertEqual(result['5']['failed'], 0)


if __name__ == '__main__': unittest.main()
