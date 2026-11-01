import importlib.util
from pathlib import Path
import struct
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
    ab = load('ab_raw_tests', 'tools/arm_transport_readonly_ab.py')
    with patch.dict(sys.modules, {'arm_transport_readonly_ab': ab}):
        capture = load('capture_tests', 'tools/arm_raw_capture.py')
replay = load('replay_tests', 'tools/replay_arm_raw_capture.py')


def frame(function, payload):
    body = bytes([function, len(payload)])+payload
    return b'\xaa\x55'+body+bytes([servo_protocol.crc8(body)])


def fixture(events):
    return {'metadata': {}, 'events': events, 'queries': [
        {'servo': 2, 'command': 9, 'cycle': 0, 'start': 10.0, 'end': 10.65, 'ok': False}]}


class RawCaptureTests(unittest.TestCase):
    def test_write_guard_and_raw_read_capture(self):
        port = Mock(); port.read.return_value = b'\xaa\x55'; port.write.return_value = 7
        wrapped = capture.CapturePort(port)
        self.assertEqual(wrapped.read(2), b'\xaa\x55')
        self.assertEqual(wrapped.write(servo_protocol.packet(bytes([9, 2]))), 7)
        self.assertEqual([e['kind'] for e in wrapped.events], ['rx', 'tx'])
        with self.assertRaises(ValueError): wrapped.write(servo_protocol.packet(bytes([1, 2])))
        self.assertEqual(port.write.call_count, 1)

    def test_fragmented_frames_replay_both_real_parsers(self):
        raw = frame(0, b'\x01\x02')+frame(5, struct.pack('<BBbB', 2, 9, 0, 29))
        events = [{'kind': 'rx', 'time': 10.1+i*.001, 'hex': bytes([b]).hex()} for i, b in enumerate(raw)]
        result = replay.analyze(fixture(events))
        self.assertEqual(result['independent_servo_frames'], 1)
        self.assertTrue(result['native_equals_independent_servo_multiset'])
        self.assertTrue(result['sdk_equals_independent_servo_multiset'])
        self.assertTrue(result['native_equals_independent_servo_sequence'])
        self.assertTrue(result['sdk_equals_independent_servo_sequence'])
        self.assertEqual(result['failed_queries'][0]['matching_valid_rx_in_query'], 1)

    def test_crc_bad_payload_not_counted_as_matching_reply(self):
        raw = bytearray(frame(5, struct.pack('<BBbB', 2, 9, 0, 29))); raw[-1] ^= 1
        result = replay.analyze(fixture([{'kind': 'rx', 'time': 10.1, 'hex': raw.hex()}]))
        self.assertEqual(result['independent_servo_frames'], 0)
        self.assertEqual(result['failed_queries'][0]['matching_valid_rx_in_query'], 0)
        self.assertEqual(result['native_parser_diagnostics'].get('crc_error'), 1)
        self.assertEqual(result['sdk_parser_diagnostics']['crc_error_messages'], 1)

    def test_other_servo_and_nonservo_frames_are_not_replies(self):
        raw = frame(0, b'\x00')+frame(5, struct.pack('<BBbB', 3, 9, 0, 29))
        result = replay.analyze(fixture([{'kind': 'rx', 'time': 10.1, 'hex': raw.hex()}]))
        q = result['failed_queries'][0]
        self.assertEqual(q['matching_valid_rx_in_query'], 0)
        self.assertGreater(q['rx_bytes_in_query'], 0)

    def test_late_reply_reported_separately_and_tx_counted(self):
        raw = frame(5, struct.pack('<BBbB', 2, 9, 0, 29))
        events = [{'kind': 'tx', 'time': 10.02+i*.2, 'hex': servo_protocol.packet(bytes([9, 2])).hex()} for i in range(3)]
        events.append({'kind': 'rx', 'time': 10.8, 'hex': raw.hex()})
        q = replay.analyze(fixture(events))['failed_queries'][0]
        self.assertEqual(q['tx_attempts'], 3)
        self.assertEqual(q['matching_valid_rx_in_query'], 0)
        self.assertEqual(q['matching_valid_rx_next_500ms'], 1)
        self.assertEqual(q['matching_valid_rx_next_500ms_before_same_requery'], 1)

    def test_next_query_reply_not_misreported_as_late_reply(self):
        events = [
            {'kind': 'tx', 'time': 10.02, 'hex': servo_protocol.packet(bytes([9, 2])).hex()},
            {'kind': 'tx', 'time': 10.8, 'hex': servo_protocol.packet(bytes([9, 2])).hex()},
            {'kind': 'rx', 'time': 10.802, 'hex': frame(5, struct.pack('<BBbB', 2, 9, 0, 29)).hex()}]
        q = replay.analyze(fixture(events))['failed_queries'][0]
        self.assertEqual(q['matching_valid_rx_next_500ms'], 1)
        self.assertEqual(q['matching_valid_rx_next_500ms_before_same_requery'], 0)
        self.assertAlmostEqual(q['next_same_requery_after_end_ms'], 150)

    def test_bad_length_does_not_hide_valid_frame_from_independent_scan(self):
        raw = b'\xaa\x55\x05\xff'+frame(5, struct.pack('<BBbB', 2, 9, 0, 29))
        events = [{'kind': 'rx', 'time': 10.1, 'hex': raw.hex()}]
        self.assertEqual(len(replay.scan_valid_frames(events)), 1)


if __name__ == '__main__': unittest.main()
