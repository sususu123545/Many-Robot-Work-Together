import math
import struct
import unittest
import threading
import time
from armpi_ros2.servo_protocol import (crc8, packet, JOINTS, ServoBoard, to_raw,
                                      to_radians, movement_payload, validated_targets)


class Port:
    def __init__(self, replies=()):
        self.written = []
        self.replies = list(replies)
        self.incoming = bytearray()
        self.closed = False

    @property
    def in_waiting(self):
        return len(self.incoming)

    def reset_input_buffer(self):
        self.incoming.clear()

    def write(self, data):
        self.written.append(data)
        if self.replies:
            self.incoming.extend(self.replies.pop(0))
        return len(data)

    def read(self, count):
        result = bytes(self.incoming[:count])
        del self.incoming[:count]
        return result

    def close(self):
        self.closed = True


class ServoTests(unittest.TestCase):
    def test_crc_matches_vendor_table_seed_entries(self):
        self.assertEqual([crc8([i]) for i in range(16)],
                         [0,94,188,226,97,63,221,131,194,156,126,32,163,253,31,65])

    def test_roundtrip_and_reversed_elbow(self):
        for name, (ident, _, _, low, high) in JOINTS.items():
            for raw in (low, (low+high)//2, high):
                self.assertEqual(to_raw(name, to_radians(name, raw)), (ident, raw))
        self.assertLess(to_radians('joint3', 832), 0)

    def test_hardware_limit_not_silently_clamped(self):
        with self.assertRaises(ValueError):
            to_raw('r_joint', to_radians('r_joint', 701))
        with self.assertRaises(ValueError):
            to_raw('joint2', to_radians('joint2', 124))
        for bad in (math.nan, math.inf):
            with self.assertRaises(ValueError):
                to_raw('joint1', bad)

    def test_gripper_uses_legacy_180_degree_scale(self):
        self.assertAlmostEqual(to_radians('r_joint', 200), -math.pi/2)
        self.assertEqual(to_raw('r_joint', -math.pi/2), (1,200))

    def test_move_format_and_no_other_function(self):
        payload = movement_payload(.5, [(1, 86)])
        self.assertEqual(payload, struct.pack('<BHBBH', 1, 500, 1, 1, 86))
        self.assertEqual(packet(payload)[2], 5)
        for duration in (.1, 10, math.nan):
            with self.assertRaises(ValueError):
                movement_payload(duration, [(1, 86)])

    def test_no_write_on_board_construction(self):
        p = Port()
        b = ServoBoard(port=p)
        self.assertEqual(p.written, [])
        b.close()
        self.assertFalse(b.reader.is_alive())

    def test_reads_only_matching_real_reply(self):
        wrong = packet(struct.pack('<BBbh', 2, 5, 0, 500))
        actual = packet(struct.pack('<BBbh', 1, 5, 0, 86))
        p = Port([wrong+actual])
        b = ServoBoard(port=p)
        self.assertEqual(b.position(1), 86)
        self.assertEqual(p.written, [packet(bytes((5,1)))])
        stats=b.diagnostics()
        self.assertEqual(stats['counters']['id_command_mismatch'],1)
        self.assertEqual(stats['last_query']['result'],'OK')
        self.assertTrue(any(e['event']=='rx_servo' and 'payload_hex' in e for e in stats['recent_events']))
        b.close()

    def test_corrupt_reply_not_reported_as_position(self):
        raw = bytearray(packet(struct.pack('<BBbh', 1, 5, 0, 86)))
        raw[-1] ^= 1
        b = ServoBoard(port=Port([raw]))
        try:
            with self.assertRaises(TimeoutError):
                b.read(1, 5, '<BBbh', timeout=.005)
        finally:b.close()
        self.assertGreaterEqual(b.diagnostics()['counters']['crc_error'],1)
        self.assertEqual(b.diagnostics()['counters']['read_timeout'],1)

    def test_step_heat_and_stale_guards(self):
        good = {'r_joint': {'raw':86, 'temperature':51, 'time':10}}
        self.assertEqual(validated_targets(['r_joint'], [to_radians('r_joint', 100)], good, 10.5), [(1,100)])
        for target, sample, now in [(200, good, 10.5), (100, good, 12),
                (100, {'r_joint':{'raw':86,'temperature':65,'time':10}}, 10.5)]:
            with self.assertRaises(ValueError):
                validated_targets(['r_joint'], [to_radians('r_joint', target)], sample, now)

    def test_stop_packet_matches_sdk_without_torque_write(self):
        p = Port(); b = ServoBoard(port=p); b.stop()
        self.assertEqual(p.written, [packet(bytes((3,6,1,2,3,4,5,6)))])
        b.close()

    def test_voltage_diagnostic_uses_only_read_packet_and_real_reply(self):
        p = Port([packet(struct.pack('<BBbH', 4, 7, 0, 11234))])
        b = ServoBoard(port=p)
        try:
            self.assertEqual(b.voltage_mv(4), 11234)
            self.assertEqual(p.written, [packet(bytes((7, 4)))])
        finally:b.close()

    def test_torque_diagnostic_reads_both_states_without_enabling_or_unloading(self):
        for state in (0, 1):
            with self.subTest(state=state):
                p = Port([packet(struct.pack('<BBbb', 4, 13, 0, state))])
                b = ServoBoard(port=p)
                try:
                    self.assertIs(b.torque_enabled(4), bool(state))
                    self.assertEqual(p.written, [packet(bytes((13, 4)))])
                finally:b.close()

    def test_torque_diagnostic_never_turns_invalid_feedback_into_enabled(self):
        p = Port([packet(struct.pack('<BBbb', 4, 13, 0, -1))])
        b = ServoBoard(port=p)
        try:
            with self.assertRaises(OSError):b.torque_enabled(4)
            self.assertEqual(p.written, [packet(bytes((13, 4)))])
        finally:b.close()

    def test_read_rejects_write_codes_wrong_format_and_broadcast_before_uart_write(self):
        p = Port();b = ServoBoard(port=p)
        try:
            for ident, command, fmt in ((4, 11, '<BBbb'), (4, 12, '<BBbb'),
                                        (4, 1, '<BBbh'), (4, 3, '<BBbh'),
                                        (4, 13, '<BBbH'), (254, 5, '<BBbh')):
                with self.subTest(ident=ident, command=command, fmt=fmt):
                    with self.assertRaises(ValueError):b.read(ident, command, fmt)
            self.assertEqual(p.written, [])
        finally:b.close()

    def test_read_retry_recovers_one_missing_reply_without_motion(self):
        actual=packet(struct.pack('<BBbh',1,5,0,85))
        p=Port([b'',actual]);b=ServoBoard(port=p)
        try:
            self.assertEqual(b.read(1,5,'<BBbh',timeout=.02),[85])
            self.assertEqual(p.written,[packet(bytes((5,1)))]*2)
            self.assertEqual(b.diagnostics()['last_query']['attempts'],2)
        finally:b.close()

    def test_read_retry_is_bounded_and_never_fabricates_feedback(self):
        p=Port();b=ServoBoard(port=p)
        with self.assertRaises(TimeoutError):b.read(1,5,'<BBbh',timeout=.001)
        self.assertEqual(len(p.written),3)
        with self.assertRaises(ValueError):b.read(1,1,'<BBbh')
        self.assertEqual(len(p.written),3)
        b.close()

    def test_continuous_rx_does_not_flush_partial_frames(self):
        class NoFlushPort(Port):
            def reset_input_buffer(self):
                raise AssertionError('Must not flush UART during telemetry')
        p=NoFlushPort([packet(struct.pack('<BBbh',1,5,0,85))]);b=ServoBoard(port=p)
        try:self.assertEqual(b.position(1),85)
        finally:b.close()

    def test_stop_preempts_missing_read_reply(self):
        p=Port();b=ServoBoard(port=p);errors=[]
        def wait_read():
            try:b.read(1,5,'<BBbh',timeout=.1,attempts=1)
            except TimeoutError:errors.append('timeout')
        worker=threading.Thread(target=wait_read);worker.start()
        deadline=time.monotonic()+.1
        while not p.written and time.monotonic()<deadline:time.sleep(.001)
        self.assertTrue(p.written)
        started=time.monotonic();b.stop()
        self.assertLess(time.monotonic()-started,.05)
        self.assertEqual(p.written[-1],packet(bytes((3,6,1,2,3,4,5,6))))
        worker.join(.2);self.assertEqual(errors,['timeout']);b.close()

    def test_diagnostics_count_invalid_length_and_bounded_event_history(self):
        invalid=packet(bytes((1,5,0)))
        actual=packet(struct.pack('<BBbh',1,5,0,85))
        b=ServoBoard(port=Port([invalid+actual]))
        try:
            self.assertEqual(b.position(1),85)
            self.assertEqual(b.diagnostics()['counters']['length_mismatch'],1)
            for i in range(50):b._record('test_event',sequence=i)
            self.assertEqual(len(b.diagnostics()['recent_events']),32)
        finally:b.close()

    def test_diagnostics_keep_failed_query_after_later_recovery(self):
        p=Port();b=ServoBoard(port=p)
        try:
            with self.assertRaises(TimeoutError):b.read(1,9,'<BBbB',timeout=.001,attempts=1)
            failed=b.diagnostics()['last_failed_query']
            p.replies.append(packet(struct.pack('<BBbB',1,9,0,31)))
            self.assertEqual(b.temperature(1),31)
            stats=b.diagnostics()
            self.assertEqual(stats['last_query']['result'],'OK')
            self.assertEqual(stats['last_failed_query'],failed)
            self.assertEqual(failed['command'],9)
            self.assertTrue(stats['last_failure_events'])
        finally:b.close()


if __name__ == '__main__':
    unittest.main()
