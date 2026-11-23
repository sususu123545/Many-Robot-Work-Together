#!/usr/bin/env python3
"""Pin the parsing rules used by the cadence capture.

These two functions decide every number in a capture report, and both have a
sign trap: joint3 has sign -1 in the car's servo table, and r_joint uses a
different degrees-per-tick scale.  A silent error here would make a cadence
report look authoritative while being wrong.

Pure Python: no rosbridge, no robot.

Usage:  python3 -B tools/test_capture_cadence.py
"""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import capture_cadence as cc  # noqa: E402

TICK_RAD = math.radians(240.0) / 1000.0
GRIPPER_RAD = math.radians(180.0) / 1000.0


def joint_state(pairs, sec=None, nanosec=None):
    message = {'name': [n for n, _ in pairs], 'position': [p for _, p in pairs]}
    if sec is not None:
        message['header'] = {'stamp': {'sec': sec, 'nanosec': nanosec or 0}}
    return {'topic': '/joint_states', 'msg': message}


class TestTicksFromMessage(unittest.TestCase):

    def test_centre_tick_reads_as_500(self):
        ticks = cc.ticks_from_message(joint_state([('joint1', 0.0)]))
        self.assertEqual(ticks['joint1'], 500)

    def test_joint3_sign_is_negative(self):
        # A positive radian must map BELOW the centre, because joint3 has sign -1.
        radians = 400 * TICK_RAD
        ticks = cc.ticks_from_message(joint_state([('joint3', radians)]))
        self.assertEqual(ticks['joint3'], 100)

    def test_joint1_same_radian_maps_above_centre(self):
        radians = 400 * TICK_RAD
        ticks = cc.ticks_from_message(joint_state([('joint1', radians)]))
        self.assertEqual(ticks['joint1'], 900)

    def test_gripper_uses_its_own_scale(self):
        # r_joint centre 700, 180 deg per 1000 ticks.
        radians = -300 * GRIPPER_RAD
        ticks = cc.ticks_from_message(joint_state([('r_joint', radians)]))
        self.assertEqual(ticks['r_joint'], 400)

    def test_round_trip_against_the_servo_table(self):
        for name, (_, centre, sign, low, high) in cc.SERVO_TABLE.items():
            scale = GRIPPER_RAD if name == 'r_joint' else TICK_RAD
            for tick in (low, (low + high) // 2, high):
                radians = sign * (tick - centre) * scale
                back = cc.ticks_from_message(joint_state([(name, radians)]))
                self.assertEqual(back[name], tick, '%s at %d' % (name, tick))

    def test_unknown_joint_is_ignored(self):
        ticks = cc.ticks_from_message(joint_state([('joint1', 0.0), ('elbow', 1.0)]))
        self.assertEqual(set(ticks), {'joint1'})

    def test_non_finite_position_is_skipped(self):
        ticks = cc.ticks_from_message(joint_state([('joint1', float('nan'))]))
        self.assertEqual(ticks, {})

    def test_missing_position_list_yields_nothing(self):
        self.assertEqual(cc.ticks_from_message({'msg': {'name': ['joint1']}}), {})


class TestStampSeconds(unittest.TestCase):

    def test_combines_sec_and_nanosec(self):
        self.assertAlmostEqual(cc.stamp_seconds(joint_state([], 12, 500000000)), 12.5, places=6)

    def test_absent_stamp_returns_none(self):
        self.assertIsNone(cc.stamp_seconds(joint_state([])))

    def test_malformed_stamp_returns_none(self):
        broken = {'msg': {'header': {'stamp': {'sec': 'soon'}}}}
        self.assertIsNone(cc.stamp_seconds(broken))


class TestCommandLineParsing(unittest.TestCase):

    def test_matches_a_real_ok_line(self):
        line = '    joint2 -20 ok  (target 588, moved 19 of 20, error 1)'
        match = cc.COMMAND_LINE.match(line)
        self.assertIsNotNone(match)
        self.assertEqual(match.group('joint'), 'joint2')
        self.assertEqual(match.group('delta'), '-20')
        self.assertEqual(match.group('status'), 'ok')

    def test_matches_a_real_no_movement_line(self):
        line = '    joint3 -14 NO MOVEMENT  (target 895, moved 7 of 14, error 7)'
        match = cc.COMMAND_LINE.match(line)
        self.assertIsNotNone(match)
        self.assertEqual(match.group('status'), 'NO MOVEMENT')
        self.assertEqual(match.group('moved'), '7')

    def test_ignores_non_command_lines(self):
        for line in ('  gates arm=True gripper=False -> ok=True ',
                     '     hop 1: dz=+101 mm -> 9 mm off',
                     'RESULT {"ok": true}'):
            self.assertIsNone(cc.COMMAND_LINE.match(line), line)


if __name__ == '__main__':
    unittest.main(verbosity=2)
