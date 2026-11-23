#!/usr/bin/env python3
"""Pin down the motion-gate semantics so they can never be swapped again.

On 2026-11-11 a refactor passed ``secure_gate``'s (arm, gripper) booleans by
position and swapped them.  Every ``arm_gate()`` then opened the gripper and
vice versa, so joint and jaw commands were silently dropped while
``/arm_controller/status`` still reported READY.  It was misdiagnosed as a
servo bus fault and cost several power cycles.  These tests need no ROS 2, no
robot and no network.

Usage:  python3 -B tools/test_gate_semantics.py
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import grasp_sequence as gs  # noqa: E402


class TestGateSemantics(unittest.TestCase):

    def setUp(self):
        self.calls = []
        self.original = gs.secure_gate
        calls = self.calls

        def fake(bridge, *, arm, gripper, timeout=120):
            calls.append((bool(arm), bool(gripper)))
            return True

        gs.secure_gate = fake

    def tearDown(self):
        gs.secure_gate = self.original

    def test_arm_gate_opens_arm_and_closes_gripper(self):
        self.assertTrue(gs.arm_gate(None))
        self.assertEqual(self.calls, [(True, False)])

    def test_gripper_gate_opens_gripper_and_closes_arm(self):
        self.assertTrue(gs.gripper_gate(None))
        self.assertEqual(self.calls, [(False, True)])

    def test_the_two_gates_are_not_both_open(self):
        gs.arm_gate(None)
        gs.gripper_gate(None)
        self.assertEqual(self.calls, [(True, False), (False, True)])
        for arm, gripper in self.calls:
            self.assertNotEqual(arm, gripper)

    def test_transient_failure_is_retried(self):
        attempts = []

        def flaky(_bridge):
            attempts.append(1)
            if len(attempts) < 3:
                raise OSError('Need 10 seconds of fresh valid five-joint feedback')
            return True

        self.assertTrue(gs.gate_with_retry(flaky, None, tries=3, wait=0))
        self.assertEqual(len(attempts), 3)

    def test_persistent_failure_returns_false(self):
        def broken(_bridge):
            raise OSError('always down')

        self.assertFalse(gs.gate_with_retry(broken, None, tries=2, wait=0))


if __name__ == '__main__':
    unittest.main(verbosity=2)
