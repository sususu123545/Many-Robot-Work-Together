"""Tests for tools/vla_rollout.py -- the joint-space gate and unit handling.

No robot, no network: everything here is the arithmetic that decides whether a
predicted action may move hardware, plus the response validation that stands
between a remote model and the arm.
"""

import importlib.util
import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
sys.path.insert(0, str(ROOT / 'ros2_ws' / 'src' / 'armpi_ros2' / 'armpi_ros2'))
sys.path.insert(0, str(ROOT / 'ros2_ws' / 'src' / 'vla_controller'))

import servo_protocol as sp  # noqa: E402

_spec = importlib.util.spec_from_file_location('vla_rollout', ROOT / 'tools' / 'vla_rollout.py')
rollout = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(rollout)

JOINTS = rollout.JOINT_ORDER


def state_of(ticks):
    return [sp.to_radians(j, ticks[j]) for j in JOINTS]


START = {'joint1': 487, 'joint2': 683, 'joint3': 920, 'joint4': 201,
         'joint5': 498, 'r_joint': 320}


class RoundTripTest(unittest.TestCase):
    """ticks -> radians -> ticks must be the identity for every joint."""

    def test_all_joints_round_trip(self):
        for joint in JOINTS:
            low, high = rollout.SAFE_RANGE[joint]
            for tick in (low + 3, 128, 320, 500, 699, high - 3):
                if not low <= tick <= high:
                    continue
                vector = [0.0] * 6
                vector[JOINTS.index(joint)] = sp.to_radians(joint, tick)
                with self.subTest(joint=joint, tick=tick):
                    self.assertEqual(rollout.radians_to_ticks(vector)[joint], tick)

    def test_gripper_uses_its_own_scale(self):
        """r_joint is 180 deg/1000 ticks, the arm joints 240.  Mixing them
        shrinks every gripper move to 75%.  This pins the two apart."""
        self.assertNotAlmostEqual(sp.tick_radians('r_joint'),
                                  sp.tick_radians('joint3'))
        # 128 is the open pose; the arm-joint constant would report 271.
        vector = [0.0] * 6
        vector[JOINTS.index('r_joint')] = sp.to_radians('r_joint', 128)
        self.assertEqual(rollout.radians_to_ticks(vector)['r_joint'], 128)
        naive = int(round(700 + sp.to_radians('r_joint', 128) / sp.TICK_RAD))
        self.assertEqual(naive, 271)

    def test_full_state_of_start_pose(self):
        targets, notes = rollout.gate(state_of(START), START)
        self.assertEqual(notes, [])
        self.assertEqual(targets, START)


class GateTest(unittest.TestCase):

    def test_within_budget_passes_untouched(self):
        ticks = dict(START, joint3=880)          # -40 ticks
        targets, notes = rollout.gate(state_of(ticks), START)
        self.assertEqual(targets, ticks)
        self.assertEqual(notes, [])

    def test_over_budget_scales_uniformly(self):
        """A 140-tick request becomes 100, and the RATIO between joints is kept.

        Scaling per component would keep every number legal while bending the
        path -- the direction is what has to survive.
        """
        ticks = dict(START, joint3=780, joint2=743)   # -140 and +60
        targets, notes = rollout.gate(state_of(ticks), START)
        self.assertTrue(notes)
        self.assertEqual(targets['joint3'], 920 - 100)
        self.assertEqual(targets['joint2'], 683 + round(60 * (100 / 140)))
        deltas = {j: targets[j] - START[j] for j in JOINTS}
        self.assertLessEqual(max(abs(v) for v in deltas.values()), 100)

    def test_out_of_range_is_rejected_not_clamped(self):
        """Clamping into range would move the arm to the limit instead of
        refusing -- a straight approach would become a pinned joint."""
        for joint, bad in (('joint2', 900), ('joint2', 100), ('r_joint', 750),
                           ('joint1', -5)):
            ticks = dict(START)
            ticks[joint] = bad
            targets, notes = rollout.gate(state_of(ticks), START)
            with self.subTest(joint=joint, value=bad):
                self.assertEqual(targets, {})
                self.assertTrue(notes)

    def test_zero_action_is_admissible(self):
        targets, notes = rollout.gate(state_of(START), START)
        self.assertTrue(targets)
        self.assertEqual(notes, [])


class ResponseValidationTest(unittest.TestCase):
    GOOD = {'actions': [[0.0] * 6, [0.1] * 6], 'policy_id': 'p',
            'control_dt': 0.5, 'validity_s': 6.0,
            'output_kind': 'physical_radians'}

    def test_accepts_a_physical_block(self):
        block = rollout.validate_response(self.GOOD)
        self.assertEqual(block['control_dt'], 0.5)
        self.assertEqual(len(block['actions']), 2)
        self.assertEqual(block['policy_id'], 'p')

    def test_rejects_normalised_output(self):
        payload = dict(self.GOOD, output_kind='normalized')
        with self.assertRaises(ValueError):
            rollout.validate_response(payload)

    def test_rejects_out_of_window_control_dt(self):
        for value in (0.1, 5.1, 0.0, -1):
            with self.subTest(control_dt=value):
                with self.assertRaises(ValueError):
                    rollout.validate_response(dict(self.GOOD, control_dt=value))

    def test_rejects_wrong_dimension(self):
        for actions in ([[0.0] * 5], [[0.0] * 7], [[]]):
            with self.subTest(dim=len(actions[0])):
                with self.assertRaises(ValueError):
                    rollout.validate_response(dict(self.GOOD, actions=actions))

    def test_rejects_empty_and_non_finite(self):
        with self.assertRaises(ValueError):
            rollout.validate_response(dict(self.GOOD, actions=[]))
        for bad in (float('nan'), float('inf')):
            actions = [[0.0] * 6]
            actions[0][2] = bad
            with self.subTest(value=bad):
                with self.assertRaises(ValueError):
                    rollout.validate_response(dict(self.GOOD, actions=actions))

    def test_defaults_policy_id(self):
        payload = {k: v for k, v in self.GOOD.items() if k != 'policy_id'}
        self.assertEqual(rollout.validate_response(payload)['policy_id'], 'remote')




class LimitEnvelopeTest(unittest.TestCase):
    """The 2026-11-28 latched arm: joint3 must be capped at the jog envelope."""

    def test_joint3_has_margin_below_the_latch_line(self):
        # The servo table allows 1000, but the arm node latches the moment raw
        # feedback passes 1000, and recovery needs a service restart.  The jog
        # envelope (40..950) is what the proven scripts use.
        self.assertEqual(rollout.SAFE_RANGE['joint3'], (40, 950))

    def test_target_that_latched_the_arm_is_rejected(self):
        ticks = dict(START, joint3=996)
        targets, notes = rollout.gate(state_of(ticks), START)
        self.assertEqual(targets, {})
        self.assertTrue(any('joint3' in note for note in notes))

    def test_target_near_the_cap_passes_but_warns(self):
        ticks = dict(START, joint3=940)
        targets, notes = rollout.gate(state_of(ticks), START)
        self.assertTrue(targets)
        self.assertTrue(any('near its upper limit' in note for note in notes))


if __name__ == '__main__':
    unittest.main(verbosity=2)
