#!/usr/bin/env python3
"""Pin the SmolVLA domain-audit decision rules.

Pure Python, no torch, no GPU, no robot.  The point of these tests is that the
domain choice documented in the handoff is reproducible and that a regression
which would silently bless a bad domain fails loudly.

Usage:  python3 -B tools/test_smolvla_domain_audit.py
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import smolvla_domain_audit as audit  # noqa: E402


def chunk_of(rows):
    """The probe stores one batch; the audit wants only the [steps][6] chunk."""
    return rows


class TestLimitClassification(unittest.TestCase):

    def test_joint_channel_uses_plus_minus_180(self):
        self.assertEqual(audit.limits_for('elbow_flex'), (-180.0, 180.0))

    def test_gripper_uses_opening_percentage(self):
        self.assertEqual(audit.limits_for('gripper'), (0.0, 100.0))


class TestDomainAudit(unittest.TestCase):

    def setUp(self):
        # A domain whose reverse map is the identity makes the arithmetic in
        # the assertions below obvious.
        self.original = audit.load_statistics
        audit.load_statistics = lambda domain, feature: (
            [0.0] * 6 if domain == 'identity' else [100.0] * 6,
            [1.0] * 6)

    def tearDown(self):
        audit.load_statistics = self.original

    def test_identity_domain_reports_no_breach_for_plausible_values(self):
        chunk = chunk_of([[0.0, 10.0, 20.0, 30.0, 40.0, 50.0],
                          [1.0, 12.0, 22.0, 32.0, 42.0, 51.0]])
        result = audit.audit_domain('identity', chunk)
        self.assertEqual(result['steps_outside_total'], 0)
        self.assertEqual(result['breaches'], [])
        self.assertEqual(result['steps'], 2)

    def test_elbow_beyond_180_counts_only_the_offending_steps(self):
        chunk = chunk_of([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                          [0.0, 0.0, 200.0, 0.0, 0.0, 0.0],
                          [0.0, 0.0, -181.0, 0.0, 0.0, 0.0]])
        result = audit.audit_domain('identity', chunk)
        self.assertEqual(result['steps_outside_total'], 2)

        breach = {b['channel']: b for b in result['breaches']}
        self.assertIn('elbow_flex', breach)
        self.assertEqual(breach['elbow_flex']['steps_outside'], 2)
        self.assertEqual(breach['elbow_flex']['allowed_range'], [-180.0, 180.0])

    def test_negative_gripper_is_a_breach(self):
        chunk = chunk_of([[0.0, 0.0, 0.0, 0.0, 0.0, -0.03]])
        result = audit.audit_domain('identity', chunk)
        self.assertEqual(result['steps_outside_total'], 1)
        self.assertEqual(result['breaches'][0]['channel'], 'gripper')

    def test_a_domain_shift_moves_the_breach_to_that_channel(self):
        # Only the elbow is shifted past its limit, so the domain choice must
        # be what decides which channel breaches -- not the raw numbers.
        # Do not touch self.original: tearDown needs the pre-setUp reference.
        original = audit.load_statistics

        def statistics(domain, feature):
            mean = [0.0] * 6
            if domain == 'elbow-shift':
                mean[2] = 200.0
            return mean, [1.0] * 6

        audit.load_statistics = statistics
        try:
            chunk = chunk_of([[0.0] * 6])
            self.assertEqual(audit.audit_domain('identity', chunk)['steps_outside_total'], 0)

            shifted = audit.audit_domain('elbow-shift', chunk)
            self.assertEqual(shifted['steps_outside_total'], 1)
            self.assertEqual(shifted['breaches'][0]['channel'], 'elbow_flex')
            self.assertEqual(shifted['breaches'][0]['observed_range'], [200.0, 200.0])
        finally:
            audit.load_statistics = original

    def test_max_step_change_is_reported(self):
        chunk = chunk_of([[0.0] * 6, [5.0] * 6, [1.0] * 6])
        self.assertEqual(audit.audit_domain('identity', chunk)['max_step_change'], 5.0)

    def test_single_step_chunk_has_no_jump(self):
        chunk = chunk_of([[0.0] * 6])
        self.assertEqual(audit.audit_domain('identity', chunk)['max_step_change'], 0.0)


class TestStateAudit(unittest.TestCase):

    def setUp(self):
        self.original = audit.load_statistics
        # Mean 0, std 1 keeps z equal to the input in degrees.
        audit.load_statistics = lambda domain, feature: ([0.0] * 6, [1.0] * 6)

    def tearDown(self):
        self.original = audit.load_statistics

    def observation(self, radians, **overrides):
        payload = {'state': radians, 'state_unit': 'rad',
                   'state_mapping': 'unadapted_armpi_joint_order',
                   'embodiment_compatible': False}
        payload.update(overrides)
        return payload

    def test_zero_radians_is_zero_sigma(self):
        result = audit.audit_state('unit', self.observation([0.0] * 6))
        self.assertEqual(result['channels_beyond_3_sigma'], 0)
        self.assertEqual(result['worst']['z'], 0.0)

    def test_radians_are_converted_to_degrees_before_comparison(self):
        # pi/2 rad = 90 deg, so with mean 0 / std 1 the z score must be 90,
        # not 1.57.  This is the regression that made the reported gap look
        # small on paper while the state was far outside the distribution.
        result = audit.audit_state('unit', self.observation([1.5707963267948966] * 6))
        self.assertAlmostEqual(result['worst']['z'], 90.0, places=3)
        self.assertEqual(result['channels_beyond_3_sigma'], 6)

    def test_rejects_observation_that_was_already_converted(self):
        with self.assertRaises(SystemExit):
            audit.audit_state('unit', self.observation([0.0] * 6, state_unit='deg'))

    def test_rejects_wrong_length(self):
        with self.assertRaises(SystemExit):
            audit.audit_state('unit', self.observation([0.0] * 5))


class TestChunkLoader(unittest.TestCase):

    def test_rejects_malformed_row(self):
        import json
        import tempfile
        payload = {'model_normalized': [[[0.0] * 5]]}
        with tempfile.NamedTemporaryFile('w', suffix='.json', delete=False) as handle:
            json.dump(payload, handle)
            path = handle.name
        try:
            from pathlib import Path
            with self.assertRaises(SystemExit):
                audit.load_chunk(Path(path))
        finally:
            os.unlink(path)


class TestShippedEvidence(unittest.TestCase):
    """The archived run must still reproduce the documented decision."""

    def setUp(self):
        from pathlib import Path
        self.evidence = (Path(__file__).resolve().parents[1] / 'artifacts' /
                         'smolvla_domain_audit_20261123')
        if not (self.evidence / 'actions.json').is_file():
            self.skipTest('archived probe evidence is not present')

    def test_so100_blue_stays_clean_over_all_fifty_steps(self):
        chunk = audit.load_chunk(self.evidence / 'actions.json')
        self.assertEqual(len(chunk), 50)
        result = audit.audit_domain('so100-blue', chunk)
        self.assertEqual(result['steps_outside_total'], 0)

    def test_so100_is_eliminated_by_elbow_and_gripper(self):
        chunk = audit.load_chunk(self.evidence / 'actions.json')
        result = audit.audit_domain('so100', chunk)
        channels = {b['channel'] for b in result['breaches']}
        self.assertIn('elbow_flex', channels)
        self.assertIn('gripper', channels)
        self.assertGreater(result['steps_outside_total'], 0)


if __name__ == '__main__':
    unittest.main(verbosity=2)
