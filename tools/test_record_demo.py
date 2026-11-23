#!/usr/bin/env python3
"""Pin the rules that decide whether a recording is a usable demonstration.

The recorder itself is a passive listener, but the scoring decides whether the
result can go into a dataset.  A wrong pairing rule or a missing action label
would produce a directory that looks like data and trains a policy on nothing,
so those two are tested here.

Pure Python: no rosbridge, no robot.  A synthetic recording directory is built
in a temp folder to exercise the verify path end to end.

Usage:  python3 -B tools/test_record_demo.py
"""

import json
import os
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import record_demo as rec  # noqa: E402

BASE = 1_795_415_000_000_000_000
SIX = ('joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'r_joint')


def state(index, gap_ms=500, joints=6, target=True):
    ticks = {name: 200 + index for name in SIX[:joints]}
    return {'t_mono': index * gap_ms / 1000.0,
            't_wall': 1.0 + index * gap_ms / 1000.0,
            'stamp_ns': BASE + index * gap_ms * 1_000_000,
            'ticks': ticks,
            'last_command': ({'sequence': index, 'joint_names': ['joint2'],
                              'targets': [[5, 300]]} if target else None)}


def image(index, gap_ms=79, skew_ms=0):
    return {'file': '%06d.jpg' % (index + 1),
            'stamp_ns': BASE + index * gap_ms * 1_000_000 + skew_ms * 1_000_000,
            'bytes': 1000,
            'sha256': 'x' * 64}


class TestStampOf(unittest.TestCase):

    def test_reads_header_stamp(self):
        message = {'msg': {'header': {'stamp': {'sec': 12, 'nanosec': 500}}}}
        self.assertEqual(rec._stamp_of(message), 12 * 10**9 + 500)

    def test_missing_stamp_is_none(self):
        self.assertIsNone(rec._stamp_of({'msg': {'header': {}}}))

    def test_non_integer_seconds_is_none(self):
        message = {'msg': {'header': {'stamp': {'sec': 'soon'}}}}
        self.assertIsNone(rec._stamp_of(message))


class TestPairing(unittest.TestCase):

    def test_states_pair_to_the_nearest_image(self):
        states = [state(i) for i in range(4)]          # every 500 ms
        images = [image(i) for i in range(26)]         # every 79 ms
        result = rec.check_pairing(states, images, 50.0)
        self.assertEqual(result['matched'], 4)
        self.assertAlmostEqual(result['rate'], 1.0)

    def test_sparse_images_drop_the_rate(self):
        states = [state(i) for i in range(4)]
        images = [image(0), image(6)]                  # only two frames
        result = rec.check_pairing(states, images, 50.0)
        self.assertLess(result['rate'], 1.0)

    def test_empty_input_does_not_crash(self):
        self.assertEqual(rec.check_pairing([], [], 50.0)['matched'], 0)
        self.assertEqual(rec.check_pairing([state(0)], [], 50.0)['rate'], 0.0)

    def test_skew_is_reported_in_milliseconds(self):
        states = [state(0)]
        images = [{'file': 'a.jpg', 'stamp_ns': BASE + 20_000_000}]  # 20 ms later
        result = rec.check_pairing(states, images, 50.0)
        self.assertAlmostEqual(result['median_skew_ms'], 20.0, places=3)


class TestSummarise(unittest.TestCase):

    def test_counts_complete_samples_only(self):
        states = [state(0), state(1), state(2, joints=5)]
        result = rec.summarise_recording(states, [image(i) for i in range(20)])
        self.assertEqual(result['state_samples'], 3)
        self.assertEqual(result['states_with_all_six_joints'], 2)

    def test_counts_samples_that_carry_a_target(self):
        states = [state(0), state(1, target=False)]
        result = rec.summarise_recording(states, [image(i) for i in range(20)])
        self.assertEqual(result['states_with_commanded_target'], 1)

    def test_duration_and_rate_follow_the_spans(self):
        states = [state(i) for i in range(4)]          # 3 gaps of 0.5 s
        result = rec.summarise_recording(states, [image(i) for i in range(30)])
        self.assertAlmostEqual(result['duration_s'], 1.5, places=3)
        self.assertAlmostEqual(result['state_hz'], 2.0, places=2)

    def test_single_sample_has_no_rate(self):
        result = rec.summarise_recording([state(0)], [image(0)])
        self.assertIsNone(result['state_hz'])


class TestAssess(unittest.TestCase):

    def clean(self):
        states = [state(i) for i in range(4)]
        return rec.summarise_recording(states, [image(i) for i in range(30)])

    def test_clean_recording_has_no_problems(self):
        self.assertEqual(rec.assess(self.clean()), [])

    def test_missing_action_labels_is_fatal(self):
        states = [state(i, target=False) for i in range(4)]
        summary = rec.summarise_recording(states, [image(i) for i in range(30)])
        problems = rec.assess(summary)
        self.assertTrue(any('commanded target' in p for p in problems))

    def test_missing_joints_is_fatal(self):
        states = [state(i, joints=5) for i in range(4)]
        summary = rec.summarise_recording(states, [image(i) for i in range(30)])
        self.assertTrue(any('six joints' in p for p in rec.assess(summary)))

    def test_stalled_state_stream_is_fatal(self):
        states = [state(0), state(1), state(20)]       # 10 s hole
        summary = rec.summarise_recording(states, [image(i) for i in range(200)])
        self.assertTrue(any('stalled' in p for p in rec.assess(summary)))

    def test_low_pairing_rate_is_fatal(self):
        states = [state(i) for i in range(4)]
        summary = rec.summarise_recording(states, [image(0)])
        self.assertTrue(any('pairing rate' in p for p in rec.assess(summary)))


class TestVerifyOnDisk(unittest.TestCase):
    """Exercise the verify subcommand against a synthetic recording tree."""

    def write_recording(self, root, states, images, task='pick the ball'):
        (root / 'images').mkdir(parents=True, exist_ok=True)
        (root / 'arm_states.jsonl').write_text(
            ''.join(json.dumps(s) + '\n' for s in states))
        (root / 'images' / 'index.jsonl').write_text(
            ''.join(json.dumps(i) + '\n' for i in images))
        (root / 'meta.json').write_text(json.dumps(
            {'schema': rec.__doc__.splitlines()[0][:20], 'task': task,
             'passive': True, 'published_anything': False}))

    def test_complete_recording_verifies(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self.write_recording(root,
                                 [state(i) for i in range(4)],
                                 [image(i) for i in range(30)])
            code = rec.main(['verify', '--dir', str(root)])
            self.assertEqual(code, 0)

    def test_recording_without_targets_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self.write_recording(root,
                                 [state(i, target=False) for i in range(4)],
                                 [image(i) for i in range(30)])
            code = rec.main(['verify', '--dir', str(root)])
            self.assertEqual(code, 1)

    def test_missing_files_fail(self):
        with tempfile.TemporaryDirectory() as tmp:
            self.assertEqual(rec.main(['verify', '--dir', tmp]), 1)


if __name__ == '__main__':
    unittest.main(verbosity=2)
