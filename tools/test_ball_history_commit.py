#!/usr/bin/env python3
"""Regression test for the two-writer history file (no ROS, no car).

Run: python tools/test_ball_history_commit.py
"""
import json
import os
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, 'perception'))

import ball_memory                                              # noqa: E402
import find_and_grasp as fg                                      # noqa: E402


class TestHistoryCommit(unittest.TestCase):

    def setUp(self):
        self.tmp = tempfile.NamedTemporaryFile(suffix='.json', delete=False)
        self.tmp.close()
        self.original = fg.HISTORY
        fg.HISTORY = type(self.original)(self.tmp.name)

    def tearDown(self):
        fg.HISTORY = self.original
        try:
            os.unlink(self.tmp.name)
        except OSError:
            pass

    def _read(self):
        return ball_memory.load(str(fg.HISTORY))

    def test_round_events_survive_memory_writes(self):
        """The 2026-11-07 bug: save(history) erased append_history() events."""
        # main() loads once at startup.
        history = ball_memory.load(str(fg.HISTORY))

        fg.append_history({'t': '12:00:01', 'round': 1, 'event': 'search',
                           'found': True})
        ball_memory.record_sighting(
            history, {'x': 0.0, 'y': 0.0, 'yaw': 87.27}, 0.0, 0.591)
        fg.commit_memory(history)
        self.assertEqual(len([e for e in self._read() if not e.get('kind')]), 1,
                         'the search event must not be erased by the sighting write')

        fg.append_history({'t': '12:00:02', 'round': 1, 'event': 'approach',
                           'ok': True})
        fg.append_history({'t': '12:00:03', 'round': 1, 'event': 'grasp',
                           'ok': True})
        ball_memory.record_outcome(history, True, note='round 1')
        fg.commit_memory(history)

        final = self._read()
        legacy = [e for e in final if not e.get('kind')]
        kinds = [e.get('kind') for e in final if e.get('kind')]
        self.assertEqual(len(legacy), 3, 'all three round events must survive')
        self.assertEqual(kinds, ['sighting', 'outcome'],
                         'both memory records must be present exactly once')
        self.assertEqual(len(final), 5)

    def test_commit_is_idempotent(self):
        history = ball_memory.load(str(fg.HISTORY))
        ball_memory.record_sighting(
            history, {'x': 0.0, 'y': 0.0, 'yaw': 0.0}, 0.0, 0.5)
        fg.commit_memory(history)
        fg.commit_memory(history)
        self.assertEqual(
            len([e for e in self._read() if e.get('kind') == 'sighting']), 1,
            'committing twice must not duplicate the sighting')

    def test_legacy_records_are_never_reordered_or_lost_on_empty_memory(self):
        for index in range(4):
            fg.append_history({'t': '12:00:0%d' % index, 'round': 1,
                               'event': 'step', 'n': index})
        fg.commit_memory([])
        self.assertEqual(len(self._read()), 4,
                         'committing an empty memory must still keep the log')


if __name__ == '__main__':
    unittest.main(verbosity=2)
