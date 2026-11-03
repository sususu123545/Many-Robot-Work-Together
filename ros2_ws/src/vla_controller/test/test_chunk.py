"""A chunk is consumed exactly k times, then thrown away."""

import unittest

from vla_controller.chunk import ChunkError, ChunkExecutor
from vla_controller.spec import ACTION_FRAME, ACTION_SPACE_ID

STAMP = 10.0


def actions(count, value=0.001):
    return [tuple([value] * 6 + [0.5]) for _ in range(count)]


def loaded(executor, count=16, validity=1.0, stamp=STAMP, **overrides):
    payload = dict(control_dt=0.35, validity_s=validity, stamp=stamp, policy_id='smolvla-0')
    payload.update(overrides)
    executor.load(actions(count), **payload)
    return executor


class RollingTest(unittest.TestCase):
    def test_only_k_actions_are_handed_out(self):
        executor = loaded(ChunkExecutor(execute_steps=4), count=16)
        handed = 0
        while executor.next_action(STAMP + 0.1) is not None:
            executor.commit()
            handed += 1
            if handed > 20:
                self.fail('executor never stopped handing out actions')
        self.assertEqual(handed, 4)
        self.assertEqual(executor.remaining(), 0)

    def test_the_rest_of_the_chunk_is_not_kept_for_later(self):
        executor = loaded(ChunkExecutor(execute_steps=2), count=16)
        for _ in range(2):
            executor.commit()
        self.assertEqual(executor.state(STAMP + 0.1), 'done')
        self.assertIsNone(executor.next_action(STAMP + 0.1))

    def test_index_advances_one_action_at_a_time(self):
        executor = loaded(ChunkExecutor(execute_steps=4), count=8)
        self.assertEqual(executor.index, 0)
        first = executor.next_action(STAMP)
        executor.commit()
        self.assertEqual(executor.index, 1)
        self.assertEqual(executor.next_action(STAMP), first)  # same payload, next slot
        self.assertEqual(executor.remaining(), 3)


class AbortTest(unittest.TestCase):
    def test_abort_drops_the_whole_rest(self):
        """Skipping one delta shifts every later target by that amount."""
        executor = loaded(ChunkExecutor(execute_steps=4), count=8)
        executor.commit()
        executor.abort('unreachable')
        self.assertIsNone(executor.next_action(STAMP))
        self.assertEqual(executor.state(STAMP), 'aborted')
        self.assertEqual(executor.aborted_reason, 'unreachable')

    def test_reloading_clears_the_abort(self):
        executor = loaded(ChunkExecutor(execute_steps=4), count=8)
        executor.abort('x')
        loaded(executor, count=8)
        self.assertEqual(executor.state(STAMP), 'executing')
        self.assertEqual(executor.aborted_reason, '')


class ExpiryTest(unittest.TestCase):
    def test_chunk_expires(self):
        executor = loaded(ChunkExecutor(execute_steps=4), count=8, validity=0.5)
        self.assertIsNone(executor.next_action(STAMP + 0.6))
        self.assertEqual(executor.state(STAMP + 0.6), 'expired')

    def test_expiry_is_independent_of_how_many_were_executed(self):
        executor = loaded(ChunkExecutor(execute_steps=4), count=8, validity=0.5)
        executor.commit()
        self.assertTrue(executor.expired(STAMP + 0.6))

    def test_nothing_loaded_is_idle(self):
        self.assertEqual(ChunkExecutor().state(STAMP), 'idle')
        self.assertIsNone(ChunkExecutor().next_action(STAMP))


class ValidationTest(unittest.TestCase):
    def test_wrong_action_space_is_refused(self):
        with self.assertRaises(ChunkError):
            ChunkExecutor().load(actions(4), control_dt=0.35, validity_s=1.0,
                                 stamp=STAMP, action_space_id='other_v1')

    def test_wrong_frame_is_refused(self):
        with self.assertRaises(ChunkError):
            ChunkExecutor().load(actions(4), control_dt=0.35, validity_s=1.0,
                                 stamp=STAMP, frame='base_link')

    def test_empty_chunk_is_refused(self):
        with self.assertRaises(ChunkError):
            ChunkExecutor().load([], control_dt=0.35, validity_s=1.0, stamp=STAMP)

    def test_chunk_shorter_than_k_is_refused(self):
        with self.assertRaises(ChunkError):
            ChunkExecutor(execute_steps=4).load(actions(2), control_dt=0.35,
                                                validity_s=1.0, stamp=STAMP)

    def test_oversized_chunk_is_refused(self):
        with self.assertRaises(ChunkError):
            ChunkExecutor(execute_steps=4, max_horizon=8).load(
                actions(9), control_dt=0.35, validity_s=1.0, stamp=STAMP)

    def test_non_positive_validity_is_refused(self):
        with self.assertRaises(ChunkError):
            ChunkExecutor().load(actions(4), control_dt=0.35, validity_s=0.0,
                                 stamp=STAMP)

    def test_wrong_component_count_is_refused_and_indexed(self):
        bad = actions(4)
        bad[2] = (0.0, 0.0, 0.0)
        with self.assertRaises(ChunkError) as caught:
            ChunkExecutor().load(bad, control_dt=0.35, validity_s=1.0, stamp=STAMP)
        self.assertIn('action 2', str(caught.exception))

    def test_defaults_are_the_platform_identity(self):
        executor = ChunkExecutor()
        self.assertEqual(executor.expected_action_space_id, ACTION_SPACE_ID)
        self.assertEqual(executor.expected_frame, ACTION_FRAME)


if __name__ == '__main__':
    unittest.main()
