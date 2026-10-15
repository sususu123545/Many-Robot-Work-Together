import unittest
from armpi_ros2.arm_health import ArmHealth, complete_feedback, motion_allowed
from armpi_ros2.servo_protocol import JOINTS


def samples(now, temperature=31):
    return {n: {'raw': v[1], 'temperature': temperature, 'time': now} for n, v in JOINTS.items()}


def healthy_window(health, start=0, seconds=10):
    for i in range(int(seconds*2)+1):
        now = start+i*.5
        health.begin_cycle()
        assert health.success(samples(now), now)
    return start+seconds


class HealthTests(unittest.TestCase):
    def test_complete_feedback_rejects_partial_stale_hot_future_invalid(self):
        good = samples(10)
        self.assertTrue(complete_feedback(good, 10.1))
        partial = dict(good); partial.pop('joint5')
        for data, now in [(partial, 10.1), (good, 11.5), (good, 9.9),
                          (samples(10, 65), 10.1), (samples(10, -1), 10.1)]:
            self.assertFalse(complete_feedback(data, now))
        bad = samples(10);bad['r_joint']['raw'] = 701
        self.assertFalse(complete_feedback(bad, 10))
        bad = samples(10);bad['joint1']['time'] = float('nan')
        self.assertFalse(complete_feedback(bad, 10))

    def test_startup_locked_until_explicit_reset_after_window(self):
        h = ArmHealth();self.assertTrue(h.estop)
        now = healthy_window(h)
        self.assertEqual(h.snapshot(now)['health_state'], 'RECOVERED_AWAIT_RESET')
        self.assertTrue(h.estop)
        self.assertTrue(h.acknowledge(now, h.generation, True))
        self.assertFalse(h.estop)

    def test_one_good_cycle_does_not_allow_reset(self):
        h = ArmHealth();h.success(samples(10), 10)
        self.assertFalse(h.acknowledge(10, h.generation, True))

    def test_error_preserves_last_complete_positions_but_not_validity(self):
        h = ArmHealth();h.success(samples(1), 1)
        h.fail('temperature timeout', 1.1)
        self.assertEqual(len(h.samples), 6)
        self.assertFalse(h.fresh(1.2))
        self.assertEqual(h.snapshot(1.2)['current_read_error'], 'temperature timeout')

    def test_recovery_never_automatically_clears_fault(self):
        h = ArmHealth();h.fail('temperature timeout', 0)
        now = healthy_window(h, 1)
        state = h.snapshot(now)
        self.assertIsNone(state['current_read_error'])
        self.assertEqual(state['health_state'], 'RECOVERED_AWAIT_RESET')
        self.assertTrue(h.estop);self.assertEqual(h.fault, 'temperature timeout')
        self.assertTrue(h.acknowledge(now, h.generation, True))
        self.assertIsNone(h.fault)
        self.assertEqual(h.last_fault['message'], 'temperature timeout')

    def test_intervening_failure_restarts_healthy_window(self):
        h = ArmHealth();healthy_window(h, 0, 9)
        h.fail('new timeout', 9.1)
        now = healthy_window(h, 9.5, 9)
        self.assertFalse(h.can_reset(now))
        h.success(samples(19.5), 19.5)
        self.assertTrue(h.can_reset(19.5))

    def test_polling_gap_does_not_count_as_continuous_recovery(self):
        h = ArmHealth();healthy_window(h)
        h.success(samples(13), 13)
        self.assertEqual(h.healthy_seconds(13), 0)
        self.assertFalse(h.can_reset(13))

    def test_stale_feedback_and_pending_poll_cannot_reset(self):
        h = ArmHealth();now = healthy_window(h)
        self.assertFalse(h.can_reset(now+1.5))
        h.begin_cycle();self.assertFalse(h.can_reset(now))

    def test_reset_cannot_overwrite_new_fault_or_failed_stop(self):
        h = ArmHealth();now = healthy_window(h);generation = h.generation
        self.assertFalse(h.acknowledge(now, generation, False))
        h.fail('new error', now)
        now = healthy_window(h, now+.5)
        self.assertFalse(h.acknowledge(now, generation, True))
        self.assertTrue(h.estop)

    def test_manual_estop_remains_after_communication_recovers(self):
        h = ArmHealth();h.manual_stop();h.fail('timeout', 0)
        now = healthy_window(h)
        self.assertTrue(h.manual_estop);self.assertTrue(h.estop)
        self.assertTrue(h.acknowledge(now, h.generation, True))
        self.assertFalse(h.manual_estop)

    def test_partial_update_is_never_published_as_complete(self):
        h = ArmHealth();partial = samples(1);partial.pop('r_joint')
        self.assertFalse(h.success(partial, 1));self.assertFalse(h.fresh(1))
        self.assertEqual(h.samples, {})

    def test_gripper_permission_cannot_enable_other_joints(self):
        self.assertTrue(motion_allowed(['r_joint'], False, True))
        for names in (['joint1'], ['joint1', 'r_joint'], ['unknown'], [], ['r_joint', 'r_joint']):
            self.assertFalse(motion_allowed(names, False, True))
        self.assertFalse(motion_allowed(['r_joint'], True, False))
        self.assertTrue(motion_allowed(['joint1'], True, False))

    def test_safety_bounds_cannot_be_relaxed(self):
        for seconds, temperature in ((0, 65), (9.9, 65), (10, 66), (float('nan'), 65)):
            with self.assertRaises(ValueError):ArmHealth(seconds, temperature)


if __name__ == '__main__':unittest.main()
