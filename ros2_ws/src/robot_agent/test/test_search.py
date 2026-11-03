"""The sweep must stop to look, and must never declare a find on one frame."""

import math
import unittest

from robot_agent.search import ScanPlanner, ScanState

STEP = math.radians(45.0)


def planner(**overrides):
    values = dict(step_rad=STEP, max_sweep_rad=2 * math.pi, spin_speed=0.18,
                  settle_s=0.8, observe_s=2.0, confirm_s=0.4,
                  max_duration_s=240.0, direction=1.0)
    values.update(overrides)
    return ScanPlanner(**values)


def advance(instance, start, duration, valid=False, step=0.05, abort_reason=''):
    """Tick from ``start`` for ``duration`` seconds; returns (commands, next_time)."""
    commands = []
    now = start
    end = start + duration
    while now < end - 1e-9:
        commands.append(instance.tick(round(now, 6), valid, abort_reason))
        now = round(now + step, 6)
    return commands, now


class StartupTest(unittest.TestCase):
    def test_idle_holds_forever(self):
        instance = planner()
        self.assertEqual(instance.tick(0.0, False).kind, 'hold')
        self.assertEqual(instance.state, ScanState.IDLE)

    def test_start_observes_before_moving(self):
        instance = planner()
        instance.start(0.0)
        commands, _ = advance(instance, 0.0, 1.9)
        self.assertTrue(all(c.kind == 'hold' for c in commands))
        self.assertTrue(all(c.wz == 0.0 for c in commands))

    def test_motion_begins_only_after_the_observation_window(self):
        instance = planner()
        instance.start(0.0)
        commands, _ = advance(instance, 0.0, 2.1)
        self.assertEqual(commands[-1].kind, 'spin')
        self.assertGreater(commands[-1].wz, 0.0)

    def test_stop_returns_to_idle(self):
        instance = planner()
        instance.start(0.0)
        advance(instance, 0.0, 2.1)
        instance.stop()
        self.assertEqual(instance.state, ScanState.IDLE)
        self.assertFalse(instance.running)


class StepCycleTest(unittest.TestCase):
    def test_spin_stops_after_one_step(self):
        instance = planner()
        instance.start(0.0)
        commands, _ = advance(instance, 0.0, 8.0)
        spun = 0.05 * sum(1 for command in commands if command.kind == 'spin')
        # 45 deg at 0.18 rad/s is 4.36 s; the spin must end, not run on.
        self.assertLess(spun, 5.0)
        self.assertGreater(spun, 3.5)
        self.assertGreater(instance.travelled_rad, STEP * 0.95)

    def test_settle_precedes_the_next_look(self):
        instance = planner()
        instance.start(0.0)
        commands = []
        now = 0.0
        while instance.state != ScanState.SETTLE and now < 30.0:
            commands.append(instance.tick(round(now, 6), False))
            now = round(now + 0.05, 6)
        self.assertEqual(instance.state, ScanState.SETTLE)
        self.assertEqual(commands[-1].kind, 'hold')
        held, now = advance(instance, now, 0.5)
        self.assertTrue(all(c.kind == 'hold' for c in held))
        held, now = advance(instance, now, 0.4)
        self.assertEqual(instance.state, ScanState.OBSERVE)

    def test_direction_flips_the_sign(self):
        instance = planner(direction=-1.0)
        instance.start(0.0)
        commands, _ = advance(instance, 0.0, 2.1)
        self.assertLess(commands[-1].wz, 0.0)

    def test_turned_angle_accumulates_across_steps(self):
        instance = planner()
        instance.start(0.0)
        advance(instance, 0.0, 40.0)
        self.assertGreater(instance.travelled_rad, STEP)


class DetectionTest(unittest.TestCase):
    def test_one_frame_is_not_a_find(self):
        instance = planner()
        instance.start(0.0)
        advance(instance, 0.0, 0.5)
        instance.tick(0.55, True)
        command = instance.tick(0.60, False)
        self.assertNotEqual(command.kind, 'found')
        self.assertNotEqual(instance.state, ScanState.FOUND)

    def test_a_held_detection_confirms(self):
        instance = planner()
        instance.start(0.0)
        advance(instance, 0.0, 0.5)
        commands, _ = advance(instance, 0.5, 0.6, valid=True)
        self.assertIn('found', [command.kind for command in commands])
        self.assertEqual(instance.state, ScanState.FOUND)

    def test_confirmation_is_not_reset_by_a_single_dropped_frame(self):
        """The detector drops frames; requiring a perfect run would never confirm."""
        instance = planner(confirm_s=0.4)
        instance.start(0.0)
        advance(instance, 0.0, 0.5)
        instance.tick(0.55, True)
        instance.tick(0.60, True)
        command = instance.tick(0.65, False)     # one miss
        self.assertNotEqual(command.kind, 'found')
        instance.tick(0.70, True)
        command = instance.tick(0.75, True)      # window restarted, not lost
        self.assertNotEqual(instance.state, ScanState.FOUND)
        commands, _ = advance(instance, 0.80, 0.4, valid=True)
        self.assertEqual(instance.state, ScanState.FOUND)

    def test_detection_during_a_turn_stops_the_turn(self):
        instance = planner()
        instance.start(0.0)
        _, now = advance(instance, 0.0, 2.2)
        self.assertEqual(instance.state, ScanState.STEP)
        command = instance.tick(round(now, 6), True)
        self.assertEqual(command.kind, 'hold')
        self.assertEqual(instance.state, ScanState.SETTLE)

    def test_found_stays_stopped(self):
        instance = planner()
        instance.start(0.0)
        advance(instance, 0.0, 0.5)
        advance(instance, 0.5, 0.5, valid=True)
        self.assertEqual(instance.state, ScanState.FOUND)
        command = instance.tick(5.0, True)
        self.assertEqual(command.kind, 'stop')
        self.assertEqual(command.wz, 0.0)


class TerminationTest(unittest.TestCase):
    def test_full_sweep_exhausts(self):
        instance = planner(max_sweep_rad=STEP)
        instance.start(0.0)
        now = 0.0
        for _ in range(4000):
            command = instance.tick(round(now, 6), False)
            now = round(now + 0.05, 6)
            if command.kind == 'stop':
                break
        self.assertEqual(instance.state, ScanState.EXHAUSTED)
        self.assertIn('swept', command.detail)

    def test_operator_abort_stops_immediately(self):
        instance = planner()
        instance.start(0.0)
        advance(instance, 0.0, 2.2)
        command = instance.tick(2.25, False, abort_reason='/safety/estop is latched')
        self.assertEqual(command.kind, 'stop')
        self.assertEqual(instance.state, ScanState.ABORTED)
        self.assertIn('estop', instance.abort_reason)

    def test_duration_cap_ends_the_sweep(self):
        instance = planner(max_duration_s=3.0, max_sweep_rad=1000.0)
        instance.start(0.0)
        commands, _ = advance(instance, 0.0, 4.0)
        self.assertEqual(instance.state, ScanState.ABORTED)
        self.assertIn('exceeded', instance.abort_reason)


class ValidationTest(unittest.TestCase):
    def test_confirm_longer_than_observe_is_refused(self):
        with self.assertRaises(ValueError):
            planner(confirm_s=3.0, observe_s=2.0)

    def test_direction_must_be_a_unit_sign(self):
        with self.assertRaises(ValueError):
            planner(direction=0.5)

    def test_non_positive_timings_are_refused(self):
        for key in ('step_rad', 'settle_s', 'observe_s', 'max_duration_s'):
            with self.assertRaises(ValueError):
                planner(**{key: 0.0})

    def test_snapshot_reports_progress(self):
        instance = planner()
        instance.start(0.0)
        advance(instance, 0.0, 2.2)
        snapshot = instance.snapshot()
        self.assertEqual(snapshot['state'], ScanState.STEP)
        self.assertIn('travelled_deg', snapshot)
        self.assertEqual(snapshot['sweep_target_deg'], 360.0)


if __name__ == '__main__':
    unittest.main()
