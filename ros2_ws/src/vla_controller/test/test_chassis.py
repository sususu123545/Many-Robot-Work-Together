"""The base must be demonstrably still, not merely not-yet-observed moving."""

import unittest

from vla_controller.chassis import ChassisGuard


def settled(guard, start=0.0, end=1.0, step=0.1):
    """Feed a run of dead-still odometry samples."""
    now = start
    while now <= end + 1e-9:
        guard.observe_odom(0.0, 0.0, 0.0, now)
        now = round(now + step, 6)
    return guard


class StartupTest(unittest.TestCase):
    def test_no_odometry_is_not_parked(self):
        parked, detail = ChassisGuard().parked(5.0)
        self.assertFalse(parked)
        self.assertIn('/odom', detail)

    def test_first_sample_alone_is_not_enough(self):
        guard = ChassisGuard()
        guard.observe_odom(0.0, 0.0, 0.0, 10.0)
        parked, detail = guard.parked(10.0)
        self.assertFalse(parked)
        self.assertIn('still for', detail)

    def test_quiet_window_makes_it_parked(self):
        guard = settled(ChassisGuard(hold=0.6, quiet=0.5))
        parked, detail = guard.parked(1.0)
        self.assertTrue(parked, detail)


class MotionTest(unittest.TestCase):
    def test_linear_motion_unparks(self):
        guard = settled(ChassisGuard())
        self.assertTrue(guard.parked(1.0)[0])
        guard.observe_odom(0.02, 0.0, 0.0, 1.1)
        self.assertFalse(guard.parked(1.1)[0])

    def test_rotation_alone_unparks(self):
        guard = settled(ChassisGuard())
        guard.observe_odom(0.0, 0.0, 0.05, 1.1)
        self.assertFalse(guard.parked(1.1)[0])

    def test_small_residual_is_tolerated(self):
        """Wheel slip and encoder noise sit below the parked thresholds."""
        guard = ChassisGuard()
        for index in range(20):
            guard.observe_odom(0.002, 0.001, 0.005, index * 0.1)
        self.assertTrue(guard.parked(1.9)[0])

    def test_non_finite_twist_is_not_evidence_of_stillness(self):
        guard = settled(ChassisGuard())
        guard.observe_odom(float('nan'), 0.0, 0.0, 1.1)
        self.assertFalse(guard.parked(1.2)[0])

    def test_clock_going_backwards_restarts_the_window(self):
        guard = settled(ChassisGuard())
        guard.observe_odom(0.0, 0.0, 0.0, 0.5)
        self.assertFalse(guard.parked(0.6)[0])


class CommandTest(unittest.TestCase):
    def test_fresh_command_unparks_even_without_visible_motion(self):
        guard = settled(ChassisGuard())
        self.assertTrue(guard.parked(1.0)[0])
        guard.observe_cmd_vel(1.05)
        parked, detail = guard.parked(1.1)
        self.assertFalse(parked)
        self.assertIn('/cmd_vel', detail)

    def test_zero_twist_does_not_restart_the_window(self):
        guard = settled(ChassisGuard())
        guard.observe_cmd_vel(1.05, magnitude=0.0)
        self.assertTrue(guard.parked(1.1)[0])

    def test_command_ages_out(self):
        guard = settled(ChassisGuard())
        guard.observe_cmd_vel(1.05, magnitude=0.05)
        self.assertFalse(guard.parked(1.1)[0])
        settled(guard, 1.1, 1.6)
        self.assertTrue(guard.parked(1.6)[0])


class StalenessTest(unittest.TestCase):
    def test_stale_odometry_is_not_parked(self):
        guard = settled(ChassisGuard(odom_timeout=0.3))
        parked, detail = guard.parked(2.0)
        self.assertFalse(parked)
        self.assertIn('stale', detail)

    def test_reset_forgets_the_evidence(self):
        guard = settled(ChassisGuard())
        self.assertTrue(guard.parked(1.0)[0])
        guard.reset()
        self.assertFalse(guard.parked(1.0)[0])


class ValidationTest(unittest.TestCase):
    def test_parked_thresholds_must_be_tighter_than_the_drive_limits(self):
        with self.assertRaises(ValueError):
            ChassisGuard(parked_linear=1.0)

    def test_non_positive_window_is_refused(self):
        with self.assertRaises(ValueError):
            ChassisGuard(hold=0.0)


if __name__ == '__main__':
    unittest.main()
