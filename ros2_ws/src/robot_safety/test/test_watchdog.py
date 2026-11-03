"""Liveness and latching: the failure that matters is the short recovery."""

import unittest

from robot_safety.watchdog import SafetyLatch, Watchdog

SOURCES = {
    'joints': {'timeout': 1.0, 'required': True},
    'camera': {'timeout': 0.5, 'required': False},
}


def watchdog(**overrides):
    sources = dict(SOURCES)
    sources.update(overrides)
    return Watchdog(sources)


def healthy(watch, start=0.0, end=1.0, step=0.2):
    now = start
    while now <= end + 1e-9:
        for name in watch.sources:
            watch.beat(name, now)
        now = round(now + step, 6)
    return watch


class StalenessTest(unittest.TestCase):
    def test_fresh_sources_are_fine(self):
        watch = healthy(watchdog())
        report = watch.evaluate(1.0)
        self.assertTrue(report['ok'])
        self.assertEqual(report['stale'], [])

    def test_stale_source_is_named(self):
        watch = healthy(watchdog())
        report = watch.evaluate(2.2)
        self.assertIn('joints', report['stale'])
        self.assertIn('camera', report['stale'])
        self.assertFalse(report['ok'])

    def test_only_required_sources_decide_ok(self):
        watch = healthy(watchdog())
        watch.beat('joints', 2.6)
        report = watch.evaluate(2.8)
        self.assertEqual(report['stale'], ['camera'])
        self.assertTrue(report['ok'])

    def test_never_seen_is_reported_separately_from_stale(self):
        watch = watchdog()
        watch.beat('joints', 1.0)
        report = watch.evaluate(1.0)
        self.assertEqual(report['missing'], ['camera'])

    def test_age_of_a_missing_source_is_infinite(self):
        self.assertEqual(watchdog().age('joints', 5.0), float('inf'))

    def test_unknown_source_is_ignored_not_invented(self):
        watch = watchdog()
        self.assertFalse(watch.beat('lidar', 1.0))
        self.assertNotIn('lidar', watch.sources)

    def test_non_finite_beat_is_ignored(self):
        watch = watchdog()
        self.assertFalse(watch.beat('joints', float('nan')))
        self.assertEqual(watch.age('joints', 1.0), float('inf'))

    def test_reset_forgets_everything(self):
        watch = healthy(watchdog())
        watch.reset()
        self.assertEqual(watch.evaluate(1.0)['missing'], ['joints', 'camera'])


class ConfigurationTest(unittest.TestCase):
    def test_empty_source_list_is_refused(self):
        with self.assertRaises(ValueError):
            Watchdog({})

    def test_non_positive_timeout_is_refused(self):
        with self.assertRaises(ValueError):
            Watchdog({'x': {'timeout': 0.0}})

    def test_optional_by_default(self):
        watch = Watchdog({'x': {'timeout': 1.0}})
        self.assertFalse(watch.sources['x']['required'])


class LatchTest(unittest.TestCase):
    def test_going_stale_latches(self):
        watch = healthy(watchdog())
        latch = SafetyLatch(watch, recovery_s=1.0)
        latch.evaluate(1.0)
        report = latch.evaluate(3.0)
        self.assertTrue(report['latched'])
        self.assertIn('joints', report['latch_reason'])

    def test_recovery_alone_does_not_clear(self):
        """The dangerous case is a link that came back for a moment."""
        watch = healthy(watchdog())
        latch = SafetyLatch(watch, recovery_s=1.0)
        latch.evaluate(1.0)
        latch.evaluate(3.0)
        watch.beat('joints', 3.1)
        report = latch.evaluate(3.2)
        self.assertTrue(report['latched'])
        self.assertFalse(report['can_clear'])

    def test_clear_after_a_full_healthy_window(self):
        watch = healthy(watchdog())
        latch = SafetyLatch(watch, recovery_s=1.0)
        latch.evaluate(1.0)
        latch.evaluate(3.0)
        # The window is measured across evaluations, so the node has to see the
        # healthy period while it happens, not only once it is over.
        healthy(watch, 3.0, 3.2)
        latch.evaluate(3.2)
        healthy(watch, 3.4, 4.5)
        latch.evaluate(4.5)
        self.assertTrue(latch.can_clear(4.5))
        self.assertTrue(latch.clear(4.5))
        self.assertFalse(latch.latched)

    def test_clear_before_recovery_is_refused(self):
        watch = healthy(watchdog())
        latch = SafetyLatch(watch, recovery_s=5.0)
        latch.evaluate(1.0)
        latch.evaluate(3.0)
        healthy(watch, 3.0, 4.0)
        latch.evaluate(4.0)
        self.assertFalse(latch.clear(4.0))

    def test_clear_on_a_clean_system_is_a_no_op(self):
        latch = SafetyLatch(healthy(watchdog()))
        latch.evaluate(1.0)
        self.assertFalse(latch.clear(1.0))

    def test_optional_source_never_latches(self):
        watch = healthy(watchdog())
        latch = SafetyLatch(watch, recovery_s=1.0)
        latch.evaluate(1.0)
        watch.beat('joints', 5.0)
        report = latch.evaluate(5.0)
        self.assertFalse(report['latched'])
        self.assertIn('camera', report['stale'])

    def test_force_latches_from_outside(self):
        latch = SafetyLatch(healthy(watchdog()), recovery_s=1.0)
        latch.evaluate(1.0)
        latch.force('operator stop', 1.1)
        report = latch.evaluate(1.2)
        self.assertTrue(report['latched'])
        self.assertEqual(report['latch_reason'], 'operator stop')


class MissingSourceTest(unittest.TestCase):
    def test_missing_does_not_latch_by_default(self):
        latch = SafetyLatch(Watchdog(SOURCES), latch_on_missing=False,
                            startup_grace_s=100.0)
        report = latch.evaluate(1.0)
        self.assertFalse(report['latched'])

    def test_missing_latches_once_opted_in(self):
        latch = SafetyLatch(Watchdog(SOURCES), latch_on_missing=True)
        report = latch.evaluate(1.0)
        self.assertTrue(report['latched'])

    def test_missing_required_latches_after_the_startup_grace(self):
        latch = SafetyLatch(Watchdog(SOURCES), latch_on_missing=False,
                            startup_grace_s=2.0)
        self.assertFalse(latch.evaluate(1.0)['latched'])
        self.assertTrue(latch.evaluate(3.5)['latched'])

    def test_a_freshly_hit_optional_gap_does_not_latch(self):
        watch = Watchdog({'cam': {'timeout': 0.5, 'required': False}})
        watch.beat('cam', 1.0)
        latch = SafetyLatch(watch, recovery_s=1.0)
        self.assertFalse(latch.evaluate(3.0)['latched'])


class ValidationTest(unittest.TestCase):
    def test_non_positive_recovery_is_refused(self):
        with self.assertRaises(ValueError):
            SafetyLatch(Watchdog(SOURCES), recovery_s=0.0)

    def test_negative_grace_is_refused(self):
        with self.assertRaises(ValueError):
            SafetyLatch(Watchdog(SOURCES), startup_grace_s=-1.0)


if __name__ == '__main__':
    unittest.main()
