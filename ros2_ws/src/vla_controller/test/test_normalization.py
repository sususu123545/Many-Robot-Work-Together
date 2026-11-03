"""De-normalisation must fail loudly, because its failure mode is silent."""

import tempfile
import unittest

from vla_controller.normalization import (MODE_MEAN_STD, MODE_MIN_MAX,
                                          NormalizationError, NormalizationStats)
from vla_controller.spec import ACTION_SPACE_ID

SCALE = (0.010, 0.010, 0.010, 0.05, 0.05, 0.05, 0.5)
CENTRE = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5)


def stats(**overrides):
    values = dict(action_space_id=ACTION_SPACE_ID, mode=MODE_MEAN_STD,
                  centre=CENTRE, scale=SCALE)
    values.update(overrides)
    return NormalizationStats(**values)


class MeanStdTest(unittest.TestCase):
    def test_unit_actions_scale_to_the_physical_unit(self):
        raw = stats().denormalize([0, 1, -1, 0, 0, 0, 0])
        self.assertAlmostEqual(raw[1], 0.010)
        self.assertAlmostEqual(raw[2], -0.010)

    def test_centre_is_added(self):
        raw = stats(centre=(0.1, 0, 0, 0, 0, 0, 0.5)).denormalize([0] * 7)
        self.assertAlmostEqual(raw[0], 0.1)

    def test_round_trip(self):
        instance = stats()
        physical = (0.003, -0.002, 0.001, 0.01, 0, 0, 0.7)
        self.assertEqual(tuple(round(v, 12) for v in
                               instance.denormalize(instance.normalize(physical))),
                         tuple(round(v, 12) for v in physical))


class MinMaxTest(unittest.TestCase):
    def test_span_acts_as_the_multiplier(self):
        instance = NormalizationStats(ACTION_SPACE_ID, MODE_MIN_MAX,
                                      centre=(0, 0, 0, 0, 0, 0, 0),
                                      scale=(0.02, 0.02, 0.02, 0.1, 0.1, 0.1, 1.0))
        raw = instance.denormalize([0.5] * 7)
        self.assertAlmostEqual(raw[0], 0.01)
        self.assertAlmostEqual(raw[6], 0.5)


class RefusalTest(unittest.TestCase):
    def test_zero_spread_is_named_component_by_component(self):
        """A gripper that never moved in the dataset is the classic case."""
        bad = list(SCALE)
        bad[6] = 0.0
        with self.assertRaises(NormalizationError) as caught:
            stats(scale=tuple(bad))
        self.assertIn('gripper', str(caught.exception))

    def test_tiny_spread_is_refused_with_the_same_message(self):
        bad = list(SCALE)
        bad[0] = 1e-9
        with self.assertRaises(NormalizationError):
            stats(scale=tuple(bad))

    def test_wrong_action_space_is_refused(self):
        with self.assertRaises(NormalizationError):
            stats(action_space_id='old_v0').denormalize([0] * 7, ACTION_SPACE_ID)

    def test_wrong_length_is_refused(self):
        with self.assertRaises(NormalizationError):
            stats().denormalize([0] * 6)

    def test_non_finite_normalised_value_is_refused(self):
        with self.assertRaises(NormalizationError):
            stats().denormalize([0, 0, 0, 0, 0, 0, float('inf')])

    def test_unknown_mode_is_refused(self):
        with self.assertRaises(NormalizationError):
            stats(mode='z_score')


class FileTest(unittest.TestCase):
    def test_dict_round_trip_keeps_the_space_and_mode(self):
        instance = stats(count=1234, source='dataset-v3')
        restored = NormalizationStats.from_dict(instance.to_dict())
        self.assertEqual(restored.action_space_id, ACTION_SPACE_ID)
        self.assertEqual(restored.mode, MODE_MEAN_STD)
        self.assertEqual(restored.count, 1234)
        self.assertEqual(restored.scale, instance.scale)

    def test_load_from_disk(self):
        instance = stats()
        import json
        with tempfile.NamedTemporaryFile('w', suffix='.json', delete=False,
                                         encoding='utf-8') as handle:
            json.dump(instance.to_dict(), handle)
            path = handle.name
        self.assertEqual(NormalizationStats.load(path).centre, instance.centre)

    def test_missing_key_reports_which_one(self):
        with self.assertRaises(NormalizationError) as caught:
            NormalizationStats.from_dict({'action_space_id': ACTION_SPACE_ID})
        self.assertIn('mode', str(caught.exception))


if __name__ == '__main__':
    unittest.main()
