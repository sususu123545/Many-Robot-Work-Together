"""Action de-normalisation, with the checks that stop it from silently lying.

A policy emits normalised actions. Turning them into metres and radians needs
the statistics of the dataset the model was trained on. Getting that wrong does
not raise anywhere: it produces a perfectly finite action that is simply the
wrong size, and the arm drives into the table at a confident 3x speed.

The two failure modes worth engineering against, both observed in practice:

* a channel whose training statistics have near-zero spread (a gripper that was
  almost never actuated), so dividing by it explodes;
* statistics from a different action space replayed against this one, which
  looks numerically healthy and is dimensionally meaningless.

Both are refused here rather than clamped.
"""

import json
import math

from .spec import ACTION_COMPONENTS, ACTION_DIM

MODE_MEAN_STD = 'mean_std'
MODE_MIN_MAX = 'min_max'
_MODES = (MODE_MEAN_STD, MODE_MIN_MAX)

#: A spread smaller than this cannot be divided by. Chosen an order of magnitude
#: below the smallest physically meaningful action (1 mm = 1e-3 m) so that any
#: channel which passes really does carry signal.
MIN_SPREAD = 1e-4


class NormalizationError(ValueError):
    """Statistics are unusable for this action space."""


def _floats(values, label):
    try:
        result = tuple(float(value) for value in values)
    except (TypeError, ValueError):
        raise NormalizationError(label + ' must be a sequence of numbers')
    if len(result) != ACTION_DIM:
        raise NormalizationError('%s must have %d entries, got %d'
                                 % (label, ACTION_DIM, len(result)))
    if not all(math.isfinite(value) for value in result):
        raise NormalizationError(label + ' contains non-finite entries')
    return result


class NormalizationStats:
    """Per-component statistics for one action space.

    ``mean_std``: ``raw = normalised * std + mean``
    ``min_max``:  ``raw = normalised * (q99 - q01) + q01``

    The attribute names are shared between the two modes on purpose: ``centre``
    is the offset and ``scale`` is the multiplier, which is the only structure
    the de-normaliser needs to know.
    """

    __slots__ = ('action_space_id', 'mode', 'centre', 'scale', 'count', 'source')

    def __init__(self, action_space_id, mode, centre, scale, count=0, source=''):
        if not isinstance(action_space_id, str) or not action_space_id:
            raise NormalizationError('action_space_id must be a non-empty string')
        if mode not in _MODES:
            raise NormalizationError('Unknown normalisation mode: %r' % (mode,))
        self.action_space_id = action_space_id
        self.mode = mode
        self.centre = _floats(centre, 'centre')
        self.scale = _floats(scale, 'scale')
        small = [name for name, value in zip(ACTION_COMPONENTS, self.scale)
                 if abs(value) < MIN_SPREAD]
        if small:
            raise NormalizationError(
                'Statistics have no usable spread for %s; de-normalising these '
                'would explode. Fix the dataset statistics, do not clamp.'
                % ', '.join(small))
        self.count = int(count)
        self.source = str(source)

    def check_space(self, expected_action_space_id):
        if self.action_space_id != expected_action_space_id:
            raise NormalizationError(
                'Statistics belong to action space %r but this platform speaks '
                '%r' % (self.action_space_id, expected_action_space_id))

    def denormalize(self, normalised, expected_action_space_id=None):
        """Normalised 7-vector -> canonical physical 7-tuple."""
        self.check_space(expected_action_space_id or self.action_space_id)
        values = _floats(normalised, 'normalised action')
        return tuple(centre + value * scale
                     for centre, scale, value in zip(self.centre, self.scale, values))

    def normalize(self, physical):
        """Physical 7-tuple -> normalised 7-tuple. Used when recording episodes."""
        values = _floats(physical, 'physical action')
        return tuple((value - centre) / scale
                     for centre, scale, value in zip(self.centre, self.scale, values))

    def to_dict(self):
        return {'action_space_id': self.action_space_id, 'mode': self.mode,
                'centre': list(self.centre), 'scale': list(self.scale),
                'count': self.count, 'source': self.source,
                'components': list(ACTION_COMPONENTS)}

    @classmethod
    def from_dict(cls, payload):
        try:
            return cls(payload['action_space_id'], payload['mode'],
                       payload['centre'], payload['scale'],
                       payload.get('count', 0), payload.get('source', ''))
        except (KeyError, TypeError) as error:
            raise NormalizationError('Statistics file is missing %s' % (error,))

    @classmethod
    def load(cls, path):
        with open(path, 'r', encoding='utf-8') as handle:
            return cls.from_dict(json.load(handle))

    def __repr__(self):
        return ('NormalizationStats(space=%r, mode=%r, count=%d)'
                % (self.action_space_id, self.mode, self.count))
