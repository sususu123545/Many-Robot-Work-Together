"""Turn ``observation + policy output`` into a goal the executor will accept.

The executor is deliberately unforgiving.  It rejects a goal whose frame or
action-space id does not match, and its safety filter rejects stale
observations, unrealisable rotations, over-large steps and a moving chassis.
Every one of those refusals is correct.  A bridge that submits such a goal and
then reports "the model failed" would be lying about where the bug is, so all
of the executor-side preconditions are checked *here*, before a request is
sent, with the same constants the executor uses (``safety.Limits``).

The most important contract is the one about normalisation: a real policy emits
normalised numbers, and de-normalising them without the dataset statistics
produces finite, plausible, wrong-sized actions.  No exception is raised
anywhere downstream.  So a normalised policy output without matching statistics
is a hard refusal, not a fallback to identity scaling.
"""

import math

from vla_controller import spec
from vla_controller.normalization import NormalizationError, NormalizationStats
from vla_controller.safety import Limits

from .policy import POLICY_OUTPUT_NORMALIZED, PolicyOutput


class BridgeError(ValueError):
    """The observation/policy pair must not be submitted to the executor."""

    def __init__(self, code, reason):
        self.code = str(code)
        super().__init__('%s: %s' % (self.code, reason))


class Code:
    OK = 'ok'
    STALE = 'stale_observation'
    FUTURE = 'future_observation'
    CONTROL_DT = 'control_dt_mismatch'
    VALIDITY = 'chunk_validity'
    HORIZON = 'short_horizon'
    STATS = 'normalization_stats_missing'
    STATS_SPACE = 'normalization_stats_space'
    UNREALIZABLE = 'unrealizable'
    GRIPPER = 'gripper_range'
    NONFINITE = 'nonfinite'
    TYPE = 'bad_input'


class ChunkSpec:
    """A validated goal, expressed only with plain Python values.

    ``as_goal_kwargs()`` returns the two dictionaries the ROS 2 shell expands
    into ``VlaObservation`` and ``VlaActionChunk``.  Keeping the shapes here
    means a unit test can assert the exact wire contract without rclpy.
    """

    __slots__ = ('observation', 'chunk')

    def __init__(self, observation, chunk):
        self.observation = dict(observation)
        self.chunk = dict(chunk)

    def as_goal_kwargs(self):
        return {'observation': dict(self.observation), 'chunk': dict(self.chunk)}

    @property
    def action_count(self):
        return len(self.chunk['actions'])

    def __repr__(self):
        return ('ChunkSpec(policy=%r, actions=%d, dt=%.2f, frame=%r)'
                % (self.chunk['policy_id'], self.action_count,
                   self.chunk['control_dt'], self.chunk['action_frame']))


class BridgePlanner:
    """Validate, de-normalise and package one step of the VLA loop."""

    def __init__(self, limits=None, stats=None, execute_steps=4,
                 allow_projection=False):
        self.limits = limits if limits is not None else Limits()
        if stats is not None and not isinstance(stats, NormalizationStats):
            raise ValueError('stats must be a NormalizationStats or None')
        self.stats = stats
        if not isinstance(execute_steps, int) or execute_steps < 1:
            raise ValueError('execute_steps must be a positive integer')
        self.execute_steps = execute_steps
        self.allow_projection = bool(allow_projection)

    def describe(self):
        return {'control_dt': self.limits.control_dt,
                'max_observation_age_s': self.limits.max_observation_age_s,
                'max_chunk_age_s': self.limits.max_chunk_age_s,
                'execute_steps': self.execute_steps,
                'stats': repr(self.stats) if self.stats else None,
                'allow_projection': self.allow_projection}

    def plan(self, observation, policy_output, now):
        """Return a :class:`ChunkSpec` or raise :class:`BridgeError`.

        ``now`` is the chunk's generation time.  The chunk is stamped with it,
        while the observation keeps its own earlier stamp: the executor needs
        both to tell a fresh observation from a stale one.
        """
        if not isinstance(policy_output, PolicyOutput):
            raise BridgeError(Code.TYPE, 'policy_output must be a PolicyOutput')
        try:
            data = observation.as_msg_kwargs() if hasattr(observation, 'as_msg_kwargs') \
                else dict(observation)
            stamp = float(observation.stamp) if hasattr(observation, 'stamp') \
                else float(data['header']['stamp'])
        except (KeyError, TypeError, ValueError) as error:
            raise BridgeError(Code.TYPE, 'observation is malformed: %s' % error)
        now = float(now)
        if not all(math.isfinite(value) for value in (stamp, now)):
            raise BridgeError(Code.NONFINITE, 'observation or chunk time is not finite')
        age = now - stamp
        if age < 0:
            raise BridgeError(Code.FUTURE,
                              'observation is %.3f s in the future' % (-age))
        if age > self.limits.max_observation_age_s:
            raise BridgeError(
                Code.STALE,
                'observation is %.3f s old (executor allows %.3f s)'
                % (age, self.limits.max_observation_age_s))

        dt = policy_output.control_dt
        if abs(dt - self.limits.control_dt) > 1e-6:
            raise BridgeError(
                Code.CONTROL_DT,
                'policy control_dt %.3f s differs from executor %.3f s; the '
                'executor would execute at its own rate while logging the '
                "policy's" % (dt, self.limits.control_dt))
        if dt < spec.MIN_MOVE_DURATION_S * 0.99 or dt > spec.MAX_MOVE_DURATION_S:
            raise BridgeError(Code.CONTROL_DT,
                              'control_dt %.3f s is outside the bus move range' % dt)
        validity = policy_output.validity_s
        if validity > self.limits.max_chunk_age_s:
            raise BridgeError(
                Code.VALIDITY,
                'chunk validity %.3f s exceeds executor window %.3f s'
                % (validity, self.limits.max_chunk_age_s))
        if len(policy_output) < self.execute_steps:
            raise BridgeError(
                Code.HORIZON,
                'policy produced %d actions but %d are executed before '
                're-observation' % (len(policy_output), self.execute_steps))

        actions = self._physical_actions(policy_output)
        self._check_actions(actions)

        chunk = {
            'header': {'stamp': now, 'frame_id': spec.ACTION_FRAME},
            'policy_id': policy_output.policy_id,
            'action_space_id': spec.ACTION_SPACE_ID,
            'action_frame': spec.ACTION_FRAME,
            'control_dt': dt,
            'validity': validity,
            'actions': [dict(zip(spec.ACTION_COMPONENTS, action))
                        for action in actions],
            'realizable_mask': list(spec.REALIZABLE_MASK),
        }
        return ChunkSpec(data, chunk)

    # -- internals ---------------------------------------------------------

    def _physical_actions(self, output):
        if output.output_kind != POLICY_OUTPUT_NORMALIZED:
            return list(output.actions)
        if self.stats is None:
            raise BridgeError(
                Code.STATS,
                'policy emits normalised actions but no dataset statistics were '
                'loaded; refusing to guess a scale')
        try:
            self.stats.check_space(spec.ACTION_SPACE_ID)
            return [self.stats.denormalize(action, spec.ACTION_SPACE_ID)
                    for action in output.actions]
        except NormalizationError as error:
            raise BridgeError(Code.STATS_SPACE, str(error))

    def _check_actions(self, actions):
        for index, action in enumerate(actions):
            if len(action) != spec.ACTION_DIM:
                raise BridgeError(Code.TYPE, 'action %d has %d components'
                                  % (index, len(action)))
            if not all(math.isfinite(value) for value in action):
                raise BridgeError(Code.NONFINITE,
                                  'action %d contains non-finite values' % index)
            opening = action[-1]
            if not 0.0 <= opening <= 1.0:
                raise BridgeError(
                    Code.GRIPPER,
                    'action %d gripper %.3f is outside [0, 1]' % (index, opening))
            bad = [name for name, value, flag
                   in zip(spec.ACTION_COMPONENTS, action, spec.REALIZABLE_MASK)
                   if flag == 0.0 and abs(value) > 1e-9]
            if bad and not self.allow_projection:
                raise BridgeError(
                    Code.UNREALIZABLE,
                    'action %d commands %s, which this arm cannot realise; '
                    'projecting silently would teach the policy a lie'
                    % (index, ', '.join(bad)))
