"""Rolling execution of a predicted action chunk.

The policy predicts H actions and the robot executes only the first k, then
re-observes and asks again (design document, section 3.4). That sentence hides
three contracts that this class makes explicit, because each of them has a
failure mode that looks like "the model is bad" from the outside:

1. **k is not a buffer length.** After k actions the remaining H-k are dropped,
   not queued for later. They were predicted from an observation that no longer
   describes the world.
2. **Actions are relative to the previous one.** Skipping a single action
   because it failed a check silently shifts every later target by that amount.
   A failed action therefore aborts the whole chunk, it does not advance the
   index.
3. **A chunk expires.** Even an unexecuted chunk becomes invalid; a stalled
   policy server must not leave a stale plan loadable.
"""

from .spec import ACTION_DIM, ACTION_FRAME, ACTION_SPACE_ID


class ChunkError(ValueError):
    """The submitted chunk cannot be executed as given."""


class ChunkExecutor:
    """Holds at most one chunk and hands out its actions in order."""

    def __init__(self, execute_steps=4, expected_action_space_id=ACTION_SPACE_ID,
                 expected_frame=ACTION_FRAME, max_horizon=64):
        if not isinstance(execute_steps, int) or execute_steps < 1:
            raise ValueError('execute_steps must be a positive integer')
        if not isinstance(max_horizon, int) or max_horizon < execute_steps:
            raise ValueError('max_horizon must be >= execute_steps')
        self.execute_steps = execute_steps
        self.expected_action_space_id = str(expected_action_space_id)
        self.expected_frame = str(expected_frame)
        self.max_horizon = max_horizon
        self._actions = ()
        self._index = 0
        self._stamp = 0.0
        self._validity = 0.0
        self._control_dt = 0.0
        self._policy_id = ''
        self._aborted_reason = ''

    # -- loading -----------------------------------------------------------

    def load(self, actions, *, control_dt, validity_s, stamp, policy_id='',
             action_space_id=ACTION_SPACE_ID, frame=ACTION_FRAME):
        """Validate and install a chunk. Replaces any chunk already loaded."""
        if action_space_id != self.expected_action_space_id:
            raise ChunkError('chunk action space %r is not %r'
                             % (action_space_id, self.expected_action_space_id))
        if frame != self.expected_frame:
            raise ChunkError('chunk frame %r is not %r' % (frame, self.expected_frame))
        if not actions:
            raise ChunkError('chunk is empty')
        if len(actions) > self.max_horizon:
            raise ChunkError('chunk has %d actions, cap is %d'
                             % (len(actions), self.max_horizon))
        if len(actions) < self.execute_steps:
            raise ChunkError('chunk has %d actions but %d are executed before '
                             're-observation' % (len(actions), self.execute_steps))
        if not isinstance(validity_s, (int, float)) or validity_s <= 0:
            raise ChunkError('validity must be positive')
        normalised = []
        for index, action in enumerate(actions):
            try:
                values = tuple(float(value) for value in action)
            except (TypeError, ValueError):
                raise ChunkError('action %d is not a sequence of numbers' % index)
            if len(values) != ACTION_DIM:
                raise ChunkError('action %d has %d components, expected %d'
                                 % (index, len(values), ACTION_DIM))
            normalised.append(values)
        self._actions = tuple(normalised)
        self._index = 0
        self._stamp = float(stamp)
        self._validity = float(validity_s)
        self._control_dt = float(control_dt)
        self._policy_id = str(policy_id)
        self._aborted_reason = ''

    # -- consuming ---------------------------------------------------------

    @property
    def loaded(self):
        return bool(self._actions)

    @property
    def index(self):
        return self._index

    @property
    def total(self):
        return len(self._actions)

    @property
    def policy_id(self):
        return self._policy_id

    @property
    def control_dt(self):
        return self._control_dt

    @property
    def stamp(self):
        return self._stamp

    @property
    def validity_s(self):
        return self._validity

    def remaining(self):
        if not self._actions:
            return 0
        return max(0, min(self.execute_steps, len(self._actions)) - self._index)

    def expired(self, now):
        return bool(self._actions) and (now - self._stamp) > self._validity

    def exhausted(self):
        """True once k actions have been consumed or the chunk was aborted."""
        return not self._actions or self._index >= min(self.execute_steps, len(self._actions))

    def next_action(self, now):
        """The next action to attempt, or None when there is nothing to do.

        Returns None rather than raising for the three states the caller has to
        tell apart anyway: no chunk, chunk consumed, chunk expired.
        """
        if self.exhausted() or self.expired(now):
            return None
        return self._actions[self._index]

    def commit(self):
        """Mark the action just handed out as executed."""
        if not self._actions or self._index >= len(self._actions):
            raise ChunkError('no action in flight to commit')
        self._index += 1

    def abort(self, reason):
        """Drop the rest of the chunk, keeping the reason for the result message.

        Used when an action fails a check. Continuing would execute deltas that
        were relative to the action that never happened.
        """
        self._aborted_reason = str(reason)
        self._index = len(self._actions) if self._actions else 0

    @property
    def aborted_reason(self):
        return self._aborted_reason

    def clear(self):
        self._actions = ()
        self._index = 0
        self._stamp = 0.0
        self._validity = 0.0
        self._control_dt = 0.0
        self._policy_id = ''
        self._aborted_reason = ''

    def state(self, now):
        """One of: idle, expired, aborted, executing, done."""
        if not self._actions:
            return 'idle'
        if self._aborted_reason:
            return 'aborted'
        if self.expired(now):
            return 'expired'
        if self.exhausted():
            return 'done'
        return 'executing'
