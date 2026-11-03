"""The gate between a predicted action and a physical arm.

Two different things get called "safety" in this project and they must not be
confused:

* the arm's own health latch (``armpi_ros2.arm_health``) decides whether the
  hardware is trustworthy. This module never re-implements it; it consumes the
  verdict published on ``/arm_controller/status``.
* this filter decides whether a *particular action* is admissible given that the
  hardware is fine -- right frame, right units, fresh plan, inside the reachable
  envelope, small enough to execute in one command.

The rejection semantics follow the design document, which is worth restating
because it is the part people get wrong:

* a magnitude limit is *limited*, not rejected: exceeding the per-step
  displacement means "slow down", so the whole action is scaled uniformly and
  the commanded direction is preserved;
* everything else is *rejected*: a wrong frame, a stale plan, an unreachable
  point or a component this platform cannot realise are not fixed by making the
  numbers smaller. Scaling them would turn a detectable bug into a small,
  undetectable one.

Scaling is uniform across all motion components on purpose. Clamping components
independently keeps every number inside its limit and bends the path, which is
how a straight approach becomes a curved one and misses a 35 mm ball.
"""

import math

from .spec import (ACTION_COMPONENTS, ACTION_DIM, ACTION_FRAME,
                   ACTION_SPACE_ID, DEFAULT_MAX_STEP_TICKS, MIN_MOVE_DURATION_S,
                   NON_REALIZABLE_COMPONENTS, REALIZABLE_MASK)


class Code:
    """Stable machine-readable rejection codes. Logged, compared, never reworded."""
    OK = 'ok'
    DIMENSION = 'dimension'
    NONFINITE = 'nonfinite'
    FRAME = 'frame'
    SPACE = 'action_space'
    REALIZABLE = 'unrealizable'
    EXPIRED = 'expired'
    STALE = 'stale_observation'
    WORKSPACE = 'workspace'
    SPEED = 'speed'
    GRIPPER = 'gripper_range'
    GRIPPER_STEP = 'gripper_step'
    BASE_MOVING = 'base_not_parked'
    ESTOP = 'estop_latched'
    HEALTH = 'arm_not_healthy'


class Workspace:
    """Conservative box in the arm-base frame, used as a cheap pre-filter.

    The real authority on reachability is the vendor IK plus the servo limits;
    this box only rejects obvious nonsense before any solver runs. It is
    deliberately narrower than the analytic reach (0.3585 m from the shoulder
    axis) because the servo limits cut the useful set far inside it, and it must
    be tightened once the reachable set has actually been measured.

    Vendor frame: x lateral, y forward, z up, origin at the arm base.
    """

    __slots__ = ('x_min', 'x_max', 'y_min', 'y_max', 'z_min', 'z_max',
                 'max_radius', 'min_radius')

    def __init__(self, x=(-0.22, 0.22), y=(0.03, 0.33), z=(0.03, 0.34),
                 max_radius=0.34, min_radius=0.05):
        for value in (*x, *y, *z, max_radius, min_radius):
            if not math.isfinite(value):
                raise ValueError('Workspace bounds must be finite')
        if not (x[0] < x[1] and y[0] < y[1] and z[0] < z[1]):
            raise ValueError('Workspace box is empty')
        if not 0.0 < min_radius < max_radius:
            raise ValueError('Workspace radii are inconsistent')
        self.x_min, self.x_max = float(x[0]), float(x[1])
        self.y_min, self.y_max = float(y[0]), float(y[1])
        self.z_min, self.z_max = float(z[0]), float(z[1])
        self.max_radius = float(max_radius)
        self.min_radius = float(min_radius)

    def contains(self, point):
        x, y, z = point
        radius = math.hypot(x, y)
        return (self.x_min <= x <= self.x_max
                and self.y_min <= y <= self.y_max
                and self.z_min <= z <= self.z_max
                and self.min_radius <= radius <= self.max_radius)

    def describe(self):
        return ('x %.3f..%.3f, y %.3f..%.3f, z %.3f..%.3f, radius %.3f..%.3f'
                % (self.x_min, self.x_max, self.y_min, self.y_max,
                   self.z_min, self.z_max, self.min_radius, self.max_radius))


class Limits:
    """Per-action limits. Defaults are derived from this arm, not from taste.

    ``max_step_m`` is set to the measured grasp tolerance band (the target point
    repeats to 9.13 mm over a 20 s static window), because a step larger than the
    tolerance cannot be corrected by the next observation. The resulting speed
    then follows from the control period instead of being asserted separately.
    """

    __slots__ = ('max_step_m', 'max_step_rad', 'control_dt', 'max_gripper_step',
                 'max_observation_age_s', 'max_chunk_age_s', 'min_scale',
                 'workspace')

    def __init__(self, max_step_m=0.010, max_step_rad=math.radians(5.0),
                 control_dt=0.35, max_gripper_step=0.35,
                 max_observation_age_s=0.75, max_chunk_age_s=1.5,
                 min_scale=0.25, workspace=None):
        numeric = (max_step_m, max_step_rad, control_dt, max_gripper_step,
                   max_observation_age_s, max_chunk_age_s, min_scale)
        if not all(math.isfinite(value) for value in numeric):
            raise ValueError('Limits must be finite')
        if max_step_m <= 0 or max_step_rad <= 0:
            raise ValueError('Per-step limits must be positive')
        # The arm node accepts one timed move of at least MIN_MOVE_DURATION_S;
        # a control period below that cannot be executed however small the step.
        if not MIN_MOVE_DURATION_S * 0.99 <= control_dt <= 5.0:
            raise ValueError('control_dt must be within the bus move duration '
                             'range (%.2f..5.0 s)' % MIN_MOVE_DURATION_S)
        if not 0.0 < max_gripper_step <= 1.0:
            raise ValueError('max_gripper_step must be within (0, 1]')
        if max_observation_age_s <= 0 or max_chunk_age_s <= 0:
            raise ValueError('Freshness windows must be positive')
        if not 0.05 <= min_scale <= 1.0:
            raise ValueError('min_scale must be within [0.05, 1]')
        self.max_step_m = float(max_step_m)
        self.max_step_rad = float(max_step_rad)
        self.control_dt = float(control_dt)
        self.max_gripper_step = float(max_gripper_step)
        self.max_observation_age_s = float(max_observation_age_s)
        self.max_chunk_age_s = float(max_chunk_age_s)
        self.min_scale = float(min_scale)
        self.workspace = workspace if workspace is not None else Workspace()

    @property
    def max_speed_mps(self):
        """Implied TCP speed limit. Reported so the config has one source of truth."""
        return self.max_step_m / self.control_dt

    @property
    def max_ticks_per_command(self):
        return DEFAULT_MAX_STEP_TICKS


class Context:
    """Everything the filter needs that is not the action itself."""

    __slots__ = ('now', 'observation_stamp', 'chunk_stamp', 'chunk_validity_s',
                 'frame', 'action_space_id', 'realizable', 'tcp_position',
                 'gripper_opening', 'arm_ready', 'arm_detail', 'base_parked',
                 'base_detail', 'latched')

    def __init__(self, now, observation_stamp, chunk_stamp, chunk_validity_s,
                 tcp_position, gripper_opening, frame=ACTION_FRAME,
                 action_space_id=ACTION_SPACE_ID, realizable=REALIZABLE_MASK,
                 arm_ready=True, arm_detail='', base_parked=True,
                 base_detail='', latched=False):
        self.now = float(now)
        self.observation_stamp = float(observation_stamp)
        self.chunk_stamp = float(chunk_stamp)
        self.chunk_validity_s = float(chunk_validity_s)
        self.frame = str(frame)
        self.action_space_id = str(action_space_id)
        self.realizable = tuple(float(flag) for flag in realizable)
        self.tcp_position = tuple(float(value) for value in tcp_position)
        self.gripper_opening = float(gripper_opening)
        self.arm_ready = bool(arm_ready)
        self.arm_detail = str(arm_detail)
        self.base_parked = bool(base_parked)
        self.base_detail = str(base_detail)
        self.latched = bool(latched)


class Verdict:
    """Result of one admission check. Immutable; feeds the published message."""

    __slots__ = ('accepted', 'code', 'reason', 'action', 'scale', 'dropped')

    def __init__(self, accepted, code=Code.OK, reason='', action=None,
                 scale=1.0, dropped=()):
        self.accepted = bool(accepted)
        self.code = str(code)
        self.reason = str(reason)
        self.action = tuple(action) if action is not None else (0.0,) * ACTION_DIM
        self.scale = float(scale)
        self.dropped = tuple(dropped)

    def __repr__(self):
        if self.accepted:
            return 'Verdict(accepted, scale=%.3f)' % self.scale
        return 'Verdict(rejected, %s: %s)' % (self.code, self.reason)


def _reject(code, reason):
    return Verdict(False, code, reason)


def _finite(values):
    return all(math.isfinite(value) for value in values)


class SafetyFilter:
    """Admission control for one action at a time.

    Stateless across calls except for the space/frame it expects, so it can be
    tested without a robot and reasoned about without a timeline.
    """

    def __init__(self, limits=None, action_space_id=ACTION_SPACE_ID,
                 action_frame=ACTION_FRAME,
                 realizable_mask=REALIZABLE_MASK, allow_projection=False):
        self.limits = limits if limits is not None else Limits()
        self.action_space_id = str(action_space_id)
        self.action_frame = str(action_frame)
        self.realizable_mask = tuple(float(flag) for flag in realizable_mask)
        if len(self.realizable_mask) != ACTION_DIM:
            raise ValueError('realizable_mask must have %d entries' % ACTION_DIM)
        self.allow_projection = bool(allow_projection)

    # -- individual gates --------------------------------------------------

    def _check_preconditions(self, context):
        if context.latched:
            return _reject(Code.ESTOP, 'safety latch active; explicit reset required')
        if not context.arm_ready:
            return _reject(Code.HEALTH,
                           'arm not ready for motion: ' + (context.arm_detail or 'unknown'))
        if not context.base_parked:
            return _reject(Code.BASE_MOVING,
                           'base is not parked; arm deltas are relative to a moving '
                           'frame: ' + (context.base_detail or 'motion detected'))
        return None

    def _check_identity(self, context):
        if context.frame != self.action_frame:
            return _reject(Code.FRAME, 'chunk frame %r is not %r'
                           % (context.frame, self.action_frame))
        if context.action_space_id != self.action_space_id:
            return _reject(Code.SPACE, 'chunk action space %r is not %r'
                           % (context.action_space_id, self.action_space_id))
        return None

    def _check_freshness(self, context):
        chunk_age = context.now - context.chunk_stamp
        if not math.isfinite(chunk_age):
            return _reject(Code.EXPIRED, 'chunk timestamp is not a finite time')
        if chunk_age > context.chunk_validity_s:
            return _reject(Code.EXPIRED,
                           'chunk is %.3f s old, validity window is %.3f s'
                           % (chunk_age, context.chunk_validity_s))
        if chunk_age > self.limits.max_chunk_age_s:
            return _reject(Code.EXPIRED,
                           'chunk is %.3f s old, platform cap is %.3f s'
                           % (chunk_age, self.limits.max_chunk_age_s))
        # Negative age means the plan is newer than the clock that judges it.
        # A small amount is harmless clock skew; a large one is a broken stamp.
        if chunk_age < -self.limits.max_chunk_age_s:
            return _reject(Code.EXPIRED,
                           'chunk timestamp is %.3f s in the future' % -chunk_age)
        observation_age = context.now - context.observation_stamp
        if observation_age > self.limits.max_observation_age_s:
            return _reject(Code.STALE,
                           'observation is %.3f s old, cap is %.3f s'
                           % (observation_age, self.limits.max_observation_age_s))
        return None

    def _check_realizable(self, action):
        blocked = [name for name, flag, value
                   in zip(ACTION_COMPONENTS, self.realizable_mask, action)
                   if flag == 0.0 and abs(value) > 1e-9]
        if not blocked:
            return None, action, ()
        if not self.allow_projection:
            return (_reject(Code.REALIZABLE,
                            'commanded %s, which this arm cannot realise; enable '
                            'projection only if the policy is trained without it'
                            % ', '.join(blocked)), action, ())
        projected = tuple(0.0 if flag == 0.0 else value
                          for flag, value in zip(self.realizable_mask, action))
        return None, projected, tuple(blocked)

    def _check_envelope(self, context, action):
        limits = self.limits
        dx, dy, dz, drx, dry, drz, gripper = action
        if not 0.0 <= gripper <= 1.0:
            return _reject(Code.GRIPPER, 'gripper target %.3f is outside [0, 1]' % gripper)
        step = abs(gripper - context.gripper_opening)
        if step > limits.max_gripper_step + 1e-9:
            return _reject(Code.GRIPPER_STEP,
                           'gripper would move %.3f in one step, limit is %.3f'
                           % (step, limits.max_gripper_step))

        position_step = math.sqrt(dx * dx + dy * dy + dz * dz)
        rotation_step = math.sqrt(drx * drx + dry * dry + drz * drz)
        scale = 1.0
        if position_step > 0.0:
            scale = min(scale, limits.max_step_m / position_step)
        if rotation_step > 0.0:
            scale = min(scale, limits.max_step_rad / rotation_step)
        if scale < 1.0:
            if scale < limits.min_scale:
                return _reject(Code.SPEED,
                               'step of %.4f m / %.4f rad needs a scale of %.3f, '
                               'below the floor of %.3f; the policy is asking for '
                               'more motion than one command can carry'
                               % (position_step, rotation_step, scale,
                                  limits.min_scale))
            dx, dy, dz = dx * scale, dy * scale, dz * scale
            drx, dry, drz = drx * scale, dry * scale, drz * scale

        target = (context.tcp_position[0] + dx,
                  context.tcp_position[1] + dy,
                  context.tcp_position[2] + dz)
        if not limits.workspace.contains(target):
            return _reject(Code.WORKSPACE,
                           'target (%.3f, %.3f, %.3f) is outside the working '
                           'envelope [%s]'
                           % (target[0], target[1], target[2],
                              limits.workspace.describe()))

        return Verdict(True, Code.OK, '', (dx, dy, dz, drx, dry, drz, gripper), scale), target

    # -- public API --------------------------------------------------------

    def check(self, action, context):
        """One action in, one verdict out. Never raises on bad input."""
        blocked = self._check_preconditions(context)
        if blocked is not None:
            return blocked
        blocked = self._check_identity(context)
        if blocked is not None:
            return blocked
        blocked = self._check_freshness(context)
        if blocked is not None:
            return blocked

        try:
            values = tuple(float(value) for value in action)
        except (TypeError, ValueError):
            return _reject(Code.DIMENSION, 'action is not a sequence of numbers')
        if len(values) != ACTION_DIM:
            return _reject(Code.DIMENSION, 'action has %d components, expected %d'
                           % (len(values), ACTION_DIM))
        if not _finite(values):
            return _reject(Code.NONFINITE, 'action contains non-finite components')

        blocked, values, dropped = self._check_realizable(values)
        if blocked is not None:
            return blocked

        outcome = self._check_envelope(context, values)
        if isinstance(outcome, Verdict):
            return outcome
        envelope, _target = outcome
        envelope.dropped = dropped
        return envelope

    def describe(self):
        limits = self.limits
        return {
            'action_space_id': self.action_space_id,
            'action_frame': self.action_frame,
            'realizable_mask': list(self.realizable_mask),
            'non_realizable': list(NON_REALIZABLE_COMPONENTS),
            'allow_projection': self.allow_projection,
            'max_step_m': limits.max_step_m,
            'max_step_rad': limits.max_step_rad,
            'max_speed_mps': round(limits.max_speed_mps, 4),
            'control_dt': limits.control_dt,
            'max_gripper_step': limits.max_gripper_step,
            'max_observation_age_s': limits.max_observation_age_s,
            'max_chunk_age_s': limits.max_chunk_age_s,
            'min_scale': limits.min_scale,
            'workspace': limits.workspace.describe(),
        }
