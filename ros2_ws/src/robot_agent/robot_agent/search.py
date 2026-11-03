"""Step-and-stare search for a target the camera cannot see yet.

Why not spin continuously
-------------------------
The detector gates on stability: it only localises a ball after five frames with
the centre moving less than a few pixels (``red_ball_detector``'s
``stability_frames`` / ``stability_center_px``). A continuous rotation at even
0.15 rad/s moves the target tens of pixels between frames, so that gate never
opens and the robot turns right past a ball it is looking at. The search
therefore has to stop, let the chassis settle, and only then look.

Why not just watch longer
-------------------------
A stationary camera cannot see behind itself, and this camera is mounted on the
arm with a field of view of roughly 70 degrees. Covering a full revolution needs
the base to turn.

The state machine
-----------------
``OBSERVE`` -> (nothing found) -> ``STEP`` -> ``SETTLE`` -> ``OBSERVE`` ...
A detection during ``STEP`` stops the turn immediately and falls through the
same settle/observe path, so a ball seen mid-turn is confirmed by the same rule
as one seen while parked. Nothing is declared found on a single frame.

All of this is pure Python: no ROS, no robot, no clock of its own.
"""

import math


class ScanState:
    OBSERVE = 'observe'
    STEP = 'step'
    SETTLE = 'settle'
    FOUND = 'found'
    EXHAUSTED = 'exhausted'
    ABORTED = 'aborted'
    IDLE = 'idle'


class Command:
    """What the caller should do this tick."""

    __slots__ = ('kind', 'wz', 'state', 'travelled_rad', 'detail')

    def __init__(self, kind, wz=0.0, state=ScanState.IDLE, travelled_rad=0.0, detail=''):
        self.kind = str(kind)
        self.wz = float(wz)
        self.state = str(state)
        self.travelled_rad = float(travelled_rad)
        self.detail = str(detail)

    def __repr__(self):
        return 'Command(%s, wz=%.3f, state=%s, turned=%.1f deg)' % (
            self.kind, self.wz, self.state, math.degrees(self.travelled_rad))


class ScanPlanner:
    """Turn a stream of "is the target visible" into a sequence of moves."""

    def __init__(self, step_rad=math.radians(45.0), max_sweep_rad=2 * math.pi,
                 spin_speed=0.18, settle_s=0.8, observe_s=2.0, confirm_s=0.4,
                 max_duration_s=240.0, direction=1.0):
        for name, value in (('step_rad', step_rad), ('max_sweep_rad', max_sweep_rad),
                            ('spin_speed', spin_speed), ('settle_s', settle_s),
                            ('observe_s', observe_s), ('confirm_s', confirm_s),
                            ('max_duration_s', max_duration_s)):
            if not math.isfinite(value) or value <= 0:
                raise ValueError('%s must be finite and positive' % name)
        if confirm_s > observe_s:
            raise ValueError('confirm_s cannot exceed observe_s; nothing could ever confirm')
        if direction not in (-1.0, 1.0):
            raise ValueError('direction must be +1 (counter-clockwise) or -1')
        self.step_rad = float(step_rad)
        self.max_sweep_rad = float(max_sweep_rad)
        self.spin_speed = float(spin_speed)
        self.settle_s = float(settle_s)
        self.observe_s = float(observe_s)
        self.confirm_s = float(confirm_s)
        self.max_duration_s = float(max_duration_s)
        self.direction = float(direction)
        self._reset_internals()

    def _reset_internals(self):
        self.state = ScanState.IDLE
        self.travelled_rad = 0.0
        self.started_at = None
        self.phase_started_at = None
        self.phase_elapsed = 0.0
        self.confirm_since = None
        self.steps_done = 0
        self.abort_reason = ''
        self._last_tick = None

    # -- lifecycle ---------------------------------------------------------

    def start(self, now):
        self._reset_internals()
        self.state = ScanState.OBSERVE
        self.started_at = float(now)
        self.phase_started_at = float(now)
        self._last_tick = float(now)

    def stop(self):
        self._reset_internals()

    @property
    def running(self):
        return self.state in (ScanState.OBSERVE, ScanState.STEP, ScanState.SETTLE)

    def phase_elapsed_s(self):
        """Seconds spent in the current phase, not since the search started."""
        return self.phase_elapsed

    # -- main step ---------------------------------------------------------

    def tick(self, now, detection_valid, abort_reason=''):
        """Advance one control tick and return the :class:`Command` to execute.

        ``detection_valid`` is the detector's own verdict (its stability gate has
        already been applied); this planner does not re-judge pixels.
        """
        if self.state == ScanState.IDLE:
            return Command('hold', state=self.state)

        dt = 0.0 if self._last_tick is None else max(0.0, float(now) - self._last_tick)
        self._last_tick = float(now)
        self.phase_elapsed = float(now) - self.phase_started_at

        if abort_reason:
            self.state = ScanState.ABORTED
            self.abort_reason = str(abort_reason)
            return Command('stop', state=self.state, travelled_rad=self.travelled_rad,
                           detail=self.abort_reason)

        if float(now) - self.started_at > self.max_duration_s:
            self.state = ScanState.ABORTED
            self.abort_reason = 'search exceeded %.1f s' % self.max_duration_s
            return Command('stop', state=self.state, travelled_rad=self.travelled_rad,
                           detail=self.abort_reason)

        if self.state in (ScanState.FOUND, ScanState.EXHAUSTED, ScanState.ABORTED):
            return Command('stop', state=self.state,
                           travelled_rad=self.travelled_rad, detail=self.abort_reason)

        if self.state == ScanState.STEP:
            return self._tick_step(now, dt, detection_valid)
        if self.state == ScanState.SETTLE:
            return self._tick_settle(now)
        return self._tick_observe(now, detection_valid)

    # -- phases ------------------------------------------------------------

    def _tick_observe(self, now, detection_valid):
        if detection_valid:
            if self.confirm_since is None:
                self.confirm_since = now
            elif now - self.confirm_since >= self.confirm_s:
                self.state = ScanState.FOUND
                return Command('found', state=self.state,
                               travelled_rad=self.travelled_rad,
                               detail='held for %.2f s' % (now - self.confirm_since))
        else:
            self.confirm_since = None

        if self.phase_elapsed >= self.observe_s:
            return self._begin_step(now)
        return Command('hold', state=self.state, travelled_rad=self.travelled_rad)

    def _begin_step(self, now):
        if self.travelled_rad >= self.max_sweep_rad:
            self.state = ScanState.EXHAUSTED
            return Command('stop', state=self.state, travelled_rad=self.travelled_rad,
                           detail='swept %.0f deg' % math.degrees(self.travelled_rad))
        self.state = ScanState.STEP
        self.phase_started_at = now
        self.phase_elapsed = 0.0
        self.confirm_since = None
        return Command('spin', wz=self.direction * self.spin_speed, state=self.state,
                       travelled_rad=self.travelled_rad)

    def _tick_step(self, now, dt, detection_valid):
        # A detection while turning is not trusted yet, but it is reason enough
        # to stop turning: the stability gate will decide in SETTLE.
        target = min(self.step_rad, self.max_sweep_rad - self.travelled_rad)
        self.travelled_rad += self.spin_speed * dt
        stop_now = (detection_valid
                    or self.travelled_rad >= self.max_sweep_rad
                    or self.spin_speed * self.phase_elapsed >= target)
        if not stop_now:
            return Command('spin', wz=self.direction * self.spin_speed, state=self.state,
                           travelled_rad=self.travelled_rad)
        self.state = ScanState.SETTLE
        self.phase_started_at = now
        self.phase_elapsed = 0.0
        self.steps_done += 1
        return Command('hold', state=self.state, travelled_rad=self.travelled_rad)

    def _tick_settle(self, now):
        if self.phase_elapsed >= self.settle_s:
            self.state = ScanState.OBSERVE
            self.phase_started_at = now
            self.phase_elapsed = 0.0
            self.confirm_since = None
            return Command('hold', state=self.state, travelled_rad=self.travelled_rad)
        return Command('hold', state=self.state, travelled_rad=self.travelled_rad)

    # -- reporting ---------------------------------------------------------

    def snapshot(self):
        return {
            'state': self.state,
            'travelled_deg': round(math.degrees(self.travelled_rad), 1),
            'sweep_target_deg': round(math.degrees(self.max_sweep_rad), 1),
            'steps_done': self.steps_done,
            'abort_reason': self.abort_reason,
            'phase_elapsed_s': round(self.phase_elapsed, 2),
        }
