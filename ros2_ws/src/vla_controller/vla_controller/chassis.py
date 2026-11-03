"""Is the base actually standing still?

Phase I of the design does manipulation with the chassis parked, and the action
frame is ``arm_base``: every predicted delta assumes the base does not move
underneath the arm. So "the base is parked" is not a comfort check, it is a
precondition for the action being true.

Two independent signals are required, because either alone lies:

* ``/odom`` says the wheels are not turning. On a mecanum base on smooth floor
  the wheels slip, so odometry under-reports motion and a quiet reading is weak
  evidence. It is still the only measurement available.
* ``/cmd_vel`` says nobody is asking for motion. A fresh command with no visible
  motion yet means the base is about to move, which is worse than already moving.

The guard therefore requires a quiet window on both: no motion observed for
``hold`` seconds and no command for ``quiet`` seconds.
"""

import math

from .spec import (CHASSIS_MAX_ANGULAR_RPS, CHASSIS_MAX_LINEAR_MPS,
                   CMD_VEL_QUIET_S, PARKED_ANGULAR_RPS, PARKED_HOLD_S,
                   PARKED_LINEAR_MPS)


class ChassisGuard:
    """Rolling evidence that the base has been still for long enough to trust."""

    def __init__(self, parked_linear=PARKED_LINEAR_MPS,
                 parked_angular=PARKED_ANGULAR_RPS,
                 hold=PARKED_HOLD_S, quiet=CMD_VEL_QUIET_S,
                 odom_timeout=0.6):
        values = (parked_linear, parked_angular, hold, quiet, odom_timeout)
        if not all(math.isfinite(value) for value in values) or min(values) <= 0:
            raise ValueError('Chassis guard thresholds must be finite and positive')
        if parked_linear > CHASSIS_MAX_LINEAR_MPS or parked_angular > CHASSIS_MAX_ANGULAR_RPS:
            raise ValueError('Parked thresholds must be tighter than the drive limits')
        self.parked_linear = float(parked_linear)
        self.parked_angular = float(parked_angular)
        self.hold = float(hold)
        self.quiet = float(quiet)
        self.odom_timeout = float(odom_timeout)
        self._quiet_since = None
        self._last_odom = None
        self._last_command = None
        self._last_motion = None
        self.first_command_at = None

    # -- inputs ------------------------------------------------------------

    def observe_odom(self, vx, vy, wz, now):
        """Feed one measured body twist (REP-103: x forward, y left, rad/s)."""
        if not all(math.isfinite(value) for value in (vx, vy, wz, now)):
            # A non-finite twist is not evidence of stillness.
            self._quiet_since = None
            self._last_odom = now if math.isfinite(now) else None
            return
        if self._last_odom is not None and now < self._last_odom:
            self._quiet_since = None  # clock went backwards; restart the window
        self._last_odom = now
        linear = math.hypot(vx, vy)
        if linear > self.parked_linear or abs(wz) > self.parked_angular:
            self._quiet_since = None
            self._last_motion = now
        elif self._quiet_since is None:
            self._quiet_since = now

    def observe_cmd_vel(self, now, magnitude=1.0):
        """Feed one /cmd_vel arrival. A zero twist does not restart the window."""
        if not math.isfinite(now):
            return
        self.first_command_at = now
        linear = abs(float(magnitude))
        if linear > 0.0:
            self._last_command = now

    # -- verdict -----------------------------------------------------------

    def parked(self, now):
        """(is_parked, explanation). Explanation is always populated."""
        if not math.isfinite(now):
            return False, 'clock is not finite'
        if self._last_odom is None:
            return False, 'no /odom received yet; base motion is unverified'
        if now - self._last_odom > self.odom_timeout:
            return False, ('/odom is %.2f s stale (cap %.2f s); base motion is unverified'
                           % (now - self._last_odom, self.odom_timeout))
        if self._quiet_since is None:
            return False, 'base motion detected on /odom'
        quiet_for = now - self._quiet_since
        if quiet_for < self.hold:
            return False, ('base has been still for %.2f s, needs %.2f s'
                           % (quiet_for, self.hold))
        if self._last_command is not None:
            since_command = now - self._last_command
            if since_command < self.quiet:
                return False, ('/cmd_vel seen %.2f s ago (cap %.2f s); a command is '
                               'pending' % (since_command, self.quiet))
        return True, ('base parked for %.2f s' % quiet_for)

    def reset(self):
        """Forget the evidence. Used after an explicit operator acknowledgement."""
        self._quiet_since = None
        self._last_odom = None
        self._last_command = None
        self._last_motion = None
