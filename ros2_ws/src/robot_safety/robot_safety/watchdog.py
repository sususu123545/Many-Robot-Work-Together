"""Liveness supervision: who stopped talking, and does that stop the robot?

The robot already has two safety mechanisms, and this one is deliberately a
third, separate thing:

* ``armpi_ros2.arm_health`` latches on bad *feedback* from the servos;
* ``armpi_ros2.motion.MotionLease`` times out a single *command*;
* this module watches the *links* -- which publishers exist, when they last
  spoke, and whether a silent one should stop the machine.

The distinction matters because all three fail differently. A dead camera does
not make the servos lie; a silent policy server does not make the wheels turn.
Lumping them together is how a single missing topic ends up disabling an arm
that was perfectly healthy.

Latching rules, which is where these things usually go wrong:

* going stale latches;
* recovering does **not** clear the latch. Clearing needs an explicit request
  and a continuous healthy window, because the interesting failure is not "it
  broke" but "it broke, recovered for 200 ms, and everything resumed".
* a source that has never been seen is reported as missing, and whether that
  latches is a configuration decision, not a default.
"""

import math


class Watchdog:
    """Per-source staleness tracking with no latch and no clock of its own."""

    def __init__(self, sources):
        """``sources``: name -> {'timeout': float, 'required': bool}.

        ``timeout`` doubles as the expected period ceiling: a source that
        publishes at 2 Hz with a 1.0 s timeout is expected to miss no more than
        one message before it counts as dead.
        """
        if not sources:
            raise ValueError('Watchdog needs at least one source')
        self.sources = {}
        for name, spec in sources.items():
            if not isinstance(name, str) or not name:
                raise ValueError('Source names must be non-empty strings')
            timeout = float(spec.get('timeout', 0.0))
            if not math.isfinite(timeout) or timeout <= 0:
                raise ValueError('Timeout for %r must be positive' % name)
            self.sources[name] = {'timeout': timeout,
                                  'required': bool(spec.get('required', False))}
        self.last_seen = {}
        self.count = {}

    def beat(self, name, now):
        """Record a message from ``name``. Unknown names are ignored, not invented."""
        if name not in self.sources or not math.isfinite(now):
            return False
        self.last_seen[name] = float(now)
        self.count[name] = self.count.get(name, 0) + 1
        return True

    def age(self, name, now):
        if name in self.last_seen:
            return max(0.0, now - self.last_seen[name])
        return float('inf')

    def evaluate(self, now):
        """{'ok': bool, 'stale': [...], 'missing': [...], 'detail': {name: age}}"""
        stale, missing, detail = [], [], {}
        for name, spec in self.sources.items():
            age = self.age(name, now)
            detail[name] = age
            if age == float('inf'):
                missing.append(name)
            elif age > spec['timeout']:
                stale.append(name)
        required_bad = [name for name in stale + missing
                        if self.sources[name]['required']]
        return {'ok': not required_bad, 'stale': stale, 'missing': missing,
                'required_bad': required_bad, 'detail': detail}

    def reset(self):
        self.last_seen.clear()
        self.count.clear()


class SafetyLatch:
    """Turn "required link is bad" into a latched stop that needs a human.

    ``latch_on_missing`` decides whether a source that has never spoken counts
    as a failure. Default is False: a robot that has not started its camera
    publisher yet is not in the same category as one whose camera died
    mid-motion, and conflating them trains operators to clear latches blindly.
    """

    def __init__(self, watchdog, latch_on_missing=False, recovery_s=2.0,
                 startup_grace_s=10.0):
        if not math.isfinite(recovery_s) or recovery_s <= 0:
            raise ValueError('recovery_s must be positive')
        if not math.isfinite(startup_grace_s) or startup_grace_s < 0:
            raise ValueError('startup_grace_s must be non-negative')
        self.watchdog = watchdog
        self.latch_on_missing = bool(latch_on_missing)
        self.recovery_s = float(recovery_s)
        self.startup_grace_s = float(startup_grace_s)
        self.latched = False
        self.latch_reason = ''
        self.latched_at = None
        self.first_eval_at = None
        self.healthy_since = None

    def evaluate(self, now):
        """Update the latch and return a status mapping."""
        if self.first_eval_at is None:
            self.first_eval_at = now
        report = self.watchdog.evaluate(now)
        bad = list(report['stale'])
        if self.latch_on_missing:
            bad += report['missing']
        elif now - self.first_eval_at > self.startup_grace_s:
            # Past the grace window a never-seen required source is a real fault.
            bad += [name for name in report['missing']
                    if self.watchdog.sources[name]['required']]
        required_bad = [name for name in bad if self.watchdog.sources[name]['required']]

        if required_bad and not self.latched:
            self.latched = True
            self.latched_at = now
            self.healthy_since = None
            self.latch_reason = 'required link down: ' + ', '.join(sorted(set(required_bad)))
        if required_bad:
            self.healthy_since = None
        else:
            if self.healthy_since is None:
                self.healthy_since = now

        report.update({
            'latched': self.latched,
            'latch_reason': self.latch_reason,
            'can_clear': self.can_clear(now),
            'healthy_for': (0.0 if self.healthy_since is None
                            else max(0.0, now - self.healthy_since)),
            'latch_on_missing': self.latch_on_missing,
        })
        return report

    def can_clear(self, now):
        return (self.latched and self.healthy_since is not None
                and now - self.healthy_since >= self.recovery_s)

    def clear(self, now):
        """Explicit acknowledgement. Refused unless the healthy window is met."""
        if not self.can_clear(now):
            return False
        self.latched = False
        self.latch_reason = ''
        self.latched_at = None
        self.healthy_since = now
        return True

    def force(self, reason, now):
        """Latch from outside (an operator stop, or a policy violation)."""
        self.latched = True
        self.latched_at = now
        self.latch_reason = str(reason)
        self.healthy_since = None
