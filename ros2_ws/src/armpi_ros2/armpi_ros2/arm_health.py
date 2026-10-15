"""ROS-independent arm safety state: complete feedback and explicit recovery.

No state transition issues motion. Communication recovery never clears a latch.
"""
import math
from .servo_protocol import JOINTS


def complete_feedback(samples, now, max_temperature=65, max_age=1.5):
    if set(samples) != set(JOINTS):
        return False
    for name, value in samples.items():
        _, _, _, low, high = JOINTS[name]
        try:
            raw, temperature, sampled = value['raw'], value['temperature'], value['time']
            if not all(math.isfinite(v) for v in (raw, temperature, sampled, now)):
                return False
            if not low <= raw <= high or not 0 <= temperature < max_temperature:
                return False
            if not 0 <= now - sampled < max_age:
                return False
        except (KeyError, TypeError, ValueError):
            return False
    return True


def motion_allowed(names, arm_enabled, gripper_enabled):
    return (bool(names) and len(set(names)) == len(names)
            and all(n in JOINTS for n in names)
            and all(gripper_enabled if n == 'r_joint' else arm_enabled for n in names))


class ArmHealth:
    def __init__(self, recovery_seconds=10.0, max_temperature=65):
        if not math.isfinite(recovery_seconds) or recovery_seconds < 10:
            raise ValueError('Recovery requires at least 10 seconds of healthy feedback')
        if not 1 <= max_temperature <= 65:
            raise ValueError('Temperature protection must not exceed 65 C')
        self.recovery_seconds = recovery_seconds
        self.max_temperature = max_temperature
        self.samples = {}
        self.complete_cycle = False
        self.pending = False
        self.healthy_since = None
        self.last_complete = None
        self.current_error = None
        self.last_fault = None
        self.fault_count = 0
        self.generation = 0
        self.startup_lock = True
        self.manual_estop = False
        self.fault_latched = False

    @property
    def estop(self):
        return self.startup_lock or self.manual_estop or self.fault_latched

    @property
    def fault(self):
        return self.last_fault['message'] if self.fault_latched and self.last_fault else None

    def begin_cycle(self):
        self.pending = True

    def fail(self, message, now, kind='communication'):
        self.pending = False
        self.complete_cycle = False
        self.healthy_since = None
        self.current_error = str(message)
        self.fault_latched = True
        self.fault_count += 1
        self.generation += 1
        self.last_fault = {'message': str(message), 'kind': kind,
                           'monotonic_time': now, 'sequence': self.fault_count}

    def success(self, samples, now):
        if not complete_feedback(samples, now, self.max_temperature):
            self.fail('Incomplete, stale or invalid whole-arm feedback', now)
            return False
        # A gap in polling is not a continuous healthy recovery window.
        if (self.last_complete is None or now < self.last_complete
                or now - self.last_complete >= 1.5 or self.healthy_since is None):
            self.healthy_since = now
        self.samples = {k: dict(v) for k, v in samples.items()}
        self.last_complete = now
        self.complete_cycle = True
        self.pending = False
        self.current_error = None
        return True

    def fresh(self, now):
        return self.complete_cycle and complete_feedback(self.samples, now, self.max_temperature)

    def healthy_seconds(self, now):
        if self.healthy_since is None or not self.fresh(now):
            return 0.0
        return max(0.0, now - self.healthy_since)

    def can_reset(self, now):
        return (not self.pending and self.current_error is None and self.fresh(now)
                and self.healthy_seconds(now) >= self.recovery_seconds)

    def acknowledge(self, now, expected_generation, stop_succeeded):
        if (not stop_succeeded or expected_generation != self.generation
                or not self.can_reset(now)):
            return False
        self.startup_lock = self.manual_estop = self.fault_latched = False
        self.generation += 1
        return True

    def manual_stop(self):
        self.manual_estop = True
        self.generation += 1

    def snapshot(self, now):
        fresh = self.fresh(now)
        if not fresh:
            state = 'READ_ERROR' if self.current_error else 'NO_FRESH_FEEDBACK'
        elif self.estop:
            state = 'RECOVERED_AWAIT_RESET' if self.can_reset(now) else 'RECOVERING'
        else:
            state = 'READY'
        reasons = []
        if self.startup_lock:
            reasons.append('startup_confirmation')
        if self.manual_estop:
            reasons.append('manual_estop')
        if self.fault_latched:
            reasons.append('fault_latched')
        return {'health_state': state, 'feedback_complete': fresh,
                'current_read_error': self.current_error,
                'last_fault': dict(self.last_fault) if self.last_fault else None,
                'fault_count': self.fault_count, 'stop_reasons': reasons,
                'healthy_seconds': round(self.healthy_seconds(now), 3),
                'reset_eligible': self.can_reset(now), 'poll_pending': self.pending}
