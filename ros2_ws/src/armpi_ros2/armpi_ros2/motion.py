"""ROS-independent motion and odometry math, in SI units.

Actuator mapping reproduces the existing chassis node exactly. The encoder
inverse uses the three measured wheel-off-ground basis pulses (2026-10-10),
not the old odometry assumptions. Physical body axes still require a ground
test; motor command signs alone are not encoder calibration evidence.
"""

import math
from dataclasses import dataclass


def finite(*values):
    return all(math.isfinite(v) for v in values)


@dataclass(frozen=True)
class Geometry:
    a: float = 0.110
    b: float = 0.0975
    wheel_diameter: float = 0.0965
    pulses_per_rev: float = 7832.0

    def __post_init__(self):
        values = (self.a, self.b, self.wheel_diameter, self.pulses_per_rev)
        if not finite(*values) or min(values) <= 0:
            raise ValueError('Chassis geometry must be finite and positive')

    @property
    def metres_per_pulse(self):
        return math.pi * self.wheel_diameter / self.pulses_per_rev


def wheel_commands(vx, vy, wz, geometry=Geometry(), max_pulse=100):
    """REP-103 x forward, y left, rad/s -> signed pulses / 10 ms.

Legacy Cartesian commands were x RIGHT, y FORWARD, mm/s. The legacy
set_velocity implementation also negates translation AND angular velocity.
Keep that hardware mapping, not the legacy names, in this conversion.
"""
    if not finite(vx, vy, wz) or max_pulse <= 0 or max_pulse > 127:
        raise ValueError('Invalid command or signed 8-bit output limit')
    turn = wz * (geometry.a + geometry.b)
    speeds = (vx + vy + turn, -vx + vy - turn,
              -vx + vy + turn, vx + vy - turn)
    pulses = [speed / geometry.metres_per_pulse * 0.01 for speed in speeds]
    scale = max(1.0, max(abs(p) for p in pulses) / max_pulse)
    return [int(p / scale) for p in pulses]


def encoder_delta(new, old):
    """Signed difference with int32 wrap, including the negative boundary."""
    return ((int(new) - int(old) + 2**31) % 2**32) - 2**31


def wheel_displacement(deltas, geometry=Geometry(), signs=(1.0, 1.0, 1.0),
                       channel_signs=(1.0, 1.0, 1.0, 1.0)):
    """Channel counts -> body dx, dy, yaw; signs must be calibrated separately.

    Measured bases: forward [+,-,-,+], left [+,+,+,+], CCW [+,-,+,-].
    channel_signs correct individual encoder polarity, signs correct body axes.
    """
    if (len(deltas) != 4 or len(signs) != 3 or len(channel_signs) != 4
            or not finite(*deltas, *signs, *channel_signs)
            or any(s not in (-1.0, 1.0) for s in (*signs, *channel_signs))):
        raise ValueError('Four finite deltas and calibrated +/-1 encoder signs required')
    m = [float(d) * s * geometry.metres_per_pulse for d, s in zip(deltas, channel_signs)]
    dx = (m[0] - m[1] - m[2] + m[3]) / 4 * signs[0]
    dy = sum(m) / 4 * signs[1]
    da = (m[0] - m[1] + m[2] - m[3]) / (4 * (geometry.a + geometry.b)) * signs[2]
    return dx, dy, da


def integrate_pose(pose, dx, dy, da):
    """Exact body displacement integration, including lateral arcs."""
    x, y, yaw = pose
    if not finite(x, y, yaw, dx, dy, da):
        raise ValueError('Non-finite odometry input')
    if abs(da) < 1e-9:
        wx = dx * math.cos(yaw) - dy * math.sin(yaw)
        wy = dx * math.sin(yaw) + dy * math.cos(yaw)
    else:
        end = yaw + da
        wx = (dx * (math.sin(end) - math.sin(yaw))
              + dy * (math.cos(end) - math.cos(yaw))) / da
        wy = (-dx * (math.cos(end) - math.cos(yaw))
              + dy * (math.sin(end) - math.sin(yaw))) / da
    return x + wx, y + wy, math.atan2(math.sin(yaw + da), math.cos(yaw + da))


def rotate_vector(vector, roll, pitch, yaw):
    """Rz(yaw) Ry(pitch) Rx(roll), matching the static TF parent<-child."""
    x, y, z = vector
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    y, z = cr * y - sr * z, sr * y + cr * z
    x, z = cp * x + sp * z, -sp * x + cp * z
    return cy * x - sy * y, sy * x + cy * y, z


class MotionLease:
    """One current command; monotonic watchdog and explicit emergency latch."""
    def __init__(self, timeout=0.35, max_linear=0.08, max_angular=0.20):
        if not finite(timeout, max_linear, max_angular) or min(timeout, max_linear, max_angular) <= 0:
            raise ValueError('Motion limits must be finite and positive')
        self.timeout = timeout
        self.max_linear = max_linear
        self.max_angular = max_angular
        self.command = (0.0, 0.0, 0.0)
        self.received_at = None
        self.estop = False

    def stop(self, latch=False):
        self.command = (0.0, 0.0, 0.0)
        self.received_at = None
        self.estop |= latch

    def accept(self, vx, vy, wz, now):
        if self.estop or not finite(vx, vy, wz, now):
            self.stop()
            return False
        speed = math.hypot(vx, vy)
        scale = max(1.0, speed / self.max_linear)
        self.command = (vx / scale, vy / scale,
                        max(-self.max_angular, min(self.max_angular, wz)))
        self.received_at = now
        return True

    def output(self, now):
        if (self.estop or self.received_at is None or not finite(now)
                or not 0 <= now - self.received_at <= self.timeout):
            self.stop()
        return self.command

    def reset(self):
        self.stop()
        self.estop = False  # A fresh command is required; never replay old motion.
