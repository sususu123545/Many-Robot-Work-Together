"""Sensor-driven, manipulation-free G1 navigation through the minefield.

The planner consumes an estimated pose and obstacle points accumulated from
RGB-D. It knows only the mission finish coordinate and lane bounds; it never
queries USD prims or uses their positions as obstacle-map inputs.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

import numpy as np


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class NavigationCommand:
    phase: str
    action: str
    reason: str
    forward_mps: float = 0.0
    lateral_mps: float = 0.0
    yaw_rate_rps: float = 0.0
    path: tuple[tuple[float, float], ...] = ()
    clearance_m: float | None = None

    @property
    def velocity(self) -> tuple[float, float, float]:
        return self.forward_mps, self.lateral_mps, self.yaw_rate_rps


class MinefieldNavigator:
    """Receding-horizon 2-D lattice planner with fail-safe sensor gates."""

    def __init__(self, goal_x: float = 22.4, lane_width: float = 4.7,
                 speed: float = 0.22, obstacle_clearance: float = 0.48,
                 sensor_timeout_s: float = 0.35, localization_timeout_s: float = 1.0,
                 horizon_m: float = 3.0, replan_period_s: float = 0.20):
        if lane_width < 1.2 or speed <= 0 or goal_x <= 0:
            raise ValueError("invalid minefield navigation geometry or speed")
        self.goal_x = float(goal_x)
        self.lane_width = float(lane_width)
        self.speed = float(speed)
        self.obstacle_clearance = float(obstacle_clearance)
        self.sensor_timeout_s = float(sensor_timeout_s)
        self.localization_timeout_s = float(localization_timeout_s)
        self.horizon_m = float(horizon_m)
        self.replan_period_s = float(replan_period_s)
        self.path = np.empty((0, 2), dtype=float)
        self.command = np.zeros(3, dtype=float)
        self.phase = "WAIT_FOR_RGBD"
        self.last_reason = "waiting_for_sensor_frames"
        self.last_plan_s = -math.inf
        self.last_clearance_m: float | None = None
        self.replans = 0
        self.blocked_plans = 0
        self.stale_stops = 0
        self._latched_reason: str | None = None

    def _stop(self, phase: str, reason: str, *, latch: bool = False) -> NavigationCommand:
        self.command[:] = 0.0
        self.phase, self.last_reason = phase, reason
        if latch:
            self._latched_reason = reason
        return self._result("halt", reason)

    def _result(self, action: str, reason: str) -> NavigationCommand:
        return NavigationCommand(
            phase=self.phase, action=action, reason=reason,
            forward_mps=float(self.command[0]), lateral_mps=float(self.command[1]),
            yaw_rate_rps=float(self.command[2]),
            path=tuple((float(x), float(y)) for x, y in self.path),
            clearance_m=self.last_clearance_m,
        )

    def plan(self, pose: Iterable[float], obstacle_points: np.ndarray) -> bool:
        """Plan to a short forward horizon using only the rolling RGB-D map."""
        x, y, _ = (float(v) for v in pose)
        x_end = min(self.goal_x, x + self.horizon_m)
        xs = np.arange(x, x_end + 1e-6, 0.25, dtype=float)
        if len(xs) < 2 and x < self.goal_x:
            xs = np.array([x, min(self.goal_x, x + 0.25)], dtype=float)
        lateral_limit = self.lane_width / 2.0 - 0.40
        ys = np.arange(-lateral_limit, lateral_limit + 0.025, 0.05, dtype=float)
        grid_x, grid_y = np.meshgrid(xs, ys, indexing="ij")
        points = np.asarray(obstacle_points, dtype=float).reshape(-1, 2)
        if len(points):
            relevant = points[(points[:, 0] >= x - 0.35)
                              & (points[:, 0] <= x_end + 0.5)
                              & (np.abs(points[:, 1]) <= self.lane_width / 2.0 + 0.2)]
        else:
            relevant = points
        if len(relevant):
            samples = np.column_stack((grid_x.ravel(), grid_y.ravel()))
            clearance = np.sqrt(np.min(np.sum(
                (samples[:, None, :] - relevant[None, :, :]) ** 2, axis=2), axis=1))
            clearance = clearance.reshape(grid_x.shape)
        else:
            clearance = np.full(grid_x.shape, 5.0, dtype=float)

        cost = 0.10 * grid_y**2 + 0.045 / np.maximum(0.04, clearance - 0.38) ** 2
        cost[clearance < self.obstacle_clearance] = np.inf
        delta = ys[:, None] - ys[None, :]
        transition = 2.0 * delta**2
        transition[np.abs(delta) > 0.30] = np.inf
        cumulative = np.full_like(cost, np.inf)
        back = np.zeros(cost.shape, dtype=np.int32)
        initial_penalty = 60.0 * (ys - y) ** 2
        cumulative[0] = cost[0] + initial_penalty
        for i in range(1, len(xs)):
            options = cumulative[i - 1][:, None] + transition
            back[i] = np.argmin(options, axis=0)
            cumulative[i] = cost[i] + np.min(options, axis=0)
        if not np.isfinite(cumulative[-1]).any():
            self.path = np.empty((0, 2), dtype=float)
            self.blocked_plans += 1
            return False
        index = int(np.argmin(cumulative[-1]))
        selected = [index]
        for i in range(len(xs) - 1, 0, -1):
            index = int(back[i, index])
            selected.append(index)
        selected.reverse()
        self.path = np.column_stack((xs, ys[np.asarray(selected)]))
        self.replans += 1
        return True

    def update(self, *, pose: Iterable[float], obstacle_points: np.ndarray,
               now_s: float, sensor_stamp_s: float, localization_stamp_s: float,
               fallen: bool = False, e_stop: bool = False, dt_s: float = 0.02,
               warmup_s: float = 2.0) -> NavigationCommand:
        """Return bounded gait velocity requests; never issues manipulation."""
        now = float(now_s)
        x, y, yaw = (float(v) for v in pose)
        if self._latched_reason is not None:
            return self._stop("SAFE_STOP", self._latched_reason, latch=True)
        if e_stop:
            return self._stop("SAFE_STOP", "e_stop", latch=True)
        if fallen:
            return self._stop("SAFE_STOP", "fall_detected", latch=True)
        if now - float(sensor_stamp_s) > self.sensor_timeout_s:
            self.stale_stops += 1
            return self._stop("SENSOR_STOP", "rgbd_stale")
        if now - float(localization_stamp_s) > self.localization_timeout_s:
            return self._stop("LOCALIZATION_STOP", "sensor_localization_stale")
        if now < warmup_s:
            self.phase = "SENSOR_WARMUP"
            self.last_reason = "collecting_initial_rgbd_map"
            self.command[:] = 0.0
            return self._result("halt", self.last_reason)
        if x >= self.goal_x:
            self.path = np.empty((0, 2), dtype=float)
            return self._stop("COMPLETE", "minefield_exit_reached")

        points = np.asarray(obstacle_points, dtype=float).reshape(-1, 2)
        if len(points):
            # A rolling RGB-D map intentionally retains recently seen cells
            # behind the robot. They remain useful to the lattice planner, but
            # must not trigger an emergency stop after the robot has passed
            # them. Clearance is therefore measured in the forward motion
            # corridor, with a small overlap behind the base for turning.
            delta = points - np.array([x, y])
            forward = delta[:, 0] * math.cos(yaw) + delta[:, 1] * math.sin(yaw)
            motion_corridor = points[forward >= -0.15]
            nearest = (float(np.min(np.linalg.norm(motion_corridor - np.array([x, y]), axis=1)))
                       if len(motion_corridor) else math.inf)
            self.last_clearance_m = nearest if math.isfinite(nearest) else None
            if nearest < self.obstacle_clearance:
                return self._stop("CLEARANCE_STOP", "obstacle_inside_safety_margin")
        else:
            nearest = math.inf
            self.last_clearance_m = None

        if now - self.last_plan_s >= self.replan_period_s:
            found = self.plan((x, y, yaw), points)
            self.last_plan_s = now
            if not found:
                return self._stop("BLOCKED_STOP", "no_clear_path")
        if not len(self.path):
            return self._stop("BLOCKED_STOP", "no_planned_path")

        self.phase, self.last_reason = "CROSS_MINEFIELD", "rgbd_path_clear"
        # Track the local lattice path with the policy's supported lateral
        # velocity. Pure-pursuit yaw alone cannot realize the lattice's fairly
        # sharp lateral transitions at low forward speed.
        lookahead_x = min(self.goal_x, x + 0.65)
        target_y = float(np.interp(lookahead_x, self.path[:, 0], self.path[:, 1]))
        dx, dy = lookahead_x - x, target_y - y
        body_x = math.cos(yaw) * dx + math.sin(yaw) * dy
        body_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
        angle = wrap_angle(math.atan2(dy, max(0.05, dx)) - yaw)
        desired_forward = self.speed * max(0.55, math.cos(min(abs(angle), math.pi / 2)))
        desired_lateral = float(np.clip(0.8 * body_y, -0.22, 0.22))
        desired_yaw = float(np.clip(0.75 * angle, -0.18, 0.18))
        desired = np.array([desired_forward, desired_lateral, desired_yaw], dtype=float)
        slew = np.array([0.30, 0.20, 0.60], dtype=float) * max(0.0, float(dt_s))
        self.command += np.clip(desired - self.command, -slew, slew)
        return self._result("walk", self.last_reason)

    def reset_latched_stop(self) -> None:
        """Require an explicit operator reset after a fall or emergency stop."""
        self._latched_reason = None
        self.phase = "WAIT_FOR_RGBD"
        self.last_reason = "operator_reset"
        self.path = np.empty((0, 2), dtype=float)
        self.command[:] = 0.0

