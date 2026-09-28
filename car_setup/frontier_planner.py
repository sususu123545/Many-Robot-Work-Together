#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Conservative, local frontier selector for OccupancyGrid maps.

This is intentionally not a replacement for a global navigation stack. It
selects only frontiers reachable along a currently known-free straight
corridor; the lidar safety state machine remains responsible for every
motion command.
"""

import math


def _yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def select_frontier_goal(grid, robot_pose, travel_direction=1,
                         clearance_radius=0.25, max_range=3.0,
                         min_range=0.45, occupied_threshold=50,
                         gain_radius=0.65, blacklist=()):
    """Return the best safe straight-line frontier goal, or ``None``.

    ``robot_pose`` is (x, y, yaw) in the OccupancyGrid frame. Unknown cells
    are never considered traversable. A candidate is accepted only if a
    circular corridor, inflated by ``clearance_radius``, is known-free up to
    its approach point. ``blacklist`` is an iterable of map-frame (x, y)
    goals that recently failed.
    """
    try:
        info = grid.info
        width, height = int(info.width), int(info.height)
        resolution = float(info.resolution)
        data = grid.data
        origin = info.origin
        ox, oy = origin.position.x, origin.position.y
        oyaw = _yaw_from_quaternion(origin.orientation)
        rx, ry, ryaw = robot_pose
    except (AttributeError, TypeError, ValueError):
        return None
    if (width <= 0 or height <= 0 or resolution <= 0.0 or
            len(data) != width * height):
        return None

    co, so = math.cos(oyaw), math.sin(oyaw)

    def world_to_cell(x, y):
        dx, dy = x - ox, y - oy
        lx, ly = co * dx + so * dy, -so * dx + co * dy
        return int(math.floor(lx / resolution)), int(math.floor(ly / resolution))

    def cell_value(ix, iy):
        if ix < 0 or iy < 0 or ix >= width or iy >= height:
            return None
        return int(data[iy * width + ix])

    cx, cy = world_to_cell(rx, ry)
    if cell_value(cx, cy) is None or cell_value(cx, cy) < 0:
        return None

    step = max(resolution * 0.75, 0.025)
    radius_cells = int(math.ceil(clearance_radius / resolution))
    footprint_offsets = []
    for iy in range(-radius_cells, radius_cells + 1):
        for ix in range(-radius_cells, radius_cells + 1):
            longitudinal, lateral = ix * resolution, iy * resolution
            if math.hypot(longitudinal, lateral) <= clearance_radius:
                footprint_offsets.append((longitudinal, lateral))
    if not footprint_offsets:
        footprint_offsets = [(0.0, 0.0)]

    blacklist = tuple(blacklist)
    movement_yaw = ryaw + (math.pi if travel_direction < 0 else 0.0)
    candidates = []

    for degrees in range(-180, 180, 15):
        angle = math.radians(degrees)
        ca, sa = math.cos(angle), math.sin(angle)
        frontier_distance = None
        blocked = False
        d = max(step, min_range)
        while d <= max_range:
            wx, wy = rx + ca * d, ry + sa * d
            gx, gy = world_to_cell(wx, wy)
            # Check a circular robot footprint around the ray center.
            has_unknown = False
            for longitudinal, lateral in footprint_offsets:
                sx = wx + ca * longitudinal - sa * lateral
                sy = wy + sa * longitudinal + ca * lateral
                ix, iy = world_to_cell(sx, sy)
                value = cell_value(ix, iy)
                if value is None or value >= occupied_threshold:
                    blocked = True
                    break
                if value < 0:
                    has_unknown = True
            if blocked:
                break
            if has_unknown:
                frontier_distance = d
                break
            d += step

        if frontier_distance is None:
            continue
        approach = frontier_distance - max(2.0 * resolution, 0.15)
        if approach < min_range:
            continue
        goal_x, goal_y = rx + ca * approach, ry + sa * approach
        frontier_x = rx + ca * frontier_distance
        frontier_y = ry + sa * frontier_distance
        if any(math.hypot(frontier_x - bx, frontier_y - by) < 0.80
               for bx, by in blacklist):
            continue

        # Estimate information gain in a shallow fan beyond the frontier.
        gain = 0
        lateral_cells = int(math.ceil(gain_radius / resolution))
        for forward in range(0, int(math.ceil(0.9 / resolution)) + 1):
            along = frontier_distance + forward * resolution
            for side in range(-lateral_cells, lateral_cells + 1):
                lateral = side * resolution
                if abs(lateral) > gain_radius:
                    continue
                wx = rx + ca * along - sa * lateral
                wy = ry + sa * along + ca * lateral
                ix, iy = world_to_cell(wx, wy)
                value = cell_value(ix, iy)
                if value is not None and value < 0:
                    gain += 1
        if gain == 0:
            continue

        heading_delta = math.atan2(math.sin(angle - movement_yaw),
                                   math.cos(angle - movement_yaw))
        turn_factor = 1.0 - 0.25 * abs(heading_delta) / math.pi
        score = (gain * resolution * resolution / (0.35 + approach)) * turn_factor
        candidates.append({
            'x': goal_x,
            'y': goal_y,
            'frontier_x': frontier_x,
            'frontier_y': frontier_y,
            'distance': approach,
            'heading': angle,
            'gain_cells': gain,
            'score': score,
        })

    if not candidates:
        return None
    # Stable tie-break: prefer the closer candidate, then smaller heading turn.
    return max(candidates, key=lambda c: (c['score'], -c['distance'],
                                         -abs(math.atan2(
                                             math.sin(c['heading'] - movement_yaw),
                                             math.cos(c['heading'] - movement_yaw)))))
