#!/usr/bin/env python3
"""Lidar cleaning and walkability analysis.

FILTERING POLICY (revised 2026-11-06 after two collisions)
----------------------------------------------------------
The original "drop everything under 30 cm" rule was a mistake: it was based on
the car's position at that moment, and once the car turned and stood next to a
table, real table legs at 0.30-0.38 m were the ONLY thing protecting the tyres
-- and one round of searching had already been run with them filtered out.
The tyre hit a table corner.

The revised policy keeps ALL real returns and drops only the KNOWN fixed
self-occlusion: a narrow chassis sliver at 339-345 deg, ~0.17 m.  The arm is
NOT filtered here either -- it occludes the lidar only while extended, and the
car must not move while the arm is extended anyway (tuck first).  If an
arm-extended scan is fed in by mistake, the caller will see a wall of close
points at 0-30 deg and the clearance checks below will refuse to move.

ROTATION CLEARANCE
------------------
`rotation_clearance()` answers "can the car spin in place right now?"  The
chassis sweep radius is ~0.35 m; anything closer is a guaranteed scrape.
search_around must call this before every 90-degree chassis rotation.

WALKABILITY
-----------
`emptiest_sector()` scores a sector by its NEAREST point, not its farthest --
the old max(r) scoring happily picked the gap between two table legs because
the wall behind them was far away.  A sector is only walkable if EVERYTHING in
it (including a +-15 deg corridor check) is beyond min_clear.
"""

import math

# Fixed chassis occlusion, measured 2026-11-05/06.  The lidar sits INSIDE the
# shell, so FOUR shell edges live in its plane -- and all four move with the
# car (proven 2026-11-06: distances stayed constant while odometry showed
# 0.27 m of travel).  Each sector has its own measured max depth.
SELF_SECTORS = (
    (339.0, 345.0, 0.25),   # right-front shell sliver (~0.17 m)
    (0.0,   30.0,  0.12),   # front-left shell edge (~0.06-0.08 m)
    (325.0, 360.0, 0.15),   # front-right shell edge (~0.06-0.08 m)
    (145.0, 210.0, 0.32),   # tail shell edges (~0.25-0.29 m)
)

# The chassis sweep radius for in-place rotation (car diagonal + margin).
# Half-diagonal of the 0.30x0.25 m chassis = 0.196 m, + 5 cm margin = 0.25 m
# (matches auto_mapping.yaml's own figure).  The previous 0.40 was the
# DIAMETER mistaken for a radius -- a 2x inflation that made
# rotation_clearance refuse turns the car could comfortably make.
ROTATION_RADIUS_M = 0.25

# A walkable corridor must be at least this deep (car length + margin).
MIN_CORRIDOR_M = 0.60


def clean_scan(ranges, angle_increment):
    """Return (angle_deg, range_m) pairs with ONLY the known chassis sliver
    removed.  Real obstacles at any distance -- including 0.2 m table legs --
    are kept: they are the whole point of having a lidar.
    """
    out = []
    for i, r in enumerate(ranges):
        # Anything under 8 cm is the sensor's own mounting bracket or
        # shell, seen from any angle -- never a real obstacle.  The
        # 324 deg / 0.04 m blocker that stalled all four search turns on
        # 2026-11-06 was exactly this.
        if r is None or r <= 0.08:
            continue
        a = i * angle_increment * 180.0 / math.pi
        is_self = any(lo <= a <= hi and r < max_m for lo, hi, max_m in SELF_SECTORS)
        if is_self:
            continue
        out.append((a, r))
    return out


def sectors(points, bucket_deg=30.0):
    buckets = {}
    for a, r in points:
        k = int((a + 180.0) // bucket_deg) * bucket_deg - 180.0
        buckets.setdefault(k, []).append((a, r))
    return buckets


def rotation_clearance(points, radius_m=ROTATION_RADIUS_M):
    """Can the car spin in place?  Returns (ok, list_of_offending_points)."""
    offenders = [(a, r) for a, r in points if r < radius_m]
    return (len(offenders) == 0), offenders


def corridor_clear(points, centre_deg, half_width_deg=15.0, min_clear=MIN_CORRIDOR_M):
    """Is a corridor of half_width around centre_deg clear to min_clear depth?"""
    lo, hi = centre_deg - half_width_deg, centre_deg + half_width_deg
    for a, r in points:
        if lo <= a <= hi and r < min_clear:
            return False
    return True


def emptiest_sector(points, front_half=False, min_clear=MIN_CORRIDOR_M):
    """Pick the direction the car can actually walk toward.

    A sector qualifies only if every point inside its +-15 deg corridor is
    beyond min_clear (table legs at 0.3 m disqualify a sector even though the
    wall behind them is at 3 m).  Among qualifying sectors the DEEPEST one
    (largest nearest-point) wins.  Returns (centre_deg or None, depth_m).
    """
    buckets = sectors(points)
    best, best_score = None, None
    for k in sorted(buckets):
        centre = k + 15.0
        if front_half and not (-95.0 <= centre <= 95.0):
            continue
        pts = buckets[k]
        nearest = min(r for _, r in pts)
        if nearest < min_clear:
            continue
        if not corridor_clear(points, centre, 15.0, min_clear):
            continue
        if best_score is None or nearest > best_score:
            best, best_score = centre, nearest
    return best, best_score


def sector_name(angle_deg):
    """Human-readable compass name for a lidar angle.

    ROS convention: 0 = nose, angles increase COUNTERCLOCKWISE, so +45 deg is
    LEFT-forward and -45 (or 315) deg is RIGHT-forward.  A briefing on
    2026-11-06 briefly used mirrored names and mislabelled left as right --
    this helper exists so reports and logs can never do that again.
    """
    a = angle_deg % 360.0
    if a < 15 or a >= 345:
        return '正前'
    if a < 75:
        return '左前'
    if a < 105:
        return '正左'
    if a < 165:
        return '左后'
    if a < 195:
        return '正后'
    if a < 255:
        return '右后'
    if a < 285:
        return '正右'
    return '右前'


def nearest_in(points, centre_deg, half_width_deg=15.0):
    """Nearest obstacle inside a corridor -- for logging how tight a spot is."""
    lo, hi = centre_deg - half_width_deg, centre_deg + half_width_deg
    rs = [r for a, r in points if lo <= a <= hi]
    return min(rs) if rs else None
