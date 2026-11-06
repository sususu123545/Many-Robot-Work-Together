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
# shell, so the shell edges, the arm and the wheels all live in its plane -- and
# everything here moves with the car (proven 2026-11-06: distances stayed
# constant while odometry showed 0.27 m of travel, and the arm's cluster slid
# from 10 deg to 38 deg when only the arm moved).
#
# Each entry is (from_deg, to_deg, max_m): drop the return when its angle is
# inside the range AND its distance is under max_m.  The max_m values are the
# measured maximum plus a 2-3 cm margin, not guesses -- see the commit that
# introduced them for the 30-frame profile behind each number.
#
# This table is per-chassis.  Change the lidar mount, the arm, or the wheels and
# it has to be re-measured; a stale entry either hides a real obstacle (too
# generous) or re-creates the "the robot thinks it is surrounded" failure (too
# tight).  tools/check_lidar_filter_sync.py keeps the web console identical.
SELF_SECTORS = (
    (139.0, 149.0, 0.18),   # rear-left shell corner (measured 0.131-0.157)
    (207.0, 238.0, 0.20),   # tail + left wheel (measured 0.131-0.172, 86 hits)
    (263.0, 282.0, 0.17),   # right wheel, outer face (measured 0.134-0.140)
    (303.0, 327.0, 0.18),   # right-front wheel (measured 0.131-0.151)
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

# Anything closer than this is the robot's own body, not an obstacle.
#
# Was 0.08 m until 2026-11-06, which was measured too low: the arm sits 0.081 to
# 0.125 m from the lidar and returns a cluster that flickers between 27 and 36
# points across 30-60 deg (five consecutive frames at one pose: 28/32/36/27/34,
# nearest 0.081-0.085 m).  Those returns made rotation_clearance refuse every
# turn AND made emptiest_sector return None, so find_and_grasp's relocation never
# ran -- the six "no-corridor" entries in artifacts/ball_history.json are this,
# not the room.
#
# 0.13 m is not a tuned guess: the chassis is 0.30 x 0.25 m and the lidar is
# mounted inside the body, so a return under ~0.13 m cannot come from outside it.
# Measured on 2026-11-06: 0.13 and 0.15 give identical decisions (rotation
# allowed, front sector 75 deg), so the value sits on a plateau rather than on a
# knife edge.  The 10 returns between 0.08 and 0.13 m are 3.2% of the scan.
# Extracted to a named constant on 2026-11-06 so the web console can be held to
# the same value: tools/check_lidar_filter_sync.py compares this, SELF_SECTORS
# and the JS mirror in web_console/sensor_console.html, and fails on drift.
MIN_RETURN_M = 0.13

# A walkable corridor must be at least this deep (car length + margin).
MIN_CORRIDOR_M = 0.60


def clean_scan(ranges, angle_increment):
    """Return (angle_deg, range_m) pairs with ONLY the known chassis sliver
    removed.  Real obstacles at any distance -- including 0.2 m table legs --
    are kept: they are the whole point of having a lidar.
    """
    out = []
    for i, r in enumerate(ranges):
        # Reject every kind of "no echo", not just None.  `NaN <= 0.08` and
        # `Inf <= 0.08` are both False, so a NaN or inf return used to sail
        # straight through this filter: it survived the self-sector test too
        # (`nan < max_m` is False), reached emptiest_sector, and made `min(r)`
        # return inf -- a sector with an unreadable point was scored as
        # infinitely clear.  The LD06 behind this rosbridge sends null today
        # (measured 2026-11-06: 89 null, 0 NaN, 0 inf in 455 points), so this
        # is a latent hole rather than an active one, but the LaserScan
        # convention is to publish inf, and one driver change would open it.
        try:
            distance = float(r)
        except (TypeError, ValueError):
            continue
        if not math.isfinite(distance) or distance <= MIN_RETURN_M:
            continue
        a = i * angle_increment * 180.0 / math.pi
        is_self = any(lo <= a <= hi and distance < max_m
                      for lo, hi, max_m in SELF_SECTORS)
        if is_self:
            continue
        out.append((a, distance))
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


def rotation_clearance_agreed(scans, radius_m=ROTATION_RADIUS_M, votes_to_refuse=2):
    """Decide from SEVERAL scans so one noisy frame cannot flip the answer.

    Measured 2026-11-06: with the self-occlusion table incomplete, two
    consecutive frames at a standstill disagreed -- (False, True) -- and a single
    frame decided whether the car was allowed to move at all.  Returns from the
    robot's own body are intermittent (a wheel edge catches the beam on some
    sweeps and not others), so a one-frame test randomly refuses work that is
    perfectly safe.

    Refuse only when at least ``votes_to_refuse`` of the supplied scans see an
    obstacle inside the radius.  With the conventional three scans and two
    votes, one bad frame is tolerated and a real object still stops the car
    (it shows up in every frame).

    Returns (ok, offending_points) where the points come from the frames that
    voted to refuse, so a caller can log what actually blocked it.
    """
    scans = [points for points in scans if points]
    if not scans:
        return False, []
    offenders = []
    voters = 0
    for points in scans:
        ok, frame_offenders = rotation_clearance(points, radius_m)
        if not ok:
            voters += 1
            offenders.extend(frame_offenders)
    if voters >= votes_to_refuse:
        # de-duplicate: the same object appears in every refusing frame
        unique = sorted(set((round(a, 1), round(r, 3)) for a, r in offenders),
                        key=lambda item: item[1])
        return False, [(a, r) for a, r in unique]
    return True, []


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
