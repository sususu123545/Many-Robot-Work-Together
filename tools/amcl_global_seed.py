#!/usr/bin/env python3
"""Give nav2_amcl an initial pose, warm if possible, broad only as a fallback.

Why this is needed at all
-------------------------
The plan for this stage said "nav2_amcl does global localisation, no initial
pose required".  That is wrong, and the source says so plainly: in
`AmclNode::publishAmclPose()` (nav2_amcl/src/amcl_node.cpp, Jazzy):

    if (!initial_pose_is_known_) {
      RCLCPP_WARN(get_logger(), "AMCL cannot publish a pose or update the
                  transform. Please set the initial pose...");
      return;
    }

`set_initial_pose: true` does not help either -- it feeds a default-constructed
PoseWithCovarianceStamped, i.e. ZERO covariance, so every particle starts on one
point.  That is a known start pose, not a global search.

Two seeding strategies, in order
--------------------------------
1. WARM. `amcl_pose_keeper.py` writes the last trustworthy pose to
   /opt/armpi-maps/last_pose.json.  Seeding there with sigma 0.25 m / 10 deg
   converges in one or two scans because the cloud has one correct place to
   collapse into.  Measured 2026-11-09: sigma 0.075 m / 5.6 deg.
2. BROAD. Sigma 1.34 m and uniform yaw spread the cloud over the whole map.
   Measured the same evening: works, but only one time out of three in a
   cluttered room -- failures sit at sigma ~1.0 m with yaw jumping between
   hypotheses.  So it is the fallback, not the default.

A warm seed is only believed after an independent check: the live scan is
projected through the resulting pose and compared with the occupied cells of the
loaded map.  Sitting 0.4 m away from the truth the mean beam error is ~0.41 m;
on the correct pose it is 0.058 m.  Without this check a tight seed can lock
onto a wrong-but-plausible pose and then report CONFIDENTLY WRONG numbers.

Run inside the core container, with map_server + amcl already active:
    source /opt/ros/jazzy/setup.bash
    python3 /opt/armpi-maps/amcl_global_seed.py --wait 20
"""
import argparse
import json
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

try:                                   # only present in the car's container
    from nav2_msgs.msg import ParticleCloud
except ImportError:                    # keeps this file importable on the PC
    ParticleCloud = None

LAST_POSE = '/opt/armpi-maps/last_pose.json'

# variance, not sigma: 1.8 -> sigma 1.34 m, which reaches ~2.7 m at 2 sigma.
VAR_XY = 1.8
VAR_YAW = math.pi ** 2

# a warm start must be tight: we are claiming "the car has not moved much".
WARM_SIGMA_XY = 0.25
WARM_SIGMA_YAW = math.radians(10.0)
WARM_MAX_DISPLACEMENT = 1.0            # metres; warm result must stay near the
                                       # pose it started from, else the seed lied

# acceptance, both for AMCL's own covariance and for the scan-to-map fit.
OK_SIGMA_XY = 0.30
OK_SIGMA_YAW = math.radians(20.0)
OK_FIT = 0.20                          # metres, mean beam-to-obstacle distance
OK_HIT = 0.70                          # share of beams within 0.15 m of a wall


class Seeder(Node):
    def __init__(self, var_xy=VAR_XY, var_yaw=VAR_YAW):
        super().__init__('amcl_global_seed')
        self.var_xy = var_xy
        self.var_yaw = var_yaw
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', QoSProfile(depth=1))
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.pose = None
        self.particles = None
        self.scan = None
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.on_pose, qos)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos)
        if ParticleCloud is not None:
            self.create_subscription(
                ParticleCloud, '/particle_cloud', self.on_particles, qos)

    def on_pose(self, message):
        self.pose = message

    def on_scan(self, message):
        self.scan = message

    def on_particles(self, message):
        self.particles = len(message.particles)

    def set_covariance(self, sigma_xy, sigma_yaw):
        self.var_xy = sigma_xy ** 2
        self.var_yaw = sigma_yaw ** 2

    def seed(self, x, y, yaw=0.0):
        # Forget any pose seen before this command: a report that silently
        # reuses the previous attempt's result would describe the wrong seed.
        self.pose = None
        message = PoseWithCovarianceStamped()
        message.header.frame_id = 'map'
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.pose.position.x = x
        message.pose.pose.position.y = y
        message.pose.pose.orientation.z = math.sin(yaw / 2.0)
        message.pose.pose.orientation.w = math.cos(yaw / 2.0)
        covariance = [0.0] * 36
        covariance[0] = self.var_xy
        covariance[7] = self.var_xy
        covariance[35] = self.var_yaw
        message.pose.covariance = covariance
        self.publisher.publish(message)


def spin_for(node, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)


def wait_for_subscribers(node, timeout=15.0):
    """Do not publish /initialpose into the void.

    2026-11-09: right after `systemctl restart armpi-localization`, this script
    ran before amcl's subscription had been discovered.  Every seed was dropped
    silently -- three broad attempts produced not a single /amcl_pose, and the
    log looked exactly like "amcl cannot localise".  Waiting for the publisher's
    subscription count removes that whole failure mode.
    """
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if node.publisher.get_subscription_count() > 0:
            return node.publisher.get_subscription_count()
        rclpy.spin_once(node, timeout_sec=0.2)
    return 0


def wait_for_scan(node, timeout=12.0):
    """Wait until at least one usable /scan has arrived.

    Discovery of a BEST_EFFORT topic is not instant.  2026-11-10: the first run
    captured the scan before any had arrived, so every "scan fit" was computed
    over zero beams and read `inf / 0 beams` at *every* pose -- including the
    correct one.  Never verify against an empty scan.
    """
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if node.scan is not None:
            count = len(scan_points(node.scan))
            if count > 20:
                return count
        rclpy.spin_once(node, timeout_sec=0.2)
    return 0


def fetch_map(node, timeout=12.0):
    box = {}

    def remember(message):
        box['map'] = message

    node.create_subscription(
        OccupancyGrid, '/map', remember,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                   reliability=ReliabilityPolicy.RELIABLE))
    end = time.monotonic() + timeout
    while time.monotonic() < end and 'map' not in box:
        rclpy.spin_once(node, timeout_sec=0.2)
    return box.get('map')


def free_centroid(grid):
    """Centroid of the free cells of whatever map is loaded right now.

    The seed used to be a hardcoded pair remembered from the first room.  After
    that room was abandoned the default sat 0.4 m from the truth in the new one
    and amcl stayed split between two hypotheses (2026-11-09: sigma_yaw 147 deg
    after 60 deg of rotation, yaw flipping 172 deg between reads).
    """
    width = grid.info.width
    resolution = grid.info.resolution
    origin = grid.info.origin.position
    count = 0
    sum_x = 0.0
    sum_y = 0.0
    for index, value in enumerate(grid.data):
        if 0 <= value < 50:
            sum_x += origin.x + (index % width + 0.5) * resolution
            sum_y += origin.y + (index // width + 0.5) * resolution
            count += 1
    if not count:
        return None
    return sum_x / count, sum_y / count


def occupied_cells(grid):
    width = grid.info.width
    return {(index % width, index // width)
            for index, value in enumerate(grid.data) if value >= 65}


def scan_points(scan, max_range=6.0):
    points = []
    angle = scan.angle_min
    for distance in scan.ranges:
        if 0.05 < distance <= max_range and not math.isinf(distance):
            points.append((distance * math.cos(angle),
                           distance * math.sin(angle)))
        angle += scan.angle_increment
    return points


def distance_field(grid, cells):
    """Distance from every cell to the nearest occupied cell, in metres.

    Chamfer 3-4 two-pass transform: exact enough (a few percent) and O(cells),
    which keeps amcl's Pi 4 free.  A bounded neighbourhood search was tried
    first and was useless exactly where it mattered: with a 0.2 m reach, 63% of
    the beams counted as "no obstacle nearby" and a pose that was 1.1 m wrong
    scored *better* (0.118 m) than the correct one (0.150 m).
    """
    width = grid.info.width
    height = grid.info.height
    resolution = grid.info.resolution
    big = 1.0e9
    dist = [big] * (width * height)
    for column, row in cells:
        dist[row * width + column] = 0.0

    diagonal = 1.41421356
    for row in range(height):                       # forward
        base = row * width
        for column in range(width):
            index = base + column
            best = dist[index]
            if column > 0 and dist[index - 1] + 1.0 < best:
                best = dist[index - 1] + 1.0
            if row > 0:
                if dist[index - width] + 1.0 < best:
                    best = dist[index - width] + 1.0
                if column > 0 and dist[index - width - 1] + diagonal < best:
                    best = dist[index - width - 1] + diagonal
                if column + 1 < width and dist[index - width + 1] + diagonal < best:
                    best = dist[index - width + 1] + diagonal
            dist[index] = best
    for row in range(height - 1, -1, -1):           # backward
        base = row * width
        for column in range(width - 1, -1, -1):
            index = base + column
            best = dist[index]
            if column + 1 < width and dist[index + 1] + 1.0 < best:
                best = dist[index + 1] + 1.0
            if row + 1 < height:
                if dist[index + width] + 1.0 < best:
                    best = dist[index + width] + 1.0
                if column + 1 < width and dist[index + width + 1] + diagonal < best:
                    best = dist[index + width + 1] + diagonal
                if column > 0 and dist[index + width - 1] + diagonal < best:
                    best = dist[index + width - 1] + diagonal
            dist[index] = best
    return [value * resolution if value < big else float('inf') for value in dist]


def fit_error(grid, field, points, x, y, yaw, near=0.15):
    """How well a live scan lies on the mapped walls at a given pose.

    Returns (mean, hit_ratio, used).  Mean alone is not enough: a pose can score
    a small mean by putting most beams in unmapped space and letting the few
    lucky ones dominate.  hit_ratio -- the share of beams within `near` of an
    obstacle -- is what separates "the walls agree" from "hardly anything
    agreed".  Measured on this map: correct pose 0.058 m / 0.95 hit,
    0.4 m off 0.41 m / 0.12 hit.
    """
    width = grid.info.width
    height = grid.info.height
    resolution = grid.info.resolution
    origin = grid.info.origin.position
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    total = 0.0
    hits = 0
    used = 0
    for px, py in points:
        wx = x + cos_yaw * px - sin_yaw * py
        wy = y + sin_yaw * px + cos_yaw * py
        column = int(math.floor((wx - origin.x) / resolution))
        row = int(math.floor((wy - origin.y) / resolution))
        if not (0 <= column < width and 0 <= row < height):
            continue
        distance = field[row * width + column]
        if distance == float('inf'):
            continue
        total += distance
        used += 1
        if distance <= near:
            hits += 1
    if not used:
        return float('inf'), 0.0, 0
    return total / used, hits / used, used


def map_id(grid):
    """Short fingerprint of the map geometry a pose is expressed in.

    A stored pose is only meaningful in the frame of the map it was recorded in.
    2026-11-10: the map was rebuilt because furniture had moved; the old pose
    then pointed into empty space in the new frame.  With this token the seed is
    rejected as "recorded in a map that no longer exists" instead of being
    silently misused.
    """
    origin = grid.info.origin.position
    return '%dx%d@%.3f,%.3f' % (grid.info.width, grid.info.height,
                                origin.x, origin.y)


def read_last_pose(path, token=None):
    try:
        with open(path) as handle:
            data = json.load(handle)
        recorded = data.get('map')
        if token is not None and recorded != token:
            return None, 'recorded in a different map (%s vs %s)' % (
                recorded, token)
        return (float(data['x']), float(data['y']), float(data['yaw']), data), None
    except (OSError, ValueError, KeyError):
        return None, None


def report(node, grid, field, label):
    """Print AMCL's own numbers plus an independent scan-to-map check.

    The scan is taken fresh here, not passed in: a scan captured at start-up may
    be older than the seed, and an empty one silently makes every pose look
    wrong (all beams "unusable").
    """
    pose = node.pose.pose.pose
    covariance = node.pose.pose.covariance
    sigma_x = math.sqrt(max(0.0, covariance[0]))
    sigma_y = math.sqrt(max(0.0, covariance[7]))
    sigma_yaw = math.sqrt(max(0.0, covariance[35]))
    yaw = math.atan2(2.0 * (pose.orientation.w * pose.orientation.z),
                     1.0 - 2.0 * pose.orientation.z ** 2)
    points = scan_points(node.scan) if node.scan is not None else []
    print('--- %s' % label)
    print('pose        : x=%+.3f  y=%+.3f  yaw=%+.1f deg'
          % (pose.position.x, pose.position.y, math.degrees(yaw)))
    print('sigma       : x=%.3f m  y=%.3f m  yaw=%.1f deg  (particles=%s)'
          % (sigma_x, sigma_y, math.degrees(sigma_yaw),
             node.particles if node.particles is not None else 'n/a'))
    tight = (sigma_x < OK_SIGMA_XY and sigma_y < OK_SIGMA_XY
             and sigma_yaw < OK_SIGMA_YAW)
    if not points:
        print('scan fit    : SKIPPED -- no usable /scan, cannot verify this pose',
              flush=True)
        print('verdict     : %s (unverified)'
              % ('CONVERGED' if tight else 'NOT converged'), flush=True)
        return tight                       # never claim a verified convergence
    mean, hit, used = fit_error(grid, field, points,
                                pose.position.x, pose.position.y, yaw)
    print('scan fit    : %.3f m mean to nearest obstacle, %.2f of beams within '
          '0.15 m (%d beams)' % (mean, hit, used), flush=True)
    agrees = mean <= OK_FIT and hit >= OK_HIT
    if tight and agrees:
        print('verdict     : CONVERGED', flush=True)
        return True
    if not agrees:
        print('verdict     : NOT converged -- the scan does not lie on the mapped '
              'walls at this pose; the filter may be confidently wrong',
              flush=True)
    else:
        print('verdict     : NOT converged -- needs motion or more particles',
              flush=True)
    return False


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--x', type=float, default=None,
                        help='seed x in the map frame; default = centroid of '
                             'the free cells of the map that is loaded')
    parser.add_argument('--y', type=float, default=None,
                        help='seed y in the map frame; default = same centroid')
    parser.add_argument('--yaw', type=float, default=0.0,
                        help='seed yaw, radians (0 = +x, map frame)')
    parser.add_argument('--wait', type=float, default=20.0,
                        help='seconds to wait for /amcl_pose per attempt')
    parser.add_argument('--attempts', type=int, default=3)
    parser.add_argument('--sigma-xy', type=float, default=math.sqrt(VAR_XY),
                        help='std dev in metres for the broad fallback search')
    parser.add_argument('--sigma-yaw', type=float, default=math.sqrt(VAR_YAW),
                        help='std dev in radians for the broad fallback search')
    parser.add_argument('--no-warm', action='store_true',
                        help='ignore last_pose.json and go straight to the '
                             'broad search')
    parser.add_argument('--last-pose', default=LAST_POSE,
                        help='where amcl_pose_keeper.py stores the last pose')
    args = parser.parse_args()

    rclpy.init()
    node = Seeder()
    grid = fetch_map(node)
    cells = occupied_cells(grid) if grid is not None else set()
    field = distance_field(grid, cells) if grid is not None else None
    if grid is None:
        print('WARNING: no /map on the wire -- cannot verify any pose', flush=True)

    centre = free_centroid(grid) if grid is not None else None
    if centre is not None:
        print('map free-cell centroid: (%+.3f, %+.3f)' % centre, flush=True)

    spin_for(node, 1.5)          # discovery + one scan
    subscribers = wait_for_subscribers(node)
    if subscribers:
        print('initialpose : %d subscriber(s)' % subscribers, flush=True)
    else:
        print('WARNING: nobody subscribed to /initialpose after 15 s -- is amcl '
              'really active?', flush=True)
    beams = wait_for_scan(node)
    print('live scan   : %d usable beams' % beams, flush=True)
    if not beams:
        print('WARNING: no /scan arrived in 12 s -- poses will be reported as '
              'unverified, not as converged', flush=True)

    explicit = args.x is not None or args.y is not None
    token = map_id(grid) if grid is not None else None
    warm = None
    if not (args.no_warm or explicit):
        warm, rejected = read_last_pose(args.last_pose, token)
        if warm is None and rejected:
            print('warm start  : ignored -- %s' % rejected, flush=True)

    # ---- 1. warm start -------------------------------------------------
    if warm is not None:
        warm_x, warm_y, warm_yaw, raw = warm
        print('warm start  : last pose (%+.3f, %+.3f, %+.1f deg) saved %s'
              % (warm_x, warm_y, math.degrees(warm_yaw),
                 raw.get('saved_at', '?')), flush=True)
        node.set_covariance(WARM_SIGMA_XY, WARM_SIGMA_YAW)
        node.seed(warm_x, warm_y, warm_yaw)
        spin_for(node, max(6.0, args.wait / 2.0))
        if node.pose is not None:
            moved = math.hypot(node.pose.pose.pose.position.x - warm_x,
                               node.pose.pose.pose.position.y - warm_y)
            if moved <= WARM_MAX_DISPLACEMENT and grid is not None:
                if report(node, grid, field, 'warm'):
                    rclpy.shutdown()
                    return 0
            else:
                print('warm start  : result moved %.2f m away from the seed -- '
                      'not trusting it' % moved, flush=True)
        else:
            print('warm start  : no /amcl_pose at all', flush=True)
        print('warm start  : rejected, falling back to a broad search',
              flush=True)

    # ---- 2. broad fallback ---------------------------------------------
    if explicit:
        seed_x = args.x if args.x is not None else (centre[0] if centre else 0.0)
        seed_y = args.y if args.y is not None else (centre[1] if centre else 0.0)
    else:
        seed_x = centre[0] if centre else 0.0
        seed_y = centre[1] if centre else 0.0
    node.set_covariance(args.sigma_xy, args.sigma_yaw)

    for attempt in range(1, args.attempts + 1):
        print('attempt %d: seeding (%+.3f, %+.3f, %.2f rad) '
              'sigma_xy=%.2f m sigma_yaw=%.2f rad'
              % (attempt, seed_x, seed_y, args.yaw,
                 args.sigma_xy, args.sigma_yaw), flush=True)
        node.seed(seed_x, seed_y, args.yaw)
        spin_for(node, args.wait)
        if node.pose is None:
            continue
        if grid is None:
            print('attempt %d: no map, cannot verify -- reporting AMCL only'
                  % attempt, flush=True)
            covariance = node.pose.pose.covariance
            if (math.sqrt(max(0.0, covariance[0])) < OK_SIGMA_XY
                    and math.sqrt(max(0.0, covariance[35])) < OK_SIGMA_YAW):
                print('verdict     : CONVERGED (unverified)', flush=True)
                rclpy.shutdown()
                return 0
            continue
        if report(node, grid, field, 'attempt %d' % attempt):
            rclpy.shutdown()
            return 0

    print('FAILED: amcl is active but did not converge, or converged onto a pose '
          'that the scan contradicts. Check that the map on the car is the map '
          'of this room (compare with tools/scan_map_match.py).', flush=True)
    rclpy.shutdown()
    return 2


if __name__ == '__main__':
    sys.exit(main())
