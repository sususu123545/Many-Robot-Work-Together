#!/usr/bin/env python3
"""Where does the live scan best fit the saved map?  Brute force, no amcl.

Why
---
amcl reported sigma_yaw of 150 degrees while sitting still, which could mean
either "the particle filter needs motion" or "the scan does not match the map at
all".  Those need completely different fixes, and amcl cannot tell them apart.

So this does the same job the hard way, with no filter and no priors: take one
live /scan, and for every candidate pose on a small grid, ask how well the beam
endpoints land on obstacles in the map.  The best pose it finds is an
independent second opinion.  If that pose is near amcl's, the map is fine and
the filter needs motion.  If it is somewhere else, the chain is inconsistent and
no amount of amcl tuning will help.

Scoring uses a distance transform: BFS outward from every occupied cell, so each
grid cell knows how far it is from the nearest obstacle.  A good pose has most
beams ending on (or very near) an obstacle; a bad pose has them in open space.
Beams that hit nothing are skipped -- they carry no position information.

Pure Python; the grid is 89x112 which is nothing.

Usage:
    python tools/scan_map_match.py --port 19091
    python tools/scan_map_match.py --map maps/workshop.pgm --span 0.8 --step 0.1
"""
import argparse
import math
import os
import sys
from collections import deque

from PIL import Image

FREE_MIN = 250
OCCUPIED_MAX = 65
INF = 999


def load_map(path):
    image = Image.open(path).convert('L')
    width, height = image.size
    data = list(image.get_flattened_data())
    return width, height, data


def read_map_yaml(pgm_path):
    """origin/resolution from the sibling .yaml (map_server format)."""
    yaml_path = os.path.splitext(pgm_path)[0] + '.yaml'
    resolution, origin = 0.05, (0.0, 0.0)
    if os.path.exists(yaml_path):
        for line in open(yaml_path, encoding='utf-8'):
            if line.startswith('resolution:'):
                resolution = float(line.split(':', 1)[1])
            elif line.startswith('origin:'):
                parts = line.split('[', 1)[1].split(']')[0].split(',')
                origin = (float(parts[0]), float(parts[1]))
    return resolution, origin


def distance_transform(width, height, data):
    """BFS distance (in cells) from the nearest occupied cell."""
    dist = [INF] * (width * height)
    queue = deque()
    for index, value in enumerate(data):
        if value <= OCCUPIED_MAX:
            dist[index] = 0
            queue.append(index)
    while queue:
        index = queue.popleft()
        row, col = divmod(index, width)
        base = dist[index] + 1
        for drow, dcol in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            nrow, ncol = row + drow, col + dcol
            if 0 <= nrow < height and 0 <= ncol < width:
                other = nrow * width + ncol
                if dist[other] > base:
                    dist[other] = base
                    queue.append(other)
    return dist


def fetch_scan(port, timeout):
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from rosbridge_client import Rosbridge
    bridge = Rosbridge('127.0.0.1', port, timeout=timeout)
    bridge.subscribe('/scan', 'sensor_msgs/msg/LaserScan')
    bridge.drain(0.8)
    message = bridge.wait_for_topic('/scan', timeout=timeout)
    bridge.close()
    return message


def scan_points(message, max_range):
    points = []
    for index, value in enumerate(message['ranges']):
        if isinstance(value, (int, float)) and value == value and 0.05 < value <= max_range:
            angle = message['angle_min'] + index * message['angle_increment']
            points.append((value * math.cos(angle), value * math.sin(angle)))
    return points


def score(points, width, height, dist, resolution, origin, x, y, yaw):
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    total = 0
    used = 0
    for px, py in points:
        mx = x + px * cos_yaw - py * sin_yaw
        my = y + px * sin_yaw + py * cos_yaw
        col = int((mx - origin[0]) / resolution)
        row = height - 1 - int((my - origin[1]) / resolution)
        if 0 <= row < height and 0 <= col < width:
            value = dist[row * width + col]
            if value == INF:
                continue
            total += value
            used += 1
    if used < len(points) * 0.3:
        return None
    return total / used, used


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--map', default='maps/workshop.pgm')
    parser.add_argument('--port', type=int, default=19091)
    parser.add_argument('--timeout', type=float, default=8.0)
    parser.add_argument('--max-range', type=float, default=6.0)
    parser.add_argument('--span', type=float, default=0.6,
                        help='search +- this many metres around x/y')
    parser.add_argument('--step', type=float, default=0.10)
    parser.add_argument('--yaw-step', type=float, default=3.0)
    args = parser.parse_args()

    width, height, data = load_map(args.map)
    resolution, origin = read_map_yaml(args.map)
    dist = distance_transform(width, height, data)
    message = fetch_scan(args.port, args.timeout)
    points = scan_points(message, args.max_range)
    print('map      : %d x %d  res=%.3f  origin=(%.3f, %.3f)'
          % (width, height, resolution, origin[0], origin[1]))
    print('scan     : %d usable beams (<= %.1f m)' % (len(points), args.max_range))

    best = None
    steps = int(args.span / args.step) + 1
    offsets = [(i - steps // 2) * args.step for i in range(steps)]
    for x in offsets:
        for y in offsets:
            for tenth in range(int(360 / args.yaw_step)):
                yaw = math.radians(tenth * args.yaw_step - 180.0)
                result = score(points, width, height, dist, resolution, origin,
                               x, y, yaw)
                if result is None:
                    continue
                mean_cells, used = result
                if best is None or mean_cells < best[0]:
                    best = (mean_cells, x, y, tenth * args.yaw_step - 180.0, used)

    if best is None:
        print('FAILED: no candidate pose kept enough beams inside the map')
        return 1
    mean_cells, x, y, yaw, used = best
    print('---')
    print('best pose   : x=%+.2f m  y=%+.2f m  yaw=%+.1f deg'
          % (x, y, yaw))
    print('mean beam-to-obstacle distance : %.2f cells = %.3f m  (%d beams)'
          % (mean_cells, mean_cells * resolution, used))
    print('interpretation: under ~1 cell means the scan really does lie on the '
          'mapped walls; several cells means the map and the world disagree')
    return 0


if __name__ == '__main__':
    sys.exit(main())
