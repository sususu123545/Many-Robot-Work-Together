#!/usr/bin/env python3
"""Measure how messy an occupancy grid actually is.

Why
---
"这个图很乱" is a fair complaint and a useless bug report: a threshold map is
either good enough to localise against or it is not, and the difference is a
handful of numbers.  This tool turns the impression into those numbers:

  speckle      occupied cells with no occupied neighbour -- lidar noise or
               isolated reflective-floor returns.  A clean wall has none.
  components   how many separate blotches the occupied cells form.  One long
               wall plus a chair is 2-4; 200 means noise.
  holes        unknown cells enclosed by free space -- usually the gaps left
               between adjacent laser rays when the range is clipped.
  bearing      the dominant line direction of the occupied cells (Hough), in
               grid degrees.  A corridor should be ONE strong peak.

Also writes a colourised PNG: occupied black, free white, unknown light grey,
ISOLATED occupied cells red, and the dominant bearing drawn as a green line.

Pure Python, no numpy: the grids here are ~12k cells, which is nothing.

Usage:
    python tools/map_quality.py maps/workshop_20261107_v2.pgm --png /tmp/q.png
"""
import argparse
import math
import os
import sys
from collections import Counter, deque

from PIL import Image

OCCUPIED_MAX = 65          # PGM value <= this is an obstacle
FREE_MIN = 250             # PGM value >= this is free
UNKNOWN_LO = 180           # 180..250 renders unknown
UNKNOWN_HI = 250

NEIGHBOURS8 = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
NEIGHBOURS4 = [(-1, 0), (1, 0), (0, -1), (0, 1)]


def load(path):
    image = Image.open(path).convert('L')
    width, height = image.size
    data = list(image.get_flattened_data())
    return width, height, data


def classify(data):
    occupied = set()
    free = set()
    for index, value in enumerate(data):
        if value <= OCCUPIED_MAX:
            occupied.add(index)
        elif value >= FREE_MIN:
            free.add(index)
    return occupied, free


def neighbours(index, width, height, offsets):
    row, col = divmod(index, width)
    for drow, dcol in offsets:
        nrow, ncol = row + drow, col + dcol
        if 0 <= nrow < height and 0 <= ncol < width:
            yield nrow * width + ncol


def components(cells, width, height, offsets):
    remaining = set(cells)
    sizes = []
    while remaining:
        seed = remaining.pop()
        queue = deque([seed])
        size = 1
        while queue:
            for neighbour in neighbours(queue.popleft(), width, height, offsets):
                if neighbour in remaining:
                    remaining.discard(neighbour)
                    queue.append(neighbour)
                    size += 1
        sizes.append(size)
    return sorted(sizes, reverse=True)


def enclosed_unknown(free, data, width, height):
    """Unknown cells that cannot reach the map border through unknown cells."""
    unknown = set(index for index, value in enumerate(data)
                  if UNKNOWN_LO <= value < UNKNOWN_HI)
    outside = set()
    queue = deque()
    for row in range(height):
        for col in (0, width - 1):
            index = row * width + col
            if index in unknown and index not in outside:
                outside.add(index)
                queue.append(index)
    for col in range(width):
        for row in (0, height - 1):
            index = row * width + col
            if index in unknown and index not in outside:
                outside.add(index)
                queue.append(index)
    while queue:
        for neighbour in neighbours(queue.popleft(), width, height, NEIGHBOURS4):
            if neighbour in unknown and neighbour not in outside:
                outside.add(neighbour)
                queue.append(neighbour)
    enclosed = unknown - outside
    # Only count holes that sit inside known space; cells touching free space
    # are the interesting ones, the rest are just ragged map edges.
    touching = set()
    for index in enclosed:
        if any(n in free for n in neighbours(index, width, height, NEIGHBOURS4)):
            touching.add(index)
    return len(unknown), len(enclosed), len(touching)


def dominant_bearing(occupied, width, height, step_tenths=5):
    """Hough over occupied cells in MAP coordinates.

    The PGM saver writes rows bottom-up (`for row in reversed(range(height))`),
    so image row r is map y = height-1-r.  Reporting the angle in image
    coordinates would silently flip its sign, which matters when the angle is
    used to rotate the robot to align with a wall.

    Returns (best_line_direction_deg, [top peaks]) where the direction is the
    line's own bearing in degrees, 0 = +x axis, positive = towards +y.
    """
    if not occupied:
        return 0.0, []
    points = []
    for index in occupied:
        row, col = divmod(index, width)
        points.append((col, height - 1 - row))
    diag = int(math.hypot(width, height)) + 2
    scored = []
    for tenths in range(0, 1800, step_tenths):
        normal = tenths / 10.0
        cos_a = math.cos(math.radians(normal))
        sin_a = math.sin(math.radians(normal))
        bins = Counter()
        for x, y in points:
            rho = int(round(x * cos_a + y * sin_a)) + diag
            bins[rho] += 1
        scored.append((max(bins.values()) if bins else 0,
                       (normal + 90.0) % 180.0))
    scored.sort(reverse=True)
    peaks = []
    for peak, direction in scored:
        if all(abs(direction - other[1]) > 10.0 for other in peaks):
            peaks.append((peak, direction))
        if len(peaks) >= 3:
            break
    return peaks[0][1], peaks


def colourise(path, data, width, height, isolated, bearing, out_png):
    image = Image.new('RGB', (width, height))
    pixels = []
    cos_a = math.cos(math.radians(bearing))
    sin_a = math.sin(math.radians(bearing))
    for index, value in enumerate(data):
        row, col = divmod(index, width)
        if index in isolated:
            pixels.append((220, 40, 40))
        elif value <= OCCUPIED_MAX:
            pixels.append((0, 0, 0))
        elif value >= FREE_MIN:
            pixels.append((255, 255, 255))
        else:
            pixels.append((200, 200, 200))
        # draw the dominant bearing through the middle
        if abs((col - width / 2) * sin_a - (row - height / 2) * cos_a) < 0.6:
            pixels[-1] = (0, 170, 0)
    image.putdata(pixels)
    scale = max(2, int(560 / max(width, height)))
    image = image.resize((width * scale, height * scale), Image.NEAREST)
    image.save(out_png)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('pgm')
    parser.add_argument('--png', default='')
    parser.add_argument('--top', type=int, default=6)
    args = parser.parse_args()

    if not os.path.exists(args.pgm):
        print('no such file: %s' % args.pgm)
        return 2
    width, height, data = load(args.pgm)
    total = width * height
    occupied, free = classify(data)
    unknown = set(range(total)) - occupied - free
    print('grid            : %d x %d = %d cells' % (width, height, total))
    print('occupied / free / unknown : %d / %d / %d'
          % (len(occupied), len(free), len(unknown)))

    isolated = set()
    for index in occupied:
        if not any(n in occupied for n in neighbours(index, width, height,
                                                     NEIGHBOURS8)):
            isolated.add(index)
    print('isolated occupied cells   : %d of %d (%.0f%%)  <- speckle'
          % (len(isolated), len(occupied),
             100.0 * len(isolated) / len(occupied) if occupied else 0))

    sizes = components(occupied, width, height, NEIGHBOURS8)
    print('occupied clusters         : %d   largest: %s'
          % (len(sizes), sizes[:args.top]))
    small = sum(1 for size in sizes if size <= 3)
    print('  clusters of <=3 cells   : %d (%.0f%% of clusters)'
          % (small, 100.0 * small / len(sizes) if sizes else 0))

    free_sizes = components(free, width, height, NEIGHBOURS4)
    print('free clusters             : %d   largest: %s'
          % (len(free_sizes), free_sizes[:args.top]))

    unknown_total, enclosed, touching_free = enclosed_unknown(
        free, data, width, height)
    print('unknown total             : %d' % unknown_total)
    print('unknown enclosed          : %d  (of which touching free space: %d)'
          % (enclosed, touching_free))

    bearing, peaks = dominant_bearing(occupied, width, height)
    print('dominant wall direction   : %s'
          % ', '.join('%.1f deg (%d cells)' % (direction, peak)
                      for peak, direction in peaks))
    print('  (map coordinates, 0 = +x.  Align the car to the first peak '
          'before starting a new map to get an axis-aligned grid.)')

    if args.png:
        colourise(args.pgm, data, width, height, isolated, bearing, args.png)
        print('wrote %s' % args.png)
    return 0


if __name__ == '__main__':
    sys.exit(main())
