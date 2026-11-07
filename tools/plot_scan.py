#!/usr/bin/env python3
"""Plot one live /scan as a top-down picture, in the lidar frame.

Why
---
Numbers like "angle_increment 0.595 deg" and "26% of beams returned nothing"
describe a scan but do not show it.  When the question is "is the robot square
to the corridor, and how wide is it", a picture answers in one glance and a
statistic does not.

Renders through rosbridge (default 127.0.0.1:19091, the SSH tunnel), so it does
not need to run on the car and it does not touch /cmd_vel.

  * black dots  : valid returns
  * red dots    : beams that returned nothing (None / NaN / <=0) -- these are
                  the ones that leave wedges in the occupancy grid
  * blue ring   : 1 m circles
  * green axes  : lidar +x (forward) and +y (left), 0 deg = +x
  * white lines : every 30 deg, labelled, so a wall's bearing can be read off

Prints the same numbers it draws, including the two widest blind sectors.

Usage:
    python tools/plot_scan.py --png artifacts/scan_plot.png
    python tools/plot_scan.py --port 19091 --range 4.0 --png ...
"""
import argparse
import math
import sys

from PIL import Image, ImageDraw

PIXELS_PER_M = 100
SIZE = 720
CENTRE = SIZE // 2

AXIS_LABELS = {
    0: '+x (front)', 90: '+y (left)', 180: '-x (rear)', 270: '-y (right)'}


def fetch(port, timeout):
    import os
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from rosbridge_client import Rosbridge
    bridge = Rosbridge('127.0.0.1', port, timeout=timeout)
    bridge.subscribe('/scan', 'sensor_msgs/msg/LaserScan')
    bridge.drain(0.8)
    message = bridge.wait_for_topic('/scan', timeout=timeout)
    bridge.close()
    return message


def valid(value):
    return isinstance(value, (int, float)) and value == value and value > 0.01


def blind_sectors(bad_angles, gap_deg=5.0):
    """Group bad beam angles into contiguous sectors, returns the widest ones."""
    if not bad_angles:
        return []
    ordered = sorted(bad_angles)
    groups = [[ordered[0]]]
    for angle in ordered[1:]:
        if angle - groups[-1][-1] <= gap_deg:
            groups[-1].append(angle)
        else:
            groups.append([angle])
    # a sector may wrap through +-180
    if len(groups) > 1 and (360.0 - groups[-1][-1] + groups[0][0]) <= gap_deg:
        groups[0] = groups.pop() + groups[0]
    sectors = [(len(group), group[0], group[-1]) for group in groups]
    sectors.sort(reverse=True)
    return sectors


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--port', type=int, default=19091)
    parser.add_argument('--timeout', type=float, default=8.0)
    parser.add_argument('--range', type=float, default=4.0,
                        help='radius of the plot in metres')
    parser.add_argument('--png', default='artifacts/scan_plot.png')
    args = parser.parse_args()

    message = fetch(args.port, args.timeout)
    ranges = message['ranges']
    increment = message['angle_increment']
    first = message['angle_min']
    scale = PIXELS_PER_M
    radius_px = args.range * scale
    if radius_px > CENTRE - 8:
        scale = (CENTRE - 8) / args.range
        radius_px = args.range * scale

    image = Image.new('RGB', (SIZE, SIZE), (250, 250, 250))
    draw = ImageDraw.Draw(image)

    # range rings
    for metres in range(1, int(args.range) + 1):
        r = metres * scale
        draw.ellipse([CENTRE - r, CENTRE - r, CENTRE + r, CENTRE + r],
                     outline=(190, 210, 240))
        draw.text((CENTRE + r - 18, CENTRE - 12), '%dm' % metres,
                  fill=(120, 150, 200))
    # bearing spokes
    for bearing in range(0, 360, 30):
        rad = math.radians(bearing)
        dx, dy = math.cos(rad), -math.sin(rad)
        draw.line([CENTRE, CENTRE,
                   CENTRE + dx * radius_px, CENTRE + dy * radius_px],
                  fill=(225, 225, 225))
        label = AXIS_LABELS.get(bearing, '%d' % bearing)
        draw.text((CENTRE + dx * (radius_px + 6) - 10,
                   CENTRE + dy * (radius_px + 6) - 6), label, fill=(90, 90, 90))
    draw.line([CENTRE - radius_px, CENTRE, CENTRE + radius_px, CENTRE],
              fill=(0, 150, 0))
    draw.line([CENTRE, CENTRE - radius_px, CENTRE, CENTRE + radius_px],
              fill=(0, 150, 0))

    bad_angles = []
    plotted = clipped = 0
    for index, value in enumerate(ranges):
        degrees = math.degrees((first + index * increment + math.pi)
                               % (2 * math.pi) - math.pi)
        if not valid(value):
            bad_angles.append(degrees)
            rad = math.radians(degrees)
            px = CENTRE + math.cos(rad) * radius_px
            py = CENTRE - math.sin(rad) * radius_px
            draw.ellipse([px - 2, py - 2, px + 2, py + 2], fill=(220, 60, 60))
            continue
        if value > args.range:
            clipped += 1
            continue
        rad = math.radians(degrees)
        px = CENTRE + math.cos(rad) * value * scale
        py = CENTRE - math.sin(rad) * value * scale
        # ImageDraw.point() silently drops float coordinates, so use a tiny
        # ellipse with rounded ints instead -- otherwise the plot looks empty
        # while the numbers say hundreds of returns.
        px, py = int(round(px)), int(round(py))
        draw.ellipse([px - 1, py - 1, px + 1, py + 1], fill=(20, 20, 20))
        plotted += 1

    image.save(args.png)

    print('beams            : %d' % len(ranges))
    print('plotted returns  : %d' % plotted)
    print('no-return beams  : %d (%.0f%%)' % (len(bad_angles),
                                              100.0 * len(bad_angles) / len(ranges)))
    print('beyond %.1f m     : %d (%.0f%%)'
          % (args.range, clipped, 100.0 * clipped / len(ranges)))
    print('widest blind sectors (deg):')
    for count, start, end in blind_sectors(bad_angles)[:4]:
        span = (end - start) % 360.0
        print('   %+7.1f .. %+7.1f  span %5.1f deg, %d beams'
              % (start, end, span, count))
    print('wrote %s' % args.png)
    return 0


if __name__ == '__main__':
    sys.exit(main())
