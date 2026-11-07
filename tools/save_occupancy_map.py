#!/usr/bin/env python3
"""Save /map (nav_msgs/OccupancyGrid) as PGM + YAML, without nav2 installed.

Why this exists
---------------
`nav2_map_server`'s `map_saver_cli` is the usual way to save a map, but nav2 is
NOT installed on this car (verified 2026-11-07) and the plan was to install it
only AFTER the mapping pass.  That is a chicken-and-egg problem: spin the car,
build a map, then have no tool to write it down.  slam_toolbox publishes /map
as a plain OccupancyGrid over rosbridge, so this tool writes the standard
PGM + YAML pair itself, with zero new dependencies.  The output is readable by
nav2_map_server / nav2_amcl once those are installed.

Usage:
    python tools/save_occupancy_map.py --output /var/lib/armpi-ros2/maps/workshop
    python tools/save_occupancy_map.py --selftest     # pure conversion check
"""
import argparse
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, 'perception'))


def occupancy_to_pixel(value):
    """nav2 map_saver convention: occupied=black, free=white, unknown=205."""
    if value is None or value < 0:
        return 205
    if value == 0:
        return 254
    return max(0, min(254, int(round((100.0 - value) * 254.0 / 100.0))))


def grid_to_bytes(grid):
    """Return (pgm_bytes, yaml_text) for a decoded OccupancyGrid dict.

    OccupancyGrid.data is row-major starting at the LOWEST y; PGM rows start at
    the TOP, so rows must be flipped or the saved map is upside down relative to
    its own origin.
    """
    info = grid['info']
    width = int(info['width'])
    height = int(info['height'])
    data = list(grid['data'])
    if len(data) != width * height:
        raise ValueError('grid size mismatch: %d cells for %dx%d'
                         % (len(data), width, height))
    header = ('P5\n%d %d\n255\n' % (width, height)).encode('ascii')
    pixels = bytearray()
    for row in reversed(range(height)):
        base = row * width
        for col in range(width):
            pixels.append(occupancy_to_pixel(data[base + col]))
    pgm = header + bytes(pixels)

    origin = info['origin']['position']
    q = info['origin']['orientation']
    yaw = math.degrees(math.atan2(
        2 * (q['w'] * q['z'] + q['x'] * q['y']),
        1 - 2 * (q['y'] ** 2 + q['z'] ** 2)))
    image_name = os.path.basename(grid.get('_output', 'map')) + '.pgm'
    yaml_text = (
        'image: %s\n'
        'resolution: %.6f\n'
        'origin: [%.6f, %.6f, %.6f]\n'
        'negate: 0\n'
        'occupied_thresh: 0.65\n'
        'free_thresh: 0.196\n'
        'mode: trinary\n'
        % (image_name, float(info['resolution']),
           float(origin['x']), float(origin['y']), yaw))
    return pgm, yaml_text


def read_map_from_bridge(seconds=20.0):
    from rosbridge_client import Rosbridge
    bridge = Rosbridge('127.0.0.1', 19091, timeout=12)
    bridge.subscribe('/map', 'nav_msgs/msg/OccupancyGrid')
    bridge.drain(0.6)
    deadline = __import__('time').monotonic() + seconds
    message = None
    while __import__('time').monotonic() < deadline:
        message = bridge.wait_for_topic('/map', timeout=1.0)
        if message is not None:
            break
    bridge.close()
    return message


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--output', default='workshop',
                        help='output prefix; writes <prefix>.pgm and <prefix>.yaml')
    parser.add_argument('--seconds', type=float, default=20.0)
    args = parser.parse_args()

    grid = read_map_from_bridge(args.seconds)
    if grid is None:
        print('no /map received -- is slam_toolbox running?', file=sys.stderr)
        return 2
    grid['_output'] = args.output
    pgm, yaml_text = grid_to_bytes(grid)
    folder = os.path.dirname(os.path.abspath(args.output))
    if folder and not os.path.isdir(folder):
        os.makedirs(folder)
    with open(args.output + '.pgm', 'wb') as handle:
        handle.write(pgm)
    with open(args.output + '.yaml', 'w', encoding='utf-8') as handle:
        handle.write(yaml_text)
    print('saved %s.pgm (%dx%d) and %s.yaml'
          % (args.output, grid['info']['width'], grid['info']['height'],
             args.output))
    return 0


def _self_test():
    """Pure conversion check -- no ROS, no car."""
    width, height = 4, 3
    # rows are y=0 (bottom) .. y=2 (top)
    data = [100, 0, -1, 50] + [0, 0, 0, 0] + [10, 20, 30, 40]
    grid = {'info': {'width': width, 'height': height, 'resolution': 0.05,
                     'origin': {'position': {'x': -1.0, 'y': -2.0, 'z': 0.0},
                                'orientation': {'x': 0.0, 'y': 0.0,
                                                'z': 0.0, 'w': 1.0}}},
            'data': data, '_output': 'test'}
    pgm, text = grid_to_bytes(grid)
    assert pgm.startswith(b'P5\n4 3\n255\n'), pgm[:20]
    pixels = list(pgm.split(b'\n', 3)[3])
    assert len(pixels) == width * height
    # first PGM row must be the LAST grid row (y=2), flipped.
    assert pixels[0] == occupancy_to_pixel(10), pixels[:4]
    assert pixels[3] == occupancy_to_pixel(40)
    assert pixels[4] == occupancy_to_pixel(0)
    assert pixels[-1] == occupancy_to_pixel(50), pixels[-4:]
    assert 'image: test.pgm' in text
    assert 'resolution: 0.050000' in text
    assert 'origin: [-1.000000, -2.000000, 0.000000]' in text
    assert occupancy_to_pixel(-1) == 205
    assert occupancy_to_pixel(100) == 0
    assert occupancy_to_pixel(0) == 254
    print('save_occupancy_map self-test: OK')
    return 0


if __name__ == '__main__':
    if '--selftest' in sys.argv:
        sys.exit(_self_test())
    sys.exit(main())
