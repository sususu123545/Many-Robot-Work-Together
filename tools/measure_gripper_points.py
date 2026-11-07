#!/usr/bin/env python3
"""Measure the gripper fingers' 3-D position in the TOOL frame, empirically.

Why
---
Every grasp attempt so far guessed "the fingertips sit N cm below the vendor IK
tool point".  Guessing is what produced the two opposite failures on 2026-11-03:

    dz = +0.030  -> fingers pressed on the floor, jaws could not open (r=367)
    dz = +0.069  -> jaws closed in mid-air, 7 cm above the ball

With a measured offset the descent target becomes exact instead of a guess.

How
---
The fingers are black against a pale floor, so they are easy to segment.  Take
the same physical finger edge from two different arm poses and triangulate the
two rays -- the reconstructed point is expressed in arm_base, then rotated into
the tool frame with the live FK.  If the two poses agree, the feature really is
rigidly attached to the gripper (and the number is trustworthy); if they
disagree by centimetres it is not the gripper and the result must be discarded.

Usage
-----
    python3 tools/measure_gripper_points.py
    python3 tools/measure_gripper_points.py --poses search_floor,search_near,search_mid
    python3 tools/measure_gripper_points.py --no-move      # just report current pose
"""

import argparse
import json
import math
import os
import sys
import time
import urllib.request
from io import BytesIO

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(REPO, 'ros2_ws', 'src', 'armpi_ros2', 'armpi_ros2'))

from PIL import Image                                     # noqa: E402
import arm_poses                                          # noqa: E402
from arm_kinematics import forward_tool_pose_raw          # noqa: E402
from ball_in_arm_base import (load_calibration, mat_mul,   # noqa: E402
                              CALIBRATION, FX, FY, CX, CY)
from rosbridge_client import Rosbridge                    # noqa: E402

SNAPSHOT = ('http://127.0.0.1:19099/snapshot'
            '?topic=/rear_camera/image_raw&type=ros_compressed')
DARK_LEVEL = 75          # the fingers read well under this; the floor is > 120
CROP_TOP = 250           # fingers live in the bottom part of the frame


def grab_frame():
    with urllib.request.urlopen(SNAPSHOT, timeout=15) as response:
        return Image.open(BytesIO(response.read())).convert('RGB')


def dark_components(image, min_area=400):
    """Biggest dark blobs in the lower part of the frame, with full pixel lists.

    ``car_scan.connected_components`` drops the pixel list, and here the extreme
    points are the whole point, so this keeps its own copy of the flood fill.
    """
    grey = image.convert('L')
    width, height = grey.size
    pixels = grey.load()
    seen = set()
    blobs = []
    for y in range(CROP_TOP, height):
        for x in range(width):
            if pixels[x, y] >= DARK_LEVEL or (x, y) in seen:
                continue
            queue = [(x, y)]
            seen.add((x, y))
            points = []
            while queue:
                cx, cy = queue.pop()
                points.append((cx, cy))
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        nx, ny = cx + dx, cy + dy
                        if (0 <= nx < width and CROP_TOP <= ny < height
                                and (nx, ny) not in seen
                                and pixels[nx, ny] < DARK_LEVEL):
                            seen.add((nx, ny))
                            queue.append((nx, ny))
            if len(points) < min_area:
                continue
            xs = [p[0] for p in points]
            ys = [p[1] for p in points]
            blobs.append({'area': len(points),
                          'bbox': [min(xs), min(ys), max(xs), max(ys)],
                          'center_x': (min(xs) + max(xs)) / 2.0,
                          'center_y': (min(ys) + max(ys)) / 2.0,
                          'points': points})
    blobs.sort(key=lambda item: item['area'], reverse=True)
    return blobs[:2]


def features(blob):
    """Three stable pixels per finger: outer extreme, top (far) tip, inner extreme."""
    points = blob['points']
    if not points:
        return {}
    left = min(points, key=lambda p: p[0])
    right = max(points, key=lambda p: p[0])
    top = min(points, key=lambda p: p[1])
    return {'outer_x': left, 'outer_x_right': right, 'top': top}


def camera_frame(ticks):
    tool = forward_tool_pose_raw(ticks)
    rotation_tool_cam, translation_tool_cam, _ = load_calibration(CALIBRATION)
    rotation_base_tool = tool['rotation_matrix']
    translation_base_tool = tool['translation_m']
    rotation_base_cam = mat_mul(rotation_base_tool, rotation_tool_cam)
    position_base_cam = [translation_base_tool[i] +
                         sum(rotation_base_tool[i][k] * translation_tool_cam[k]
                             for k in range(3)) for i in range(3)]
    return {'tool': tool, 'R_bt': rotation_base_tool, 't_bt': translation_base_tool,
            'R_bc': rotation_base_cam, 't_bc': position_base_cam}


def ray_in_base(frame, u, v):
    """Unit direction in arm_base through pixel (u, v) of the equidistant model."""
    offset_x = (u - CX) / FX
    offset_y = (v - CY) / FY
    theta = math.hypot(offset_x, offset_y)
    phi = math.atan2(offset_y, offset_x)
    direction_cam = [math.sin(theta) * math.cos(phi),
                     math.sin(theta) * math.sin(phi),
                     math.cos(theta)]
    return [sum(frame['R_bc'][i][k] * direction_cam[k] for k in range(3))
            for i in range(3)]


def triangulate(origin_a, dir_a, origin_b, dir_b):
    """Closest point to two rays; also return how far apart the rays pass."""
    # Minimise |a + s*u - (b + t*v)| over s, t.
    w = [origin_a[i] - origin_b[i] for i in range(3)]
    uu = sum(dir_a[i] * dir_a[i] for i in range(3))
    vv = sum(dir_b[i] * dir_b[i] for i in range(3))
    uv = sum(dir_a[i] * dir_b[i] for i in range(3))
    uw = sum(dir_a[i] * w[i] for i in range(3))
    vw = sum(dir_b[i] * w[i] for i in range(3))
    denominator = uu * vv - uv * uv
    if abs(denominator) < 1e-9:
        return None, None
    s = (uv * vw - vv * uw) / denominator
    t = (uu * vw - uv * uw) / denominator
    point_a = [origin_a[i] + s * dir_a[i] for i in range(3)]
    point_b = [origin_b[i] + t * dir_b[i] for i in range(3)]
    return [(point_a[i] + point_b[i]) / 2.0 for i in range(3)], \
           math.dist(point_a, point_b)


def to_tool(frame, point_base):
    delta = [point_base[i] - frame['t_bt'][i] for i in range(3)]
    return [sum(frame['R_bt'][k][i] * delta[k] for k in range(3)) for i in range(3)]


def sample(bridge, pose_name):
    if pose_name:
        ticks = arm_poses.goto_pose(bridge, pose_name, verbose=False)
    else:
        ticks = arm_poses.read_ticks(bridge)
    if not ticks:
        raise SystemExit('no joint feedback')
    image = grab_frame()
    blobs = dark_components(image)
    if len(blobs) < 2:
        raise SystemExit('%s: found %d dark blobs, need 2 (fingers not in view)'
                         % (pose_name or 'current', len(blobs)))
    blobs.sort(key=lambda item: item['center_x'])
    frame = camera_frame(ticks)
    return {'pose': pose_name or 'current', 'ticks': ticks, 'frame': frame,
            'blobs': blobs, 'image': image,
            'features': [features(blob) for blob in blobs]}


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--poses', default='search_floor,search_near,search_mid',
                        help='comma separated pose names to sample')
    parser.add_argument('--no-move', action='store_true',
                        help='sample the current pose only (no arm motion)')
    parser.add_argument('--capture', default=None,
                        help='directory to dump the frames for eyeballing')
    args = parser.parse_args()

    bridge = Rosbridge('127.0.0.1', 19091, timeout=8)
    samples = []
    try:
        if not arm_poses.secure_arm_gate(bridge):
            raise SystemExit('cannot secure the arm gate')
        time.sleep(1)
        names = ([None] if args.no_move
                 else [name.strip() for name in args.poses.split(',') if name.strip()])
        for name in names:
            print('[sample] %s' % (name or 'current pose'), flush=True)
            samples.append(sample(bridge, name))
            print('   ticks %s' % samples[-1]['ticks'], flush=True)
            for index, (blob, feature) in enumerate(zip(samples[-1]['blobs'],
                                                        samples[-1]['features'])):
                print('   finger%d area=%d bbox=%s outer_x=%s top=%s'
                      % (index, blob['area'], blob['bbox'],
                         feature.get('outer_x'), feature.get('top')), flush=True)
    finally:
        bridge.close()

    if args.capture:
        os.makedirs(args.capture, exist_ok=True)
        for record in samples:
            path = os.path.join(args.capture, '%s.png' % record['pose'])
            record['image'].save(path)
            print('frame -> %s' % path)

    if len(samples) < 2:
        print('need at least two poses to triangulate')
        return 0

    print('\n== triangulated finger features, expressed in the TOOL frame ==')
    report = {}
    for feature_name in ('outer_x', 'top'):
        for finger in (0, 1):
            values = []
            for index in range(len(samples) - 1):
                for other in range(index + 1, len(samples)):
                    pair = []
                    good = True
                    for record in (samples[index], samples[other]):
                        pixel = record['features'][finger].get(feature_name)
                        if pixel is None:
                            good = False
                            break
                        pair.append((record['frame'], ray_in_base(record['frame'],
                                                                  pixel[0], pixel[1]),
                                     pixel))
                    if not good:
                        continue
                    point, gap = triangulate(pair[0][0]['t_bc'], pair[0][1],
                                             pair[1][0]['t_bc'], pair[1][1])
                    if point is None:
                        continue
                    local = to_tool(pair[0][0], point)
                    values.append({'poses': '%s|%s' % (samples[index]['pose'],
                                                       samples[other]['pose']),
                                   'ray_gap_mm': round(gap * 1000, 1),
                                   'tool_xyz_mm': [round(v * 1000, 1) for v in local],
                                   'pixels': [pair[0][2], pair[1][2]],
                                   })
            if not values:
                continue
            key = '%s_finger%d' % (feature_name, finger)
            report[key] = values
            print('\n%-18s' % key)
            for entry in values:
                print('   %-26s gap=%5.1f mm  tool=%s  px=%s'
                      % (entry['poses'], entry['ray_gap_mm'],
                         entry['tool_xyz_mm'], entry['pixels']))
            if len(values) > 1:
                spread = [max(entry['tool_xyz_mm'][axis] for entry in values) -
                          min(entry['tool_xyz_mm'][axis] for entry in values)
                          for axis in range(3)]
                print('   pose-to-pose spread: %.1f / %.1f / %.1f mm'
                      % tuple(spread))
                print('   -> %s' % ('rigid, trustworthy' if max(spread) < 12
                                    else 'NOT rigid: these pixels are not the gripper'))

    if report:
        print('\nJSON ' + json.dumps(report))
    return 0


if __name__ == '__main__':
    sys.exit(main())
