#!/usr/bin/env python3
"""Locate the red ball in the arm-base frame, purely from live data.

Chain (same maths as the on-robot pipeline, done on the workstation because the
board cannot start new processes over SSH):

    pixel -> camera ray (equidistant fisheye) -> camera frame
          -> tool frame            (tool_T_camera from the hand-eye calibration)
          -> arm_base frame        (forward_tool_pose_raw of the live joint ticks)

Why this exists: the hand-eye calibration IS an eye-in-hand result -- it is valid
at every pose as long as the live joint angles are composed in. Treating it as a
startup-pose-only static transform (and falling back to pixel guessing) is what
made the first grasp attempts chase the ball blindly.

Vendor frame convention (arm_kinematics docstring): x lateral, y forward, z up.
"""

import argparse
import json
import math
import os
import sys
import urllib.request
from io import BytesIO

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(REPO, 'ros2_ws', 'src', 'armpi_ros2', 'armpi_ros2'))

from PIL import Image                                          # noqa: E402
import car_approach as ca                                      # noqa: E402
from arm_kinematics import forward_tool_pose_raw               # noqa: E402

SNAPSHOT = ('http://127.0.0.1:19099/snapshot'
            '?topic=/rear_camera/image_raw&type=ros_compressed')
CALIBRATION = os.path.join(REPO, 'tools', 'runtime', 'calib',
                           'rear_camera_hand_eye.json')

BALL_RADIUS_M = 0.0175
# Calibrated equidistant fisheye model (RMS 1.158 px, 29 views).
FX, FY, CX, CY = 541.286, 536.832, 308.972, 220.808


def rotation_from_quaternion(x, y, z, w):
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if abs(norm - 1.0) > 1e-3:
        raise SystemExit('calibration quaternion is not normalised (|q|=%.4f)' % norm)
    return [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
            [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
            [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]


def mat_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)]
            for i in range(3)]


def mat_vec(a, v):
    return [sum(a[i][k] * v[k] for k in range(3)) for i in range(3)]


def mat_t_vec(t, v):
    return [v[i] - t[i] for i in range(3)]


def transpose(m):
    return [[m[j][i] for j in range(3)] for i in range(3)]


def load_calibration(path):
    with open(path, 'r', encoding='utf-8') as handle:
        data = json.load(handle)
    if data.get('schema') != 'armpi_ros2_hand_eye_v1':
        raise SystemExit('unexpected calibration schema: %r' % data.get('schema'))
    block = data['tool_T_camera']
    translation = [float(v) for v in block['translation_m']]
    quaternion = [float(v) for v in block['quaternion_xyzw']]
    rotation = rotation_from_quaternion(*quaternion)
    return rotation, translation, data


def camera_point(u, v, radius_px):
    """Ball centre in the camera optical frame (x right, y down, z forward).

    Two different angles matter here and mixing them up puts the ball 4x too
    close: the OFFSET angle (u,v from the principal point) gives the ray
    direction, while the ANGULAR RADIUS (blob radius / focal length) gives the
    range, because sin(angular_radius) = ball_radius / distance.
    """
    offset_x = (u - CX) / FX
    offset_y = (v - CY) / FY
    theta = math.hypot(offset_x, offset_y)          # ray angle off the axis
    phi = math.atan2(offset_y, offset_x)
    alpha = max(1e-6, radius_px / FX)               # angular radius of the disc
    distance = BALL_RADIUS_M / math.sin(min(alpha, 1.4))
    direction = [math.sin(theta) * math.cos(phi),
                 math.sin(theta) * math.sin(phi),
                 math.cos(theta)]
    return [distance * component for component in direction], distance


def read_ticks(port=19091):
    from rosbridge_client import Rosbridge
    bridge = Rosbridge('127.0.0.1', port, timeout=8)
    try:
        bridge.drain(0.3)
        message = bridge.wait_for_topic('/joint_states', 8.0, newest=True)
        if message is None:
            raise SystemExit('no joint feedback')
        ticks = {}
        for name, position in zip(message['name'], message['position']):
            if name in ('joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'r_joint'):
                ticks[name] = int(round(500 + position / (math.pi / 1000.0)))
        # joint3's sign is inverted in the vendor mapping
        if 'joint3' in ticks:
            radians = None
        return ticks, message.get('header', {}).get('stamp')
    finally:
        bridge.close()


def raw_from_message(message):
    """Convert a JointState back to raw ticks (same signs as the arm node)."""
    signs = {'joint1': 1.0, 'joint2': 1.0, 'joint3': -1.0,
             'joint4': 1.0, 'joint5': 1.0, 'r_joint': 1.0}
    # 1000 ticks span 240 degrees (not 180): 1 tick = 0.24 deg
    tick_rad = math.radians(240.0) / 1000.0
    raw = {}
    for name, position in zip(message['name'], message['position']):
        if name in signs:
            raw[name] = int(round(500.0 + signs[name] * float(position) / tick_rad))
    return raw


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--port', type=int, default=19091)
    parser.add_argument('--json', action='store_true', help='print one JSON line')
    args = parser.parse_args()

    from rosbridge_client import Rosbridge
    bridge = Rosbridge('127.0.0.1', args.port, timeout=8)
    try:
        bridge.drain(0.3)
        message = bridge.wait_for_topic('/joint_states', 8.0, newest=True)
        if message is None:
            raise SystemExit('no joint feedback')
        raw = raw_from_message(message)
    finally:
        bridge.close()

    with urllib.request.urlopen(SNAPSHOT, timeout=15) as response:
        image = Image.open(BytesIO(response.read())).convert('RGB')
    blob, reason = ca.locate_ball(image)
    if blob is None:
        # The floor-zone gate assumes the ball sits low in the frame; when the
        # camera is pitched down the ball rides near the top. Fall back to the
        # largest red blob of plausible size anywhere in the picture.
        from car_scan import connected_components
        candidates = []
        for component in connected_components(ca.redness_mask(image)):
            radius = (component['area'] / 3.14159) ** 0.5
            if 15.0 <= radius <= 140.0 and component['fill'] >= 0.5:
                candidates.append((component['area'], component, radius))
        if not candidates:
            raise SystemExit('ball not detected (%s) and no size-plausible blob' % reason)
        candidates.sort(key=lambda item: item[0], reverse=True)
        _, component, radius = candidates[0]
        blob = {'center_x': component['center_x'], 'center_y': component['center_y'],
                'radius_px': radius}

    rotation_tool_cam, translation_tool_cam, calibration = load_calibration(CALIBRATION)
    tool = forward_tool_pose_raw(raw)
    rotation_base_tool = tool['rotation_matrix']
    translation_base_tool = tool['translation_m']

    point_cam, distance = camera_point(blob['center_x'], blob['center_y'],
                                       blob['radius_px'])
    # camera -> tool -> base
    point_tool = [sum(rotation_tool_cam[i][k] * point_cam[k] for k in range(3))
                  + translation_tool_cam[i] for i in range(3)]
    point_base = [sum(rotation_base_tool[i][k] * point_tool[k] for k in range(3))
                  + translation_base_tool[i] for i in range(3)]

    result = {
        'joints_raw': raw,
        'ball_pixel': {'u': round(blob['center_x'], 1),
                       'v': round(blob['center_y'], 1),
                       'radius_px': round(blob['radius_px'], 1)},
        'ball_in_camera_m': [round(v, 4) for v in point_cam],
        'camera_distance_m': round(distance, 4),
        'ball_in_arm_base_m': [round(v, 4) for v in point_base],
        'tool_in_arm_base_m': [round(v, 4) for v in translation_base_tool],
        'pitch_deg': round(sum(tool['angles_deg'].values()), 2),
    }
    if args.json:
        print(json.dumps(result))
        return 0

    print('joints (raw ticks): %s' % raw)
    print('ball pixel: u=%.1f v=%.1f r=%.1f px'
          % (blob['center_x'], blob['center_y'], blob['radius_px']))
    print('camera distance: %.3f m' % distance)
    print('ball in camera frame (x right, y down, z fwd): %s' % result['ball_in_camera_m'])
    print('tool point in arm_base: %s' % result['tool_in_arm_base_m'])
    print('BALL IN ARM BASE (x lat, y fwd, z up): %s' % result['ball_in_arm_base_m'])
    print('tool pitch: %.1f deg' % result['pitch_deg'])
    return 0


if __name__ == '__main__':
    sys.exit(main())
