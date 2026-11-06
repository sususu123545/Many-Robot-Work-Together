#!/usr/bin/env python3
"""Locate an object on a known plane WITHOUT knowing how big it is.

The problem with the current pipeline
-------------------------------------
``ball_in_arm_base.camera_point`` converts one pixel plus the blob's apparent
radius into a distance:

    distance = BALL_RADIUS_M / sin(radius_px / FX)

That is exact -- for a 35 mm ball.  Feed it a 60 mm ball and the distance comes
out 1.7x too small, silently, because nothing in the maths knows the real size.
Every "grasp a different object" idea inherits that error.

What this module does instead
-----------------------------
Two facts that hold for anything resting on a flat surface, no size needed:

  1. the LOWEST pixel of the silhouette is where the object touches the plane,
     so casting that pixel's ray onto the plane gives the contact point in x/y;
  2. once you know the distance to that contact point, the pixel scale there is
     known, so the blob's pixel height converts to a real height in millimetres.

Centre = contact point + height/2 straight up.  A ball, a bottle, a box and a
can of drink all work through the same code path.

Sanity check: a sphere's silhouette bottom is the contact point when viewed from
above the surface, which is the case here (camera is ~0.2 m up looking down).

    python3 tools/ground_localizer.py                 # locate the red blob now
    python3 tools/ground_localizer.py --plane -0.094  # different surface height
"""

import argparse
import math
import os
import sys
import urllib.request
from io import BytesIO

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(REPO, 'ros2_ws', 'src', 'armpi_ros2', 'armpi_ros2'))

from PIL import Image                                       # noqa: E402
import car_approach as ca                                    # noqa: E402
from arm_kinematics import forward_tool_pose_raw             # noqa: E402
from ball_in_arm_base import (load_calibration, mat_mul,     # noqa: E402
                              CALIBRATION, FX, FY, CX, CY)
from rosbridge_client import Rosbridge                       # noqa: E402

SNAPSHOT = ('http://127.0.0.1:19099/snapshot'
            '?topic=/rear_camera/image_raw&type=ros_compressed')
SIGNS = {'joint1': 1.0, 'joint2': 1.0, 'joint3': -1.0, 'joint4': 1.0, 'joint5': 1.0}
TICK_RAD = math.radians(240.0) / 1000.0
FLOOR_Z = -0.094          # measured, in the arm_base frame


def camera_frame(ticks):
    """Camera origin and orientation in arm_base, from live joint ticks."""
    tool = forward_tool_pose_raw(ticks)
    rotation_tool_cam, translation_tool_cam, _ = load_calibration(CALIBRATION)
    rotation_base_cam = mat_mul(tool['rotation_matrix'], rotation_tool_cam)
    origin = [tool['translation_m'][i] + sum(tool['rotation_matrix'][i][k]
                                             * translation_tool_cam[k]
                                             for k in range(3)) for i in range(3)]
    return rotation_base_cam, origin


def ray(rotation_base_cam, u, v):
    """Unit direction in arm_base through pixel (u, v), equidistant model."""
    ux, uy = (u - CX) / FX, (v - CY) / FY
    theta = math.hypot(ux, uy)
    phi = math.atan2(uy, ux)
    direction_cam = [math.sin(theta) * math.cos(phi),
                     math.sin(theta) * math.sin(phi),
                     math.cos(theta)]
    return [sum(rotation_base_cam[i][k] * direction_cam[k] for k in range(3))
            for i in range(3)]


def intersect_plane(origin, direction, plane_z):
    """Where the ray meets the horizontal plane z = plane_z, or None."""
    if direction[2] > -1e-6:
        return None                       # ray is horizontal or pointing away
    s = (plane_z - origin[2]) / direction[2]
    if s <= 0:
        return None
    return [origin[0] + s * direction[0],
            origin[1] + s * direction[1],
            plane_z], s


def pixel_scale(distance_mm, focal_px=FX):
    """Millimetres of object per pixel, at that distance."""
    return distance_mm / focal_px


def _bounds(source):
    """Pixel extents of a detection.

    Accepts either a ``detectors.Detection`` (preferred -- it states its own
    bottom edge) or the legacy blob dict from which the bottom edge is inferred.
    Worth being explicit: for a YOLO box the true bottom is ``y + h/2``, while
    ``center_y + radius`` only coincides with it for a circle.
    """
    if hasattr(source, 'bottom_v'):
        return source.center_x, source.bottom_v, source.top_v
    center_y = source['center_y']
    radius = source['radius_px']
    return source['center_x'], center_y + radius, center_y - radius


def locate(source, ticks, plane_z=FLOOR_Z):
    """Contact point, height and centre of an object resting on the plane."""
    rotation, origin = camera_frame(ticks)
    u, bottom_v, top_v = _bounds(source)

    contact = intersect_plane(origin, ray(rotation, u, bottom_v), plane_z)
    if contact is None:
        return None, 'the bottom ray never reaches the plane (object too far or horizon)'
    point, _ = contact
    distance = math.dist(origin, point)
    scale = pixel_scale(distance)

    # height from the pixel height, using the scale at the contact point
    height = (bottom_v - top_v) * scale
    centre = [point[0], point[1], plane_z + height / 2.0]

    # Both distances are reported so the two methods can be compared on equal
    # terms: the old radius method produced a CAMERA-TO-CENTRE distance, and
    # comparing it against camera-to-contact is meaningless (they differ by the
    # object's own radius plus the grazing angle).
    centre_distance = math.dist(origin, centre)

    # old method, for comparison only
    old_distance = 0.0175 / math.sin(max(1e-6, (bottom_v - top_v) / 2.0 / FX))

    return {'pixel': [round(u, 1), round(bottom_v, 1), round((bottom_v - top_v) / 2.0, 1)],
            'contact': [round(v, 4) for v in point],
            'centre_distance_m': round(centre_distance, 4),
            'camera_distance_m': round(distance, 4),
            'height_mm': round(height * 1000, 1),
            'centre': [round(v, 4) for v in centre],
            'radius_estimate_mm': round(height * 500, 1),
            'old_radius_method_distance_m': round(old_distance, 4)}, ''


def read_ticks(bridge):
    bridge.drain(0.3)
    message = bridge.wait_for_topic('/joint_states', 8.0, newest=True)
    if message is None:
        return None
    return {name: int(round(500.0 + SIGNS[name] * float(position) / TICK_RAD))
            for name, position in zip(message['name'], message['position'])
            if name in SIGNS}


def find_object(image, backend='color', model=None, label=None):
    """Run the configured detector and keep the most promising object."""
    sys.path.insert(0, os.path.join(HERE, 'perception'))
    import detectors

    detector = detectors.build(backend, model=model)
    found = detector.detect(image)
    return detectors.pick_best(found, label=label)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--plane', type=float, default=FLOOR_Z,
                        help='height of the supporting surface in arm_base (m)')
    parser.add_argument('--port', type=int, default=19091)
    parser.add_argument('--backend', default='color', choices=('color', 'onnx'),
                        help='which detector finds the object')
    parser.add_argument('--model', default='/opt/vision/models/yolo11n.onnx')
    parser.add_argument('--label', default=None,
                        help='keep only this class name, e.g. bottle or sports ball')
    args = parser.parse_args()

    bridge = Rosbridge('127.0.0.1', args.port, timeout=10)
    try:
        ticks = read_ticks(bridge)
    finally:
        bridge.close()
    if ticks is None:
        print('no joint feedback'); return 2

    with urllib.request.urlopen(SNAPSHOT, timeout=15) as response:
        image = Image.open(BytesIO(response.read())).convert('RGB')
    detection = find_object(image, args.backend, args.model, args.label)
    if detection is None:
        print('nothing detected by the %s backend' % args.backend); return 2

    u, bottom_v, top_v = _bounds(detection)
    result, error = locate(detection, ticks, args.plane)
    if result is None:
        print('cannot localise: %s' % error); return 2

    print('joint ticks       : %s' % ticks)
    print('detection         : %s  (backend %s)' % (detection.label, args.backend))
    print('pixel             : u=%.0f  bottom_v=%.0f  top_v=%.0f  (%.0fx%.0f box)'
          % (u, bottom_v, top_v, detection.width, detection.height))
    print()
    print('contact point     : %s   (ray hit the plane here)' % result['contact'])
    print('distance to contact: %.3f m' % result['camera_distance_m'])
    print('distance to CENTRE : %.3f m   <- comparable with the old method'
          % result['centre_distance_m'])
    print('object height     : %.1f mm   (from pixel height x scale at that distance)'
          % result['height_mm'])
    print('implies radius    : %.1f mm  if it is a sphere' % result['radius_estimate_mm'])
    print('CENTRE            : %s' % result['centre'])
    print()
    print('old method (assumed 35 mm ball, centre distance): %.3f m'
          % result['old_radius_method_distance_m'])
    print('new method needs no size, so a 60 mm ball or a bottle lands on the same numbers')
    return 0


if __name__ == '__main__':
    sys.exit(main())
