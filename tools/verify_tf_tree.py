#!/usr/bin/env python3
"""Check the live TF tree on the car, without RViz.

``robot_state_publisher`` is only useful if the tree it publishes is the tree the
Python code assumes.  This subscribes to ``/tf`` and ``/tf_static``, rebuilds the
child -> parent map, and then does the check that actually matters: compose the
chain ``base_link -> grasping_frame`` and compare it against
``arm_kinematics.forward_tool_pose_raw`` evaluated on the same ``/joint_states``.

If those two agree, the URDF is a faithful description of this robot and MoveIt
can be trusted to plan on it.  If they disagree, the URDF is lying and every
collision check built on it is worthless -- which is exactly the failure mode
worth catching before a single motion command is sent.

    python3 tools/verify_tf_tree.py
    python3 tools/verify_tf_tree.py --expect base_link:grasping_frame
"""

import argparse
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(REPO, 'ros2_ws', 'src', 'armpi_ros2', 'armpi_ros2'))

from arm_kinematics import forward_tool_pose_raw                # noqa: E402
from rosbridge_client import Rosbridge                          # noqa: E402

TICK_RAD = math.radians(240.0) / 1000.0
SIGNS = {'joint1': 1.0, 'joint2': 1.0, 'joint3': -1.0, 'joint4': 1.0, 'joint5': 1.0}

CHAIN = ['base_footprint', 'base_link', 'arm_base', 'link1', 'link2', 'link3',
         'link4', 'link5', 'grasping_frame']


def collect(bridge, seconds=6.0):
    """Gather transform edges from /tf and /tf_static."""
    edges = {}
    frames = {}
    bridge.drain(0.4)
    for topic in ('/tf_static', '/tf'):
        deadline = time.monotonic() + seconds / 2.0
        while time.monotonic() < deadline:
            message = bridge.wait_for_topic(topic, 2.0)
            if message is None:
                break
            for transform in message.get('transforms', []):
                parent = transform['header']['frame_id'].lstrip('/')
                child = transform['child_frame_id'].lstrip('/')
                translation = transform['transform']['translation']
                rotation = transform['transform']['rotation']
                edges[child] = parent
                frames[child] = ([float(translation['x']), float(translation['y']),
                                  float(translation['z'])], rotation)
    return edges, frames


def quaternion_to_matrix(q):
    x, y, z, w = q['x'], q['y'], q['z'], q['w']
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]


def compose(frames, edges, base, tip):
    """Transform of ``tip`` in ``base``, composing from base down to tip.

    Order matters and getting it backwards is easy: the chain has to be walked
    base -> tip and post-multiplied, ``T = T * T_child``.  An earlier version
    walked tip -> base, which computes ``T_tip_parent * T_parent_base`` and
    silently produces a plausible-looking but wrong point.
    """
    chain = []
    link = tip
    while link != base:
        parent = edges.get(link)
        if parent is None:
            return None, None, link
        chain.append(link)
        link = parent
        if len(chain) > 40:
            return None, None, link
    chain.reverse()

    rotation = [[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]]
    translation = [0.0, 0.0, 0.0]
    for child in chain:
        t, q = frames[child]
        r = quaternion_to_matrix(q)
        offset = [sum(rotation[i][k] * t[k] for k in range(3)) for i in range(3)]
        translation = [translation[i] + offset[i] for i in range(3)]
        rotation = [[sum(rotation[i][k] * r[k][j] for k in range(3)) for j in range(3)]
                    for i in range(3)]
    return rotation, translation, None


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--port', type=int, default=19091)
    parser.add_argument('--seconds', type=float, default=6.0)
    parser.add_argument('--expect', default='arm_base:grasping_frame')
    args = parser.parse_args()

    bridge = Rosbridge('127.0.0.1', args.port, timeout=10)
    try:
        bridge.drain(0.4)
        joints = bridge.wait_for_topic('/joint_states', 8.0, newest=True)
        edges, frames = collect(bridge, args.seconds)
    finally:
        bridge.close()

    print('TF edges seen: %d' % len(edges))
    missing = [link for link in CHAIN if link not in edges and link != 'base_footprint']
    for link in CHAIN[1:]:
        parent = edges.get(link)
        mark = 'OK ' if parent else '-- '
        print('  %s%-16s parent: %s' % (mark, link, parent or '(no transform)'))

    if missing:
        print()
        print('MISSING (no transform received): %s' % ', '.join(missing))
        print('NOTE: static transforms live on /tf_static with transient_local QoS;')
        print('      rosbridge may not deliver them to a fresh subscriber.')

    if joints is None:
        print('no /joint_states -- cannot compare against arm_kinematics')
        return 2
    raw = {name: int(round(500.0 + SIGNS[name] * float(position) / TICK_RAD))
           for name, position in zip(joints['name'], joints['position'])
           if name in SIGNS}
    truth = forward_tool_pose_raw(raw)['translation_m']
    print()
    print('joint ticks: %s' % raw)
    print('arm_kinematics arm_base->tool: %s'
          % [round(v, 4) for v in truth])

    base, tip = args.expect.split(':')
    rotation, translation, stuck = compose(frames, edges, base, tip)
    if translation is None:
        print('cannot compose %s -> %s (stuck at %s)' % (base, tip, stuck))
        return 2
    print('TF          %s->%s: %s'
          % (base, tip, [round(v, 4) for v in translation]))
    residual = math.dist(translation, truth)
    print('residual: %.1f mm' % (residual * 1000))
    if residual < 0.005:
        print('-> TF agrees with the kinematic model; the URDF describes this robot.')
        return 0
    print('-> DISAGREES. Do not plan motion on this URDF.')
    return 1


if __name__ == '__main__':
    sys.exit(main())
