#!/usr/bin/env python3
"""量化每个寻找姿态的地面覆盖范围：把画面每一行反投影到地面平面。

为什么值得这么做
----------------
寻找姿态好不好，不能靠"照片看着还行"判断——同一颗球，相机俯角差 10°，可能
在 0.3 m 处就掉出画面，也可能要到 1.2 m 才进画面。有手眼标定之后，这件事是
可以算的：相机光心位姿 = FK(关节角) ∘ tool_T_camera，画面像素 → 射线 →
与地面平面求交，就得到该像素看到的地面点距基座多远。

地面高度（arm_base 的 z）用实测值：抓取成功那次球心 z=-0.076 m，球半径 17.5 mm，
所以地面约在 z=-0.094 m。可以随时用 --floor-z 覆盖。

输出：每个姿态下，画面若干行的地面距离、以及"有效覆盖 0.15~1.2 m"的区间。
"""

import argparse
import math
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(REPO, 'ros2_ws', 'src', 'armpi_ros2', 'armpi_ros2'))

from arm_kinematics import forward_tool_pose_raw                   # noqa: E402
from ball_in_arm_base import load_calibration, CALIBRATION, FX, FY, CX, CY  # noqa: E402
import arm_poses                                                  # noqa: E402

FLOOR_Z = -0.094          # arm_base 坐标下的地面高度（实测推算）
TICK_RAD = math.radians(240.0) / 1000.0
SIGNS = {'joint1': 1.0, 'joint2': 1.0, 'joint3': -1.0, 'joint4': 1.0}


def matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)]
            for i in range(3)]


def matvec(a, v):
    return [sum(a[i][k] * v[k] for k in range(3)) for i in range(3)]


def camera_pose(ticks):
    """返回 (R_base_cam, t_base_cam)。"""
    tool = forward_tool_pose_raw(ticks)
    rotation_tc, translation_tc, _ = load_calibration(CALIBRATION)
    rotation = matmul(tool['rotation_matrix'], rotation_tc)
    translation = [sum(tool['rotation_matrix'][i][k] * translation_tc[k]
                       for k in range(3)) + tool['translation_m'][i] for i in range(3)]
    return rotation, translation


def ground_distance(rotation, translation, u, v):
    """像素 (u,v) 的射线与地面 z=FLOOR_Z 的交点，返回 (前向距离, 横向偏移)。"""
    theta_x = (u - CX) / FX
    theta_y = (v - CY) / FY
    theta = math.hypot(theta_x, theta_y)
    phi = math.atan2(theta_y, theta_x)
    direction_cam = [math.sin(theta) * math.cos(phi),
                     math.sin(theta) * math.sin(phi),
                     math.cos(theta)]
    direction = matvec(rotation, direction_cam)
    if abs(direction[2]) < 1e-6:
        return None, None
    scale = (FLOOR_Z - translation[2]) / direction[2]
    if scale <= 0:
        return None, None
    x = translation[0] + scale * direction[0]
    y = translation[1] + scale * direction[1]
    return y, x


def coverage(ticks, rows=(470, 430, 390, 350, 310, 270, 230, 190)):
    rotation, translation = camera_pose(ticks)
    rows_out = []
    for v in rows:
        forward, lateral = ground_distance(rotation, translation, CX, v)
        rows_out.append((v, forward, lateral))
    valid = [item[1] for item in rows_out if item[1] and 0.05 < item[1] < 3.0]
    span = (min(valid), max(valid)) if valid else (None, None)
    return {'camera': [round(value, 4) for value in translation],
            'rows': [(v, None if d is None else round(d, 3),
                      None if x is None else round(x, 3)) for v, d, x in rows_out],
            'span_m': span}


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--pose', default=None, help='姿态名；缺省则逐个姿态计算')
    parser.add_argument('--move', action='store_true', help='真的把臂移过去再算')
    args = parser.parse_args()

    names = [args.pose] if args.pose else list(arm_poses.POSES)
    bridge = None
    if args.move:
        from rosbridge_client import Rosbridge
        bridge = Rosbridge('127.0.0.1', 19091, timeout=8)
        if not arm_poses.secure_arm_gate(bridge):
            raise SystemExit('拿不到臂门控')

    try:
        for name in names:
            if args.move:
                ticks = arm_poses.goto_pose(bridge, name, verbose=False)
            else:
                target = arm_poses.POSES[name]['joints']
                ticks = {'joint1': 500, 'joint2': target['joint2'],
                         'joint3': target['joint3'], 'joint4': target['joint4']}
            result = coverage(ticks)
            span = result['span_m']
            print('\n== %s ==' % name)
            print('  关节: %s' % {k: ticks[k] for k in ('joint2', 'joint3', 'joint4')})
            print('  相机位置(arm_base): %s' % result['camera'])
            if span[0]:
                print('  地面覆盖: %.2f ~ %.2f m（前向）' % span)
            for v, forward, lateral in result['rows']:
                print('    行 v=%3d -> 前向 %s m, 横向 %s m'
                      % (v, '—' if forward is None else '%.2f' % forward,
                         '—' if lateral is None else '%+.2f' % lateral))
    finally:
        if bridge:
            bridge.close()


if __name__ == '__main__':
    sys.exit(main())
