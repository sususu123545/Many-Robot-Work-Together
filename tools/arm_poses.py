#!/usr/bin/env python3
"""命名机械臂姿态库：把"看地面""看高处""抓取预备"固化成可复用的姿态。

为什么需要它
------------
同一条臂，看地上的球和看架子上的东西，需要完全不同的姿态：
  * 看地面：相机要下俯、臂要前伸，否则相机离基座太近，稍远的球就超出可达范围
    （2026-11-03 实测：只抬 joint2 不前伸，0.2 m 的球在 arm_base 里被算成 0.50 m）；
  * 看高处：相机要接近水平、臂要抬高，否则视野全被地板占据。

姿态用原始 tick 表示（1000 tick = 240°，joint3 的符号是反的）。
每个姿态都可以用 `--pose NAME --capture out.jpg` 落一张实拍图来验证，
不要凭想象定姿态——这是这台臂反复吃亏的地方。

用法
----
    python3 tools/arm_poses.py --list
    python3 tools/arm_poses.py --pose search_floor --capture %TEMP%/floor.jpg
"""

import argparse
import json
import math
import os
import subprocess
import sys
import time
import urllib.request
from io import BytesIO

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(REPO, 'ros2_ws', 'src', 'armpi_ros2', 'armpi_ros2'))

from arm_kinematics import forward_tool_pose_raw            # noqa: E402
from ball_in_arm_base import load_calibration, CALIBRATION  # noqa: E402
from gripper_jog import status, reset_estop, set_gates      # noqa: E402
from rosbridge_client import Rosbridge                      # noqa: E402

SNAPSHOT = ('http://127.0.0.1:19099/snapshot'
            '?topic=/rear_camera/image_raw&type=ros_compressed')
TICK_RAD = math.radians(240.0) / 1000.0
SIGNS = {'joint1': 1.0, 'joint2': 1.0, 'joint3': -1.0, 'joint4': 1.0}
JOG_STEP = 60          # arm_jog.py's own per-message ceiling; never exceed it

# 姿态表：joint2=肩、joint3=肘、joint4=腕俯仰（joint1 是底座偏航，保持不动）
POSES = {
    'ready': {
        'joints': {'joint2': 300, 'joint3': 910, 'joint4': 490},
        'desc': '抓取预备位：臂前伸、相机俯视工作区（2026-11-03 抓取成功时的姿态）',
    },
    'search_home': {
        'joints': {'joint2': 681, 'joint3': 925, 'joint4': 204},
        'desc': '用户认可的通用寻找姿态：覆盖地面 0.22~0.88 m，相机高 0.18 m、俯角 40°',
    },
    'search_near': {
        'joints': {'joint2': 550, 'joint3': 950, 'joint4': 280},
        'desc': '找近处 0.17~0.30 m（爪尖离地 10 cm）',
    },
    'search_mid': {
        'joints': {'joint2': 640, 'joint3': 930, 'joint4': 280},
        'desc': '找中距 0.28~0.62 m（爪尖离地 18 cm）',
    },
    'search_far': {
        'joints': {'joint2': 300, 'joint3': 600, 'joint4': 350},
        'desc': '找远处 0.53~1.30 m（爪尖离地 20 cm）',
    },
    'search_floor': {
        'joints': {'joint2': 430, 'joint3': 880, 'joint4': 430},
        'desc': '旧的地面姿态（覆盖 0.33~0.57 m），保留兼容',
    },
    'search_high': {
        'joints': {'joint2': 620, 'joint3': 700, 'joint4': 480},
        'desc': '看高处：抬肩收肘，相机接近水平，覆盖桌面/架子高度',
    },
    'carry': {
        'joints': {'joint2': 520, 'joint3': 760, 'joint4': 470},
        'desc': '搬运位：夹住东西后收臂，重心靠近车体',
    },
}


def read_ticks(bridge, timeout=8.0):
    bridge.drain(0.3)
    message = bridge.wait_for_topic('/joint_states', timeout, newest=True)
    if message is None:
        return None
    return {name: int(round(500.0 + SIGNS[name] * float(position) / TICK_RAD))
            for name, position in zip(message['name'], message['position'])
            if name in SIGNS}


def secure_arm_gate(bridge, timeout=120):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        state = status(bridge)
        if state.get('reset_eligible') and (state.get('healthy_seconds') or 0) > 11:
            if reset_estop(bridge) and set_gates(bridge, arm=True, gripper=False):
                return True
        time.sleep(2)
    return False


def jog(joint, delta, duration=1.6):
    result = subprocess.run(
        [sys.executable, os.path.join(HERE, 'arm_jog.py'), '--joint', joint,
         '--delta-ticks', str(int(delta)), '--duration', str(duration)],
        capture_output=True, text=True, timeout=150, cwd=REPO)
    return 'moved as commanded' in (result.stdout or '')


def goto_pose(bridge, name, tolerance=15, verbose=True):
    """把臂移动到命名姿态；逐关节靠近，最多六轮回补（死区会吃掉小步）。

    每步都看 jog 的返回值：曾经出现过"六轮全部被拒、函数照样返回成功"的情况
    （2026-11-03，搜索时姿态没动但日志显示已切换）。被拒就 sleep 8 s 再试，
    不要连着重试——连续高频命令会把舵机总线搞僵。
    """
    if name not in POSES:
        raise SystemExit('未知姿态 %r，可用：%s' % (name, ', '.join(POSES)))
    joints_spec = POSES[name]['joints']
    # 顺序很关键：先把 joint2 抬高给前臂让出空间，再压 joint3；反过来前臂会顶在地上
    target = {key: joints_spec[key] for key in ('joint2', 'joint3', 'joint4')
              if key in joints_spec}
    ticks = read_ticks(bridge)
    if ticks is None:
        raise SystemExit('没有关节反馈')
    refused = 0
    for round_index in range(6):
        pending = False
        for joint, value in target.items():
            delta = value - ticks[joint]
            if abs(delta) <= tolerance:
                continue
            pending = True
            # arm_jog.py refuses anything above 60 ticks per message as its own
            # safety limit, and a refusal is silent here -- it looks exactly
            # like a servo that will not move.  Asking for 70 made every single
            # command die in the caller on 2026-11-04, so the arm never budged.
            step = max(-JOG_STEP, min(JOG_STEP, delta))
            if verbose:
                print('    %s %+d (-> %d)' % (joint, step, value), flush=True)
            moved = jog(joint, step)
            fresh = read_ticks(bridge)
            if fresh:
                ticks.update(fresh)
            if not moved:
                refused += 1
                if refused >= 2:
                    print('    %s 连续被拒，等 8 s 让总线缓过来' % joint, flush=True)
                    time.sleep(8)
                    refused = 0
        if not pending:
            break
    return ticks


def capture(path):
    with urllib.request.urlopen(SNAPSHOT, timeout=15) as response:
        data = response.read()
    with open(path, 'wb') as handle:
        handle.write(data)
    return path


def describe_camera(ticks):
    """用 FK + 手眼外参算出相机在 arm_base 里的位置和俯仰，便于判断姿态是否合适。"""
    tool = forward_tool_pose_raw(ticks)
    rotation_tool_cam, translation_tool_cam, _ = load_calibration(CALIBRATION)

    def matmul(a, b):
        return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)]
                for i in range(3)]

    rotation_base_cam = matmul(tool['rotation_matrix'], rotation_tool_cam)
    cam_position = [sum(tool['rotation_matrix'][i][k] * translation_tool_cam[k]
                        for k in range(3)) + tool['translation_m'][i] for i in range(3)]
    # 光轴在 arm_base 里的方向：相机坐标 z 轴
    axis = [rotation_base_cam[i][2] for i in range(3)]
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, axis[2]))))
    return {'camera_position': [round(v, 4) for v in cam_position],
            'axis': [round(v, 3) for v in axis],
            'pitch_deg': round(pitch, 1)}


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--list', action='store_true')
    parser.add_argument('--pose', default=None)
    parser.add_argument('--capture', default=None, help='移动后保存一张实拍图')
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args()

    if args.list or not args.pose:
        for name, info in POSES.items():
            print('%-13s %s' % (name, info['desc']))
            print('              %s' % info['joints'])
        return 0

    bridge = Rosbridge('127.0.0.1', 19091, timeout=8)
    try:
        before = read_ticks(bridge)
        print('当前: %s' % before)
        if not args.dry_run:
            if not secure_arm_gate(bridge):
                raise SystemExit('拿不到臂门控')
            time.sleep(1)
            print('移动到 %s（%s）' % (args.pose, POSES[args.pose]['desc']))
            after = goto_pose(bridge, args.pose)
        else:
            after = before
        print('到位: %s' % after)
        if after:
            print('相机：%s' % json.dumps(describe_camera(after), ensure_ascii=False))
        if args.capture:
            print('实拍: %s' % capture(args.capture))
        return 0
    finally:
        bridge.close()


if __name__ == '__main__':
    sys.exit(main())
