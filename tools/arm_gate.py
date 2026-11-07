#!/usr/bin/env python3
"""带自愈的关节移动：失败就复位、重新授权、再试。

这台臂有个固定脾气：温度读超时会让节点周期性锁存（约每 30~50 s 一次），
锁存期间所有关节"命令被接受但不动"。任何移动代码只要不处理这个，就会出现
"连续 8 次 no move" 的假死（2026-11-03 实测）。

本模块把三件事封在一起：
  * secure_gate()  盯住 reset_eligible 窗口拿门控（不盯窗口的重试必然失败）
  * move_joint()   单关节移动，最多 6 轮；每轮失败就复位重试
  * goto()         多关节依次移动（关节逐个走，别指望多关节单报文）

其它工具（arm_poses / grasp_sequence / servo_approach）都应当通过这里动关节。
"""

import math
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)

from gripper_jog import status, reset_estop, set_gates      # noqa: E402
from rosbridge_client import Rosbridge                      # noqa: E402

TICK_RAD = math.radians(240.0) / 1000.0
SIGNS = {'joint1': 1.0, 'joint2': 1.0, 'joint3': -1.0, 'joint4': 1.0}
PYTHON = sys.executable


def connect(port=19091):
    return Rosbridge('127.0.0.1', port, timeout=8)


def read_ticks(bridge, timeout=8.0):
    bridge.drain(0.3)
    message = bridge.wait_for_topic('/joint_states', timeout, newest=True)
    if message is None:
        return None
    return {name: int(round(500.0 + SIGNS[name] * float(position) / TICK_RAD))
            for name, position in zip(message['name'], message['position'])
            if name in SIGNS}


def secure_gate(bridge, arm=True, gripper=False, timeout=120, quiet=False):
    """盯住健康窗口拿门控。窗口会周期归零，所以必须轮询而不是定时重试。"""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        state = status(bridge)
        if state.get('reset_eligible') and (state.get('healthy_seconds') or 0) > 11:
            if reset_estop(bridge, quiet=quiet) and set_gates(
                    bridge, arm=arm, gripper=gripper, quiet=quiet):
                return True
        time.sleep(2)
    return False


def jog_raw(joint, delta, duration=1.8, timeout=150):
    """发一条单关节命令，返回是否真的动了。"""
    result = subprocess.run(
        [PYTHON, os.path.join(HERE, 'arm_jog.py'), '--joint', joint,
         '--delta-ticks', str(int(delta)), '--duration', str(duration)],
        capture_output=True, text=True, timeout=timeout, cwd=REPO)
    return 'moved as commanded' in (result.stdout or '')


def move_joint(bridge, joint, target, tolerance=18, rounds=6, quiet=False):
    """把单个关节走到 target；每轮不死就复位重试一次。返回最终 tick。"""
    ticks = read_ticks(bridge) or {}
    for round_index in range(rounds):
        current = ticks.get(joint)
        if current is None:
            ticks = read_ticks(bridge) or ticks
            continue
        delta = target - current
        if abs(delta) <= tolerance:
            break
        step = max(-70, min(70, delta))
        moved = jog_raw(joint, step)
        if not quiet:
            print('    %s %+d -> %s' % (joint, step, 'ok' if moved else 'no move'),
                  flush=True)
        if not moved:
            # 锁存了：复位 + 重新授权，然后立刻重试同一步
            if secure_gate(bridge, arm=True, quiet=quiet):
                moved = jog_raw(joint, step)
                if not quiet:
                    print('    复位后续试 %s %+d -> %s'
                          % (joint, step, 'ok' if moved else 'still stuck'), flush=True)
        ticks = read_ticks(bridge) or ticks
    return ticks.get(joint)


def goto(bridge, targets, tolerance=18, quiet=False):
    """按 dict 依次移动多个关节，返回最终 ticks。"""
    ticks = read_ticks(bridge) or {}
    for joint, value in targets.items():
        final = move_joint(bridge, joint, value, tolerance=tolerance, quiet=quiet)
        if final is not None:
            ticks[joint] = final
    return read_ticks(bridge) or ticks


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--set', nargs='+', required=True,
                        help='形如 joint2=370 joint3=700')
    args = parser.parse_args()
    targets = {}
    for item in args.set:
        name, _, value = item.partition('=')
        targets[name] = int(value)

    bridge = connect()
    try:
        if not secure_gate(bridge):
            raise SystemExit('拿不到臂门控')
        print('开始: %s' % read_ticks(bridge))
        final = goto(bridge, targets)
        print('结束: %s' % final)
    finally:
        bridge.close()
