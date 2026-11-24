#!/usr/bin/env python3
"""Jog the arm back to the recorded startup pose and reopen the jaw.

For after an interrupted grasp leaves the arm parked mid-sweep and the camera
can no longer see the ball.  Uses the same gate and jog helpers as the grasp
run itself, so it obeys the estop ladder and reads feedback after every step.

Startup pose: joint1=486 joint2=682 joint3=915 joint4=194 joint5=499, jaw open.
"""
import sys
import time

sys.path.insert(0, 'tools')
sys.path.insert(0, 'ros2_ws/src/armpi_ros2/armpi_ros2')

import grasp_sequence as gs

STARTUP = {'joint1': 486, 'joint2': 682, 'joint3': 915, 'joint4': 194}
OPEN_TICK = 128


def main():
    bridge = gs.open_bridge() if hasattr(gs, 'open_bridge') else None
    if bridge is None:
        from rosbridge_client import Rosbridge
        bridge = Rosbridge('127.0.0.1', 19091)
        bridge.drain(0.5)
    ticks, _ = gs.read_ticks_and_stamp(bridge)
    if ticks is None:
        print('没有新鲜反馈，退出')
        return 2
    print('当前:', {k: ticks.get(k) for k in sorted(ticks) if k.startswith('joint') or k == 'r_joint'})

    if not gs.secure_gate(bridge, arm=True, gripper=False):
        print('拿不到运动授权，退出')
        return 2
    for name, goal in STARTUP.items():
        while True:
            ticks, _ = gs.read_ticks_and_stamp(bridge)
            if ticks is None:
                print('  %s 反馈丢失，中止' % name)
                return 2
            delta = goal - ticks[name]
            if abs(delta) <= 8:
                break
            step = max(-90, min(90, delta))
            _, after = gs.jog_ex(name, step, bridge=bridge)
            if after:
                ticks.update(after)
            time.sleep(0.5)
        print('  %s -> %d（目标 %d）' % (name, ticks[name], goal))

    if not gs.secure_gate(bridge, arm=False, gripper=True):
        print('拿不到夹爪授权，退出')
        return 2
    gs.open_wide(bridge, OPEN_TICK)
    print('夹爪已张开')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
