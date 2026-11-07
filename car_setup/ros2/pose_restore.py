#!/usr/bin/env python3
"""开机姿态恢复：把机械臂从当前姿态分步慢速恢复到目标姿态。

在容器启动前通过临时容器执行，此时 /dev/rrc 未被 arm_node 独占。
"""
import json
import math
import sys
import time

sys.path.insert(0, '/opt/armpi_ros2/install/armpi_ros2/lib/python3.12/site-packages')
from armpi_ros2.servo_protocol import JOINTS, ServoBoard, to_radians

TARGET = {'joint1': 503, 'joint2': 697, 'joint3': 844, 'joint4': 189,
          'joint5': 499, 'r_joint': 96}
MAX_STEP = 60
DURATION = 2.0
TEMP_LIMIT = 60
MAX_ROUNDS = 20
STALL_ROUNDS = 3        # consecutive rounds with no progress before giving up


def main():
    board = ServoBoard()
    print('当前姿态:')
    for j, (ident, _, _, lo, hi) in JOINTS.items():
        try:
            print('  %-8s raw=%-4d temp=%s' % (j, board.position(ident), board.temperature(ident)))
        except Exception as e:
            print('  %-8s 读取失败 %s' % (j, e))
    print()

    # Stall guard.  Measured 2026-11-07 at 09:49: this loop ran all 20 rounds and
    # gave up with return 5, i.e. 64 seconds of arm motion at every boot, because
    # TARGET joint3=844 is not a pose the arm can hold -- it settles at about 882,
    # so every round still "needs" 38 more ticks.  The caller
    # (run_native_core.prepare_pose) ignores the result and boots anyway, so the
    # cost is silent: heat, wear, and 64 s of boot delay.
    #
    # Three rounds with no improvement in the worst residual means the target is
    # out of reach, not that the arm needs another nudge.
    best_worst = 10 ** 6
    stalled_rounds = 0

    for rnd in range(1, MAX_ROUNDS + 1):
        targets = []
        done = True
        worst = 0
        for j, goal in TARGET.items():
            ident, _, _, lo, hi = JOINTS[j]
            try:
                cur = board.position(ident)
                temp = board.temperature(ident)
            except Exception as e:
                print('第%d轮读取 %s 失败: %s' % (rnd, j, e))
                board.close()
                return 2
            if temp >= TEMP_LIMIT:
                print('温度超限中止: %s=%dC' % (j, temp))
                board.close()
                return 3
            delta = goal - cur
            worst = max(worst, abs(delta))
            if abs(delta) > 6:
                done = False
            step = max(-MAX_STEP, min(MAX_STEP, delta))
            targets.append((ident, max(lo, min(hi, cur + step))))

        if done:
            print('全部到达目标姿态')
            break

        if worst < best_worst - 1:
            best_worst = worst
            stalled_rounds = 0
        else:
            stalled_rounds += 1
            if stalled_rounds >= STALL_ROUNDS:
                print('连续 %d 轮无进展（最好残差 %d tick），目标可能超出行程，中止'
                      % (stalled_rounds, best_worst))
                board.close()
                return 6

        try:
            board.move(DURATION, targets)
        except Exception as e:
            print('第%d轮下发失败: %s' % (rnd, e))
            board.close()
            return 4
        time.sleep(DURATION + 1.2)
        print('第%d轮完成' % rnd)
    else:
        print('达到最大轮次仍未到位')
        board.close()
        return 5

    print('\n最终姿态:')
    ok = True
    result = {}
    for j, goal in TARGET.items():
        ident = JOINTS[j][0]
        try:
            cur = board.position(ident)
        except Exception:
            cur = None
        result[j] = cur
        delta = None if cur is None else cur - goal
        if delta is not None and abs(delta) > 12:
            ok = False
        print('  %-8s raw=%-4s 目标=%-4d 差=%s' % (j, cur, goal, delta))
    board.close()
    print('\nRESULT_JSON=' + json.dumps({'ok': ok, 'final': result}, sort_keys=True))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
