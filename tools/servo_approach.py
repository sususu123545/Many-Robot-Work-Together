#!/usr/bin/env python3
"""缓慢逼近 + 机械臂跟随目标（视觉伺服）。

为什么要这样：固定姿态切换会让球掉出画面（前伸的预备位相机下俯更陡，0.25 m
的球直接落到画面下缘以外，2026-11-03 实测）。改成**边走边追**之后，球一直被
joint1/joint2 压在画面中部，枪口（爪）自然对准，最后只需合爪。

控制律（每个周期，约 0.5 s）：
    joint1 += -(u - 320) / 2.1     # 水平：把球拉回画面中线
    joint2 += -(v - 320) / 2.1     # 垂直：把球压到画面中下部
    底盘    前进 0.05 m/s × 0.4 s   # 距离靠球半径判定
停止条件：球半径 >= 52 px（约 0.18 m）或球已到画面下缘。

注意：整段只用**臂门控**（关节要动），底盘不受门控，所以前进不需要切门。
夹爪在这期间会自然回位，抓取阶段再单独张开。
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

from PIL import Image                                        # noqa: E402
import car_approach as ca                                    # noqa: E402
from rosbridge_client import Rosbridge                       # noqa: E402

SNAPSHOT = ('http://127.0.0.1:19099/snapshot'
            '?topic=/rear_camera/image_raw&type=ros_compressed')
PYTHON = sys.executable
TARGET_U = 320.0
TARGET_V = 330.0
PX_PER_CMD_TICK = 2.1
MIN_RADIUS, MAX_RADIUS = 18.0, 140.0     # 可抓尺寸（约 0.08~0.6 m）
FAR_MIN_RADIUS = 6.0                     # "远处可能是个球"的下限
MIN_FILL = 0.55                          # 球投影是实心圆，fill≈0.78
ASPECT_RANGE = (0.6, 1.7)                # 球的宽高比接近 1


def log(message):
    print(message, flush=True)


def frame(attempts=4):
    """取一帧**可用**的图像。

    web_video_server 的 /snapshot 每次都是新订阅，偶发返回全黑或异常帧
    （已知问题，日志里记过）。抓取循环靠单帧判定，撞上一帧坏图就会误判
    "目标丢失"，所以这里连取几帧，取第一张平均亮度正常的。
    """
    last = None
    for _ in range(attempts):
        try:
            with urllib.request.urlopen(SNAPSHOT, timeout=15) as response:
                image = Image.open(BytesIO(response.read())).convert('RGB')
        except Exception:                          # noqa: BLE001
            time.sleep(0.3)
            continue
        last = image
        gray = image.convert('L')
        mean = sum(gray.getdata()) / float(gray.width * gray.height)
        if mean > 18:                              # 正常场景远高于此
            return image
        time.sleep(0.4)
    return last


def locate(min_radius=MIN_RADIUS):
    """按尺寸筛选找球，**不限画面区域**。

    伺服过程中球会从画面顶部一路被压到中下部；如果沿用"只看画面下 65%"的
    地板闸门，球在顶部时会被判成不存在（2026-11-03 实测 cycle 2 直接丢目标）。
    """
    from car_scan import connected_components
    image = frame()
    if image is None:
        return None, '相机取帧失败'
    best = None
    for component in connected_components(ca.redness_mask(image)):
        radius = (component['area'] / 3.14159) ** 0.5
        aspect = component['width'] / float(max(1, component['height']))
        if not min_radius <= radius <= MAX_RADIUS:
            continue
        if component['fill'] < MIN_FILL:
            continue
        if not ASPECT_RANGE[0] <= aspect <= ASPECT_RANGE[1]:
            continue
        if best is None or radius > best['radius_px']:
            best = {'center_x': component['center_x'],
                    'center_y': component['center_y'],
                    'radius_px': radius}
    if best is None:
        return None, '没有可抓尺寸的红色候选'
    return best, ''


def read_limits():
    """读一次关节原始 tick，用于判断是否贴近限位。"""
    import json as _json
    bridge = Rosbridge('127.0.0.1', 19091, timeout=8)
    try:
        bridge.drain(0.2)
        message = bridge.wait_for_topic('/joint_states', 4.0, newest=True)
        if message is None:
            return {}
        tick_rad = math.radians(240.0) / 1000.0
        signs = {'joint1': 1.0, 'joint2': 1.0}
        return {name: int(round(500.0 + signs[name] * float(pos) / tick_rad))
                for name, pos in zip(message['name'], message['position'])
                if name in signs}
    finally:
        bridge.close()


def rotate_chassis(bridge, degrees, rate=0.18):
    """原地转向（负角度=顺时针）。"""
    seconds = abs(math.radians(degrees)) / rate
    twist = {'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
             'angular': {'x': 0.0, 'y': 0.0,
                         'z': math.radians(degrees) / max(seconds, 1e-3)}}
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        bridge.publish('/cmd_vel', twist, 'geometry_msgs/msg/Twist')
        time.sleep(0.1)
    stop(bridge)
    time.sleep(0.4)


def jog(joint, delta, duration=1.0):
    result = subprocess.run(
        [PYTHON, os.path.join(HERE, 'arm_jog.py'), '--joint', joint,
         '--delta-ticks', str(int(delta)), '--duration', str(duration)],
        capture_output=True, text=True, timeout=150, cwd=REPO)
    return 'moved as commanded' in (result.stdout or '')


def drive(bridge, linear, seconds):
    twist = {'linear': {'x': float(linear), 'y': 0.0, 'z': 0.0},
             'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}}
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        bridge.publish('/cmd_vel', twist, 'geometry_msgs/msg/Twist')
        time.sleep(0.05)


def stop(bridge):
    zero = {'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}}
    for _ in range(3):
        bridge.publish('/cmd_vel', zero, 'geometry_msgs/msg/Twist')
        time.sleep(0.05)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--target-radius', type=float, default=42.0,
                        help='球半径达到这个像素值就停（42 px ≈ 0.22 m，已在可抓范围）')
    parser.add_argument('--max-cycles', type=int, default=14)
    parser.add_argument('--speed', type=float, default=0.07,
                        help='前进速度；低于 0.05 会落进麦轮死区，实测原地不动')
    parser.add_argument('--min-radius', type=float, default=MIN_RADIUS,
                        help='接受的最小半径；找远处目标时用 6~10')
    parser.add_argument('--verify-growth', action='store_true',
                        help='远距候选需要证明球在变大，否则判为假目标')
    args = parser.parse_args()

    report = {'ok': False, 'cycles': []}
    bridge = Rosbridge('127.0.0.1', 19091, timeout=8)
    try:
        misses = 0
        first_radius = None
        for cycle in range(1, args.max_cycles + 1):
            blob, reason = locate(args.min_radius)
            if blob is None:
                misses += 1
                log('cycle %d: 漏检 %d/2（%s）' % (cycle, misses, reason))
                if misses >= 2:
                    # 球在近场掉出画面 = 相机已经贴到球上方，这就是到位了
                    if last_good and (last_good[0] >= 30 or last_good[1] >= 435):
                        log('  球已进入近场盲区（上一帧 r=%.0f v=%.0f），判定到位'
                            % last_good)
                        stop(bridge)
                        report.update(ok=True,
                                      detail='已逼近（球离开近场视野，r=%.0f）' % last_good[0])
                        return finish(report, 0)
                    report['detail'] = '连续漏检: %s' % reason
                    return finish(report, 2)
                time.sleep(0.3)
                continue
            misses = 0
            last_good = None
            u, v, r = blob['center_x'], blob['center_y'], blob['radius_px']
            last_good = (r, v)
            distance = ca.FX * 0.0175 / max(1.0, r)
            log('cycle %d: u=%.0f v=%.0f r=%.0f (~%.2f m)' % (cycle, u, v, r, distance))
            report['cycles'].append([round(u), round(v), round(r, 1)])
            # 远距候选验证：真球靠近时半径必须变大；墙面红色花纹不会
            if args.verify_growth:
                radii = [item[2] for item in report['cycles']]
                # 每周期只前进 2 cm，半径增长本来就慢：阈值放 6% 且至少 4 个周期
                # （原来 12%/3 周期会把真目标误判成假目标，2026-11-03 实测 25->28 被误杀）
                if len(radii) >= 4 and radii[-1] < radii[0] * 1.06:
                    log('  半径没有增长（%.0f -> %.0f），判定为假目标'
                        % (radii[0], radii[-1]))
                    report['detail'] = 'false_positive'
                    report['ok'] = False
                    return finish(report, 3)
            # 距离窗口：理想工作距离 0.20~0.26 m（r 约 44~52）
            # 太近（r 太大）时球会掉进相机近场盲区，抓取阶段根本看不见它，
            # 所以要主动后退——只前进不后退的调节器在这里会把自己逼死。
            if r > max(args.target_radius + 8, 50) and cycle >= 1:
                log('  太近了（r=%.0f），后退拉回工作距离' % r)
                drive(bridge, -0.07, 0.5)
                time.sleep(0.3)
                continue

            if r >= args.target_radius or (r >= 38 and abs(u - TARGET_U) <= 70 and cycle >= 2):
                log('  已到位（半径 %.0f >= %.0f）' % (r, args.target_radius))
                stop(bridge)
                report.update(ok=True, detail='已逼近到可抓距离 %.2f m' % distance)
                return finish(report, 0)

            # 关节接近限位就别再纠偏了（顶到限位只会原地空转，还浪费周期）
            limits = read_limits()
            pan = max(-45, min(45, round(-(u - TARGET_U) / PX_PER_CMD_TICK)))
            tilt = max(-35, min(35, round(-(v - TARGET_V) / PX_PER_CMD_TICK)))
            if limits.get('joint1') is not None and (
                    limits['joint1'] < 60 and pan < 0 or limits['joint1'] > 940 and pan > 0):
                log('    joint1 接近限位，跳过水平纠偏')
                pan = 0
            if limits.get('joint2') is not None and (
                    limits['joint2'] < 155 and tilt < 0 or limits['joint2'] > 855 and tilt > 0):
                log('    joint2 接近限位，跳过俯仰纠偏')
                tilt = 0
            if abs(pan) >= 5:
                jog('joint1', pan, duration=0.9)
            elif abs(u - TARGET_U) > 110:
                # joint1 顶到限位但球还在画面边上：改用**车身转向**把球拉回来。
                # 相机随臂转，所以车身左转 = 球在画面里右移（与 joint1 同向）。
                yaw_deg = max(-22.0, min(22.0, (TARGET_U - u) / 9.4))
                log('    车身转向 %+.0f° 把球拉回画面' % yaw_deg)
                rotate_chassis(bridge, yaw_deg)
            if abs(tilt) >= 5:
                jog('joint2', tilt, duration=0.9)
            drive(bridge, args.speed, 0.5)
            time.sleep(0.3)
        stop(bridge)
        report['detail'] = '达到周期上限仍未到可抓距离'
        return finish(report, 2)
    finally:
        try:
            stop(bridge)
        except Exception:                              # noqa: BLE001
            pass
        bridge.close()


def finish(report, code):
    print('RESULT ' + json.dumps(report, ensure_ascii=False), flush=True)
    return code


if __name__ == '__main__':
    sys.exit(main())
