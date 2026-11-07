#!/usr/bin/env python3
"""抓取红球全流程：搜索 → 接近 → 抓取 → 抬起。

为什么要有这一层：模型只该负责"理解用户说了什么"，不该负责拆解三段几何动作。
find → approach → grasp 各自都要抢一次门控、各自有超时，串起来时任何一环失败
都会让整个任务半途而废。这里把它们做成一条确定性流水线，模型只需要说
"执行 fetch_ball"。

阶段间是**强依赖**的：
  搜索 失败 -> 直接结束（没目标就没必要动）
  接近 失败 -> 结束并报告球在哪一帧丢的
  抓取 失败 -> 报告失败在 IK / 合爪 / 抬升 哪一步

输出最后一行是 `RESULT {...}`，供控制台/适配器解析。
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
from gripper_jog import status                                # noqa: E402

SNAPSHOT = ('http://127.0.0.1:19099/snapshot'
            '?topic=/rear_camera/image_raw&type=ros_compressed')
PYTHON = sys.executable
MIN_RADIUS, MAX_RADIUS = 18.0, 140.0      # 约 0.08..0.6 m，超出就不是能抓的目标
SCAN_STEP_DEG = 30
SCAN_STEPS = 12


def log(message):
    print(message, flush=True)


def grab_frame(attempts=4):
    """取一帧可用图像（/snapshot 偶发全黑，见 web_console 的已知问题记录）。"""
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
        if mean > 18:
            return image
        time.sleep(0.4)
    return last


def find_ball_in_frame(min_radius=MIN_RADIUS):
    """一帧里找球。min_radius 调小就能接受"远处的球"，交给接近阶段验证。"""
    from car_scan import connected_components
    image = grab_frame()
    best = None
    for component in connected_components(ca.redness_mask(image)):
        radius = (component['area'] / 3.14159) ** 0.5
        aspect = component['width'] / float(max(1, component['height']))
        if not min_radius <= radius <= MAX_RADIUS:
            continue
        if component['fill'] < 0.55:
            continue
        if not 0.6 <= aspect <= 1.7:
            continue
        if best is None or radius > best['radius_px']:
            best = {'center_x': component['center_x'],
                    'center_y': component['center_y'],
                    'radius_px': radius}
    if best is None:
        return None, '没有像球的红色候选'
    return best, ''


def rotate(bridge, degrees, rate=0.18):
    seconds = abs(math.radians(degrees)) / rate
    twist = {'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
             'angular': {'x': 0.0, 'y': 0.0, 'z': math.radians(degrees) / max(seconds, 1e-3)}}
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        bridge.publish('/cmd_vel', twist, 'geometry_msgs/msg/Twist')
        time.sleep(0.1)
    stop(bridge)


def stop(bridge):
    zero = {'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}}
    for _ in range(3):
        bridge.publish('/cmd_vel', zero, 'geometry_msgs/msg/Twist')
        time.sleep(0.05)


def set_pose(bridge, name):
    """切到命名搜索姿态（见 tools/arm_poses.py）。"""
    import arm_poses
    try:
        arm_poses.goto_pose(bridge, name, verbose=False)
        return True
    except Exception as error:                     # noqa: BLE001
        log('    姿态切换失败(%s): %s' % (name, error))
        return False


def stage_search(bridge, poses=('search_home', 'search_far', 'search_high'),
                 steps=SCAN_STEPS, min_radius=8.0):
    """两级搜索：先按地面目标扫一圈，再换高处姿态扫一圈。

    实测（2026-11-03）：只用一个姿态扫，地面上的球和架子上的东西只能看到一种；
    而姿态又必须由"臂门控 + 关节到位"才谈得上，所以先摆姿态再转。
    """
    log('[1/3] 搜索红球（按目标高度切换姿态）')
    import arm_poses
    if not arm_poses.secure_arm_gate(bridge):
        log('    拿不到臂门控，姿态切换会失败')
    attempts = []
    best = None
    for pose in poses:
        log('  用 %s 姿态扫描（%d 个角度 x %d°）' % (pose, steps, SCAN_STEP_DEG))
        set_pose(bridge, pose)
        for step in range(steps):
            blob, reason = find_ball_in_frame(min_radius=min_radius)
            if blob:
                distance = ca.FX * 0.0175 / max(1.0, blob['radius_px'])
                log('    找到：u=%.0f v=%.0f r=%.0f 约 %.2f m（%s 第 %d 个角度）'
                    % (blob['center_x'], blob['center_y'], blob['radius_px'],
                       distance, pose, step + 1))
                return {'found': True, 'pose': pose,
                        'pixel': [blob['center_x'], blob['center_y'],
                                  round(blob['radius_px'], 1)],
                        'distance_m': round(distance, 3), 'scan_index': step}
            attempts.append('%s#%d:%s' % (pose, step + 1, reason))
            log('    第 %d 个角度没看到（%s）' % (step + 1, reason))
            rotate(bridge, SCAN_STEP_DEG)
            time.sleep(0.5)
    return {'found': False, 'detail': '%d 个姿态各转一圈都没找到可抓的红球' % len(poses),
            'attempts': attempts[-6:]}


def run_stage(script, args, timeout):
    result = subprocess.run([PYTHON, os.path.join(HERE, script)] + args,
                            capture_output=True, text=True, timeout=timeout, cwd=REPO)
    tail = [line for line in (result.stdout or '').splitlines() if line.strip()]
    payload = None
    for line in reversed(tail):
        if line.startswith('RESULT '):
            try:
                payload = json.loads(line[len('RESULT '):])
            except ValueError:
                payload = None
            break
    return result.returncode == 0, payload, tail[-3:]


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--stop-distance', type=float, default=0.30,
                        help='接近阶段停在离球多近（相机距离，抓取需要比"看得见"更近）')
    parser.add_argument('--no-grasp', action='store_true', help='只找并靠近，不抓')
    parser.add_argument('--search-only', action='store_true', help='只搜索，不动底盘接近')
    parser.add_argument('--poses', default='search_home,search_far,search_high',
                        help='搜索姿态，逗号分隔，按顺序试')
    parser.add_argument('--steps', type=int, default=SCAN_STEPS,
                        help='每个姿态转多少个 %d° 的角' % SCAN_STEP_DEG)
    parser.add_argument('--min-radius', type=float, default=8.0,
                        help='接受多小的红色候选（像素半径）')
    args = parser.parse_args()

    report = {'ok': False, 'stage': 'search'}
    bridge = Rosbridge('127.0.0.1', 19091, timeout=8)
    try:
        poses = tuple(part.strip() for part in args.poses.split(',') if part.strip())
        search = stage_search(bridge, poses=poses, steps=args.steps,
                              min_radius=args.min_radius)
        report['search'] = search
        if not search.get('found'):
            report['detail'] = '没找到红球'
            return finish(report, 2)

        if args.search_only:
            report.update(ok=True, stage='search_done',
                          detail='找到红球：%s 姿态，约 %.2f m'
                          % (search['pose'], search['distance_m']))
            return finish(report, 0)

        log('[2/3] 缓慢逼近，机械臂跟随目标（视觉伺服）')
        import arm_poses
        # 只在门控真的关着时才 reset：reset 会给六个舵机发一次 stop 报文，
        # 臂会轻微下沉、视野跟着变，刚找到的球就丢了（2026-11-03 实测）。
        current = status(bridge)
        if current.get('estop') or not current.get('enable_arm_motion'):
            arm_poses.secure_arm_gate(bridge)
            time.sleep(1.0)
        ok, payload, tail = run_stage(
            'servo_approach.py',
            ['--target-radius', '42', '--min-radius', '8', '--verify-growth'],
            timeout=520)
        report['approach'] = {'ok': ok, 'tail': tail, 'payload': payload}
        if payload and payload.get('detail') == 'false_positive':
            report['false_positive'] = True
            report['detail'] = '候选是假目标（靠近后不变大）'
            return finish(report, 3)
        if not ok:
            report['stage'] = 'approach'
            report['detail'] = '接近失败: %s' % (' | '.join(tail)[:160])
            return finish(report, 2)

        if args.no_grasp:
            report.update(ok=True, stage='approach_done', detail='已靠近，未抓取')
            return finish(report, 0)

        log('[3/3] 抓取并抬起')
        ok, payload, tail = run_stage('grasp_sequence.py', [], timeout=600)
        report['grasp'] = {'ok': ok, 'result': payload, 'tail': tail}
        if ok and payload and payload.get('ok'):
            report.update(ok=True, stage='done',
                          detail='抓取成功，夹爪停在 %s' % payload.get('r_after_lift'))
            return finish(report, 0)
        report['stage'] = 'grasp'
        report['detail'] = (payload or {}).get('detail') or ' | '.join(tail)[:160]
        return finish(report, 2)
    finally:
        try:
            stop(bridge)
        except Exception:                       # noqa: BLE001
            pass
        bridge.close()


def finish(report, code):
    print('RESULT ' + json.dumps(report, ensure_ascii=False), flush=True)
    return code


if __name__ == '__main__':
    sys.exit(main())
