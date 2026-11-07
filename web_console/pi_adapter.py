# -*- coding: utf-8 -*-
"""真车执行层：把 LLM 规划出的一个动作，落到已经在实车上验证过的工具上。

设计要点
--------
* **每个动作都是一个独立子进程**（tools/*.py）。原因是这些工具里有大量阻塞式
  往返（等健康窗口、等舵机走完、等检测），放进 FastAPI 的事件循环会拖住整个控制台；
  子进程还能超时杀掉，不会把控制台进程一起带崩。
* **结果回灌给模型**。run() 返回的 summary 是人类可读的一句话，会作为 observation
  发回模型，让它决定下一步——这正是"模型自己给小车发布任务"的闭环。
* 所有动作都要求隧道（19091 rosbridge / 19099 web_video_server）在线；不在线就
  明确报错，而不是假装执行。

动作清单与 tools/ 的对应关系：
    find_ball      -> 本文件内的扫描循环（底盘小角度旋转 + 红球检测）
    approach_ball  -> tools/ball_approach.py
    grasp_ball     -> tools/grasp_sequence.py
    release_ball   -> 本文件内（夹爪门 + 张开）
    report_status  -> 本文件内（关节/夹爪/底盘/球是否可见）
    stop           -> 本文件内（零速 + 撤销命令）
"""

import json
import math
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
TOOLS = os.path.join(REPO, 'tools')
sys.path.insert(0, TOOLS)

PYTHON = sys.executable


def _run_tool(script, args, timeout, progress=None):
    """跑一个工具子进程，返回 (ok, stdout, stderr)。"""
    command = [PYTHON, os.path.join(TOOLS, script)] + [str(item) for item in args]
    if progress:
        progress('开始执行 %s' % script)
    try:
        result = subprocess.run(command, capture_output=True, text=True,
                                timeout=timeout, cwd=REPO)
    except subprocess.TimeoutExpired:
        return False, '', '超时（%d s）被终止' % timeout
    return result.returncode == 0, result.stdout or '', result.stderr or ''


def _bridge():
    from rosbridge_client import Rosbridge
    return Rosbridge('127.0.0.1', 19091, timeout=8)


def _last_result_line(text):
    for line in reversed((text or '').splitlines()):
        if line.startswith('RESULT '):
            try:
                return json.loads(line[len('RESULT '):])
            except ValueError:
                return None
    return None


# ---------------------------------------------------------------------------
# 各动作实现
# ---------------------------------------------------------------------------
def _raise_arm_for_search(progress=None):
    """把机械臂抬到"平视"搜索位（joint2 上抬），失败也不阻塞扫描。

    捡完球后相机是朝下看地板的，直接转圈会漏掉远处的球——这是 2026-11-03
    第一次让模型自主找球时踩到的坑（转满一圈什么都没看见）。
    """
    from gripper_jog import status, reset_estop, set_gates
    bridge = _bridge()
    try:
        deadline = time.monotonic() + 120
        granted = False
        while time.monotonic() < deadline:
            state = status(bridge)
            if state.get('reset_eligible') and (state.get('healthy_seconds') or 0) > 11:
                if reset_estop(bridge) and set_gates(bridge, arm=True, gripper=False):
                    granted = True
                    break
            time.sleep(2)
        if not granted:
            return False
        if progress:
            progress('抬臂到搜索位…')
        for _ in range(2):
            result = subprocess.run(
                [PYTHON, os.path.join(TOOLS, 'arm_jog.py'), '--joint', 'joint2',
                 '--delta-ticks', '35', '--duration', '1.6'],
                capture_output=True, text=True, timeout=150, cwd=REPO)
            if 'moved as commanded' not in (result.stdout or ''):
                break
        return True
    except Exception:                              # noqa: BLE001
        return False
    finally:
        bridge.close()


def action_find_ball(progress=None):
    """原地小角度旋转扫描，最多一圈半；找到就报方向和像素半径。"""
    import car_approach as ca
    import urllib.request
    from io import BytesIO
    from PIL import Image

    snapshot = ('http://127.0.0.1:19099/snapshot'
                '?topic=/rear_camera/image_raw&type=ros_compressed')
    # 先抬臂：捡完球后相机是朝下看地板的，直接转圈会漏掉远处的球
    _raise_arm_for_search(progress)

    bridge = _bridge()
    try:
        for step in range(12):                     # 12 x 30° = 一整圈
            if progress:
                progress('扫描中：第 %d 个角度' % (step + 1))
            with urllib.request.urlopen(snapshot, timeout=15) as response:
                image = Image.open(BytesIO(response.read())).convert('RGB')
            blob, _ = ca.locate_ball(image)
            if blob and blob['radius_px'] >= 12:
                return {'ok': True,
                        'summary': '找到红球：画面 u=%.0f（偏向%s），估计距离 %.2f m'
                                   % (blob['center_x'],
                                      '左' if blob['center_x'] < 280 else
                                      ('右' if blob['center_x'] > 360 else '正前'),
                                      ca.FX * 0.0175 / max(1.0, blob['radius_px']))}
            # 转 30 度（0.18 rad/s 已标定）
            twist = {'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                     'angular': {'x': 0.0, 'y': 0.0, 'z': math.radians(30) / 3.33}}
            deadline = time.monotonic() + 3.33
            while time.monotonic() < deadline:
                bridge.publish('/cmd_vel', twist, 'geometry_msgs/msg/Twist')
                time.sleep(0.1)
            bridge.publish('/cmd_vel', {'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                                        'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}},
                           'geometry_msgs/msg/Twist')
            time.sleep(0.6)
        return {'ok': False, 'summary': '转了一整圈也没看到红球'}
    except Exception as error:                     # noqa: BLE001
        return {'ok': False, 'summary': '扫描失败: %s' % error}
    finally:
        bridge.close()


def action_approach_ball(progress=None):
    ok, out, err = _run_tool('ball_approach.py', [], timeout=420, progress=progress)
    tail = [line for line in out.splitlines() if line.strip().startswith('step')][-1:] \
        or out.strip().splitlines()[-1:]
    detail = tail[0].strip() if tail else (err.strip()[:120] or '无输出')
    return {'ok': ok, 'summary': ('已经开到球附近，' if ok else '接近失败，') + detail}


def action_fetch_ball(progress=None):
    """一条龙：按姿态搜索 -> 视觉伺服接近 -> 标定定位抓取 -> 抬升。

    这就是"抓取红色小球"对应的完整链路，模型只需要选这一个动作，
    中间所有几何/姿态/门控都由确定性代码负责。
    """
    ok, out, err = _run_tool('fetch_ball.py', [], timeout=900, progress=progress)
    result = _last_result_line(out)
    if result:
        if result.get('ok'):
            grasp = (result.get('grasp') or {}).get('result') or {}
            return {'ok': True,
                    'summary': '抓到了：搜索阶段在 %s 姿态发现目标，抓取后夹爪停在 %s'
                               % ((result.get('search') or {}).get('pose'),
                                  grasp.get('r_after_lift')),
                    'data': result}
        return {'ok': False,
                'summary': '没完成（卡在 %s 阶段）：%s'
                           % (result.get('stage'), (result.get('detail') or '')[:120]),
                'data': result}
    return {'ok': False, 'summary': '抓取流水线异常: %s' % (err.strip()[:160] or '无输出')}


def action_grasp_ball(progress=None):
    ok, out, err = _run_tool('grasp_sequence.py', [], timeout=600, progress=progress)
    result = _last_result_line(out)
    if result:
        summary = ('抓取成功：夹爪停在 %s，球被夹住并抬起'
                   % result.get('r_after_lift')) if result.get('ok') else \
                  ('抓取未完成（%s）：%s' % (result.get('stage'), result.get('detail')))
        return {'ok': bool(result.get('ok')), 'summary': summary, 'data': result}
    return {'ok': False, 'summary': '抓取脚本异常: %s' % (err.strip()[:160] or '无输出')}


def action_release_ball(progress=None):
    from gripper_jog import status, reset_estop, set_gates
    bridge = _bridge()
    try:
        deadline = time.monotonic() + 120
        granted = False
        while time.monotonic() < deadline:
            state = status(bridge)
            if state.get('reset_eligible') and (state.get('healthy_seconds') or 0) > 11:
                if reset_estop(bridge) and set_gates(bridge, arm=False, gripper=True):
                    granted = True
                    break
            time.sleep(2)
        if not granted:
            return {'ok': False, 'summary': '拿不到夹爪门控，没法放球'}
        from gripper_jog import read_r
        start = read_r(bridge)
        if progress:
            progress('夹爪张开中（当前 r_joint=%s）…' % start)
        # 夹着球时张开要克服摩擦，必须逐级给目标并**每次验证**：
        # 之前一次性发 169 就报成功，实际爪子还夹在 321（2026-11-03 实测）。
        position = start or 320
        for target, duration in ((250, 2.0), (200, 2.0), (169, 2.5), (140, 2.5)):
            bridge.drain(0.3)
            message = bridge.wait_for_topic('/joint_states', 6.0)
            stamp = message.get('header', {}).get('stamp') if message else {'sec': 0, 'nanosec': 0}
            bridge.publish('/gripper_controller/command', {
                'header': {'stamp': stamp, 'frame_id': ''},
                'joint_names': ['r_joint'],
                'points': [{'positions': [(target - 700) * math.pi / 1000.0],
                            'velocities': [], 'accelerations': [], 'effort': [],
                            'time_from_start': {'sec': int(duration), 'nanosec': 0}}],
            }, 'trajectory_msgs/msg/JointTrajectory')
            time.sleep(duration + 1.2)
            position = read_r(bridge) or position
            if progress:
                progress('r_joint=%s' % position)
            if position <= 200:
                break
        opened = position <= 230
        return {'ok': opened,
                'summary': ('夹爪已张开到 %s，球放下' % position) if opened else
                           ('夹爪只开到 %s，球可能还夹着' % position)}
    except Exception as error:                     # noqa: BLE001
        return {'ok': False, 'summary': '放球失败: %s' % error}
    finally:
        bridge.close()


def action_report_status(progress=None):
    from gripper_jog import status, read_r
    from arm_jog import read_ticks
    import car_approach as ca
    import urllib.request
    from io import BytesIO
    from PIL import Image

    bridge = _bridge()
    try:
        state = status(bridge)
        joints = (state.get('joints') or {})
        ticks, _ = read_ticks(bridge)
        gripper = read_r(bridge)
        holding = bool(gripper and 280 <= gripper <= 400)
        parts = ['机械臂 %s' % state.get('health_state'),
                 '关节 %s' % ticks,
                 '夹爪 r_joint=%s（%s）' % (gripper, '看起来夹着东西' if holding else '空'),
                 '运动门 arm=%s gripper=%s' % (state.get('enable_arm_motion'),
                                               state.get('enable_gripper_motion'))]
        try:
            with urllib.request.urlopen(
                    'http://127.0.0.1:19099/snapshot?topic=/rear_camera/image_raw&type=ros_compressed',
                    timeout=10) as response:
                image = Image.open(BytesIO(response.read())).convert('RGB')
            blob, _ = ca.locate_ball(image)
            if blob:
                parts.append('画面里能看到红球（u=%.0f, r=%.0f px）'
                             % (blob['center_x'], blob['radius_px']))
            else:
                parts.append('画面里看不到红球')
        except Exception:                          # noqa: BLE001
            parts.append('相机不可用')
        return {'ok': True, 'summary': '；'.join(parts)}
    except Exception as error:                     # noqa: BLE001
        return {'ok': False, 'summary': '读状态失败: %s' % error}
    finally:
        bridge.close()


def action_stop(progress=None):
    bridge = _bridge()
    try:
        for _ in range(10):
            bridge.publish('/cmd_vel', {'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                                        'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}},
                           'geometry_msgs/msg/Twist')
            time.sleep(0.1)
        return {'ok': True, 'summary': '已发零速，底盘停住（机械臂保持当前姿态）'}
    finally:
        bridge.close()


ACTIONS = {
    'find_ball': action_find_ball,
    'approach_ball': action_approach_ball,
    'fetch_ball': action_fetch_ball,
    'grasp_ball': action_grasp_ball,
    'release_ball': action_release_ball,
    'report_status': action_report_status,
    'stop': action_stop,
    'done': lambda progress=None: {'ok': True, 'summary': '任务结束'},
}


def run(action, params=None, progress=None):
    handler = ACTIONS.get(action)
    if handler is None:
        return {'ok': False, 'summary': '未知动作: %s' % action}
    return handler(progress)


if __name__ == '__main__':
    name = sys.argv[1] if len(sys.argv) > 1 else 'report_status'
    print(json.dumps(run(name, progress=lambda message: print('  .', message)),
                     ensure_ascii=False, indent=2))
