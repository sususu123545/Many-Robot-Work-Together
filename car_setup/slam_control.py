#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# slam_control.py — 话题控制的 SLAM 开关 + 地图保存（hector / slam_toolbox 双引擎）
#
# 订阅: /slam_control/cmd  (std_msgs/String)
#   "start"                用「当前引擎」启动建图
#   "stop"                 停止「当前引擎」（两个引擎都清干净）
#   "save"                 保存当前 /map 到 /home/ubuntu/maps/map_YYYYMMDD_HHMMSS.{pgm,yaml}
#   "save:<name>"          保存到 /home/ubuntu/maps/<name>.{pgm,yaml}
#   "engine:hector"        切换引擎为 hector
#   "engine:slam_toolbox"  切换引擎为 slam_toolbox
#
#   ⚠️ 两个引擎互斥：都在跑会同时发布 map→odom，TF 冲突（表现为地图错乱/节点卡死）。
#      所以 start 前会先停掉另一个。
#
# 发布: /slam_control/status (std_msgs/String, JSON, latched, 2s 心跳)
#   {"running":bool, "busy":bool, "engine":"hector|slam_toolbox",
#    "last":"...", "detail":"...", "ts":...}

import json
import os
import re
import signal
import subprocess
import threading
import time

import rospy
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid

# 用 launch 文件全路径而不是包名 —— 该车 devel 空间未注册 ldlidar_stl_ros，
# rospack 找不到包，按包名 launch 会 RLException。
HECTOR_LAUNCH = ['roslaunch',
                 '/home/ubuntu/ld06_ws/src/ldlidar_stl_ros/launch/ld06_hector.launch']
SLAM_TOOLBOX_LAUNCH = ['roslaunch',
                       '/home/ubuntu/ld06_ws/src/ldlidar_stl_ros/launch/ld06_slam_toolbox.launch']
SLAM_TOOLBOX_RUN = ['bash', '/home/ubuntu/run_slam_toolbox.sh']

MAP_DIR = '/home/ubuntu/maps'
HECTOR_LOG = '/home/ubuntu/hector_run.log'
SLAM_TOOLBOX_LOG = '/home/ubuntu/slam_toolbox_console.log'

# 引擎定义：名字 -> (roslaunch 命令, 存活检测模板, pgrep 模式)
ENGINES = {
    'hector': {
        'launch': HECTOR_LAUNCH,
        'log': HECTOR_LOG,
        'pgrep': '[h]ector_mapping',
        'label': 'hector',
    },
    'slam_toolbox': {
        'launch': SLAM_TOOLBOX_LAUNCH,
        'log': SLAM_TOOLBOX_LOG,
        'pgrep': '[a]sync_slam_toolbox_node',
        'label': 'slam_toolbox',
    },
}
DEFAULT_ENGINE = 'hector'

# 另一个引擎的「清场」模式（互斥用）
OTHER_PGREPS = {
    'hector': ['[l]d06_hector.launch', '[h]ector_mapping'],
    'slam_toolbox': ['[l]d06_slam_toolbox.launch', '[a]sync_slam_toolbox_node'],
}


def write_map_files(grid, path):
    """OccupancyGrid -> map_saver 同款 trinary PGM + YAML"""
    w = grid.info.width
    h = grid.info.height
    res = grid.info.resolution
    org = grid.info.origin.position
    data = grid.data
    with open(path + '.pgm', 'wb') as f:
        f.write(b'P5\n# CREATOR: slam_control.py %.3f m/pix\n%d %d\n255\n'
                % (res, w, h))
        for row in range(h - 1, -1, -1):      # PGM 首行 = 地图最上面一行
            line = bytearray(w)
            base = row * w
            for col in range(w):
                v = data[base + col]
                if v < 0:
                    line[col] = 205            # unknown 灰
                elif v >= 65:
                    line[col] = 0              # occupied 黑
                else:
                    line[col] = 254            # free 白
            f.write(bytes(line))
    with open(path + '.yaml', 'w') as f:
        f.write('image: %s.pgm\n'
                'resolution: %g\n'
                'origin: [%g, %g, 0.0]\n'
                'negate: 0\n'
                'occupied_thresh: 0.65\n'
                'free_thresh: 0.196\n'
                % (os.path.basename(path), res, org.x, org.y))


def pgrep_alive(pattern):
    try:
        r = subprocess.run(['pgrep', '-f', pattern],
                           capture_output=True, timeout=3)
        return r.returncode == 0
    except Exception:
        return False


def pkill_pattern(pattern):
    try:
        subprocess.run(['pkill', '-f', pattern], capture_output=True, timeout=5)
    except Exception:
        pass


class SlamControl(object):
    def __init__(self):
        self.proc = None          # 当前引擎的 roslaunch Popen
        self.engine = DEFAULT_ENGINE
        self.busy = False
        self.last = '就绪'
        self.detail = ''
        self.lock = threading.Lock()
        self.latest_map = None    # (stamp, OccupancyGrid)
        self.engine_pub = rospy.Publisher('/slam_control/engine', String,
                                         queue_size=1, latch=True)
        self.status_pub = rospy.Publisher('/slam_control/status', String,
                                          queue_size=1, latch=True)
        rospy.Subscriber('/slam_control/cmd', String, self.on_cmd)
        rospy.Subscriber('/map', OccupancyGrid, self.on_map, queue_size=1)
        rospy.on_shutdown(self.cleanup)
        os.makedirs(MAP_DIR, exist_ok=True)

    def on_map(self, grid):
        self.latest_map = (time.time(), grid)

    # ---------- 引擎存活 ----------
    def engine_alive(self, name=None):
        name = name or self.engine
        if name == self.engine and self.proc is not None and self.proc.poll() is None:
            return True
        # 进程句柄丢了（例如节点重启过）就用 pgrep 兜底
        return pgrep_alive(ENGINES[name]['pgrep'])

    def any_engine_alive(self):
        return any(self.engine_alive(n) for n in ENGINES)

    # ---------- 状态发布 ----------
    def publish_status(self):
        msg = {
            'running': self.engine_alive(),
            'busy': self.busy,
            'engine': self.engine,
            'last': self.last,
            'detail': self.detail,
            'ts': time.time(),
        }
        self.status_pub.publish(String(json.dumps(msg, ensure_ascii=False)))

    def set_state(self, last, detail='', busy=None):
        with self.lock:
            self.last = last
            self.detail = detail
            if busy is not None:
                self.busy = busy

    # ---------- 命令分发 ----------
    def on_cmd(self, msg):
        cmd = (msg.data or '').strip()
        rospy.loginfo('slam_control got cmd: %s', cmd)
        if cmd == 'start':
            threading.Thread(target=self.do_start).start()
        elif cmd == 'stop':
            threading.Thread(target=self.do_stop).start()
        elif cmd.startswith('save'):
            name = cmd[5:] if len(cmd) > 4 else ''
            threading.Thread(target=self.do_save, args=(name,)).start()
        elif cmd.startswith('engine:'):
            self.do_set_engine(cmd[7:].strip())

    # ---------- 切换引擎 ----------
    def do_set_engine(self, name):
        name = (name or '').strip().lower()
        if name not in ENGINES:
            self.set_state('切换失败', '未知引擎 %s' % name)
            return
        with self.lock:
            if self.busy:
                return
        if name == self.engine:
            self.set_state('已是 %s' % ENGINES[name]['label'], '', busy=False)
            self.publish_status()
            return
        was_running = self.any_engine_alive()
        self.engine = name
        self.proc = None
        self.engine_pub.publish(String(name))
        note = '（原引擎已在运行，需重新点开始建图）' if was_running else ''
        self.set_state('已切换到 %s' % ENGINES[name]['label'], note, busy=False)
        self.publish_status()

    # ---------- 停掉指定引擎（含清场） ----------
    def _kill_engine(self, name):
        if name == self.engine and self.proc is not None and self.proc.poll() is None:
            try:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
            except Exception:
                pass
            for _ in range(12):
                time.sleep(0.5)
                if self.proc.poll() is not None:
                    break
            if self.proc.poll() is None:
                try:
                    os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)
                except Exception:
                    pass
        # 兜底／清场：句柄丢失时直接 pkill
        for pat in OTHER_PGREPS[name]:
            pkill_pattern(pat)
        time.sleep(1)

    # ---------- start ----------
    def do_start(self):
        with self.lock:
            if self.busy:
                return
            self.busy = True
        eng = ENGINES[self.engine]
        try:
            if self.engine_alive():
                self.set_state('%s 已在运行' % eng['label'], busy=False)
                return

            # 互斥：先把另一个引擎停掉，否则两个都发 map→odom
            other = 'slam_toolbox' if self.engine == 'hector' else 'hector'
            if self.engine_alive(other):
                self.set_state('正在停掉 %s …' % ENGINES[other]['label'], busy=True)
                self._kill_engine(other)

            self.set_state('正在启动 %s …' % eng['label'], busy=True)
            logf = open(eng['log'], 'ab')
            self.proc = subprocess.Popen(
                eng['launch'], stdout=logf, stderr=subprocess.STDOUT,
                preexec_fn=os.setsid)
            # 等节点真正注册到 master
            ok = False
            for _ in range(24):
                time.sleep(0.5)
                if self.proc.poll() is not None:
                    break
                if pgrep_alive(eng['pgrep']):
                    ok = True
                    break
            if ok:
                self.set_state('%s 已启动' % eng['label'],
                               'PID %d，推送小车即可建图' % self.proc.pid,
                               busy=False)
            else:
                self.set_state('启动失败', '见容器内 %s' % eng['log'], busy=False)
                self.proc = None
        except Exception as e:
            self.set_state('启动异常', str(e), busy=False)

    # ---------- stop ----------
    def do_stop(self):
        with self.lock:
            if self.busy:
                return
            self.busy = True
        try:
            self.set_state('正在停止 …', busy=True)
            # 两个引擎都停，避免任何一半残留导致 TF 冲突
            self._kill_engine('hector')
            self._kill_engine('slam_toolbox')
            self.proc = None
            alive = [n for n in ENGINES if self.engine_alive(n)]
            if not alive:
                self.set_state('已停止', 'hector / slam_toolbox 均已停止', busy=False)
            else:
                self.set_state('停止失败',
                               '仍有进程: %s' % ','.join(alive), busy=False)
        except Exception as e:
            self.set_state('停止异常', str(e), busy=False)

    # ---------- save ----------
    def do_save(self, name):
        with self.lock:
            if self.busy:
                return
            self.busy = True
        try:
            latest = self.latest_map
            if not self.any_engine_alive():
                self.set_state('保存失败', '没有建图节点在运行，/map 不存在', busy=False)
                return
            if latest is None or time.time() - latest[0] > 10:
                self.set_state('保存失败', '超过 10s 没收到 /map 数据', busy=False)
                return
            name = re.sub(r'[^\w\-\u4e00-\u9fff]', '_', (name or '').strip())
            if not name:
                name = time.strftime('map_%Y%m%d_%H%M%S')
            path = os.path.join(MAP_DIR, name)
            self.set_state('正在保存地图 …', path, busy=True)
            write_map_files(latest[1], path)
            if os.path.exists(path + '.pgm') and os.path.exists(path + '.yaml'):
                sz = os.path.getsize(path + '.pgm') // 1024
                self.set_state('地图已保存',
                               '%s.pgm / .yaml（%d KB）' % (path, sz), busy=False)
            else:
                self.set_state('保存失败', '文件未生成', busy=False)
        except Exception as e:
            self.set_state('保存异常', str(e), busy=False)

    # ---------- 退出清理 ----------
    def cleanup(self):
        try:
            if self.proc is not None and self.proc.poll() is None:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
        except Exception:
            pass

    def spin(self):
        self.engine_pub.publish(String(self.engine))
        rate = rospy.Rate(0.5)   # 2s 心跳，控制台据此判断节点在线
        while not rospy.is_shutdown():
            self.publish_status()
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('slam_control')
    SlamControl().spin()
