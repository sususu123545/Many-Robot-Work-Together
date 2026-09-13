#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# slam_control.py — 话题控制的 hector 建图开关 + 地图保存
#
# 订阅: /slam_control/cmd  (std_msgs/String)
#   "start"          启动 hector (roslaunch ldlidar_stl_ros ld06_hector.launch)
#   "stop"           停止 hector (进程组 SIGINT -> SIGKILL)
#   "save"           保存地图到 /home/ubuntu/maps/map_YYYYMMDD_HHMMSS.{pgm,yaml}
#   "save:<name>"    保存地图到 /home/ubuntu/maps/<name>.{pgm,yaml}
#
# 地图落盘由本节点直接完成（OccupancyGrid -> PGM/YAML，与 map_saver 同格式），
# 不依赖 map_server 包 —— ROS apt 源签名已过期，装不上。
#
# 发布: /slam_control/status (std_msgs/String, JSON, latched, 2s 心跳)
#   {"running":bool, "busy":bool, "last":"...", "detail":"...", "ts":...}

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
MAP_DIR = '/home/ubuntu/maps'
HECTOR_LOG = '/home/ubuntu/hector_run.log'


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


class SlamControl(object):
    def __init__(self):
        self.proc = None          # roslaunch Popen
        self.busy = False
        self.last = '就绪'
        self.detail = ''
        self.lock = threading.Lock()
        self.latest_map = None    # (stamp, OccupancyGrid)
        self.status_pub = rospy.Publisher('/slam_control/status', String,
                                          queue_size=1, latch=True)
        rospy.Subscriber('/slam_control/cmd', String, self.on_cmd)
        rospy.Subscriber('/map', OccupancyGrid, self.on_map, queue_size=1)
        rospy.on_shutdown(self.cleanup)
        os.makedirs(MAP_DIR, exist_ok=True)

    def on_map(self, grid):
        self.latest_map = (time.time(), grid)

    # ---------- 状态发布 ----------
    def hector_alive(self):
        if self.proc is not None and self.proc.poll() is None:
            return True
        # 进程句柄丢了（例如节点重启过）就用 pgrep 兜底
        try:
            r = subprocess.run(['pgrep', '-f', '[h]ector_mapping'],
                               capture_output=True, timeout=3)
            return r.returncode == 0
        except Exception:
            return False

    def publish_status(self):
        msg = {
            'running': self.hector_alive(),
            'busy': self.busy,
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
        self.publish_status()

    # ---------- 命令入口 ----------
    def on_cmd(self, msg):
        cmd = (msg.data or '').strip()
        rospy.loginfo('slam_control cmd: %s', cmd)
        if cmd == 'start':
            threading.Thread(target=self.do_start, daemon=True).start()
        elif cmd == 'stop':
            threading.Thread(target=self.do_stop, daemon=True).start()
        elif cmd == 'save' or cmd.startswith('save:'):
            name = cmd[5:] if ':' in cmd else ''
            threading.Thread(target=self.do_save, args=(name,), daemon=True).start()
        else:
            self.set_state('未知命令', cmd)

    # ---------- start ----------
    def do_start(self):
        with self.lock:
            if self.busy:
                return
            self.busy = True
        try:
            if self.hector_alive():
                self.set_state('hector 已在运行', busy=False)
                return
            self.set_state('正在启动 hector …', busy=True)
            logf = open(HECTOR_LOG, 'ab')
            self.proc = subprocess.Popen(
                HECTOR_LAUNCH, stdout=logf, stderr=subprocess.STDOUT,
                preexec_fn=os.setsid)
            # 等 hector_mapping 真正注册到 master
            ok = False
            for _ in range(20):
                time.sleep(0.5)
                if self.proc.poll() is not None:
                    break
                try:
                    r = subprocess.run(['pgrep', '-f', '[h]ector_mapping'],
                                       capture_output=True, timeout=3)
                    if r.returncode == 0:
                        ok = True
                        break
                except Exception:
                    pass
            if ok:
                self.set_state('hector 已启动', 'PID %d，推送小车即可建图' % self.proc.pid,
                               busy=False)
            else:
                self.set_state('启动失败', '见容器内 %s' % HECTOR_LOG, busy=False)
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
            self.set_state('正在停止 hector …', busy=True)
            stopped = False
            if self.proc is not None and self.proc.poll() is None:
                try:
                    os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
                except Exception:
                    pass
                for _ in range(12):
                    time.sleep(0.5)
                    if self.proc.poll() is not None:
                        stopped = True
                        break
                if not stopped:
                    try:
                        os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)
                    except Exception:
                        pass
            # 兜底：句柄丢失时直接 pkill
            if not stopped:
                subprocess.run(['pkill', '-f', '[l]d06_hector.launch'],
                               capture_output=True, timeout=5)
                subprocess.run(['pkill', '-f', '[h]ector_mapping'],
                               capture_output=True, timeout=5)
                time.sleep(1)
            self.proc = None
            alive = self.hector_alive()
            self.set_state('hector 已停止' if not alive else '停止失败',
                           '' if not alive else '仍有 hector_mapping 进程',
                           busy=False)
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
            if not self.hector_alive():
                self.set_state('保存失败', 'hector 未在运行，/map 不存在', busy=False)
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
        rate = rospy.Rate(0.5)   # 2s 心跳，控制台据此判断节点在线
        while not rospy.is_shutdown():
            self.publish_status()
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('slam_control')
    SlamControl().spin()
