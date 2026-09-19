#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# wall_follow.py — 自动绕墙巡航建图 (路线A, 2026-09-19)
#
# 原理: 贴右侧墙慢速巡航, 用麦轮"横移"控制墙距(不旋转 → 避开麦轮打滑重灾区),
#       绕房间一圈自然形成闭合路线 → slam_toolbox 回环校正.
#
# 状态机:
#   IDLE    待命 (等 /wall_follow/cmd "start")
#   FOLLOW  沿墙巡航: 前进 vy=巡航速; 横移 vx=P控墙距; 前距随近减速
#   TURN    前方受阻: 原地慢转向"离墙侧", 直到前方清空
#   LOST    墙丢了(门口/凹角): 先加横移追墙, 超时则向墙侧慢转找回
#   RECOVER 急停恢复: 停→慢退→转离墙→回 FOLLOW (stuck 计数, 超限→ABORT)
#   DONE    回到起点附近且路程足够 → 停车成功
#   ABORT   卡死次数过多 / 超时 → 停车待命
#
# 安全:
#   - 前向 45° 锥内任何有效点 < estop_dist → RECOVER
#   - 全程爬行速度(默认 30mm/s ≈ 3cm/s), 失控也可手抓
#   - 节点退出(on_shutdown)先连发 3 次停车命令
#   - ⚠️ 底盘无看门狗: 最后一条命令会一直执行, 头几次运行必须在旁看护!
#
# 话题:
#   订阅: /scan (LaserScan), /odom (Odometry, 位姿兜底),
#         /wall_follow/cmd (String: start|stop)
#   发布: /chassis_control/set_translation (平移), /chassis_control/set_velocity (仅转向),
#         /wall_follow/status (String, JSON, latched, 2Hz)
#
# 位姿来源: 优先 TF map→base_link (slam_toolbox 校正后, 回环判定准);
#           拿不到退化为 /odom (有打滑漂移, 仅兜底).
#
# 运行(容器内):
#   source /opt/ros/noetic/setup.bash
#   source /home/ubuntu/armpi_pro/devel/setup.bash
#   export ROS_MASTER_URI=http://raspberrypi:11311 ROS_HOSTNAME=raspberrypi
#   python3 /home/ubuntu/wall_follow.py
# 另开终端发令: rostopic pub -1 /wall_follow/cmd std_msgs/String "data: start"

import json
import math
import threading
import time

import rospy
import tf2_ros
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion

from chassis_control.msg import SetTranslation, SetVelocity


def norm_ang(a):
    """规整到 (-pi, pi]"""
    while a > math.pi:
        a -= 2 * math.pi
    while a <= -math.pi:
        a += 2 * math.pi
    return a


# ---------------- 参数 ----------------
class P(object):
    cruise_speed = 30.0        # 巡航前进 mm/s
    backup_speed = 25.0        # RECOVER 倒退 mm/s
    strafe_max = 20.0          # 横移上限 mm/s
    turn_rate = 0.25           # 转向角速度 rad/s (低速, 降打滑)
    target_wall_dist = 0.55    # 目标墙距 m
    wall_kp = 60.0             # 墙距 P 增益 (mm/s per m)
    front_slow_dist = 0.80     # 前距小于此开始减速 m
    front_turn_dist = 0.45     # 前距小于此进入 TURN m
    front_clear_dist = 0.65    # TURN 中前距大于此回 FOLLOW m
    wall_lost_dist = 1.05      # 侧距大于此视为墙丢失 m
    wall_lost_secs = 6.0       # 墙丢失持续 -> 向墙侧转
    estop_dist = 0.16          # 急停距离 m
    min_valid_range = 0.10     # 滤车体自射 (实测 0.06~0.08m)
    max_valid_range = 8.0
    # ⭐ 车体自射方位屏蔽(扫描坐标系角度, 09-19 多帧实测 72 帧):
    #    310°~330° 线缆/支架 0.06m (371 hits) —— 主遮挡, 落急停锥内会触发假急停
    #    240°~270° 右侧结构 0.081m (16 hits) —— 次遮挡, 污染右侧墙距测量
    #    两侧各留 ±4° 余量。此区间内读数一律丢弃。
    self_mask = [(306.0, 334.0), (236.0, 274.0)]
    loop_close_dist = 0.5      # 距起点小于此(且路程足够) → DONE
    min_path_len = 5.0         # DONE 所需最小路程 m
    max_seconds = 300          # 总超时
    stuck_secs = 6.0           # FOLLOW 中位姿不动判卡死
    stuck_abort = 4            # 卡死次数上限
    cmd_rate = 10.0            # 指令发布 Hz
    wall_side = 1              # +1=沿右墙, -1=沿左墙
    laser_yaw = None           # 强制指定 base_laser 在 base_link 中的朝向(rad); None=从 TF 读
    enable_motion = True       # False=dry-run 只打日志不动车


class WallFollower(object):
    def __init__(self):
        P.cruise_speed = rospy.get_param('~cruise_speed', P.cruise_speed)
        P.target_wall_dist = rospy.get_param('~target_wall_dist', P.target_wall_dist)
        P.max_seconds = rospy.get_param('~max_seconds', P.max_seconds)
        P.wall_side = int(rospy.get_param('~wall_side', P.wall_side))
        P.enable_motion = bool(rospy.get_param('~enable_motion', P.enable_motion))
        P.laser_yaw = rospy.get_param('~laser_yaw', None)

        self.state = 'IDLE'
        self.scan = None
        self.scan_stamp = 0.0
        self.odom_pose = None       # (x, y, yaw)
        self.lock = threading.Lock()

        self.pub_trans = rospy.Publisher('/chassis_control/set_translation',
                                         SetTranslation, queue_size=1)
        self.pub_vel = rospy.Publisher('/chassis_control/set_velocity',
                                       SetVelocity, queue_size=1)
        self.pub_status = rospy.Publisher('/wall_follow/status', String,
                                          queue_size=1, latch=True)
        rospy.Subscriber('/scan', LaserScan, self.on_scan, queue_size=1)
        rospy.Subscriber('/odom', Odometry, self.on_odom, queue_size=1)
        rospy.Subscriber('/wall_follow/cmd', String, self.on_cmd)

        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)

        # 运行期数据
        self.t0 = 0.0
        self.start_pose = None
        self.path_len = 0.0
        self.last_pose = None
        self.last_move_t = 0.0
        self.last_pose_check = None
        self.stuck_count = 0
        self.lost_since = None
        self.pose_src = 'none'
        self.msg = 'boot'
        self.last_cmd = (0.0, 0.0, 0.0)   # (vx, vy, ang) 记录用于日志

        rospy.on_shutdown(self.on_shutdown)

    # ---------------- 订阅回调 ----------------
    def on_scan(self, msg):
        with self.lock:
            self.scan = msg
            self.scan_stamp = time.time()

    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        self.odom_pose = (p.x, p.y, yaw)

    def on_cmd(self, msg):
        cmd = (msg.data or '').strip().lower()
        rospy.loginfo('wall_follow cmd: %s', cmd)
        if cmd == 'start' and self.state in ('IDLE', 'DONE', 'ABORT'):
            self.start_run()
        elif cmd == 'stop':
            self.stop_motors()
            self.state = 'IDLE'
            self.msg = '手动停止'

    # ---------------- 工具 ----------------
    def laser_yaw_offset(self):
        """base_laser 在 base_link 中的朝向(rad). 静态, 缓存."""
        if P.laser_yaw is not None:
            return P.laser_yaw
        if getattr(self, '_lyaw', None) is not None:
            return self._lyaw
        try:
            t = self.tf_buf.lookup_transform('base_link', 'base_laser',
                                             rospy.Time(0), rospy.Duration(1.0))
            q = t.transform.rotation
            self._lyaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        except Exception:
            self._lyaw = 0.0   # 读不到先按 0, 校准模式可人工给 P.laser_yaw
        return self._lyaw

    def sectors(self):
        """从最新一帧 scan 计算关键扇区的最近距离(m). 无数据返回 None."""
        with self.lock:
            msg = self.scan
            age = time.time() - self.scan_stamp
        if msg is None or age > 1.0:
            return None
        lyaw = self.laser_yaw_offset()
        side = P.wall_side  # +1 右墙(负方位角侧)
        # 扇区(车体方位角, 度) —— 已避开 self_mask 遮挡带:
        #   右墙侧墙距用 -87..-66°(=扫描 273..294°, 实测干净窗口);
        #   front_side 拆两段, 绕过 306..334° 遮挡带。
        if side > 0:
            secs = {
                'front':      [(-20, 20)],
                'front_side': [(-64, -53), (-27, -21)],
                'side':       [(-87, -66)],
                'estop':      [(-45, 45)],
            }
        else:
            secs = {
                'front':      [(-20, 20)],
                'front_side': [(53, 64), (21, 27)],
                'side':       [(66, 87)],
                'estop':      [(-45, 45)],
            }
        out = {k: P.max_valid_range for k in secs}
        out_ang = {k: 0.0 for k in secs}
        counts = {k: 0 for k in secs}
        ang = msg.angle_min
        inc = msg.angle_increment
        masks = P.self_mask
        for i, r in enumerate(msg.ranges):
            a = ang + i * inc
            if math.isnan(r) or r < P.min_valid_range or r > P.max_valid_range:
                continue
            deg = math.degrees(a) % 360.0
            skip = False
            for lo, hi in masks:
                if lo <= deg <= hi:
                    skip = True
                    break
            if skip:
                continue
            b = math.degrees(norm_ang(a + lyaw))
            for k, ranges in secs.items():
                for lo, hi in ranges:
                    if lo <= b <= hi:
                        counts[k] += 1
                        if r < out[k]:
                            out[k] = r
                            out_ang[k] = b
                        break
        out['counts'] = counts
        out['bearings'] = out_ang
        return out

    def pose(self):
        """当前位姿 (x, y, yaw). 优先 map TF, 退化 /odom."""
        try:
            t = self.tf_buf.lookup_transform('map', 'base_link',
                                             rospy.Time(0), rospy.Duration(0.2))
            p = t.transform.translation
            q = t.transform.rotation
            yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
            self.pose_src = 'map'
            return (p.x, p.y, yaw)
        except Exception:
            pass
        if self.odom_pose is not None:
            self.pose_src = 'odom'
            return self.odom_pose
        self.pose_src = 'none'
        return None

    # ---------------- 运动输出 ----------------
    def send_translation(self, vx, vy):
        """平移: vx 右+, vy 前+, mm/s"""
        self.last_cmd = (vx, vy, 0.0)
        if not P.enable_motion:
            return
        m = SetTranslation()
        m.velocity_x = float(vx)
        m.velocity_y = float(vy)
        self.pub_trans.publish(m)

    def send_rotate(self, ang):
        """原地转: ang>0 逆时针(左). 用 set_velocity 通道, 与平移不同时发."""
        self.last_cmd = (0.0, 0.0, ang)
        if not P.enable_motion:
            return
        m = SetVelocity()
        m.velocity = 0.0
        m.direction = 0.0
        m.angular = float(ang)
        self.pub_vel.publish(m)

    def stop_motors(self):
        self.last_cmd = (0.0, 0.0, 0.0)
        if not P.enable_motion:
            return
        m1 = SetTranslation()
        m1.velocity_x = 0.0
        m1.velocity_y = 0.0
        m2 = SetVelocity()
        m2.velocity = 0.0
        m2.direction = 0.0
        m2.angular = 0.0
        for _ in range(3):
            self.pub_trans.publish(m1)
            self.pub_vel.publish(m2)
            time.sleep(0.05)

    def on_shutdown(self):
        try:
            self.stop_motors()
        except Exception:
            pass

    # ---------------- 运行控制 ----------------
    def start_run(self):
        pose = self.pose()
        self.t0 = time.time()
        self.start_pose = pose
        self.last_pose = pose
        self.last_pose_check = (pose, time.time()) if pose else None
        self.path_len = 0.0
        self.stuck_count = 0
        self.lost_since = None
        self.state = 'FOLLOW'
        self.msg = '已出发(位姿源:%s)' % self.pose_src
        rospy.loginfo('wall_follow START pose=%s src=%s',
                      str(pose), self.pose_src)

    def finish(self, state, msg):
        self.stop_motors()
        self.state = state
        self.msg = msg
        rospy.loginfo('wall_follow %s: %s path=%.1fm', state, msg, self.path_len)

    # ---------------- 主循环 ----------------
    def spin(self):
        # 先等雷达数据就绪
        t_wait = time.time()
        while not rospy.is_shutdown():
            if self.sectors() is not None:
                break
            if time.time() - t_wait > 30:
                rospy.logwarn('等待 /scan 超时, 继续等待...')
                t_wait = time.time()
            time.sleep(0.2)

        rate = rospy.Rate(P.cmd_rate)
        status_div = 0
        while not rospy.is_shutdown():
            self.step()
            status_div += 1
            if status_div >= int(P.cmd_rate / 2):   # 2Hz 状态
                status_div = 0
                self.publish_status()
            rate.sleep()

    def step(self):
        if self.state in ('IDLE', 'DONE', 'ABORT'):
            return

        # 超时
        if time.time() - self.t0 > P.max_seconds:
            self.finish('ABORT', '超时 %.0fs' % P.max_seconds)
            return

        sec = self.sectors()
        if sec is None:
            # 雷达断流 >1s: 立即停车等恢复
            self.stop_motors()
            self.msg = '雷达断流, 停车等待'
            return

        front, side = sec['front'], sec['side']
        fside = sec['front_side']

        # 急停: 前向锥内有极近点 —— ⭐ 需连续 3 帧触发(防单帧毛刺/路过物体)
        if sec['estop'] < P.estop_dist:
            self.estop_hits = getattr(self, 'estop_hits', 0) + 1
        else:
            self.estop_hits = 0
        if self.estop_hits >= 3:
            self.estop_hits = 0
            self.stuck_count += 1
            if self.stuck_count >= P.stuck_abort:
                self.finish('ABORT', '急停次数超限(%d)' % self.stuck_count)
                return
            self.state = 'RECOVER'
            self.recover_t0 = time.time()
            self.msg = '急停! %.2fm @%.0f°' % (sec['estop'], sec['bearings']['estop'])
            return

        # 位姿/路程/回环/卡死
        pose = self.pose()
        if pose is not None and self.last_pose is not None:
            dx = pose[0] - self.last_pose[0]
            dy = pose[1] - self.last_pose[1]
            d = math.hypot(dx, dy)
            if d < 1.0:    # 防位姿源切换跳变
                self.path_len += d
            self.last_pose = pose
        if pose is not None:
            # 回环判定
            if (self.start_pose is not None and
                    self.path_len > P.min_path_len):
                dd = math.hypot(pose[0] - self.start_pose[0],
                                pose[1] - self.start_pose[1])
                if dd < P.loop_close_dist:
                    self.finish('DONE',
                                '回到起点附近(%.2fm), 路程 %.1fm' % (dd, self.path_len))
                    return
            # 卡死判定(仅 FOLLOW)
            if self.state == 'FOLLOW':
                if self.last_pose_check is None:
                    self.last_pose_check = (pose, time.time())
                pc, pt = self.last_pose_check
                if time.time() - pt > P.stuck_secs:
                    moved = math.hypot(pose[0] - pc[0], pose[1] - pc[1])
                    self.last_pose_check = (pose, time.time())
                    if moved < 0.02:
                        self.stuck_count += 1
                        if self.stuck_count >= P.stuck_abort:
                            self.finish('ABORT', '卡死次数超限(%d)' % self.stuck_count)
                            return
                        self.state = 'RECOVER'
                        self.recover_t0 = time.time()
                        self.msg = '疑似卡死(%d)' % self.stuck_count
                        return

        # ---------------- 状态机 ----------------
        if self.state == 'FOLLOW':
            if front < P.front_turn_dist or fside < P.front_turn_dist * 0.8:
                # 前方/前侧受阻 → 向离墙侧转
                self.state = 'TURN'
                self.msg = '遇障碍转向(前 %.2f 侧 %.2f)' % (front, fside)
                return
            if side > P.wall_lost_dist:
                if self.lost_since is None:
                    self.lost_since = time.time()
                elif time.time() - self.lost_since > P.wall_lost_secs:
                    self.state = 'LOST'
                    self.msg = '墙丢失, 向墙侧找回'
                    return
            else:
                self.lost_since = None
            # 前进: 前距越近越慢
            vy = P.cruise_speed
            if front < P.front_slow_dist:
                span = P.front_slow_dist - P.front_turn_dist
                vy = max(12.0, P.cruise_speed * (front - P.front_turn_dist) / span)
            # 横移 P 控墙距: 右墙(side>0)太远→向右挪(vx+)
            err = side - P.target_wall_dist
            vx = P.wall_kp * err * P.wall_side
            vx = max(-P.strafe_max, min(P.strafe_max, vx))
            self.send_translation(vx, vy)
            self.msg = '巡航 前%.2f 侧%.2f vx%.0f vy%.0f' % (front, side, vx, vy)

        elif self.state == 'TURN':
            if front > P.front_clear_dist and fside > P.front_clear_dist * 0.7:
                self.state = 'FOLLOW'
                self.msg = '转向完成, 恢复巡航'
                return
            # 右墙→左转(+), 左墙→右转(-)
            self.send_rotate(P.turn_rate * P.wall_side)
            self.msg = '转向中 前%.2f' % front

        elif self.state == 'LOST':
            # 向墙侧慢转, 找回墙
            if side < P.target_wall_dist + 0.3:
                self.state = 'FOLLOW'
                self.lost_since = None
                self.msg = '找回墙, 恢复巡航'
                return
            self.send_rotate(-P.turn_rate * P.wall_side)
            self.msg = '找墙中 侧%.2f' % side

        elif self.state == 'RECOVER':
            dt = time.time() - self.recover_t0
            if dt < 0.6:
                self.stop_motors()
                self.msg = '急停缓冲'
            elif dt < 2.2:
                self.send_translation(0.0, -P.backup_speed)
                self.msg = '慢退中'
            elif dt < 3.4:
                self.send_rotate(P.turn_rate * P.wall_side)
                self.msg = '恢复转向'
            else:
                self.state = 'FOLLOW'
                self.last_pose_check = None
                self.msg = '恢复巡航'

    def publish_status(self):
        pose = self.pose()
        d_start = None
        if pose is not None and self.start_pose is not None:
            d_start = math.hypot(pose[0] - self.start_pose[0],
                                 pose[1] - self.start_pose[1])
        s = {
            'state': self.state,
            'msg': self.msg,
            'path_len': round(self.path_len, 2),
            'dist_from_start': None if d_start is None else round(d_start, 2),
            'pose_src': self.pose_src,
            'stuck': self.stuck_count,
            'elapsed': 0 if self.state == 'IDLE' else round(time.time() - self.t0, 1),
            'cmd': [round(c, 1) for c in self.last_cmd],
            'motion': P.enable_motion,
            'ts': time.time(),
        }
        self.pub_status.publish(String(json.dumps(s, ensure_ascii=False)))


if __name__ == '__main__':
    rospy.init_node('wall_follow')
    WallFollower().spin()
