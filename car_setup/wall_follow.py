#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# wall_follow.py — 自动绕墙巡航建图 (路线A, 2026-09-19)
#
# 原理: 贴右侧墙慢速巡航, 用麦轮"横移"控制墙距(不旋转 → 避开麦轮打滑重灾区),
#       绕房间一圈自然形成闭合路线 → slam_toolbox 回环校正.
#
# 状态机:
#   IDLE    待命 (等 /wall_follow/cmd "start")
#   EXPLORE 根据地图选择当前可达前沿点; 到达后重选
#   FOLLOW  无可达前沿时的保守沿墙兜底巡航
#   TURN    行驶前方受阻: 原地慢转向"离墙侧", 直到行驶前方清空
#   LOST    墙丢了(门口/凹角): 先加横移追墙, 超时则向墙侧慢转找回
#   WAIT_OBSTACLE 动态避障: 停车等待障碍物离开; 持续存在才选择安全侧转弯
#   RECOVER 卡死恢复: 停→确认后方安全再慢退→转离墙→回 FOLLOW
#   DONE    回到起点附近且路程足够 → 停车成功
#   ABORT   卡死次数过多 / 超时 → 停车待命
#
# 安全:
#   - 行驶方向 50° 锥内任何有效点 < estop_dist → 立即停车等待
#   - 扫描使用空间中值 + 时间 EMA；安全急停仍使用原始点，不会被滤波延迟
#   - 动态障碍先等待 clear_secs；持续存在才按左右空旷程度选择转向
#   - 全程爬行速度(默认 25mm/s ≈ 2.5cm/s), 失控也可手抓
#   - 节点退出(on_shutdown)先连发 3 次停车命令
#   - chassis_control_node 监视本节点 /wall_follow/status 心跳;
#     心跳中断约 1.5s 会切断电机输出
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
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion

from chassis_control.msg import SetTranslation, SetVelocity
from frontier_planner import select_frontier_goal


def norm_ang(a):
    """规整到 (-pi, pi]"""
    while a > math.pi:
        a -= 2 * math.pi
    while a <= -math.pi:
        a += 2 * math.pi
    return a


def norm_deg(a):
    """Wrap degrees to [-180, 180) for rear-facing scan sectors."""
    return (a + 180.0) % 360.0 - 180.0


# ---------------- 参数 ----------------
class P(object):
    cruise_speed = 25.0        # 巡航前进 mm/s; 自动建图优先安全
    backup_speed = 15.0        # RECOVER 倒退 mm/s
    strafe_max = 6.0           # 横移上限 mm/s; 建图优先转向, 少用麦轮横移
    turn_rate = 0.20           # 转向角速度 rad/s (低速, 降打滑)
    target_wall_dist = 0.55    # 目标墙距 m
    wall_kp = 30.0             # 墙距 P 增益 (mm/s per m)
    prefer_turning = True      # 墙距偏差较大时原地转向, 不用大幅横移拉回
    wall_turn_threshold = 0.15 # 墙距误差超过此值就优先转向 m
    wall_turn_rate = 0.10      # 墙距修正角速度 rad/s
    front_slow_dist = 0.70     # 前距小于此开始减速 m
    front_turn_dist = 0.40     # 前距小于此进入 TURN m
    front_clear_dist = 0.55    # TURN 中前距大于此回 FOLLOW m
    wall_lost_dist = 1.05      # 侧距大于此视为墙丢失 m
    wall_lost_secs = 6.0       # 墙丢失持续 -> 向墙侧转
    estop_dist = 0.30          # 硬停车距离 m; 10cm 对车体来说过近
    obstacle_pause_dist = 0.40 # 先停车观察的距离; 低速下减少无谓等待
    obstacle_clear_dist = 0.50 # 连续超过此距离才允许恢复
    obstacle_wait_secs = 2.5   # 障碍持续存在超过此时间, 视为静态障碍并尝试绕行
    obstacle_clear_secs = 1.0  # 连续安全时间, 防止人短暂移开就立即起步
    side_pause_dist = 0.25     # 麦轮横移方向的动态障碍观察距离
    side_clear_dist = 0.35     # 侧向障碍离开后的恢复距离
    side_estop_dist = 0.22     # 侧向硬停车距离
    turn_clear_dist = 0.40     # 原地转向一侧至少需要的空闲距离
    turn_stop_dist = 0.32      # 转向路径停车距离; 不随准入距离缩小
    rear_clear_dist = 0.45     # 倒退恢复所需的后方空闲距离
    scan_timeout = 0.50        # LD06 约 10Hz; 超过半秒无新扫描即停车
    min_valid_range = 0.10     # 滤车体自射 (实测 0.06~0.08m)
    max_valid_range = 8.0
    # ⭐ 车体自射方位屏蔽(扫描坐标系角度, 09-19 多帧实测 72 帧):
    #    310°~330° 线缆/支架 0.06m (371 hits) —— 主遮挡
    #    240°~270° 右侧结构 0.081m (16 hits) —— 次遮挡
    # ⭐⭐ 09-19 二修: 这些自射读数全部 <0.12m (贴着雷达的结构),
    #    所以屏蔽带内只丢 <mask_max_range 的读数、保留远处读数——
    #    否则右前斜方(沿右墙最先撞家具腿的方向)会被整个戳瞎!
    self_mask = [(306.0, 334.0), (236.0, 274.0)]
    mask_max_range = 0.12      # 屏蔽带内只丢 <此距离 的读数
    loop_close_dist = 0.5      # 距起点小于此(且路程足够) → DONE
    min_path_len = 5.0         # DONE 所需最小路程 m
    max_seconds = 300          # 总超时
    stuck_secs = 6.0           # FOLLOW 中位姿不动判卡死
    stuck_abort = 4            # 连续卡死次数上限
    stuck_reset_dist = 0.05    # 有效移动达到此距离后清零卡死计数(m)
    cmd_rate = 10.0            # 指令发布 Hz
    wall_side = 1              # +1=沿右墙, -1=沿左墙
    # +1=车头向前巡航；-1=车尾向后巡航。扇区、安全和墙侧均以行驶方向为准。
    travel_direction = 1
    # 区域驻留：即使没有触发硬障碍，也不能在同一小片区域反复转圈。
    area_dwell_secs = 30.0       # 区域内最长驻留时间
    area_dwell_radius = 0.60     # 以起始位姿为中心的“同一片区域”半径(m)
    area_escape_turn_secs = 2.0  # 需要换向时先原地转向的时间
    area_escape_min_clear = 0.55 # 直行方向达到此距离才认为是开阔
    area_escape_speed = 25.0     # 区域脱离速度(mm/s)
    area_escape_progress = 0.45 # 必须由位姿确认实际离开此距离(m)
    area_escape_timeout = 35.0  # 未达到位移则停车，不把超时当成成功(s)
    frontier_enabled = True
    frontier_clearance_radius = 0.25
    frontier_max_range = 3.0
    frontier_min_range = 0.45
    frontier_goal_tolerance = 0.35
    frontier_turn_tolerance = 0.18
    frontier_map_timeout = 4.0
    frontier_blacklist_secs = 30.0
    laser_yaw = None           # 强制指定 base_laser 在 base_link 中的朝向(rad); None=从 TF 读
    filter_enabled = True      # 空间中值 + 时间 EMA
    spatial_filter_radius = 1  # 每个点取左右各 N 个点的中值
    ema_alpha = 0.35           # 越大越灵敏, 越小越平滑
    sector_percentile = 10.0   # 扇区控制距离使用低百分位, 抑制单点毛刺
    enable_motion = True       # False=dry-run 只打日志不动车


class WallFollower(object):
    def __init__(self):
        P.cruise_speed = rospy.get_param('~cruise_speed', P.cruise_speed)
        P.backup_speed = rospy.get_param('~backup_speed', P.backup_speed)
        P.strafe_max = rospy.get_param('~strafe_max', P.strafe_max)
        P.turn_rate = rospy.get_param('~turn_rate', P.turn_rate)
        P.target_wall_dist = rospy.get_param('~target_wall_dist', P.target_wall_dist)
        P.wall_kp = rospy.get_param('~wall_kp', P.wall_kp)
        P.prefer_turning = bool(rospy.get_param('~prefer_turning', P.prefer_turning))
        P.wall_turn_threshold = rospy.get_param(
            '~wall_turn_threshold', P.wall_turn_threshold)
        P.wall_turn_rate = rospy.get_param('~wall_turn_rate', P.wall_turn_rate)
        P.front_slow_dist = rospy.get_param('~front_slow_dist', P.front_slow_dist)
        P.front_turn_dist = rospy.get_param('~front_turn_dist', P.front_turn_dist)
        P.front_clear_dist = rospy.get_param('~front_clear_dist', P.front_clear_dist)
        P.max_seconds = rospy.get_param('~max_seconds', P.max_seconds)
        P.wall_side = int(rospy.get_param('~wall_side', P.wall_side))
        P.travel_direction = int(rospy.get_param(
            '~travel_direction', P.travel_direction))
        P.area_dwell_secs = float(rospy.get_param(
            '~area_dwell_secs', P.area_dwell_secs))
        P.area_dwell_radius = float(rospy.get_param(
            '~area_dwell_radius', P.area_dwell_radius))
        P.area_escape_turn_secs = float(rospy.get_param(
            '~area_escape_turn_secs', P.area_escape_turn_secs))
        P.area_escape_min_clear = float(rospy.get_param(
            '~area_escape_min_clear', P.area_escape_min_clear))
        P.area_escape_speed = float(rospy.get_param(
            '~area_escape_speed', P.area_escape_speed))
        P.area_escape_progress = float(rospy.get_param(
            '~area_escape_progress', P.area_escape_progress))
        P.area_escape_timeout = float(rospy.get_param(
            '~area_escape_timeout', P.area_escape_timeout))
        P.frontier_enabled = bool(rospy.get_param(
            '~frontier_enabled', P.frontier_enabled))
        P.frontier_clearance_radius = float(rospy.get_param(
            '~frontier_clearance_radius', P.frontier_clearance_radius))
        P.frontier_max_range = float(rospy.get_param(
            '~frontier_max_range', P.frontier_max_range))
        P.frontier_min_range = float(rospy.get_param(
            '~frontier_min_range', P.frontier_min_range))
        P.frontier_goal_tolerance = float(rospy.get_param(
            '~frontier_goal_tolerance', P.frontier_goal_tolerance))
        P.frontier_turn_tolerance = float(rospy.get_param(
            '~frontier_turn_tolerance', P.frontier_turn_tolerance))
        P.frontier_map_timeout = float(rospy.get_param(
            '~frontier_map_timeout', P.frontier_map_timeout))
        P.frontier_blacklist_secs = float(rospy.get_param(
            '~frontier_blacklist_secs', P.frontier_blacklist_secs))
        P.area_escape_progress = max(0.30, min(2.0, P.area_escape_progress))
        P.area_escape_timeout = max(5.0, min(120.0, P.area_escape_timeout))
        P.frontier_clearance_radius = max(0.15, min(1.0, P.frontier_clearance_radius))
        P.frontier_max_range = max(0.8, min(6.0, P.frontier_max_range))
        P.frontier_min_range = max(0.35, min(
            P.frontier_max_range - 0.15, P.frontier_min_range))
        P.frontier_goal_tolerance = max(0.10, min(0.80, P.frontier_goal_tolerance))
        P.frontier_turn_tolerance = max(0.05, min(0.60, P.frontier_turn_tolerance))
        P.frontier_map_timeout = max(0.5, min(10.0, P.frontier_map_timeout))
        P.frontier_blacklist_secs = max(5.0, min(120.0, P.frontier_blacklist_secs))
        P.enable_motion = bool(rospy.get_param('~enable_motion', P.enable_motion))
        P.laser_yaw = rospy.get_param('~laser_yaw', None)
        P.obstacle_pause_dist = rospy.get_param('~obstacle_pause_dist', P.obstacle_pause_dist)
        P.obstacle_clear_dist = rospy.get_param('~obstacle_clear_dist', P.obstacle_clear_dist)
        P.obstacle_wait_secs = rospy.get_param('~obstacle_wait_secs', P.obstacle_wait_secs)
        P.obstacle_clear_secs = rospy.get_param('~obstacle_clear_secs', P.obstacle_clear_secs)
        P.side_pause_dist = rospy.get_param('~side_pause_dist', P.side_pause_dist)
        P.side_clear_dist = rospy.get_param('~side_clear_dist', P.side_clear_dist)
        P.side_estop_dist = rospy.get_param('~side_estop_dist', P.side_estop_dist)
        P.turn_clear_dist = rospy.get_param('~turn_clear_dist', P.turn_clear_dist)
        P.turn_stop_dist = rospy.get_param('~turn_stop_dist', P.turn_stop_dist)
        P.rear_clear_dist = rospy.get_param('~rear_clear_dist', P.rear_clear_dist)
        P.scan_timeout = rospy.get_param('~scan_timeout', P.scan_timeout)
        P.stuck_reset_dist = float(rospy.get_param(
            '~stuck_reset_dist', P.stuck_reset_dist))
        P.filter_enabled = bool(rospy.get_param('~filter_enabled', P.filter_enabled))
        P.spatial_filter_radius = int(rospy.get_param(
            '~spatial_filter_radius', P.spatial_filter_radius))
        P.ema_alpha = float(rospy.get_param('~ema_alpha', P.ema_alpha))
        P.sector_percentile = float(rospy.get_param(
            '~sector_percentile', P.sector_percentile))
        P.spatial_filter_radius = max(0, min(4, P.spatial_filter_radius))
        P.ema_alpha = max(0.05, min(1.0, P.ema_alpha))
        P.sector_percentile = max(0.0, min(40.0, P.sector_percentile))
        P.travel_direction = -1 if P.travel_direction < 0 else 1

        self.state = 'IDLE'
        self.scan = None
        self.raw_ranges = None
        self.filtered_ranges = None
        self.scan_stamp = 0.0
        self.odom_pose = None       # (x, y, yaw)
        self.lock = threading.Lock()
        self.map_grid = None
        self.map_stamp = 0.0

        self.pub_trans = rospy.Publisher('/chassis_control/set_translation',
                                         SetTranslation, queue_size=1)
        self.pub_vel = rospy.Publisher('/chassis_control/set_velocity',
                                       SetVelocity, queue_size=1)
        self.pub_status = rospy.Publisher('/wall_follow/status', String,
                                          queue_size=1, latch=True)
        rospy.Subscriber('/scan', LaserScan, self.on_scan, queue_size=1)
        rospy.Subscriber('/odom', Odometry, self.on_odom, queue_size=1)
        rospy.Subscriber('/map', OccupancyGrid, self.on_map, queue_size=1)
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
        self.estop_hits = 0
        self.obstacle_hits = 0
        self.obstacle_since = None
        self.obstacle_clear_since = None
        self.obstacle_kind = 'front'
        self.turn_sign = P.wall_side
        self.area_anchor = None
        self.area_since = 0.0
        self.escape_sign = 0
        self.escape_t0 = 0.0
        self.escape_move_pose = None
        self.escape_move_t0 = 0.0
        self.escape_pose_src = None
        self.resume_state = 'FOLLOW'
        self.frontier_goal = None
        self.frontier_blacklist = []
        self.goal_reached_count = 0
        self.filter_stats = {'valid': 0, 'total': 0}

        rospy.on_shutdown(self.on_shutdown)

    # ---------------- 订阅回调 ----------------
    def on_scan(self, msg):
        raw = []
        for i, value in enumerate(msg.ranges):
            r = float(value)
            deg = math.degrees(msg.angle_min + i * msg.angle_increment) % 360.0
            if not math.isfinite(r) or r < P.min_valid_range or r > P.max_valid_range:
                raw.append(float('inf'))
                continue
            masked = False
            for lo, hi in P.self_mask:
                if lo <= deg <= hi and r < P.mask_max_range:
                    masked = True
                    break
            raw.append(float('inf') if masked else r)

        filtered = self.filter_ranges(raw)
        with self.lock:
            self.scan = msg
            self.raw_ranges = raw
            self.filtered_ranges = filtered
            self.scan_stamp = time.time()

    @staticmethod
    def _median(values):
        values = sorted(values)
        if not values:
            return float('inf')
        mid = len(values) // 2
        if len(values) % 2:
            return values[mid]
        return 0.5 * (values[mid - 1] + values[mid])

    def filter_ranges(self, raw):
        """Filter scan for steering while preserving raw data for emergency stop.

        A spatial median removes isolated LD06 spikes. The temporal EMA is
        asymmetric: a newly appearing close object is accepted immediately,
        while a disappearing object is released gradually. This makes the
        steering stable without delaying a safety stop.
        """
        if not raw:
            return []
        if not P.filter_enabled:
            return list(raw)
        radius = P.spatial_filter_radius
        spatial = []
        n = len(raw)
        for i, value in enumerate(raw):
            if not math.isfinite(value):
                spatial.append(float('inf'))
                continue
            if radius <= 0:
                spatial.append(value)
                continue
            nearby = []
            for off in range(-radius, radius + 1):
                j = i + off
                if 0 <= j < n and math.isfinite(raw[j]):
                    nearby.append(raw[j])
            spatial.append(self._median(nearby) if nearby else float('inf'))

        with self.lock:
            previous = self.filtered_ranges
        if not previous or len(previous) != len(spatial):
            return list(spatial)
        alpha = P.ema_alpha
        out = []
        for current, old in zip(spatial, previous):
            if not math.isfinite(current):
                out.append(float('inf'))
            elif not math.isfinite(old) or current < old - 0.08:
                # Close obstacles must never wait for the EMA to catch up.
                out.append(current)
            else:
                out.append(alpha * current + (1.0 - alpha) * old)
        return out

    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        self.odom_pose = (p.x, p.y, yaw)

    def on_map(self, msg):
        with self.lock:
            self.map_grid = msg
            self.map_stamp = time.time()

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

    @staticmethod
    def sector_definitions(travel_direction, wall_side):
        """Define canonical travel-facing sectors in base_link degrees.

        The rest of the controller always consumes ``front``, ``left`` and
        ``right`` relative to the direction of travel.  Reversing therefore
        rotates all safety sectors by 180 degrees instead of only negating the
        motor command (which would leave the rear path unprotected).
        """
        heading = 0.0 if travel_direction >= 0 else 180.0

        # Ranges that cross +/-180 are split because the scan matcher below
        # compares simple inclusive degree intervals.
        def wrapped(center, low, high):
            lo, hi = norm_deg(center + low), norm_deg(center + high)
            return [(lo, hi)] if lo <= hi else [(-180.0, hi), (lo, 180.0)]

        side_center = heading - 90.0 * wall_side
        return {
            'front': wrapped(heading, -20, 20),
            'front_side': wrapped(heading, -65, -25) if wall_side > 0 else wrapped(heading, 25, 65),
            'side': wrapped(side_center, -20, 20),
            'front_left': wrapped(heading, 20, 70),
            'front_right': wrapped(heading, -70, -20),
            'left': wrapped(heading, 70, 110),
            'right': wrapped(heading, -110, -70),
            'rear': wrapped(heading + 180, -20, 20),
            'estop': wrapped(heading, -50, 50),
            'front_wide': wrapped(heading, -70, 70),
        }

    def sectors(self):
        """Return robust sector distances and raw safety minima.

        Steering uses the filtered low percentile.  The hard-stop fields use
        raw ranges, so a filter cannot make a close obstacle look farther away.
        """
        with self.lock:
            msg = self.scan
            raw = self.raw_ranges
            filtered = self.filtered_ranges
            age = time.time() - self.scan_stamp
        if msg is None or not raw or not filtered or age > P.scan_timeout:
            return None
        lyaw = self.laser_yaw_offset()
        secs = self.sector_definitions(P.travel_direction, P.wall_side)
        values = {k: [] for k in secs}
        raw_values = {k: [] for k in secs}
        out_ang = {k: 0.0 for k in secs}
        nearest = {k: (P.max_valid_range, 0.0) for k in secs}
        counts = {k: 0 for k in secs}
        ang = msg.angle_min
        inc = msg.angle_increment
        for i, r in enumerate(filtered):
            a = ang + i * inc
            b = norm_deg(math.degrees(a + lyaw))
            for k, ranges in secs.items():
                for lo, hi in ranges:
                    if lo <= b <= hi:
                        if math.isfinite(r):
                            values[k].append(r)
                            if r < nearest[k][0]:
                                nearest[k] = (r, b)
                        if i < len(raw) and math.isfinite(raw[i]):
                            raw_values[k].append(raw[i])
                        if math.isfinite(r):
                            counts[k] += 1
                        break
        out = {}
        raw_min = {}
        percentile = P.sector_percentile / 100.0
        for k in secs:
            vals = sorted(values[k])
            if vals:
                idx = min(len(vals) - 1, int(percentile * (len(vals) - 1)))
                out[k] = vals[idx]
                # Bearing is informational; report the closest filtered return.
                out_ang[k] = nearest[k][1]
            else:
                out[k] = P.max_valid_range
            raw_min[k] = min(raw_values[k]) if raw_values[k] else P.max_valid_range
        self.filter_stats = {
            'valid': sum(1 for value in filtered if math.isfinite(value)),
            'total': len(msg.ranges),
            'sector_samples': sum(counts.values()),
        }
        out['counts'] = counts
        out['bearings'] = out_ang
        out['raw_min'] = raw_min
        out['raw_estop'] = raw_min['estop']
        out['raw_rear'] = raw_min['rear']
        out['age'] = age
        return out

    def map_pose(self, frame='map'):
        """Return a SLAM-frame pose only; never substitute odom for map goals."""
        try:
            t = self.tf_buf.lookup_transform(frame, 'base_link',
                                             rospy.Time(0), rospy.Duration(0.2))
            p = t.transform.translation
            q = t.transform.rotation
            yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
            self.pose_src = 'map'
            return (p.x, p.y, yaw)
        except Exception:
            return None

    def pose(self):
        """Current pose (x, y, yaw); prefer map TF, fall back to /odom."""
        pose = self.map_pose()
        if pose is not None:
            return pose
        if self.odom_pose is not None:
            self.pose_src = 'odom'
            return self.odom_pose
        self.pose_src = 'none'
        return None

    def select_frontier(self, pose=None):
        """Select a reachable map frontier along a known-free corridor."""
        if not P.frontier_enabled:
            return False
        with self.lock:
            grid, stamp = self.map_grid, self.map_stamp
        if grid is None or time.time() - stamp > P.frontier_map_timeout:
            return False
        frame = getattr(getattr(grid, 'header', None), 'frame_id', '') or 'map'
        map_pose = self.map_pose(frame)
        if map_pose is None:
            return False
        now = time.time()
        self.frontier_blacklist = [entry for entry in self.frontier_blacklist
                                   if entry[2] > now]
        blacklist = [(entry[0], entry[1]) for entry in self.frontier_blacklist]
        goal = select_frontier_goal(
            grid, map_pose, P.travel_direction,
            clearance_radius=P.frontier_clearance_radius,
            max_range=P.frontier_max_range,
            min_range=P.frontier_min_range,
            blacklist=blacklist)
        if goal is None:
            self.frontier_goal = None
            return False
        self.frontier_goal = goal
        self.pose_src = 'map'
        self.msg = ('地图前沿目标 %.2fm, 信息增益 %d 格' %
                    (goal['distance'], goal['gain_cells']))
        return True

    def blacklist_frontier_goal(self):
        if self.frontier_goal is not None:
            self.frontier_blacklist.append((
                self.frontier_goal.get('frontier_x', self.frontier_goal['x']),
                self.frontier_goal.get('frontier_y', self.frontier_goal['y']),
                time.time() + P.frontier_blacklist_secs))
        self.frontier_goal = None

    def resume_after_turn(self):
        """Resume the interrupted behavior, replanning after blocked goals."""
        if self.resume_state == 'EXPLORE':
            self.state = 'EXPLORE'
            if not self.select_frontier():
                self.state = 'FOLLOW'
                self.msg = '前沿暂不可直达, 恢复保守沿墙巡航'
            else:
                pose = self.pose()
                self.last_pose_check = (pose, time.time()) if pose else None
        elif self.resume_state == 'ESCAPE':
            self.state = 'ESCAPE'
            self.escape_sign = 0
            self.escape_move_pose = None
            self.msg = '转向完成, 继续以实际位移确认区域脱离'
        else:
            self.state = 'FOLLOW'
            self.msg = '转向完成, 恢复巡航'

    def step_frontier(self, sec):
        """Track a selected frontier while retaining the lidar safety checks."""
        with self.lock:
            grid, stamp = self.map_grid, self.map_stamp
        if grid is None or time.time() - stamp > P.frontier_map_timeout:
            self.stop_motors()
            self.state = 'WAIT_MAP'
            self.last_pose_check = None
            self.msg = '地图数据过期, 停车等待新地图'
            return
        frame = getattr(getattr(grid, 'header', None), 'frame_id', '') or 'map'
        pose = self.map_pose(frame)
        if pose is None:
            self.stop_motors()
            self.state = 'WAIT_MAP'
            self.last_pose_check = None
            self.msg = '缺少地图到小车位姿TF, 停车等待'
            return
        self.pose_src = 'map'
        if self.frontier_goal is None and not self.select_frontier(pose):
            self.state = 'FOLLOW'
            self.msg = '当前无可达前沿, 暂用保守沿墙巡航'
            return

        goal = self.frontier_goal
        dx, dy = goal['x'] - pose[0], goal['y'] - pose[1]
        distance = math.hypot(dx, dy)
        if distance <= P.frontier_goal_tolerance:
            self.goal_reached_count += 1
            self.frontier_goal = None
            if self.select_frontier(pose):
                self.last_pose_check = (pose, time.time())
                self.msg = '前沿点已到达, 选择下一目标(累计%d)' % (
                    self.goal_reached_count)
            else:
                self.state = 'FOLLOW'
                self.msg = '附近没有新的直线路径前沿, 暂用沿墙巡航'
            return

        travel_heading = math.atan2(dy, dx)
        body_heading = (travel_heading - math.pi
                        if P.travel_direction < 0 else travel_heading)
        heading_error = norm_ang(body_heading - pose[2])
        if abs(heading_error) > P.frontier_turn_tolerance:
            physical_sign = 1 if heading_error > 0.0 else -1
            travel_sign = physical_sign * P.travel_direction
            if self.turn_path_distance(sec, travel_sign) < P.turn_stop_dist:
                self.resume_state = 'EXPLORE'
                self.enter_obstacle_wait(sec, '前沿目标转向路径被占用')
                return
            self.send_rotate(P.turn_rate * physical_sign)
            self.msg = '对准前沿目标 %.2fm, 转角%.0f°' % (
                distance, math.degrees(heading_error))
            return

        if sec['front'] < P.front_turn_dist:
            self.resume_state = 'EXPLORE'
            self.enter_obstacle_wait(sec, '前沿路径受阻')
            return
        speed = min(P.cruise_speed,
                    max(12.0, P.cruise_speed * distance / 0.8))
        self.send_translation(0.0, speed * P.travel_direction)
        self.msg = '前沿探索 距目标%.2fm 增益%d格' % (
            distance, goal['gain_cells'])

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
        self.estop_hits = 0
        self.obstacle_hits = 0
        self.obstacle_since = None
        self.obstacle_clear_since = None
        self.obstacle_kind = 'front'
        self.turn_sign = P.wall_side
        self.area_anchor = pose
        self.area_since = time.time()
        self.escape_sign = 0
        self.escape_t0 = 0.0
        self.escape_move_pose = None
        self.escape_move_t0 = 0.0
        self.escape_pose_src = None
        self.state = 'FOLLOW'
        direction = '倒车巡航' if P.travel_direction < 0 else '前进巡航'
        self.msg = '%s已出发(位姿源:%s)' % (direction, self.pose_src)
        self.frontier_goal = None
        self.frontier_blacklist = []
        self.goal_reached_count = 0
        if self.select_frontier():
            self.state = 'EXPLORE'
            self.msg = '已选择地图前沿目标 %.2fm, 信息增益 %d 格' % (
                self.frontier_goal['distance'], self.frontier_goal['gain_cells'])
        rospy.loginfo('wall_follow START pose=%s src=%s',
                      str(pose), self.pose_src)

    def choose_turn_sign(self, sec):
        """Choose the more open side for an in-place turn.

        Return +1 for left, -1 for right, or 0 when neither side has enough
        clearance.  The wall side is only a tie breaker, never a blind order.
        """
        left = min(sec.get('front_left', P.max_valid_range),
                   sec.get('left', P.max_valid_range))
        right = min(sec.get('front_right', P.max_valid_range),
                    sec.get('right', P.max_valid_range))
        # 原始近点也参与转向准入判断：反侧的近点不阻止绕行，
        # 但候选转向路径上的近点会被拒绝。
        raw = sec.get('raw_min', {})
        left = min(left, raw.get('front_left', P.max_valid_range),
                   raw.get('left', P.max_valid_range))
        right = min(right, raw.get('front_right', P.max_valid_range),
                    raw.get('right', P.max_valid_range))
        if max(left, right) < P.turn_clear_dist:
            return 0
        # 只允许在满足准入距离的方向间择优，不能因相差不足 10cm
        # 而按沿墙偏好选到未达标的另一侧。
        if left < P.turn_clear_dist:
            return -1
        if right < P.turn_clear_dist:
            return 1
        if abs(left - right) < 0.10:
            return 1 if P.wall_side < 0 else -1
        return 1 if left > right else -1

    @staticmethod
    def turn_path_distance(sec, sign):
        side = 'left' if sign > 0 else 'right'
        diagonal = 'front_left' if sign > 0 else 'front_right'
        raw = sec.get('raw_min', {})
        return min(sec.get(side, P.max_valid_range),
                   sec.get(diagonal, P.max_valid_range),
                   raw.get(side, P.max_valid_range),
                   raw.get(diagonal, P.max_valid_range))

    def enter_obstacle_wait(self, sec, reason, kind='front'):
        if self.state not in ('WAIT_OBSTACLE', 'TURN'):
            self.resume_state = self.state
        self.state = 'WAIT_OBSTACLE'
        self.obstacle_since = self.obstacle_since or time.time()
        self.obstacle_clear_since = None
        self.obstacle_kind = kind
        self.stop_motors()
        distance = sec.get('side' if kind == 'side' else 'front', P.max_valid_range)
        self.msg = '%s; %s %.2fm' % (reason, '侧方' if kind == 'side' else '行驶前方', distance)

    def area_escape_sign(self, sec):
        """Return 0 for a clear current travel direction, otherwise a turn sign."""
        # 中央行驶方向开阔时，不要因为侧墙误差而原地打转；直接离开当前区域。
        if (sec['front'] >= P.area_escape_min_clear and
                sec['raw_estop'] >= P.estop_dist):
            return 0
        return self.choose_turn_sign(sec)

    def start_area_escape(self, sec, pose):
        """Leave a low-information local area using the safest open direction."""
        if self.select_frontier(pose):
            self.state = 'EXPLORE'
            self.last_pose_check = (pose, time.time())
            self.msg = '驻留超时, 改选地图前沿(增益%d格)' % (
                self.frontier_goal['gain_cells'])
            return True
        sign = self.area_escape_sign(sec)
        if sign == 0 and sec['front'] < P.area_escape_min_clear:
            self.state = 'PAUSED_RECOVERY'
            self.msg = '驻留超时且无可达前沿/安全脱离方向, 保持停车'
            self.stop_motors()
            return True
        self.escape_sign = sign
        self.escape_t0 = time.time()
        self.escape_move_pose = None
        self.escape_move_t0 = 0.0
        self.stuck_count = 0
        self.state = 'ESCAPE'
        self.stop_motors()
        if sign:
            self.msg = '当前区域驻留过久, 向%s侧寻找开阔区' % (
                '左' if sign > 0 else '右')
        else:
            self.msg = '当前区域驻留过久, 沿开阔方向脱离'
        rospy.loginfo('wall_follow area escape sign=%d pose=%s', sign, str(pose))
        return True

    def step_area_escape(self, sec):
        """Escape only counts as successful after measured displacement."""
        dt = time.time() - self.escape_t0
        pose = self.pose()
        if (self.escape_move_pose is not None and pose is not None and
                self.pose_src != self.escape_pose_src):
            self.stop_motors()
            self.state = 'PAUSED_RECOVERY'
            self.msg = '脱离期间位姿源切换, 位移不可比, 保持停车'
            return
        if self.escape_move_pose is not None and pose is not None:
            moved = math.hypot(pose[0] - self.escape_move_pose[0],
                               pose[1] - self.escape_move_pose[1])
            if moved >= P.area_escape_progress:
                self.state = 'FOLLOW'
                self.area_anchor = pose
                self.area_since = time.time()
                self.escape_sign = 0
                self.last_pose_check = None
                self.msg = '区域脱离已确认, 实际移动 %.2fm, 恢复巡航' % moved
                return
        if dt >= P.area_escape_timeout:
            self.stop_motors()
            self.state = 'PAUSED_RECOVERY'
            self.msg = '区域脱离位移不足 %.2fm, 超时停车待诊断' % (
                P.area_escape_progress)
            return
        if pose is None:
            self.stop_motors()
            self.state = 'PAUSED_RECOVERY'
            self.msg = '无可靠位姿, 无法确认区域脱离; 保持停车'
            return
        if self.escape_sign and dt < P.area_escape_turn_secs:
            if self.turn_path_distance(sec, self.escape_sign) < P.turn_stop_dist:
                self.enter_obstacle_wait(sec, '开阔区转向路径被占用')
                return
            self.send_rotate(P.turn_rate * self.escape_sign * P.travel_direction)
            self.msg = '区域脱离转向 %s %.1fs' % (
                '左' if self.escape_sign > 0 else '右',
                P.area_escape_turn_secs - dt)
            return
        if self.escape_move_pose is None:
            self.escape_move_pose = pose
            self.escape_move_t0 = time.time()
            self.escape_pose_src = self.pose_src
        # Escape does not re-enter wall-follow merely because a timer expired.
        if (self.escape_move_pose is not None and
                time.time() - self.escape_move_t0 >= P.area_escape_timeout and
                math.hypot(pose[0] - self.escape_move_pose[0],
                           pose[1] - self.escape_move_pose[1]) < P.area_escape_progress):
            self.stop_motors()
            self.state = 'PAUSED_RECOVERY'
            self.msg = '区域脱离未达到目标位移, 停车待诊断'
            return
        self.send_translation(0.0, P.area_escape_speed * P.travel_direction)
        moved = math.hypot(pose[0] - self.escape_move_pose[0],
                           pose[1] - self.escape_move_pose[1])
        self.msg = '区域脱离中, 已移动 %.2f/%.2fm' % (
            moved, P.area_escape_progress)

    def step_obstacle_wait(self, sec):
        """Stop first; clear briefly to resume, persistently blocked to turn."""
        self.stop_motors()
        now = time.time()
        front = sec['front']
        hard_front = sec['raw_estop'] < P.estop_dist
        hard_side = sec['raw_min']['side'] < P.side_estop_dist
        clear = (front >= P.obstacle_clear_dist and
                 not hard_front and not hard_side)
        if self.obstacle_kind == 'side':
            clear = (sec['side'] >= P.side_clear_dist and
                     not hard_front and not hard_side)
        else:
            # 前方障碍等待期间，也不能让新的侧向人员/障碍触发恢复。
            clear = clear and sec['side'] >= P.side_pause_dist
        if clear:
            if self.obstacle_clear_since is None:
                self.obstacle_clear_since = now
            elif now - self.obstacle_clear_since >= P.obstacle_clear_secs:
                self.obstacle_since = None
                self.obstacle_clear_since = None
                self.last_pose_check = None
                self.area_anchor = self.pose()
                self.area_since = time.time()
                if self.resume_state == 'EXPLORE':
                    self.state = 'EXPLORE'
                    if self.frontier_goal is None and not self.select_frontier():
                        self.state = 'FOLLOW'
                        self.msg = '障碍物已离开, 无前沿目标, 恢复巡航'
                    else:
                        self.last_pose_check = (self.pose(), now)
                        self.msg = '障碍物已离开, 恢复前沿目标'
                elif self.resume_state == 'ESCAPE':
                    self.state = 'ESCAPE'
                    self.msg = '障碍物已离开, 继续确认脱离位移'
                else:
                    self.state = 'FOLLOW'
                    self.msg = '障碍物已离开, 安全恢复巡航'
            else:
                self.msg = '障碍物暂时离开, 继续确认 %.1fs' % (
                    P.obstacle_clear_secs - (now - self.obstacle_clear_since))
            return

        self.obstacle_clear_since = None
        waited = now - (self.obstacle_since or now)
        if waited >= P.obstacle_wait_secs:
            sign = self.choose_turn_sign(sec)
            if sign:
                if self.resume_state == 'EXPLORE':
                    self.blacklist_frontier_goal()
                self.turn_sign = sign
                self.state = 'TURN'
                self.obstacle_since = None
                self.msg = '障碍物持续存在, 向%s侧绕行' % ('左' if sign > 0 else '右')
                return
            self.msg = '前方障碍且左右均不足 %.2fm, 保持停车' % P.turn_clear_dist
        else:
            self.msg = '发现障碍, 停车观察 %.1fs' % (P.obstacle_wait_secs - waited)

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
        if self.state in ('IDLE', 'DONE', 'ABORT', 'PAUSED_RECOVERY'):
            return

        # 超时
        if time.time() - self.t0 > P.max_seconds:
            self.finish('ABORT', '超时 %.0fs' % P.max_seconds)
            return

        sec = self.sectors()
        if sec is None:
            # 雷达断流 >1s: 立即停车等恢复
            self.stop_motors()
            if self.state != 'WAIT_SCAN':
                self.resume_state = self.state
            self.state = 'WAIT_SCAN'
            self.msg = '雷达断流, 停车等待'
            return

        if self.state == 'WAIT_SCAN':
            self.state = ('FOLLOW' if self.resume_state == 'WAIT_SCAN'
                          else self.resume_state)
            self.msg = '雷达数据恢复, 继续安全检查'

        if self.state == 'WAIT_MAP':
            with self.lock:
                grid, stamp = self.map_grid, self.map_stamp
            fresh = (grid is not None and
                     time.time() - stamp <= P.frontier_map_timeout)
            frame = getattr(getattr(grid, 'header', None), 'frame_id', '') or 'map'
            localized = fresh and self.map_pose(frame) is not None
            if not localized:
                self.stop_motors()
                self.msg = '等待有效地图与 map->base_link TF'
                return
            if self.select_frontier():
                self.state = 'EXPLORE'
                self.last_pose_check = (self.pose(), time.time())
                self.msg = '地图/TF恢复, 继续前沿探索'
            else:
                self.state = 'FOLLOW'
                self.msg = '地图/TF已恢复但无可达前沿, 回退沿墙巡航'

        # 车尾扇区无有效回波时无法判断倒车路径，必须停下。
        if P.travel_direction < 0 and sec.get('counts', {}).get('front', 0) == 0:
            self.stop_motors()
            if self.state != 'WAIT_SCAN':
                self.resume_state = self.state
            self.state = 'WAIT_SCAN'
            self.msg = '车尾雷达无有效回波, 停车等待'
            return

        front, side = sec['front'], sec['side']
        fside = sec['front_side']

        # 急停使用原始点: 首帧立即停车，连续 2 帧确认后进入等待。
        # WAIT/TURN 状态由各自的路径安全检查处理，避免一个位于
        # 转向反侧的近点永久阻止绕行。
        side_hard = sec['raw_min']['side'] < P.side_estop_dist
        hard_obstacle = sec['raw_estop'] < P.estop_dist or side_hard
        if self.state in ('FOLLOW', 'LOST', 'RECOVER', 'ESCAPE', 'EXPLORE') and hard_obstacle:
            self.estop_hits += 1
            self.stop_motors()
            if self.estop_hits < 2:
                self.msg = '安全急停确认中, 保持停车'
                return
            self.estop_hits = 0
            self.enter_obstacle_wait(sec, '安全急停',
                                     'side' if side_hard else 'front')
            return
        if self.state not in ('WAIT_OBSTACLE', 'TURN'):
            self.estop_hits = 0

        # 普通前方障碍先停车观察。短暂横穿的人员离开后自动恢复；
        # 持续存在的家具/墙体才进入 TURN，避免一见人就盲目转向。
        side_blocked = side < P.side_pause_dist
        if self.state in ('FOLLOW', 'LOST', 'ESCAPE', 'EXPLORE') and (front < P.obstacle_pause_dist or side_blocked):
            self.obstacle_hits += 1
        else:
            self.obstacle_hits = 0
        if self.obstacle_hits >= 2:
            self.obstacle_hits = 0
            self.enter_obstacle_wait(sec, '侧方障碍' if side_blocked else '前方障碍',
                                     'side' if side_blocked else 'front')
            return
        if self.obstacle_hits:
            # 第一帧先停再确认，防止近障直接落入下面的 TURN 分支，
            # 绕过动态障碍等待。持续两帧后才切换等待状态。
            self.stop_motors()
            self.msg = '障碍确认中, 保持停车'
            return

        if self.state == 'WAIT_OBSTACLE':
            self.step_obstacle_wait(sec)
            return

        # 位姿/路程/回环/卡死
        pose = self.pose()
        if pose is not None and self.last_pose is not None:
            dx = pose[0] - self.last_pose[0]
            dy = pose[1] - self.last_pose[1]
            d = math.hypot(dx, dy)
            if d < 1.0:    # 防位姿源切换跳变
                self.path_len += d
                # 卡死计数只表示连续失败的脱困次数。此前计数在成功
                # 脱困并正常行驶后仍会保留，多个相隔很久的小卡顿会
                # 累加到 stuck_abort，误触发 ABORT。
                if (self.state in ('FOLLOW', 'EXPLORE') and
                        self.stuck_count > 0 and
                        d >= P.stuck_reset_dist):
                    self.stuck_count = 0
                    rospy.loginfo('wall_follow effective motion %.2fm; '
                                  'reset stuck counter', d)
            self.last_pose = pose
        if pose is not None:
            if self.area_anchor is None:
                self.area_anchor = pose
                self.area_since = time.time()
            elif (self.state == 'FOLLOW' and
                  math.hypot(pose[0] - self.area_anchor[0],
                             pose[1] - self.area_anchor[1]) >= P.area_dwell_radius):
                self.area_anchor = pose
                self.area_since = time.time()
        # 区域驻留优先于普通“卡死”计数：在低信息区域反复转向时，
        # 应主动换到开阔区，而不是累计几次后直接 ABORT。
        if (self.state == 'FOLLOW' and pose is not None and
                self.area_anchor is not None and
                time.time() - self.area_since >= P.area_dwell_secs):
            if self.start_area_escape(sec, pose):
                return
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
            # 沿墙巡航与前沿直行均需要进度看门狗，不能让 EXPLORE 无进度地跑满整轮。
            if self.state in ('FOLLOW', 'EXPLORE'):
                if self.last_pose_check is None:
                    self.last_pose_check = (pose, time.time())
                pc, pt = self.last_pose_check
                if time.time() - pt > P.stuck_secs:
                    moved = math.hypot(pose[0] - pc[0], pose[1] - pc[1])
                    self.last_pose_check = (pose, time.time())
                    if moved < 0.02:
                        self.stuck_count += 1
                        self.resume_state = self.state
                        if self.state == 'EXPLORE':
                            self.blacklist_frontier_goal()
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
                # 前方/前侧受阻 → 只向已确认更空旷的一侧转
                sign = self.choose_turn_sign(sec)
                if not sign:
                    self.enter_obstacle_wait(sec, '转向两侧均不安全')
                    return
                self.turn_sign = sign
                self.resume_state = 'FOLLOW'
                self.state = 'TURN'
                self.msg = '遇障碍向%s转(前 %.2f 侧 %.2f)' % (
                    '左' if sign > 0 else '右', front, fside)
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
            # 沿行驶方向巡航：倒车模式把整套安全扇区转到车尾。
            vy = P.cruise_speed
            if front < P.front_slow_dist:
                span = P.front_slow_dist - P.front_turn_dist
                vy = max(12.0, P.cruise_speed * (front - P.front_turn_dist) / span)
            # 横移 P 控墙距: 右墙(side>0)太远→向右挪(vx+)
            err = side - P.target_wall_dist
            if (P.prefer_turning and abs(err) > P.wall_turn_threshold and
                    front < P.front_clear_dist):
                # 右墙太远时向右转, 右墙太近时向左转; 转向代替大幅侧移。
                turn_sign = -P.wall_side if err > 0.0 else P.wall_side
                self.send_rotate(P.wall_turn_rate * turn_sign * P.travel_direction)
                self.msg = '转向修正 前%.2f 侧%.2f 误差%.2f' % (
                    front, side, err)
                return
            vx = P.wall_kp * err * P.wall_side * P.travel_direction
            vx = max(-P.strafe_max, min(P.strafe_max, vx))
            self.send_translation(vx, vy * P.travel_direction)
            self.msg = '%s巡航 前%.2f 侧%.2f vx%.0f vy%.0f' % (
                '倒车' if P.travel_direction < 0 else '前进', front, side,
                vx, vy * P.travel_direction)

        elif self.state == 'TURN':
            turn_side = sec['left'] if self.turn_sign > 0 else sec['right']
            turn_path = self.turn_path_distance(sec, self.turn_sign)
            if turn_path < P.turn_stop_dist:
                self.enter_obstacle_wait(sec, '转向路径被占用')
                return
            # 不能只看滤波后的 front：孤立近点可能被 EMA 拉远到 0.6m，
            # 但车体实际仍面对 < estop_dist 的原始回波。此时若直接判定
            # TURN 完成，会回到 FOLLOW，下一帧再次急停，形成停车循环。
            # 只有原始行驶方向也清空后，才允许结束转向。
            if (front > P.front_clear_dist and
                    sec['raw_estop'] >= P.estop_dist and
                    turn_side > P.turn_clear_dist * 0.8):
                self.resume_after_turn()
                return
            self.send_rotate(P.turn_rate * self.turn_sign * P.travel_direction)
            self.msg = '向%s转向 前%.2f 侧%.2f' % (
                '左' if self.turn_sign > 0 else '右', front, turn_side)

        elif self.state == 'ESCAPE':
            self.step_area_escape(sec)

        elif self.state == 'EXPLORE':
            self.step_frontier(sec)

        elif self.state == 'LOST':
            # 向墙侧慢转, 找回墙
            if side < P.target_wall_dist + 0.3:
                self.state = 'FOLLOW'
                self.lost_since = None
                self.msg = '找回墙, 恢复巡航'
                return
            self.send_rotate(-P.turn_rate * P.wall_side * P.travel_direction)
            self.msg = '找墙中 侧%.2f' % side

        elif self.state == 'RECOVER':
            dt = time.time() - self.recover_t0
            if dt < 0.6:
                self.stop_motors()
                self.msg = '急停缓冲'
            elif dt < 2.2:
                if sec['raw_rear'] < P.rear_clear_dist:
                    self.enter_obstacle_wait(sec, '恢复方向不安全, 取消脱困')
                    return
                self.send_translation(0.0, -P.backup_speed * P.travel_direction)
                self.msg = '脱困方向安全, 慢速移动中'
            elif dt < 3.4:
                if self.turn_path_distance(sec, P.wall_side) < P.turn_stop_dist:
                    self.enter_obstacle_wait(sec, '脱困转向路径被占用')
                    return
                self.send_rotate(P.turn_rate * P.wall_side * P.travel_direction)
                self.msg = '恢复转向'
            else:
                if self.resume_state == 'EXPLORE':
                    self.state = 'EXPLORE'
                    if not self.select_frontier():
                        self.state = 'FOLLOW'
                        self.msg = '脱困后无可达前沿, 恢复沿墙巡航'
                else:
                    self.state = 'FOLLOW'
                self.last_pose_check = None
                if self.state == 'EXPLORE':
                    self.msg = '脱困完成, 重新选择地图前沿'
                elif not self.msg.startswith('脱困后无可达前沿'):
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
            'travel_direction': P.travel_direction,
            'travel_mode': 'reverse' if P.travel_direction < 0 else 'forward',
            'area': {
                'dwell_secs': P.area_dwell_secs,
                'dwell_radius': P.area_dwell_radius,
                'escape_timeout': P.area_escape_timeout,
                'escape_state': self.state == 'ESCAPE',
                'escape_progress': P.area_escape_progress,
            },
            'frontier': None if self.frontier_goal is None else {
                'x': round(self.frontier_goal['x'], 2),
                'y': round(self.frontier_goal['y'], 2),
                'distance': round(self.frontier_goal['distance'], 2),
                'gain_cells': self.frontier_goal['gain_cells'],
                'score': round(self.frontier_goal['score'], 3),
                'reached': self.goal_reached_count,
            },
            'wait_secs': (round(time.time() - self.obstacle_since, 1)
                          if self.state == 'WAIT_OBSTACLE' and self.obstacle_since
                          else 0.0),
            'safety': {
                'estop_dist': P.estop_dist,
                'pause_dist': P.obstacle_pause_dist,
                'clear_dist': P.obstacle_clear_dist,
                'front_slow_dist': P.front_slow_dist,
                'front_turn_dist': P.front_turn_dist,
                'front_clear_dist': P.front_clear_dist,
                'turn_clear_dist': P.turn_clear_dist,
                'turn_stop_dist': P.turn_stop_dist,
                'side_pause_dist': P.side_pause_dist,
                'side_estop_dist': P.side_estop_dist,
                'scan_timeout': P.scan_timeout,
                'stuck_reset_dist': P.stuck_reset_dist,
                'filter_enabled': P.filter_enabled,
                'spatial_radius': P.spatial_filter_radius,
                'ema_alpha': P.ema_alpha,
                'sector_percentile': P.sector_percentile,
            },
            'filter': self.filter_stats,
            'ts': time.time(),
        }
        self.pub_status.publish(String(json.dumps(s, ensure_ascii=False)))


if __name__ == '__main__':
    rospy.init_node('wall_follow')
    WallFollower().spin()
