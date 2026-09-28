#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""自动建图编排器。

这个节点只负责编排：启动/停止 slam_control，并把运动控制交给
WallFollower。WallFollower 内含雷达滤波、动态障碍等待、左右安全转向和
底盘心跳看门狗。节点启动后保持 IDLE，不会自动让小车运动。

命令:
  /auto_mapping/cmd (std_msgs/String): start | stop

状态:
  /auto_mapping/status (std_msgs/String, JSON, latched)
"""

import json
import threading
import time

import rospy
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Empty, String

from wall_follow import WallFollower


class AutoMappingController(object):
    def __init__(self):
        self.state = 'IDLE'
        self.msg = '待命, 不会自动移动'
        self.lock = threading.Lock()
        self.started_at = 0.0
        self.slam_status = {}
        self.wall_status = {}
        self.last_map_at = 0.0

        self.pub_slam = rospy.Publisher('/slam_control/cmd', String,
                                        queue_size=1)
        self.pub_wall = rospy.Publisher('/wall_follow/cmd', String,
                                        queue_size=1)
        self.pub_chassis_stop = rospy.Publisher('/chassis_control/stop', Empty,
                                                queue_size=1, latch=True)
        self.pub_status = rospy.Publisher('/auto_mapping/status', String,
                                          queue_size=1, latch=True)
        # 给控制台提供统一的地图坐标位姿。/poseupdate 只在部分 SLAM
        # 引擎存在，直接查 map->base_link 可同时兼容 hector 和 slam_toolbox。
        self.pub_pose = rospy.Publisher('/console/robot_pose',
                                        PoseWithCovarianceStamped, queue_size=1)
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)
        rospy.Subscriber('/auto_mapping/cmd', String, self.on_cmd)
        rospy.Subscriber('/slam_control/status', String, self.on_slam_status,
                         queue_size=1)
        rospy.Subscriber('/wall_follow/status', String, self.on_wall_status,
                         queue_size=1)
        rospy.Subscriber('/map', OccupancyGrid, self.on_map, queue_size=1)

        # WallFollower 在同一 ROS 进程内运行，避免另外启动一个没有看门狗
        # 绑定关系的运动节点。它初始为 IDLE，只有收到 start 才会发速度。
        self.follower = WallFollower()
        self.worker = threading.Thread(target=self.follower.spin,
                                       name='wall-follow', daemon=True)
        self.worker.start()
        rospy.Timer(rospy.Duration(0.25), self.tick)
        rospy.Timer(rospy.Duration(0.10), self.publish_pose)
        rospy.on_shutdown(self.on_shutdown)
        self.publish_status()

    def publish_pose(self, _event):
        """Publish map->base_link for the web console when SLAM is available."""
        try:
            tf_msg = self.tf_buf.lookup_transform(
                'map', 'base_link', rospy.Time(0), rospy.Duration(0.08))
        except Exception:
            # 建图未启动或 TF 暂时不完整时不发布伪造位置。
            return
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = tf_msg.header.stamp
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = tf_msg.transform.translation.x
        msg.pose.pose.position.y = tf_msg.transform.translation.y
        msg.pose.pose.position.z = tf_msg.transform.translation.z
        msg.pose.pose.orientation = tf_msg.transform.rotation
        self.pub_pose.publish(msg)

    def on_map(self, _msg):
        self.last_map_at = time.time()

    def on_slam_status(self, msg):
        try:
            self.slam_status = json.loads(msg.data or '{}')
        except (TypeError, ValueError):
            self.slam_status = {}

    def on_wall_status(self, msg):
        try:
            self.wall_status = json.loads(msg.data or '{}')
        except (TypeError, ValueError):
            self.wall_status = {}

    def publish(self, publisher, value):
        publisher.publish(String(value))

    def on_cmd(self, msg):
        cmd = (msg.data or '').strip().lower()
        if cmd == 'start':
            with self.lock:
                if self.state not in ('IDLE', 'STOPPED', 'DONE', 'ERROR'):
                    return
                self.state = 'STARTING_SLAM'
                self.msg = '正在启动 SLAM; 小车保持停止'
                self.started_at = time.time()
            self.publish(self.pub_slam, 'start')
            self.publish_status()
        elif cmd == 'stop':
            self.stop_all('已停止, 小车保持停止')

    def stop_all(self, message):
        # 先直接切断底盘硬件输出，再停墙跟随和 SLAM。底盘停止回调会
        # 取消尚未完成的缓变速线程，避免旧速度在 stop 之后补写回来。
        for _ in range(3):
            self.pub_chassis_stop.publish(Empty())
            time.sleep(0.02)
        self.publish(self.pub_wall, 'stop')
        self.publish(self.pub_slam, 'stop')
        self.pub_chassis_stop.publish(Empty())
        with self.lock:
            self.state = 'STOPPED'
            self.msg = message
        self.publish_status()

    def tick(self, _event):
        now = time.time()
        with self.lock:
            state = self.state
            started_at = self.started_at

        if state == 'STARTING_SLAM':
            slam_running = bool(self.slam_status.get('running'))
            fresh_map = self.last_map_at >= started_at - 0.2
            if slam_running and fresh_map and now - started_at >= 1.0:
                self.publish(self.pub_wall, 'start')
                with self.lock:
                    self.state = 'RUNNING'
                    self.msg = 'SLAM 已就绪, 雷达辅助巡航建图已启动'
            elif now - started_at > 30.0:
                self.stop_all('SLAM 启动超时, 未启动运动')

        elif state in ('RUNNING', 'PAUSED'):
            wall_state = self.wall_status.get('state')
            if wall_state == 'DONE':
                with self.lock:
                    self.state = 'DONE'
                    self.msg = '已回到起点附近; 请保存地图后再停止 SLAM'
            elif wall_state == 'ABORT':
                with self.lock:
                    self.state = 'ERROR'
                    self.msg = '自动巡航中止: %s' % self.wall_status.get('msg', '')
            elif wall_state in ('WAIT_OBSTACLE', 'WAIT_MAP', 'WAIT_SCAN',
                                'PAUSED_RECOVERY'):
                with self.lock:
                    self.state = 'PAUSED'
                    self.msg = self.wall_status.get('msg') or '安全暂停, 等待条件恢复'
            elif state == 'PAUSED' and wall_state in (
                    'FOLLOW', 'EXPLORE', 'TURN', 'LOST', 'RECOVER', 'ESCAPE'):
                with self.lock:
                    self.state = 'RUNNING'
                    self.msg = '巡航条件恢复, 继续自动建图'

        self.publish_status()

    def publish_status(self):
        wall = self.wall_status or {}
        data = {
            'state': self.state,
            'msg': self.msg,
            'slam_running': bool(self.slam_status.get('running')),
            'wall_state': wall.get('state', 'UNKNOWN'),
            'wall_msg': wall.get('msg', ''),
            'path_len': wall.get('path_len'),
            'filter': wall.get('filter'),
            'safety': wall.get('safety'),
            'travel_mode': wall.get('travel_mode', 'forward'),
            'frontier': wall.get('frontier'),
            'wait_secs': wall.get('wait_secs', 0.0),
            'ts': time.time(),
        }
        self.pub_status.publish(String(json.dumps(data, ensure_ascii=False)))

    def on_shutdown(self):
        try:
            self.publish(self.pub_wall, 'stop')
            self.publish(self.pub_slam, 'stop')
        except Exception:
            pass


if __name__ == '__main__':
    rospy.init_node('auto_mapping_control')
    AutoMappingController()
    rospy.spin()
