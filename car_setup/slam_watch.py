#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# slam_watch.py — 建图冻结诊断: 车动时同步采样 SLAM位姿/里程计/地图尺寸/扫描统计
# 用法: python3 slam_watch.py <秒数>  (输出到 stdout, 建议重定向到文件)
import math
import sys
import time

import rospy
import tf2_ros
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from tf.transformations import euler_from_quaternion

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 40.0

odom_pose = None
scan_info = None
map_info = None


def on_odom(m):
    global odom_pose
    p = m.pose.pose.position
    q = m.pose.pose.orientation
    yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
    odom_pose = (p.x, p.y, yaw)


def on_scan(m):
    global scan_info
    n = 0
    near = 0
    far = 0
    for r in m.ranges:
        if math.isnan(r) or r < 0.10:
            continue
        n += 1
        if r < 3.0:
            near += 1
        elif r > 7.5:
            far += 1
    scan_info = (n, near, far, m.header.stamp.to_sec())


def on_map(m):
    global map_info
    map_info = (m.info.width, m.info.height)


def main():
    rospy.init_node('slam_watch', anonymous=True)
    rospy.Subscriber('/odom', Odometry, on_odom, queue_size=1)
    rospy.Subscriber('/scan', LaserScan, on_scan, queue_size=1)
    rospy.Subscriber('/map', OccupancyGrid, on_map, queue_size=1)
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf)
    time.sleep(1.5)
    print('t(s) | slam_pose(map->base) | odom_pose | scan:valid/near/far age | map WxH')
    t0 = time.time()
    while time.time() - t0 < DUR and not rospy.is_shutdown():
        t = time.time() - t0
        try:
            tr = buf.lookup_transform('map', 'base_link', rospy.Time(0),
                                      rospy.Duration(0.3))
            p = tr.transform.translation
            q = tr.transform.rotation
            yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
            slam_s = '%+.2f,%+.2f,%+.0fd' % (p.x, p.y, math.degrees(yaw))
        except Exception as e:
            slam_s = 'TF_FAIL:%s' % type(e).__name__
        od_s = 'none' if odom_pose is None else '%+.2f,%+.2f,%+.0fd' % (
            odom_pose[0], odom_pose[1], math.degrees(odom_pose[2]))
        if scan_info is None:
            sc_s = 'none'
        else:
            age = time.time() - scan_info[3] if scan_info[3] > 1e9 else -1
            sc_s = '%d/%d/%d' % (scan_info[0], scan_info[1], scan_info[2])
        mp_s = 'none' if map_info is None else '%dx%d' % map_info
        print('%5.1f | %-22s | %-20s | %-12s | %s' % (t, slam_s, od_s, sc_s, mp_s))
        sys.stdout.flush()
        time.sleep(0.5)


if __name__ == '__main__':
    main()
