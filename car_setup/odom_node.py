#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
odom_node.py -- wheel-odometry for Hiwonder ArmPi Pro (mecanum chassis).

Subscribes to /chassis_control/encoder_counts (std_msgs/Int32MultiArray)
published by chassis_control_node.py, computes mecanum forward kinematics,
and publishes /odom (nav_msgs/Odometry) + tf odom -> base_link.

Kinematics reverse-engineered from chassis_control_node.py:
  The chassis node sends wheel speeds as s = [-v1, v4, v2, -v3]
  where v1..v4 are intermediate mecanum wheel speeds derived from
  vx, vy, vp (vp = omega*(a+b)) via:
      v1 = vy - vx + vp
      v2 = vy + vx - vp
      v3 = vy - vx - vp
      v4 = vy + vx + vp
  Encoder deltas are proportional to commanded s. Forward command produced
  all encoder counts decreasing, so we define m_i = -delta_counts_i to make
  forward motion positive. Inverting:
      vx_robot = (m0+m1+m2+m3)/4
      vy_robot = (-m0+m1+m2-m3)/4
      vp       = (-m0+m1-m2+m3)/4
      omega    = vp / (a+b)

Params (mm):
  a=110, b=97.5, wheel_d=96.5, pulses_per_rev=44*178=7832
"""
import math

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32MultiArray

# ---------- chassis geometry (mm) ----------
A = 110.0
B = 97.5
WHEEL_DIAMETER = 96.5
PULSES_PER_REV = 44 * 178  # 7832
PI = math.pi
MM_PER_PULSE = (PI * WHEEL_DIAMETER) / PULSES_PER_REV

# ---------- sign calibration ----------
# These can be flipped if strafe/rotation tests show inverted axes.
SIGN_FORWARD = 1.0
SIGN_STRAFE = 1.0
SIGN_ROTATION = 1.0


def euler_to_quaternion(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    return Quaternion(
        x=sr * cp * cy - cr * sp * sy,
        y=cr * sp * cy + sr * cp * sy,
        z=cr * cp * sy - sr * sp * cy,
        w=cr * cp * cy + sr * sp * sy,
    )


class MecanumOdom:
    def __init__(self):
        rospy.init_node('mecanum_odom')
        self.frame_id = rospy.get_param('~frame_id', 'odom')
        self.child_frame_id = rospy.get_param('~child_frame_id', 'base_link')
        self.a = rospy.get_param('~a', A)
        self.b = rospy.get_param('~b', B)
        self.mm_per_pulse = rospy.get_param('~mm_per_pulse', MM_PER_PULSE)
        self.sign_forward = rospy.get_param('~sign_forward', SIGN_FORWARD)
        self.sign_strafe = rospy.get_param('~sign_strafe', SIGN_STRAFE)
        self.sign_rotation = rospy.get_param('~sign_rotation', SIGN_ROTATION)
        self.base_width = self.a + self.b

        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        rospy.Subscriber('/chassis_control/encoder_counts',
                         Int32MultiArray, self.on_encoders)

        self.last_counts = None
        self.last_time = None
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.ok = False

        rospy.loginfo('mecanum_odom waiting for /chassis_control/encoder_counts ...')

    def normalize_delta(self, new_val, old_val):
        delta = new_val - old_val
        if delta > 2147483648:
            delta -= 4294967296
        elif delta < -2147483648:
            delta += 4294967296
        return delta

    def on_encoders(self, msg):
        now = rospy.Time.now()
        counts = list(msg.data)
        if len(counts) != 4:
            rospy.logwarn_throttle(5.0, 'encoder message has %d values, expected 4', len(counts))
            return

        if self.last_counts is None:
            self.last_counts = counts
            self.last_time = now
            self.ok = True
            rospy.loginfo('mecanum_odom first counts: %s', counts)
            return

        dt = (now - self.last_time).to_sec()
        if dt <= 0:
            self.last_counts = counts
            self.last_time = now
            return

        deltas = [self.normalize_delta(counts[i], self.last_counts[i]) for i in range(4)]
        self.last_counts = counts
        self.last_time = now

        # forward command produced negative raw deltas; negate to make forward positive m_i
        m = [self.sign_forward * (-d) * self.mm_per_pulse for d in deltas]

        vx_robot = (m[0] + m[1] + m[2] + m[3]) / 4.0 / dt
        vy_robot = self.sign_strafe * (-m[0] + m[1] + m[2] - m[3]) / 4.0 / dt
        vp = self.sign_rotation * (-m[0] + m[1] - m[2] + m[3]) / 4.0
        omega = vp / self.base_width / dt

        # integrate in odom frame
        # Exact arc integration over the step (theta goes from th1 to th2):
        #   dx = (vx/w)(sin th2 - sin th1) + (vy/w)(cos th2 - cos th1)
        #   dy = -(vx/w)(cos th2 - cos th1) + (vy/w)(sin th2 - sin th1)
        # The vy term in dx must be PLUS. (An earlier version had it as minus,
        # which silently corrupted every strafe/rotation step while leaving
        # pure-forward motion looking correct.)
        if abs(omega) * dt < 1e-9:
            self.x += (vx_robot * math.cos(self.theta) - vy_robot * math.sin(self.theta)) * dt
            self.y += (vx_robot * math.sin(self.theta) + vy_robot * math.cos(self.theta)) * dt
            self.theta += omega * dt
        else:
            k = vx_robot / omega
            l = vy_robot / omega
            th_new = self.theta + omega * dt
            self.x += (k * (math.sin(th_new) - math.sin(self.theta))
                       + l * (math.cos(th_new) - math.cos(self.theta)))
            self.y += (k * (-math.cos(th_new) + math.cos(self.theta))
                       + l * (math.sin(th_new) - math.sin(self.theta)))
            self.theta = th_new

        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # publish Odometry
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id
        odom.pose.pose.position.x = self.x / 1000.0
        odom.pose.pose.position.y = self.y / 1000.0
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = euler_to_quaternion(0, 0, self.theta)
        odom.twist.twist.linear.x = vx_robot / 1000.0
        odom.twist.twist.linear.y = vy_robot / 1000.0
        odom.twist.twist.angular.z = omega
        odom.pose.covariance = [0.01, 0, 0, 0, 0, 0,
                                0, 0.01, 0, 0, 0, 0,
                                0, 0, 1e6, 0, 0, 0,
                                0, 0, 0, 1e6, 0, 0,
                                0, 0, 0, 0, 1e6, 0,
                                0, 0, 0, 0, 0, 0.05]
        odom.twist.covariance = [0.005, 0, 0, 0, 0, 0,
                                  0, 0.005, 0, 0, 0, 0,
                                  0, 0, 1e6, 0, 0, 0,
                                  0, 0, 0, 1e6, 0, 0,
                                  0, 0, 0, 0, 1e6, 0,
                                  0, 0, 0, 0, 0, 0.02]
        self.odom_pub.publish(odom)

        # publish tf
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = self.frame_id
        t.transform.translation.x = self.x / 1000.0
        t.transform.translation.y = self.y / 1000.0
        t.transform.translation.z = 0.0
        t.transform.rotation = odom.pose.pose.orientation
        t.child_frame_id = self.child_frame_id
        self.tf_broadcaster.sendTransform(t)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    node = MecanumOdom()
    node.run()
