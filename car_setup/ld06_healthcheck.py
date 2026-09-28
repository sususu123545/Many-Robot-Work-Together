#!/usr/bin/env python3
"""Return success only after one real LaserScan message is received."""

import rospy
from sensor_msgs.msg import LaserScan


rospy.init_node('ld06_healthcheck', anonymous=True, disable_signals=True)
rospy.wait_for_message('/scan', LaserScan, timeout=4.0)
