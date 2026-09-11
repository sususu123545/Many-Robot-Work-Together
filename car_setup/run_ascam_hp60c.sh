#!/bin/bash
# Launch the Angstrong HP60C depth camera node inside the armpi_pro container.
# Created 2026-09-11 by WorkBuddy.
#
# Inside the container:
#   bash /home/ubuntu/run_ascam_hp60c.sh > /tmp/ascam_run.log 2>&1
#
# Notes:
#  - roscore/rosmaster is already running (started by the boot chain), so we
#    launch ONLY the node, we do NOT start a second roscore.
#  - ROS_MASTER_URI / ROS_HOSTNAME are copied from armpi_pro's source_env.bash so
#    this node joins the SAME ROS graph as the rest of the robot.
#  - The vendor lib dir is put FIRST in LD_LIBRARY_PATH on purpose: the SDK ships
#    its own libjpeg/turbojpeg and CMake linked against those.

WS=/home/ubuntu/ascam/linux_ros/ros
LIBS=$WS/src/ascamera/libs/lib/aarch64-linux-gnu
CONF=$WS/src/ascamera/configurationfiles

export ROS_MASTER_URI=http://raspberrypi:11311
export ROS_HOSTNAME=raspberrypi

source /opt/ros/noetic/setup.bash
export LD_LIBRARY_PATH=$LIBS:$LD_LIBRARY_PATH
source $WS/devel/setup.bash

echo "=== env ==="
echo "ROS_MASTER_URI = $ROS_MASTER_URI"
echo "ROS_HOSTNAME   = $ROS_HOSTNAME"
echo "LD_LIBRARY_PATH= $LD_LIBRARY_PATH"
echo "roslaunch      = $(which roslaunch)"
echo "ascamera pkg   = $(rospack find ascamera)"
echo "config dir     = $CONF"
echo "=== ls /dev/video0 ==="
ls -l /dev/video0
echo "=== launching ==="

cd $WS
exec roslaunch ascamera hp60c.launch argPath:=$CONF
