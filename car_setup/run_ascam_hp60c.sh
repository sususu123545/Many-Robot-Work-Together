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
#  - MUST run as root (docker exec without -u ubuntu): the 'ubuntu' user is not in
#    the 'video' group, so it gets "uvc_open:Access denied" / "open camera, ret: -82".
#
# IDEMPOTENT (2026-09-11): running this twice used to start a SECOND ascamera_node,
# which then fought the first one over /dev/video0:
#     [ERROR] [UvcCamera.cpp] uvc_open:Busy
#     [ERROR] open camera, ret: -82
#     [ WARN] Shutdown request received.
#     [ WARN] Reason given for shutdown: [[/ascamera_hp60c] Reason: new node registered with same name]
#      => the old node gets killed, the new one can't open the device, camera dies.
# So we always clean up any existing instance first. Safe to run repeatedly.

WS=/home/ubuntu/ascam/linux_ros/ros
LIBS=$WS/src/ascamera/libs/lib/aarch64-linux-gnu
CONF=$WS/src/ascamera/configurationfiles

# --- 1. 幂等：先清掉已在跑的实例，避免抢设备 / 同名节点互杀 ---------------
if pgrep -f "[h]p60c.launch" >/dev/null 2>&1 || pgrep -x ascamera_node >/dev/null 2>&1; then
    echo "=== 发现已在运行的实例，先清理 ==="
    pgrep -a -f "[h]p60c.launch" 2>/dev/null
    pgrep -a -x ascamera_node   2>/dev/null
    pkill -f "[h]p60c.launch" 2>/dev/null
    pkill -x ascamera_node    2>/dev/null
    sleep 4
    echo "=== 清理后剩余 ==="
    pgrep -a -f "[h]p60c.launch" 2>/dev/null || echo "(roslaunch 已清)"
    pgrep -a -x ascamera_node   2>/dev/null || echo "(ascamera_node 已清)"
fi

# --- 2. 环境 ------------------------------------------------------------
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

# 设备节点编号会变（并不总是 /dev/video0），所以列表全打，不要只 look video0
echo "=== /dev/video* ==="
ls -l /dev/video* 2>/dev/null | head -20 || echo "(没有 /dev/video* 节点)"

echo "=== USB 里的相机 ==="
lsusb 2>/dev/null | grep 3482 || echo "(未在 USB 上看到 3482:6723)"

echo "=== launching ==="

cd $WS
exec roslaunch ascamera hp60c.launch argPath:=$CONF
