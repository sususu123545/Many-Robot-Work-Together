#!/bin/bash
set -eo pipefail
source /opt/ros/noetic/setup.bash
source /home/ubuntu/armpi_pro/devel/setup.bash
export ROS_MASTER_URI=http://raspberrypi:11311
# ROS1 is retained ONLY for the optional camera/video/legacy diagnostic bridge.
# No ROS1 chassis, arm, lidar, IMU, odometry or autonomous-task launch here.
if ! pgrep -f '[r]osmaster' >/dev/null; then
    roscore >/home/ubuntu/roscore_run.log 2>&1 &
fi
for i in {1..30}; do
    if rosparam get /run_id >/dev/null 2>&1; then break; fi
    sleep 1
done
rosparam get /run_id >/dev/null
camera=false
if [ -e /dev/video0 ]; then camera=true; fi
exec roslaunch /home/ubuntu/armpi_ros2_legacy_sidecar.launch camera:="$camera"
