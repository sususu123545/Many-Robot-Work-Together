#!/bin/bash
# 自动建图控制节点：启动后待命，不会自动移动。
# The systemd unit owns this process. A stale process from an earlier
# docker exec must be cleaned up during deployment; silently skipping here
# would leave the service inactive and keep old parameters in memory.
source /opt/ros/noetic/setup.bash
source /home/ubuntu/ld06_ws/devel/setup.bash
source /home/ubuntu/armpi_pro/devel/setup.bash
export ROS_MASTER_URI=http://raspberrypi:11311
export ROS_HOSTNAME=raspberrypi
if [ -f /home/ubuntu/auto_mapping.yaml ]; then
    rosparam load /home/ubuntu/auto_mapping.yaml /auto_mapping_control
fi
exec python3 /home/ubuntu/auto_mapping_control.py > /home/ubuntu/auto_mapping.log 2>&1
