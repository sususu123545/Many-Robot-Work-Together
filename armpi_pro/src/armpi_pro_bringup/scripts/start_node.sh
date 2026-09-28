#!/bin/bash

# ROS launch locates Python nodes by executable bit. Keep this true even when
# a source file was copied into the container with a mode-preserving tool off.
chmod +x /home/ubuntu/armpi_pro/src/chassis_control/scripts/chassis_control_node.py

# The container does not start roscore by itself, while odom/ld06 and every
# roslaunch below require the same master at raspberrypi:11311. Start one
# idempotently and wait for its rosmaster process before registering nodes.
if ! pgrep -f '[r]osmaster' > /dev/null 2>&1; then
    echo 'ROS master is not running; starting roscore.'
    roscore > /home/ubuntu/roscore_run.log 2>&1 &
    i=0
    while [ "$i" -lt 30 ] && ! pgrep -f '[r]osmaster' > /dev/null 2>&1; do
        i=$((i + 1))
        sleep 1
    done
fi
if ! pgrep -f '[r]osmaster' > /dev/null 2>&1; then
    echo 'ROS master failed to start; aborting node launch.' >&2
    exit 1
fi

roslaunch /home/ubuntu/armpi_pro/src/armpi_pro_bringup/launch/start_dependence.launch &
sleep 10
# The rear USB camera has one owner: this launch script.  Older boot attempts
# could leave two usb_cam_node processes behind, which made ROS unregister
# /rear_camera and caused VIDIOC_REQBUFS(Device or resource busy).  Clean up
# stale instances once, then keep the launch itself under a lock so a second
# start_node invocation cannot create another camera owner.
if [ -e /dev/video0 ]; then
    stale_camera_pids=$(pgrep -f '[u]sb_cam_node' || true)
    if [ -n "$stale_camera_pids" ]; then
        echo "Stopping stale USB camera processes: $stale_camera_pids"
        kill $stale_camera_pids 2>/dev/null || true
        sleep 2
        remaining_camera_pids=$(pgrep -f '[u]sb_cam_node' || true)
        if [ -n "$remaining_camera_pids" ]; then
            kill -9 $remaining_camera_pids 2>/dev/null || true
        fi
    fi
    (
        exec 9>/tmp/rear_camera.launch.lock
        if ! flock -n 9; then
            echo 'Rear camera launch already owned by another start_node instance.'
            exit 0
        fi
        exec roslaunch /home/ubuntu/armpi_pro/src/armpi_pro_bringup/launch/start_camera.launch camera_name:=rear_camera
    ) &
    sleep 5
else
    echo 'No /dev/video0 device; skipping optional camera launch.'
fi
roslaunch /home/ubuntu/armpi_pro/src/armpi_pro_bringup/launch/start_sensor.launch &
sleep 5
roslaunch /home/ubuntu/armpi_pro/src/armpi_pro_bringup/launch/start_functions.launch
