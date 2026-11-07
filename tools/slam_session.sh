#!/bin/bash
# One mapping session, start to finish, with a live record of /map growth.
#
#   bash /tmp/slam_session.sh [SECONDS] [OUTPUT_PREFIX] [MAX_LASER_RANGE]
#
# Starts slam_toolbox against the real mapping params, activates the lifecycle
# node, records every /map frame for SECONDS, saves the final map, then stops
# the node.  Run this in the BACKGROUND and drive the survey from the PC while
# it watches -- that is the only way to see whether scans made it into the grid.
#
# Sends nothing to the wheels itself.
set +u

DURATION=${1:-100}
OUT=${2:-/tmp/workshop}
# 6 m, not 3 m.  Measured 2026-11-07: 26% of LD06 beams return nothing and the
# returns are mostly beyond 1 m, so clipping at 3 m throws away another 19% and
# shreds the free space into ~122 disconnected pieces (looks like radial
# streaks).  At 6 m the free space is ONE connected region and speckle is zero.
MAX_RANGE=${3:-6.0}

source /opt/ros/jazzy/setup.bash
source /opt/armpi_ros2/install/setup.bash 2>/dev/null || true
export ROS_LOG_DIR=/tmp/slam_logs
mkdir -p "$ROS_LOG_DIR"

rm -f /tmp/map_growth.log /tmp/slam_session.out /tmp/session_ready
pkill -f async_slam_toolbox_node 2>/dev/null
sleep 1

timeout $((DURATION + 40)) ros2 run slam_toolbox async_slam_toolbox_node \
    --ros-args --params-file /tmp/slam_toolbox_mapping.yaml \
    -p max_laser_range:=$MAX_RANGE \
    --log-level info >/tmp/slam_session.out 2>&1 &
PID=$!

sleep 9
echo "configure : $(ros2 lifecycle set /slam_toolbox configure 2>&1 | tail -1)"
echo "activate  : $(ros2 lifecycle set /slam_toolbox activate 2>&1 | tail -1)"
sleep 1
echo "state     : $(ros2 lifecycle get /slam_toolbox 2>&1 | tail -1)"
touch /tmp/session_ready

echo "=== watching /map for ${DURATION}s -- drive the survey now ==="
python3 /tmp/watch_map_growth.py --seconds "$DURATION" 2>&1 | tee /tmp/map_growth.log

echo "=== saving map ==="
python3 /tmp/save_map_in_container.py --output "$OUT" 2>&1 | tail -3
ls -l "$OUT".pgm "$OUT".yaml 2>&1

kill -INT "$PID" 2>/dev/null
sleep 2
kill -9 "$PID" 2>/dev/null
wait "$PID" 2>/dev/null
echo "SESSION DONE"
