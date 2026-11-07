#!/bin/bash
# One-shot acceptance run for the localisation stack.  Run INSIDE the container.
#
#   bash /tmp/amcl_acceptance.sh [DEGREES]
#
# Brings up map_server + amcl + lifecycle_manager, seeds a GLOBAL initial pose,
# turns the car (that is what makes the particle filter converge), and reports
# whether sigma collapsed and whether amcl's yaw tracked the wheels.
#
# Everything happens in one script on purpose: starting the stack by hand and
# then starting it again leaves TWO sets of same-named nodes in the graph, and
# they fight over /initialpose and /amcl_pose.  This script refuses to run if a
# stale node is left over, and cleans up after itself.
#
# Config lives on the HOST bind mount (/opt/armpi-maps) so tweaking a parameter
# never needs a new `docker commit`.
set +u

DEGREES=${1:-120}
CFG=/opt/armpi-ros2-localization
PARAMS=/opt/armpi-maps/nav2_localization_params.yaml

source /opt/ros/jazzy/setup.bash
source /opt/armpi_ros2/install/setup.bash

# `ps` truncates comm to 15 characters, so lifecycle_manager shows up as
# "lifecycle_manag".  Matching the full name silently never matches and lets a
# duplicate manager survive -- that bug cost a whole debugging round.
cleanup() {
    pkill -9 -x amcl 2>/dev/null
    pkill -9 -x map_server 2>/dev/null
    pkill -9 -x lifecycle_manag 2>/dev/null
}
cleanup
sleep 2
stale=$(ps -eo comm= | grep -cxE 'amcl|map_server|lifecycle_manag')
if [ "$stale" != "0" ]; then
    echo "REFUSING: $stale stale node(s) still in the graph."
    echo "Zombie nodes cannot be killed -- restart the core to clear them:"
    echo "  sudo systemctl restart armpi-ros2"
    exit 1
fi

timeout 150 ros2 launch "$CFG/nav2_localization.launch.py" \
    params_file:="$PARAMS" >/tmp/amcl_acceptance.out 2>&1 &
PID=$!

sleep 16
echo "map_server : $(ros2 lifecycle get /map_server 2>&1 | tail -1)"
echo "amcl       : $(ros2 lifecycle get /amcl 2>&1 | tail -1)"
echo "---"
timeout 90 python3 "$CFG/amcl_converge_test.py" --degrees "$DEGREES" 2>&1 | tail -14

kill -INT "$PID" 2>/dev/null
sleep 2
kill -9 "$PID" 2>/dev/null
cleanup
echo ACCEPTANCE_DONE
