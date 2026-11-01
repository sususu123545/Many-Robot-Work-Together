#!/usr/bin/env bash
# Runs target_stability.py in a sidecar sharing the live core's IPC namespace,
# alongside grasp_vision.launch.py so red_ball_detector + arm_camera_tf +
# grasp_target_transform are all publishing.
#
# Needs the arm in READY: arm_camera_tf refuses to publish arm_base->rear_camera
# while estop is latched, and then every target_base sample is missing. Run
# run_arm_reset_in_core_ipc.sh first.
#
# USAGE (repo root):
#   python tools/runtime/upload_via_exec.py \
#       tools/perception/target_stability.py /home/pi/target_stability.py \
#       tools/perception/measure_target_stability.sh /home/pi/measure_target_stability.sh
#   python tools/pi_run.py --host <car> --cmd 'bash /home/pi/measure_target_stability.sh'
set +u

CORE=$(sudo -n docker ps --filter label=armpi.owner=ros2-native-core --format '{{.Names}}' | head -1)
if [ -z "$CORE" ]; then
  echo "NO_CORE_CONTAINER"
  exit 2
fi
echo "CORE=$CORE"
IMAGE=$(sudo -n docker inspect "$CORE" --format '{{.Config.Image}}')
echo "IMAGE=$IMAGE"

sudo -n docker run --rm --entrypoint /bin/bash \
  --network host \
  --ipc "container:$CORE" \
  -e ROS_DOMAIN_ID=42 \
  -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -v /home/pi/target_stability.py:/stability.py:ro \
  -v /home/pi/armpi-calibration:/opt/armpi-calibration:ro \
  "$IMAGE" -lc '
    set +u
    source /opt/armpi_ros2/install/setup.bash
    ros2 launch armpi_ros2 grasp_vision.launch.py > /tmp/stability.log 2>&1 &
    LAUNCH_PID=$!
    sleep 8
    python3 /stability.py
    kill -INT $LAUNCH_PID 2>/dev/null
    sleep 2
    pkill -f grasp_vision.launch.py 2>/dev/null
    pkill -f armpi_ros2/red_ball_detector 2>/dev/null
    pkill -f grasp_target_transform 2>/dev/null
    pkill -f arm_camera_transform 2>/dev/null
  '
