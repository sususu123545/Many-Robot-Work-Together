#!/usr/bin/env bash
# Runs the grasp perception probe as a SIDECAR container that shares the live
# core container's IPC namespace.
#
# WHY --ipc container:<core> instead of --ipc host:
#   Fast DDS defaults to shared-memory transport for same-host peers. Each
#   container gets its OWN /dev/shm unless told otherwise, so a sidecar with
#   --ipc host matches the host namespace (which the core does not use) and the
#   graph resolves but ZERO payload arrives: `ros2 node list` shows every node,
#   while `ros2 topic echo --once` sits at the timeout. Sharing the core's IPC
#   namespace makes /dev/shm identical on both sides and data flows immediately.
#
# This starts NO core process and touches no device: the perception nodes only
# subscribe to topics the core already publishes.
#
# USAGE (from the repo root; the pair of files must be copied to the car first)
#   python tools/runtime/upload_via_exec.py \
#       tools/perception/grasp_probe.sh /home/pi/grasp_probe.sh \
#       tools/perception/run_probe_in_core_ipc.sh /home/pi/run_probe_in_core_ipc.sh
#   python tools/pi_run.py --host <car> --cmd 'bash /home/pi/run_probe_in_core_ipc.sh'
# Run run_arm_reset_in_core_ipc.sh first when the chain stays empty: arm_camera_tf
# will not publish while the arm reports estop.
set -u

CORE=$(sudo -n docker ps --filter label=armpi.owner=ros2-native-core --format '{{.Names}}' | head -1)
if [ -z "$CORE" ]; then
  echo "NO_CORE_CONTAINER"
  exit 2
fi
echo "CORE=$CORE"

# Take the image from the RUNNING core instead of hardcoding a tag. The probe
# launches grasp_vision.launch.py inside this sidecar, so its
# grasp_target_transform is the one under test; pinning a tag here once meant
# the probe kept exercising the previous generation of the code while the car
# ran the new one.
IMAGE=$(sudo -n docker inspect "$CORE" --format '{{.Config.Image}}')
echo "IMAGE=$IMAGE"
PROBE=/home/pi/grasp_probe.sh
CALIB=/home/pi/armpi-calibration

sudo -n docker run --rm --entrypoint /bin/bash \
  --network host \
  --ipc "container:$CORE" \
  -e ROS_DOMAIN_ID=42 \
  -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -v "$PROBE:/probe.sh:ro" \
  -v "$CALIB:/opt/armpi-calibration:ro" \
  "$IMAGE" -lc 'bash /probe.sh'
