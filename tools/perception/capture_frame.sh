#!/usr/bin/env bash
# Grab one frame from the core's wrist camera into /home/pi/snapshots/frame.png
# on the CAR (the sidecar is --rm, so the image has to land on a host mount).
#
# USAGE (repo root):
#   python tools/runtime/upload_via_exec.py \
#       tools/perception/snapshot.py /home/pi/snapshot.py \
#       tools/perception/capture_frame.sh /home/pi/capture_frame.sh
#   python tools/pi_run.py --host <car> --cmd 'bash /home/pi/capture_frame.sh'
#   python tools/runtime/download_via_exec.py /home/pi/snapshots/frame.png snapshots/frame.png
set -u

CORE=$(sudo -n docker ps --filter label=armpi.owner=ros2-native-core --format '{{.Names}}' | head -1)
if [ -z "$CORE" ]; then
  echo "NO_CORE_CONTAINER"
  exit 2
fi
IMAGE=$(sudo -n docker inspect "$CORE" --format '{{.Config.Image}}')
mkdir -p /home/pi/snapshots

sudo -n docker run --rm --entrypoint /bin/bash \
  --network host \
  --ipc "container:$CORE" \
  -e ROS_DOMAIN_ID=42 \
  -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -v /home/pi/snapshot.py:/snapshot.py:ro \
  -v /home/pi/snapshots:/snapshots \
  "$IMAGE" -lc 'set +u; source /opt/armpi_ros2/install/setup.bash; python3 /snapshot.py /snapshots/frame.png'
