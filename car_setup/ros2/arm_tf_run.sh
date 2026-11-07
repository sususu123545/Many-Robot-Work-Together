#!/bin/sh
# Start the TF publisher as a sidecar, the same way frame_relay runs.
#
# Why a sidecar and not part of the core launch: adding a node to
# armpi_ros2.service means rebuilding/redeploying the vendor image, and a bad
# node in that launch takes the whole ROS graph down with it (measured: any one
# node dying deactivates the unit).  A sidecar shares the core's network and IPC
# namespace, subscribes to the same topics, and cannot hurt anything if it dies.
#
# Usage: sh /home/pi/arm_tf/run.sh
set -eu

CONTAINER=arm_tf_publisher
MOUNT_DIR=/home/pi/arm_tf
CALIB_DIR=/var/lib/armpi-ros2/calibration

CORE=$(docker ps --format '{{.Names}} {{.ID}}' | awk '/^armpi_ros2_native /{print $2; exit}')
if [ -z "${CORE:-}" ]; then
    echo "core container armpi_ros2_native is not running" >&2
    exit 1
fi
IMG=$(docker inspect "$CORE" --format '{{.Config.Image}}')
echo "core=$CORE image=$IMG"

docker rm -f "$CONTAINER" >/dev/null 2>&1 || true
docker run -d --name "$CONTAINER" \
    --network host \
    --ipc "container:$CORE" \
    -v "$MOUNT_DIR":/tf_pub:ro \
    -v "$CALIB_DIR":/opt/armpi-calibration:ro \
    "$IMG" bash -lc 'source /opt/armpi_ros2/install/setup.bash && exec python3 /tf_pub/arm_tf_publisher.py'

sleep 4
docker ps --filter "name=$CONTAINER" --format '{{.Names}} {{.Status}}'
echo '--- logs ---'
docker logs --tail 25 "$CONTAINER" 2>&1
