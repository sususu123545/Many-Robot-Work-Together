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
CORE=armpi_ros2_native
MOUNT_DIR=/home/pi/arm_tf
CALIB_DIR=/var/lib/armpi-ros2/calibration
WAIT_SECONDS=${WAIT_SECONDS:-120}

# The core is recreated on every restart, and a sidecar that joins it with
# --ipc container:<name> must not even try until the NEW container exists.
# Measured 2026-11-07: systemd's PartOf restart fires both units at once, the
# sidecar found no running core, exited 1 and stayed dead -- the TF tree then
# silently had no arm_base while every other health check looked fine.
waited=0
while [ "$waited" -lt "$WAIT_SECONDS" ]; do
    if docker inspect "$CORE" --format '{{.State.Running}}' 2>/dev/null | grep -q true; then
        break
    fi
    sleep 3
    waited=$((waited + 3))
done
if ! docker inspect "$CORE" --format '{{.State.Running}}' 2>/dev/null | grep -q true; then
    echo "core container $CORE did not come up within ${WAIT_SECONDS}s" >&2
    exit 1
fi

IMG=$(docker inspect "$CORE" --format '{{.Config.Image}}')
echo "core=$CORE image=$IMG (waited ${waited}s)"

# A killed `docker run` client leaves its container behind, and --name then
# conflicts on the next start. Clear it unconditionally.
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
