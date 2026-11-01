#!/usr/bin/env bash
# Runs arm_reset_once.sh in a sidecar sharing the live core container's IPC
# namespace. See run_probe_in_core_ipc.sh for why --ipc container: is required
# and why --ipc host silently yields an empty graph payload.
set -u

CORE=$(sudo -n docker ps --filter label=armpi.owner=ros2-native-core --format '{{.Names}}' | head -1)
if [ -z "$CORE" ]; then
  echo "NO_CORE_CONTAINER"
  exit 2
fi
echo "CORE=$CORE"

IMAGE=$(sudo -n docker inspect "$CORE" --format '{{.Config.Image}}')

sudo -n docker run --rm --entrypoint /bin/bash \
  --network host \
  --ipc "container:$CORE" \
  -e ROS_DOMAIN_ID=42 \
  -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -v /home/pi/arm_reset_once.sh:/reset.sh:ro \
  "$IMAGE" -lc 'bash /reset.sh'
