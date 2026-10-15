#!/bin/bash
# Install the ROS 2 camera driver into the pinned image's build container.
# Runs INSIDE armpi-cam-build.
#
# Lessons baked in here, each earned in this session:
#  1. Signed-By in ros2.sources is an INLINE PGP block. Rebuilding the stanza from
#     a grep of line 1 drops the key -> "NO_PUBKEY ... repository is not signed",
#     which looks like a mirror fault but is self-inflicted. sed the URI only.
#  2. BOTH package servers are unusable at their defaults from this network:
#     packages.ros.org ~50 KB/s (2 MB index never completes) and ports.ubuntu.com
#     answers "Undetermined Error" on most .deb fetches. Both are repointed.
#  3. The image_transport_plugins metapackage drags in the full OpenCV and VTK9
#     stacks. Only compressed_image_transport is actually needed to give
#     rosbridge a JPEG topic, so the single package is installed instead.
# All three temporary source files are restored before commit, so the committed
# image keeps its original apt configuration.
set -u

ROS_SRC=/usr/share/ros-apt-source/ros2.sources
UBUNTU_SRC=/etc/apt/sources.list.d/ubuntu.sources
UBUNTU_BAK=/tmp/ubuntu.sources.orig

echo "=== disk before ==="
df -h / | tail -1

echo "=== repoint ubuntu ports mirror ==="
cp -a "$UBUNTU_SRC" "$UBUNTU_BAK"
sed -i 's#http://ports.ubuntu.com/ubuntu-ports/#https://mirrors.aliyun.com/ubuntu-ports/#' "$UBUNTU_SRC"
grep -v '^ ' "$UBUNTU_SRC" | head -8

echo "=== repoint ros mirror ==="
rm -f /etc/apt/sources.list.d/ros2.sources
sed -e 's#^Types: deb deb-src#Types: deb#' \
    -e 's#http://packages.ros.org/ros2/ubuntu#https://mirrors.aliyun.com/ros2/ubuntu#' \
    "$ROS_SRC" > /etc/apt/sources.list.d/ros2.sources
grep -v '^ ' /etc/apt/sources.list.d/ros2.sources

echo "=== apt-get update ==="
apt-get update -q -o Acquire::Retries=3 2>&1 | tail -6

PKGS="ros-jazzy-v4l2-camera ros-jazzy-compressed-image-transport"
echo "=== simulate (sizes) ==="
apt-get -s install --no-install-recommends $PKGS 2>&1 | grep -E '^(Need to get|After this operation|The following NEW|E:)' | head -6

echo "=== policy ==="
apt-cache policy $PKGS 2>&1 | grep -E '^ros-|Candidate:|Installed:' | head -10

echo "=== install ==="
export DEBIAN_FRONTEND=noninteractive
apt-get install -y --no-install-recommends -o Acquire::Retries=5 -o Acquire::http::Timeout=120 $PKGS 2>&1 | tail -14
echo "INSTALL_RC=${PIPESTATUS[0]}"

echo "=== restore apt sources ==="
cp -a "$UBUNTU_BAK" "$UBUNTU_SRC"
rm -f /etc/apt/sources.list.d/ros2.sources
ln -s "$ROS_SRC" /etc/apt/sources.list.d/ros2.sources
grep -v '^ ' "$UBUNTU_SRC" | grep URIs
ls -la /etc/apt/sources.list.d/

echo "=== verify ==="
source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
ros2 pkg list 2>/dev/null | grep -E '^(v4l2_camera|compressed_image_transport)$' || echo PKG_GREP_EMPTY
ls /opt/ros/jazzy/lib/ | grep -iE 'compressed|v4l2' || echo NO_SO
echo "=== disk after ==="
df -h / | tail -1
echo "CAM_CAMPAIGN_DONE"
