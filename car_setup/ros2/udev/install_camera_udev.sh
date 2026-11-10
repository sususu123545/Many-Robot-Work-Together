#!/bin/bash
# Pin the UVC camera to a stable device node.
#
# The icspring camera drops off the USB bus and comes back with different minor
# numbers (/dev/video1 and /dev/video2 instead of /dev/video0).  The core
# container maps one device path, so a re-enumeration silently removed the
# camera from the boot: `camera:=false`, no images, and every grasp attempt after
# that ran blind (2026-11-10, twice).
set -e
RULE=/etc/udev/rules.d/99-armpi-camera.rules
cat > /tmp/99-armpi-camera.rules <<'EOF'
# icSpring UVC camera: give the capture node a stable name.
# index 0 is the video capture function; index 1 is the metadata node.
SUBSYSTEM=="video4linux", ATTRS{idVendor}=="32e6", ATTRS{idProduct}=="9005", ATTR{index}=="0", SYMLINK+="armpi_cam"
EOF
cp /tmp/99-armpi-camera.rules "$RULE"
udevadm control --reload-rules
udevadm trigger --subsystem-match=video4linux
sleep 2
echo "--- 规则 ---"
cat "$RULE"
echo "--- 符号链接 ---"
ls -l /dev/armpi_cam 2>&1 || echo "armpi_cam 还没出现"
