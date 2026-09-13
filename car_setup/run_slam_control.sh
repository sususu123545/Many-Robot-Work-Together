#!/bin/bash
# slam_control 启动脚本(幂等) — 由 slam-control.service 调用
# 注意: 幂等检查必须锚定 "^python3 /home/ubuntu/slam_control.py"。
# 不能用 '[s]lam_control.py' 这种宽松模式 —— 服务内联命令的自身 cmdline
# 里也含 "python3 /home/ubuntu/slam_control.py" 字样, 会误判"已在跑"而跳过启动
# (2026-09-13 首次开机自启就是踩了这个坑: 服务 active 但节点没起来, 节点日志为空)。
if pgrep -f '^python3 /home/ubuntu/slam_control.py' >/dev/null; then
    echo "slam_control already running, skip"
    exit 0
fi
source /opt/ros/noetic/setup.bash
source /home/ubuntu/ld06_ws/devel/setup.bash
source /home/ubuntu/armpi_pro/devel/setup.bash
export ROS_MASTER_URI=http://raspberrypi:11311
export ROS_HOSTNAME=raspberrypi
exec python3 /home/ubuntu/slam_control.py > /home/ubuntu/slam_control.log 2>&1
