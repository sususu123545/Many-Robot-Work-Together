#!/bin/bash
# ============================================================
# 轮式里程计 odom 节点启动脚本（幂等版 2026-09-18）
#
# 调用方: systemd odom.service（宿主侧，root 身份执行 docker exec）
#   docker exec -d -u ubuntu -w /home/ubuntu armpi_pro bash -lc \
#       'bash /home/ubuntu/run_odom.sh > /home/ubuntu/odom_run.log 2>&1'
#
# 前置: chassis_control_node.py 已在跑 —— 它**独家持有 I2C** 并发布
#       /chassis_control/encoder_counts；odom 节点只订阅该话题，
#       完全不碰 I2C（两个进程抢总线会 TimeoutError(110)）。
#       因此 odom 不需要 root / dialout / video 权限，以 ubuntu 身份跑即可。
#
# 幂等: 重复执行会先清掉旧实例再启动，不会出现同名节点互杀。
# ⚠️ 数进程用 pgrep -f "[o]dom_node.py"；直接写 pgrep -af odom_node.py
#    会匹配到执行这条命令的 shell 自己，导致"永远有进程"的假象。
# ============================================================

export ROS_MASTER_URI=http://raspberrypi:11311
export ROS_HOSTNAME=raspberrypi

# --- 1. 幂等：先清掉已在跑的实例 ------------------------------
if pgrep -f "[o]dom_node.py" >/dev/null 2>&1; then
    echo "=== 发现已在运行的 odom 实例，先清理 ==="
    pgrep -a -f "[o]dom_node.py" 2>/dev/null
    pkill -f "[o]dom_node.py" 2>/dev/null
    sleep 3
    pgrep -a -f "[o]dom_node.py" 2>/dev/null || echo "(旧实例已清)"
fi

# --- 2. 环境（与机器人其余节点同一 ROS 图） --------------------
source /opt/ros/noetic/setup.bash
source /home/ubuntu/armpi_pro/devel/setup.bash

# --- 3. 等编码器话题就绪（chassis 节点可能还在启动中）----------
# start_node.service 是 oneshot，它返回时 roslaunch 里的节点未必都已就绪，
# 所以这里主动等话题出现（最多 60s）。等不到也照样启动，节点会自行等待。
echo "=== 等 /chassis_control/encoder_counts（最多 60s）==="
i=0
while [ $i -lt 30 ]; do
    if rostopic list 2>/dev/null | grep -q "^/chassis_control/encoder_counts$"; then
        echo "话题已就绪（第 $i 次探测）"
        break
    fi
    i=$((i+1))
    sleep 2
done
if rostopic list 2>/dev/null | grep -q "^/chassis_control/encoder_counts$"; then
    echo "OK: /chassis_control/encoder_counts 存在"
else
    echo "WARN: 未等到 encoder_counts，仍启动 odom（节点会自行等待）"
fi

# --- 4. 启动 -------------------------------------------------
echo "=== launching odom_node.py ==="
cd /home/ubuntu
exec python3 -u /home/ubuntu/odom_node.py
