#!/bin/bash
# ============================================================
# LD06 激光雷达启动脚本（幂等版 2026-09-11）
#
# 用法（宿主侧，root 身份，不带 -u ubuntu —— ubuntu 不在 dialout 组）:
#   docker exec -d -w /home/ubuntu armpi_pro bash -lc \
#       'bash /home/ubuntu/run_ld06.sh > /tmp/ld06_run.log 2>&1'
#
# 幂等: 重复执行会先清掉旧实例再启动，不会出现同名节点互杀。
# ⚠️ 数进程不要用 pgrep -x ldlidar_stl_ros_node —— Linux 进程名
#    截断到 15 字符（变成 ldlidar_stl_ros），-x 永远匹配不上。
#    用: pgrep -f "[l]dlidar_stl_ros_node"
# ============================================================

WS=/home/ubuntu/ld06_ws

# --- 1. 幂等：先清掉已在跑的实例 ------------------------------
if pgrep -f "[l]d06.launch" >/dev/null 2>&1 || pgrep -f "[l]dlidar_stl_ros_node" >/dev/null 2>&1; then
    echo "=== 发现已在运行的实例，先清理 ==="
    pgrep -a -f "[l]d06.launch"           2>/dev/null
    pgrep -a -f "[l]dlidar_stl_ros_node"  2>/dev/null
    pkill -f "[l]d06.launch"          2>/dev/null
    pkill -f "[l]dlidar_stl_ros_node" 2>/dev/null
    sleep 3
    echo "=== 清理后剩余 ==="
    pgrep -a -f "[l]d06.launch"          2>/dev/null || echo "(roslaunch 已清)"
    pgrep -a -f "[l]dlidar_stl_ros_node" 2>/dev/null || echo "(ldlidar 节点已清)"
fi

# --- 2. 环境（与机器人其余节点同一 ROS 图） --------------------
export ROS_MASTER_URI=http://raspberrypi:11311
export ROS_HOSTNAME=raspberrypi

source /opt/ros/noetic/setup.bash
source $WS/devel/setup.bash

# --- 3. 等设备和 ROS Master 完全稳定 ---------------------------
# 开机时 /dev/ttyUSB0 可能先枚举、后建立稳定串口；同时 roscore
# 可能在 start_node 中刚启动或刚换 run_id。只检查 pgrep 会让
# roslaunch 在 run_id 切换窗口注册，产生“run_id 不匹配”并退出。
echo "=== 等待 LD06 设备和稳定 ROS run_id（最多 180s）==="
stable_run_id=''
stable_count=0
i=0
while [ "$i" -lt 90 ]; do
    PORT=/dev/ld06
    [ -e "$PORT" ] || PORT=/dev/ttyUSB0
    run_id=$(rosparam get /run_id 2>/dev/null | tr -d '[:space:]' || true)
    if [ -e "$PORT" ] && [ -n "$run_id" ]; then
        if [ "$run_id" = "$stable_run_id" ]; then
            stable_count=$((stable_count + 1))
        else
            stable_run_id="$run_id"
            stable_count=1
        fi
        if [ "$stable_count" -ge 3 ]; then
            echo "ROS run_id stable: $run_id"
            break
        fi
    else
        stable_count=0
        stable_run_id=''
    fi
    i=$((i + 1))
    sleep 2
done
if [ "$stable_count" -lt 3 ]; then
    echo "LD06 device or stable ROS run_id not ready; retrying via systemd" >&2
    exit 1
fi

# --- 4. 设备检查（串口是 root:dialout，本脚本必须 root 跑）----
echo "=== /dev/ttyUSB* ==="
ls -l /dev/ttyUSB* 2>/dev/null || echo "(没有 /dev/ttyUSB* 节点)"
echo "=== /dev/ld06 (udev 固定名, 见 /etc/udev/rules.d/99-ld06.rules) ==="
ls -l /dev/ld06 2>/dev/null || echo "(/dev/ld06 不存在 → 回退 ttyUSB0)"

# 固定端口名优先; 没有就回退 ttyUSB0
PORT=/dev/ld06
[ -e "$PORT" ] || PORT=/dev/ttyUSB0
echo "=== 使用端口: $PORT ==="

echo "=== launching ld06.launch (topic=/scan 230400) ==="
cd $WS
exec roslaunch ldlidar_stl_ros ld06.launch port_name:=$PORT
