#!/bin/bash
# run_slam_toolbox.sh — 容器内启动 slam_toolbox 建图 (幂等)
# 用法:
#   docker exec -d -u ubuntu -w /home/ubuntu armpi_pro bash -lc \
#     'bash /home/ubuntu/run_slam_toolbox.sh > /home/ubuntu/slam_toolbox_run.log 2>&1'
#
# ⚠️ 三个必须:
#   1. 必须带 ROS_MASTER_URI=http://raspberrypi:11311
#      (不带会连 localhost:11311 失败, 节点安静退出, 表现为"什么都没发生")
#   2. 必须先用 bash <脚本> 调用 (脚本可能没 x 权限)
#   3. 必须与 hector 互斥! 两者都发 map->odom, 同时跑会打架
#      → 本脚本启动前自动停掉 hector

export ROS_MASTER_URI=http://raspberrypi:11311
export ROS_HOSTNAME=raspberrypi

source /opt/ros/noetic/setup.bash
source /home/ubuntu/armpi_pro/devel/setup.bash

LAUNCH=/home/ubuntu/ld06_ws/src/ldlidar_stl_ros/launch/ld06_slam_toolbox.launch

echo "=== run_slam_toolbox.sh $(date '+%F %T') ==="

# ---- 1. 先停 hector (互斥) ----
if pgrep -f "[h]ector_mapping" >/dev/null 2>&1 || pgrep -f "[l]d06_hector.launch" >/dev/null 2>&1; then
    echo "[1/3] 检测到 hector 在跑, 先停掉 (两者互斥)"
    pkill -f "[l]d06_hector.launch" 2>/dev/null
    pkill -f "[h]ector_mapping" 2>/dev/null
    sleep 4
fi
echo "[1/3] hector 已清: $(pgrep -f '[h]ector_mapping' >/dev/null 2>&1 && echo '仍在!' || echo 'OK')"

# ---- 2. 清掉旧的 slam_toolbox 实例 (幂等) ----
if pgrep -f "[a]sync_slam_toolbox_node" >/dev/null 2>&1; then
    echo "[2/3] 发现旧 slam_toolbox 实例, 清理"
    pgrep -a -f "[a]sync_slam_toolbox_node" 2>/dev/null
    pkill -f "[a]sync_slam_toolbox_node" 2>/dev/null
    pkill -f "[l]d06_slam_toolbox.launch" 2>/dev/null
    sleep 4
fi
echo "[2/3] 旧实例已清: $(pgrep -f '[a]sync_slam_toolbox_node' >/dev/null 2>&1 && echo '仍在!' || echo 'OK')"

# ---- 3. 等 /scan + /odom 就绪后启动 ----
i=0
while [ $i -lt 30 ]; do
    if rostopic list 2>/dev/null | grep -q "^/scan$" && \
       rostopic list 2>/dev/null | grep -q "^/odom$"; then
        echo "[3/3] /scan + /odom 就绪 (等了 $((i*2))s)"
        break
    fi
    i=$((i+1))
    sleep 2
done

if [ $i -ge 30 ]; then
    echo "[3/3] ⚠️ 等 /scan 或 /odom 超时 60s, 仍尝试启动"
fi

echo "[3/3] 启动 slam_toolbox ..."
# ⚠️ 必须用 setsid 脱离当前进程组:
#    docker exec -d 在会话结束时会向进程组发信号, 直接跑 roslaunch 会被一起带走
#    (症状: 日志出现 "started with pid [N]" 紧跟 "killing on exit", 进程消失)
exec setsid roslaunch "$LAUNCH"
