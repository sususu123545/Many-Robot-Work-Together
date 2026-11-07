# ARMPI ROS2 开发规范

> 本文件由维护者整理，记录本机（树莓派 ARMPI 小车）ROS2 核心的架构、改动流程与硬性约束。
> 容器内 `/opt/armpi_ros2/DEVELOPMENT.md` 为副本；**以宿主机 `/opt/armpi-ros2/DEVELOPMENT.md` 为准**（容器重建会丢失容器内文件）。

---

## 1. 系统架构

```
systemd: armpi-ros2.service
  └─ /opt/armpi-ros2/run_native_core.py
       ├─ 预检 1: prepare_boot_arm()  → joint3 超限时归位（仅超 /dev/rrc）
       ├─ 预检 2: prepare_lidar()     → 读 /dev/ld06，15s 内需 5 个 CRC 有效包，否则拒绝启动
       └─ docker run --name armpi_ros2_native  (固定名，启动前自动 rm -f 同名残留)
            └─ ros2 launch armpi_ros2 base.launch.py
```

**核心特性：容器无状态。** 每次启动都从镜像重建，容器内的任何文件改动在重启后全部丢失。

- 镜像 pin 在 `/etc/armpi-ros2/core.json` 的 `image` + `image_id`
- `run_native_core.py` 启动时**强校验** `docker image inspect` 的 ID 必须等于 `image_id`，不匹配直接拒绝启动
- 总开关：`/etc/armpi-ros2/active` 存在才启动（`touch` 启用，删除停用）

---

## 2. 代码修改流程（必须严格遵守）

```bash
# 1) 进容器改源码
sudo docker exec -it armpi_ros2_native bash
cd /opt/armpi_ros2/src/armpi_ros2

# 2) 编译
source /opt/ros/jazzy/setup.bash
cd /opt/armpi_ros2 && colcon build --packages-select armpi_ros2

# 3) 跑测试（必须全绿）
cd /opt/armpi_ros2/src/armpi_ros2
PYTHONPATH=/opt/armpi_ros2/src/armpi_ros2:$PYTHONPATH python3 -m pytest test/ -q

# 4) 固化镜像（在宿主机执行）
NEWTAG="armpi-ros2-migration:$(date +%Y%m%d-%H%M)"
sudo docker commit armpi_ros2_native $NEWTAG
sudo docker image inspect $NEWTAG --format '{{.Id}}'

# 5) 更新 core.json（两处都要改：image 和 image_id）
sudo python3 -c "
import json
p='/etc/armpi-ros2/core.json'
d=json.load(open(p)); d['image']='$NEWTAG'
d['image_id']='<上一步输出的 sha256>'
json.dump(d,open(p,'w')); print(d)"

# 6) 重启生效
sudo systemctl restart armpi-ros2
```

**跳过第 5 步 = 服务起不来**（ID 强校验会拦截）。

---

## 3. 硬性约束（代码里写死的，改配置突破不了）

| 约束 | 值 | 位置 |
|---|---|---|
| 温度上限 | `1 <= max_temperature <= 65` | `arm_node.py:40` |
| 单步位移 | `1 <= max_step_ticks <= 100` tick | `arm_node.py:40` |
| 指令时长 | `0.3 <= duration <= 5` 秒 | `on_command` |
| 时钟戳窗口 | `-0.1 <= now - stamp <= 1` 秒 | `on_command` |
| 恢复等待 | 连续健康 ≥ 10 秒才能 reset | `arm_health.py` |
| 运动授权 | 每次启动默认 false | `base.yaml` |

### 急停与故障策略（2026-10-29 修订）
- **温度故障** → 立即锁存急停（不妥协）
- **通信抖动** → 连续 3 次才锁存；单次只中断动作、不锁急停
- 单次串口读超时 `0.3s`，最多重试 3 次

### 运动授权（每次重启都要重新开）
```bash
ros2 param set /arm_controller enable_arm_motion true
ros2 param set /arm_controller enable_gripper_motion true
```
前置条件：`estop=false` 且 feedback 健康。
⚠️ `validate_permissions` 含 `pending` 判断，轮询占掉一半时间窗口，**单次调用容易失败，需重试**。

---

## 4. 关节参数表

| 关节 | 舵机ID | centre | sign | 限位 | 说明 |
|---|---|---|---|---|---|
| joint1 | 6 | 500 | +1 | 0–1000 | 底座旋转 |
| joint2 | 5 | 500 | +1 | 125–875 | 大臂 |
| joint3 | 4 | 500 | **−1** | 0–1000 | 小臂，**承载整臂自重** |
| joint4 | 3 | 500 | +1 | 0–1000 | 腕俯仰 |
| joint5 | 2 | 500 | +1 | 0–1000 | 腕旋转 |
| r_joint | 1 | 700 | +1 | 0–700 | 夹爪 |

**角度换算**：
```python
tick = radians(180)/1000 if joint == 'r_joint' else radians(240)/1000
radians = sign * (raw - centre) * tick
```
⚠️ r_joint 的合法角度是 **−126° ~ 0°**，全为负值 —— 这是设计使然，不是故障。

---

## 5. 串口协议（RRC 舵机总线）

```
帧格式: AA 55 | 功能码 | 长度 | 载荷 | CRC8
读位置: 0x05  载荷 '<BBbh'  (4B)
读温度: 0x09  载荷 '<BBbB'  (4B)
读限位: 0x32  载荷 '<BBbHH' (6B)
```
载荷结构统一为 `[舵机ID, 命令码, 保留位, 数据...]`，`read()` 内部已裁掉前三字节。

---

## 6. 已知坑（都踩过）

1. **`docker save` 需要额外等于镜像大小的临时空间**（`/var/lib/docker/tmp`）。1.12GB 镜像要先留 1.12GB，否则磁盘打满、命令卡死。
2. **重启容器时相机 `v4l2_camera_node` 会 SIGABRT（exit −6）**，因为 `/dev/video0` 未释放，进而拖垮整个容器。等设备释放后重试即可。
3. **rosbridge 9091 只绑 `127.0.0.1`**，外部连不上，必须 SSH 隧道。
4. **rosbridge 的 `advertise` 到真正能 publish 有约 2 秒延迟**。立即 publish 会被静默丢弃，且目标节点连 reject 日志都不会打。
5. **串口抖动频繁**（实测约每 56 秒一次 `No physical reply from servo N`），这是硬件/固件层面的抖动，不是代码问题。
6. **树莓派开机后前 5 分钟 SSH 可能未就绪**，会误报 `Permission denied`，别急着判定密码错误。
7. **机械臂无支撑时会下垂，joint3 越过 1000 触发超限**，导致开机归位流程失败并留下 `ok:false` 标记，此后每次启动都拒绝 —— 需删除 `/run/armpi-ros2/arm-bootstrap.json` 并确保周围安全后重试。

---

## 7. 常用命令

```bash
# 服务
systemctl status armpi-ros2
sudo systemctl restart armpi-ros2

# 容器
sudo docker ps -a
sudo docker logs -f armpi_ros2_native
sudo docker exec -it armpi_ros2_native bash

# 状态
ros2 topic echo /arm_controller/status --once
ros2 topic echo /joint_states --once
ros2 node list

# 解除急停
ros2 service call /arm_controller/reset_estop std_srvs/srv/Trigger

# 切到 ROS1 环境（两年前的工作容器）
sudo docker start -ai armpi_pro
```

---

## 8. 禁止事项

1. **禁止删除 `armpi_pro` 容器** —— 内含 5.3GB、约两年的 ROS1 工作数据，删除不可恢复。
2. **禁止只在容器内改文件而不固化成镜像** —— 重启即丢失。
3. **禁止跳过 `core.json` 的 `image_id` 更新** —— 服务会因强校验失败而拒绝启动。
4. **禁止在无支撑状态下让机械臂下垂** —— joint3 会超限，触发归位流程失败。
5. **禁止把 `max_temperature` 视为可随意调高的参数** —— 65 是代码硬上限，调整需改代码并评估安全风险。
6. **禁止在未确认周边安全时授权运动** —— `enable_arm_motion=true` 后机械臂会立即响应轨迹指令。

---

## 9. 当前状态（2026-11-02）

- 运行镜像：`armpi-ros2-migration:command-fix-20261101` (`sha256:00c8bb7402d7...`)
- 回滚镜像：`armpi-ros2-migration:stage-zjt99z`（2026-10-29 的急停策略修复版）
- 容器策略：固定名单实例 `armpi_ros2_native`，启动前自动清理残留
- 系统镜像总数：4（从 29 个清理而来）
