# VLA 架构落地 · Round 1

> 依据 `docs/VLA_ROS2_论文规划.docx`（2026-11 方案讨论稿）搭建第一批 ROS 2 包。
> 本轮范围：**§5.1 的包骨架 + §5.3 的安全执行边界**，载体按现有 ArmPi Pro 实车。
> 日期：2026-11-03

## 1. 本轮交付

| 包 | 类型 | 内容 |
|---|---|---|
| `vla_interfaces` | ament_cmake | `VlaAction`、`VlaActionChunk`、`VlaObservation`、`VlaSafetyVerdict`、`GetPolicyInfo.srv`、`ExecuteActionChunk.action` |
| `vla_controller` | ament_python | 规范层 `spec`、反归一化 `normalization`、安全过滤 `safety`、动作块 `chunk`、底盘静止判定 `chassis`、几何适配 `adapter`、ROS 节点 `chunk_executor_node` |
| `robot_safety` | ament_python | 链路看门狗 `watchdog`、监督节点 `supervisor_node` |

测试：`vla_controller` 114 例、`robot_safety` 24 例，**全部可在无 ROS 2 的 Windows 本机运行**。

```bash
python3 tools/vla_offline_selftest.py        # 跑两套测试 + 打一张决策轨迹表
```

## 2. 与方案文档的六处落地差异

这些不是"没做完"，是按实车事实改的口径。每一条都写进了代码注释和配置注释。

### 2.1 控制频率：20 Hz → 0.35 s/步

文档 §3.4 建议 20 Hz 控制步长。实车做不到：一次命令就是一条**定时舵机报文**，
`armpi_ros2.servo_protocol.movement_payload` 只接受 `0.3..5.0 s`。20 Hz 会丢掉
6 个预测里的 5 个。`Limits` 在构造时就把 `control_dt` 校验在这个区间内，
所以这个错误配不出来。

→ 默认 `control_dt: 0.35`，隐含 TCP 速度上限 28.6 mm/s。

### 2.2 动作帧：`arm_base_link` → `arm_base`

`base.launch.py` 里**没有任何静态 TF 把 `base_link` 连到 `arm_base`**
（见 `armpi_ros2/arm_base` 相关注释）。写 `arm_base_link` 会得到一个不存在的帧，
而且 `tf2` 不会报错——它只是永远查不到。安全过滤对帧名做白名单校验，不匹配直接拒收。

### 2.3 7 维动作里有 2 维这台臂做不出来

厂商解算器吃 `(x, y, z, pitch)`，返回 4 个舵机 tick：

- `drx`（pitch，绕侧向 x 轴）→ **可实现**；
- `dry`（工具轴滚转）→ 在 `joint5` 上，而 `joint5` **不在位姿模型里**，必须保持不动 → **不可实现**；
- `drz`（偏航）→ `joint1` 由解算器**为了指向目标点**而选定，不是独立自由度 → **不可实现**。

`spec.REALIZABLE_MASK = (1,1,1,1,0,0,1)`。默认策略是**拒收**而不是悄悄丢掉：
一个"看起来在学旋转、其实什么都没学"的策略，比一个明显跑不通的策略危害大得多。
`allow_projection:=true` 才降级为投影，并在 `VlaSafetyVerdict.dropped_components` 里点名。

### 2.4 双相机 → 单目

文档 §3.2 的观测是 `{I_front, I_wrist, ...}`。实车目前只有 `/rear_camera`（尾部单目，
已做鱼眼等距标定，RMS 1.158 px）。`VlaObservation` 保留了 `front_image` 字段，
空数据表示"本平台没有场景相机"，与"相机死了"区分开。

### 2.5 底盘锁定是硬前置条件，不是提醒

底盘用现有麦轮（`/cmd_vel`，REP-103）。动作帧是 `arm_base`，底盘一动，
预测的每个增量都变成假的。所以 `ChassisGuard` 要求**两个独立证据**同时成立才放行：

- `/odom` 连续 `0.6 s` 低于 `4 mm/s` / `0.012 rad/s`；
- `/cmd_vel` 连续 `0.5 s` 没有非零命令。

单看一个都会骗人：麦轮在光滑地面打滑，里程计会低报运动；而"刚下命令、还没动"
比"已经在动"更危险。

### 2.6 没有 MoveIt Servo

文档 §2.1 L7 写的是 `MoveIt Servo + ros2_control`。现状是 `armpi_ros2` 的
`arm_controller` 直接吃单点 `JointTrajectory`，`max_step_ticks=100`，
串行写入 + `busy_until` 节流。本轮的安全层是**架在这条链上**的，
不假装下面有 Servo。要接 Servo 得先有 URDF/SRDF 和 `ros2_control` 硬件接口，
那是 `robot_description` 包的事，还没开始。

## 3. 安全层的两条语义

文档 §5.3 说"必要时拒绝执行而非简单截断"。代码里把这句话拆成了两类：

| 情况 | 处理 | 理由 |
|---|---|---|
| 单步位移 / 旋转超限 | **等比缩小**（scale） | 超了就是"慢一点"，方向要保住 |
| 缩小超过 4 倍（`min_scale=0.25`）仍超限 | **拒收** `speed` | 说明策略的时间尺度不对，缩到 1/10 只会永久爬行 |
| 帧不符 / 动作空间不符 / 计划过期 / 观测过期 / 工作空间外 / 不可实现分量 / 夹爪越界 | **拒收** | 这不是"数值偏大"，缩小只会把一个能发现的 bug 变成一个小 bug |

缩放是**整体等比**的。按分量各自 clamp 能让每个数都合规，代价是把直线路径掰弯——
35 mm 的球就是这么丢的。

`adapter` 里还有第二道：笛卡尔步长合规不等于**关节**步长合规。姿态不同，同样的
10 mm 可能让某个关节转 20 tick，也可能 80 tick。做法是按同一个因子逐级缩，
每级都重新 IK + 正解自检残差，直到所有关节都 ≤ `max_step_ticks` 且残差 ≤ 2 mm。

实测（臂展开空中位 `j1=495 j2=644 j3=696 j4=166`）：

```
+10 mm y    scale 1.00  down  residual 0.38 mm   j4 185  j3 707  j2 636  j1 496
+30 mm y    filter 先缩到 0.33，adapter 再 scale 1.00，末端 10 mm
+120 mm y   filter 拒收 speed（需要 0.083 的缩放）
roll 命令   filter 拒收 unrealizable（dry）
```

残差 0.4~1.0 mm 是舵机量化（1 tick = 0.24°）的下限，不是拟合误差。

## 4. 在这台车上怎么跑

```bash
# 1. 构建（车上，容器内）
cd /opt/armpi_ros2/ros2_ws   # 按实际源码路径
colcon build --packages-select vla_interfaces vla_controller robot_safety
source install/setup.bash

# 2. 离线自检（不动硬件）
python3 src/vla_controller/run_tests.py
python3 src/robot_safety/run_tests.py

# 3. 起执行器（不授权、不动臂）
ros2 launch vla_controller vla_controller.launch.py
ros2 topic echo /vla/executor_status      # 看 arm_ready / base_parked

# 4. 起监督器（默认只发 /vla/safety_estop，不接硬件急停）
ros2 launch robot_safety robot_safety.launch.py
ros2 topic echo /safety/watchdog_status
```

影子模式（不动硬件，只验证决策）：
`python3 tools/vla_offline_selftest.py` 打出的表就是影子模式的离线版本。

## 5. 验收清单

真机联调前必须逐条过：

- [ ] `arm_ready: true` 且 `arm_detail` 为空 —— 意味着 `enable_arm_motion` 已由人打开、
      `feedback_complete: true`、`dry_run: false`
- [ ] `base_parked: true`，且推一下车能立刻变 `false`（证明 `/odom` 真的在动）
- [ ] 发一条假 chunk，`/vla/safety_verdict` 有 `reason_code: ok`，`/arm_controller/command_result` 有 `accepted: true`
- [ ] 故意发 `frame: base_link` 的 chunk，必须在 goal 阶段被拒
- [ ] 故意发带 `drz` 的 chunk，必须 `reason_code: unrealizable`
- [ ] 拔掉 `/joint_states` 的发布者，`/vla/executor_status` 的 `pose_ready` 变 false 并在 1.5 s 内失败
- [ ] 只有在上面全过之后，才考虑把 `publish_to_hardware_estop:=true` 接到 `/safety/estop`

## 6. 明确没做

1. **夹爪控制**：`enable_gripper_output` 设为 `true` 会直接报错退出，不是桩。
   原因：`arm_controller` 一次只允许一条定时报文在飞，夹爪必须和臂**交替**下发；
   而且本机 `r_joint` 实测可动区间被压到 298..379 tick、向"开"方向走不动
   （见记忆/`tools/gripper_guarded.py`），先修硬件再说。
2. **`vla_ros2_bridge`**：模型推理服务客户端（HTTP/gRPC、序列队列、动作有效期）没写。
   本轮先把"模型输出之后"这一段做严。bridge 要等 SmolVLA 服务端形态定了再写。
3. **`vla_data_collection`**：episode 录制、LeRobot 转换。属于 S2。
4. **`robot_description` / `moveit_config`**：URDF/SRDF 不存在，TF 链缺
   `base_link → arm_base`。这是 MoveIt Servo 和碰撞检查的前置，也是 §5.3
   "关键轨迹必须经过碰撞/可达性检查"目前只能给 IK+舵机限位的原因。
5. **论文部分**：§7 的三条主线一个字没写。

## 7. 下一步建议

按依赖顺序，只有一条路能走通：

1. 补 `robot_description`：URDF（含 `base_link → arm_base` 静态变换）+ SRDF，
   顺手把 `arm_camera_tf` 那条手眼外参并进去。
2. 接 `vla_data_collection`，先录 50–100 条单一固定位置抓取（文档 §6.2 第一档）。
3. 再写 `vla_ros2_bridge`，用录到的数据反推真实归一化统计量灌进
   `NormalizationStats`——没有统计量，反归一化这一步永远是假的。
4. 最后接 `robot_agent` 和 Nav2 停靠位姿（阶段 II）。
