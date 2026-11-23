# ArmPi 红球抓取交接

更新至 2026 年 11 月 23 日，时间按北京时间记。采样目录名中的 `Z` 表示 UTC，查找文件时按原名使用。

## 1. 目标与交接状态

本轮现场目标是在 ROS 2 真车上定位直径 **36 mm** 的红球，让夹爪抓住它并抬离地面，然后记录可复核的结果。最近一轮走的是 **车载相机 → 红球定位 → ArmPi 运动学与路径筛查 → ROS 2 底盘、机械臂和夹爪控制**。项目另外有 SmolVLA 真机接入目标，进度见第 7 节。

**2026-11-23 17:00 新场地首抓成功**，过程与判据见 1.1。换场地前那一轮的末次状态保留在第 2 节，只作新旧对比；当时影响连续执行的是底盘前方的间歇近回波和肘轴停稳后回偏。新场地已不再出现 0.147 m 那种近回波（前方最近回波 2.17 m，仅剩车体自身的 0.12 m 回波），肘轴停稳在首段通过、第二段出现 +9 ticks 超差。

本轮成功需要同时留下以下证据。

1. 抓前能在图像中确认球在地上，夹爪和球的位置关系清楚。
2. 合爪后可见球位于两指之间。记录夹爪开度、合爪前后编码器值和关节状态。
3. 受控抬升后，连续多帧显示球随夹爪移动并离开地面。近景容易遮挡时，用现场侧视或固定相机补证。
4. 动作结束后核对四轮输出、关节停稳值和两项运动许可，保留前后图像、轨迹回执、反馈和停车记录。

编码器能支持判断夹爪确实动过。既有阈值曾把空载闭合的 377 ticks 判为抓住，单张相机图也曾无法区分地面球和指间球。采用上面的实物验收。[抓取判据回放](SmolVLA离线验证.md#抓取成功判据回放2026-11-22) · [历史抓球记录](抓球成功记录-2026-11-02.md)

### 1.1 新场地首抓结果（2026-11-23 16:44）

一条链路走通：相机定位 → 固定档位 IK → 4 跳下降 → 合爪 → 抬升。现场记录在同目录 `tmp/run3.log`（会话产物，不入库），连拍三张确认球在两指之间且离地。

| 项目 | 实测 |
| --- | --- |
| 球位（`arm_base`） | `(0.0137, 0.2120, -0.0773)`，前向 **21.2 cm**，横向偏 14 mm |
| 起点六轴 | `j1=486 j2=683 j3=940 j4=200` |
| 下降 | 4 跳 `+101/+56/+11/+8` mm，误差 **9/9/8/7** mm，`aborted_by_floor_guard=false` |
| 落点 | `-1 mm`（`z_bias=-0.6`），工具点离地 **24.1 mm**，距保护线余 9.1 mm |
| 夹爪 | `r_open=128 → r_closed=383 → r_after_lift=370`，`detail="jaws held at 370"` |
| 底盘/健康 | `wheels [0,0,0,0]`、`fault_count=0`、`health=READY`、两闸门事后关闭 |

**六个可以直接用的判据：**

1. 🔴 **球必须 ≤24 cm**。`y=0.320 m` 时 `solve` 直接返回 `no IK`，而抓取流程的底盘自动前移是禁用的（会推球），那一轮 8 秒即退出。21.2 cm 一次成功。
2. 🔴 **`joint3` 每 tick ≈ 1 mm 工具点高度**。40~49 tick 的单步会一次花光整跳的垂直余量，穿过保护线（`landed +73 mm`、空夹 450）。`descent_step_cap` 系数已收紧到 **0.20**，单步上限 **20/16** ticks（refine 16）。
3. ⚠️ **手眼对相机姿态敏感**。同一颗球，`j4=200` 测 31.5~32.0 cm，`j4=282` 测 24.3 cm，**差 7 cm**。定位与执行必须用同一姿态。
4. ⚠️ **夹爪最大只开到 183 ticks（约 48 mm）**，开不到 115~130。球 36 mm，每侧余量 6 mm。
5. ⚠️ **`joint3` 有静态上漂**：停稳后 +7、+9，空闲 20 分钟再涨 20（874→894）；`joint2` 下降还会把它顶升（947→955→964）。所以「允许肘轴参与的高位前伸」难走：IK 要 j3 降到 791~871，实际越走越高。
6. ⚠️ **`DZ=+8` 属于偏上夹持**。现场观察夹的是球的上半球；据球心离地 16.7 mm 与工具点 24.1 mm 反推，指垫只比工具点低约 4 mm。若要夹得更实，下一档应把 `DZ_LEVELS` 首档从 **0.008 降到 0.004**（工具点 20.6 mm，距保护线仍有 5.6 mm），而不是继续压步长。当前 383→370 抬升后球未掉，够稳，暂未改。



## 2. 换场地前的最后状态

最后一次采样目录是 `grasp_dwell_combined_20261123T072332Z`，约北京时间 15:23。以下全是**换场地前**的数据。

| 项目 | 末次读数或状态 |
| --- | --- |
| 机械臂状态 | `READY`，末次无急停、无活动锁存故障。9 秒中有 18 条状态，采样脚本判为有效的有 17 条 |
| 机械臂及夹爪许可 | `enable_arm_motion=false`，`enable_gripper_motion=false` |
| 六轴编码器 | `joint1=484`，`joint2=687`，`joint3=931`，`joint4=206`，`joint5=499`，`r_joint=247`，单位均为 ticks |
| 底盘 | `enable_motion=true`，底盘无急停或硬件故障，四轮输出 `[0, 0, 0, 0]` |
| 最近一次接受的机械臂轨迹 | 本次启动周期序号 2，仅肘轴，目标 907 ticks。暂停前的观察试验没有产生新序号 |
| 串口整笔读取超时 | 本次启动后的累计值为 70，末次 9 秒窗口内保持不变。历史末次失败是舵机 4 的 `0x09` 温度查询，3 次尝试未收到匹配回包 |
| 红球 | 最近一次底盘试移后球心前向距离约 297 mm。红球、车体和相机视角在现场调整后都需重新量测 |

[末次原始状态和雷达](../datasets/smolvla_probe/grasp_dwell_combined_20261123T072332Z/capture_report.json) · [末次现场画面](../datasets/smolvla_probe/grasp_dwell_combined_20261123T072332Z/latest.jpg)

## 3. 已经做过的真机试验

### 3.1 腕轴和肩轴已有停稳通过记录

11 月 23 日较早的一次重启恢复中，腕轴 202→222 ticks，停稳 215，最大误差 7 ticks。随后肩轴 686→706，停稳 702，最大误差 4 ticks。之后小车再次进入开机确认锁并在不同场景上线，重新定位后执行腕轴 193→213，停稳 206，误差 7 ticks，停后 9 秒有 18 条状态均为 `READY`，两项许可关闭，底盘静止。按当时约 ±8 ticks 的标准，这些短步通过。每次换场地或重启均应读实际起点。

[较早的重启复测](SmolVLA离线验证.md#关机前末两段和重启复测2026-11-22-至-2026-11-23) · [最新腕轴真机结果](../datasets/smolvla_probe/pregrasp_new_scene_serial_watch_20261123T061330Z/guarded_resume_wrist_20261123T061505Z/result.json) · [停后反馈](../datasets/smolvla_probe/pregrasp_new_scene_wrist_post_20261123T061630Z/capture_report.json)

### 3.2 底盘短移被雷达保护终止

先前两次前移尝试分别在球位超出当时的试验范围、最终就绪检查变化时拦截，两次均为零条速度指令。第二次脚本用一个条件汇总了底盘、机械臂和雷达状态，该次的具体触发项无法从日志确定。

随后改用连续就绪窗口，要求至少 5 帧不同且连续通过的雷达扫描，覆盖至少 0.85 秒，并记录拦截原因。第三次确实发出 **4 条**非零速度指令，目标速度 0.05 m/s，发令时间上限 0.40 秒，里程计触发停车的行程阈值为 20 mm。途中出现位于扫描 **+8.72°、约 0.147 m** 的近回波，守护流程发停车。停稳里程计显示前移 **3.28 mm**、侧移约 0.22 mm、偏航约 −0.056°，球心前向估计从 301.53 降到 297.18 mm。后者是视觉估计。

在此前场景也采到过相近方向的间歇近回波。现有记录尚不能确认其来自车身或外界物体。已有 `scan_clean.py` 的 0.19 m 自身半径用于原地旋转筛查，前移复测沿用更窄的已核验车身回波范围，并保留范围外近回波的停车触发。

[第一次零发令](../datasets/smolvla_probe/single_forward_ball_approach_20261123T062737Z/trial_result.json) · [第二次零发令](../datasets/smolvla_probe/single_forward_ball_approach_20261123T063147Z/trial_result.json) · [实际试移与停车](../datasets/smolvla_probe/single_forward_ball_approach_20261123T065328Z/trial_result.json) · [雷达处理代码](../tools/scan_clean.py)

### 3.3 肘轴到位和保持仍未通过

| 时点 | 实测 |
| --- | --- |
| 发令 | 仅 `joint3`，起点 927，目标 907 ticks，时长 3.2 秒 |
| 临近结束 | 回读 916 ticks，误差 9 ticks |
| 撤销许可后的停稳窗口 | 回读 920 ticks，最大误差 13 ticks，超出 ±8 ticks |
| 此后空闲只读 | 回读回到 931 ticks，随后一个 10 秒窗口内保持在 931 |
| 独立诊断 | 两次读到舵机 4 扭矩使能为真，静态端电压分别约 7.501 和 7.536 V，末次肘轴温度约 30 °C |

肘轴试验的动作及停稳窗口中，整笔读取超时计数保持 27。跟踪偏差在撤销许可之前已经存在，撤销后又出现回偏。静态电压、使能位和串口写入回执无法量化负载下的保持力，机械、电气或控制时序原因尚未定。用户从现场观察，没有报告明显下沉、抖动或连杆松动。历史上一次 30 ticks 肘轴短步也留下 12 ticks 的停稳误差。

拟验证的替代步骤是从最新姿态筛查一条肘轴短步，在同一条指令的常规等待结束后额外观察约 **1.8 秒**，再撤销许可并按原容差验收。两轮单独采样因图像时效过期未发令。之后把采样、对图和复算接到同一流程，依次遇到 Windows 中文路径读图、画面差异、相机曝光变化的拦截。读图改用了 `Path.read_bytes()` 配合 `cv2.imdecode`，最后一次仍在曝光检查处停住。**额外等待是否改善肘轴跟踪尚未经真机验证。**

[肘轴轨迹和回读](../datasets/smolvla_probe/grasp_high_entry_preflight_20261123T063417Z/guarded_grasp_elbow_20261123T063548Z/result.json) · [继续保持 931 ticks 的记录](../datasets/smolvla_probe/grasp_elbow_idle_drift_20261123T064141Z/capture_report.json) · [静态诊断记录](SmolVLA离线验证.md#当前红球的前移检查与肘轴试验2026-11-23)

### 3.4 路径筛查说明

- 在球前向距离约 303 mm 的一组观测中，允许肘轴参与时，72 个候选中有 49 个通过高位模型筛查。代表性候选的最大单关节行程约 386 ticks。整条高位路线仍处于离线状态，真机只试了首条肘轴短步。[多轴高位预演](../datasets/smolvla_probe/grasp_approach_recheck_20261123T062942Z/pan_held_pregrasp_screen.json)
- 以约 302 mm 球距和固定肘轴 931 ticks 复算，所测 130 和 150 mm 高位目标的端点误差分别约 18.6 和 19.7 mm，测试的先腕、先肩两类路线均未通过净空筛查。约 **22–24 cm** 是换场地时可供摆球的初始参考，来自固定肘轴方案的离线估算。该距离从机械臂基座沿地面前向估计，摆好后以新图像和运动学复算为准。[固定肘轴预演](../datasets/smolvla_probe/grasp_elbow_idle_drift_20261123T064141Z/fixed_elbow_high_pregrasp_review.json)
- 高位筛查用仓库 URDF 和网格近似检查地面、车身、部分自碰撞及球体间隙。当前室内家具和人员的净空由现场及传感器复核。高位通过之后还需独立验证下探、接触、夹住和抬升。[模型筛查代码](../tools/runtime/check_armpi_high_pregrasp.py)

## 4. 换场地后的操作顺序

### 4.1 恢复连接与建立新起点

1. 等用户确认场地换好、红球和小车已放稳，再连接原 ArmPi。当前网络经 Tailscale 时曾出现连接超时和同名主机混淆。使用已登记的地址和 SSH 主机密钥，并核对 ArmPi 的 ROS 2 容器、相机和机械臂话题。记录车端启动时间、镜像名及本轮最新动作序号。
2. 先只读采集相机、内参、TF、雷达、里程计、机械臂和底盘状态。核对手、脚、尺子、家具等与车轮及机械臂活动范围的距离。取一组足够覆盖短时波动的样本，记录两项许可、急停原因、六轴位置、四轮输出、反馈年龄以及温度读取超时计数的起止值。
3. 对 `startup_confirmation` 单独处理。当真实回读完整、控制器达到 `RECOVERED_AWAIT_RESET` 且 `reset_eligible=true`，经现场确认后，可通过控制器复位开机确认锁，再按下一段需要独立授权相应关节。遇到 `manual_estop`、仍在恢复、位置超界或活动故障时，先读明原因并排除。舵机位置指令只经现有控制器，由它独占串口。
4. 旧场景的读数、球位和净空确认留作对比。对新画面用车载标定和 `arm_base → rear_camera` TF 定位。优先用有充分支撑的红球上缘拟合，减少地面红色反光对半径的干扰。记下多帧球心中位数、最大偏差和横向残差。

已有试验复用了至少 3 帧有效配对，其参考条件为六轴位置年龄不超过 0.40 秒、图像和关节 header 时间差不超过 200 ms、接收时间差不超过 250 ms、TF 与图像相差不超过 125 ms。配对不足、球被挡、画面不稳定或环境改变时，先重新采集。归档的 `latest.jpg` 可供现场检查，计算球位使用有姿态与 TF 的配对帧。

### 4.2 先选可执行路线，再决定底盘是否移动

| 新场地读数 | 接手动作 |
| --- | --- |
| 球已摆近且有通过筛查的固定肘轴路线 | 先尝试底盘原地的高位路线。以当前实测肘轴为固定值，按新球位计算腕、肩的分段路线。每段按真实回读重新判断下一段 |
| 球距仍偏大，现场与雷达均确认前方净空 | 需要缩短球距时，使用单次、有时间及里程计上限的低速前移。发布前确认其他 `/cmd_vel` 发布者为空、底盘无故障、连续新鲜雷达帧通过、当前轮输出为零。途中遇近回波、回读丢失、偏航或超程立即停。停稳后检查里程计和重测球位 |
| 前移近回波持续或被遮挡，固定肘轴路线也过不了 | 请现场确认回波来源和轮边，或者先把球摆到约 22–24 cm 的候选范围再定位。仍按新球位筛查是否可达 |
| 规划确实依赖肘轴 | 先做空闲关节诊断。若电压、温度、回读和场地状态允许，只发一条新筛查的短步，记录发令前、动作期间、撤销许可前后和随后静止窗。需要试额外等待时，观察上限约 1.8 秒，到时收回许可，期间继续监测状态与停车条件 |

底盘实际朝向优先由历史的实物正向移动和本轮里程计核对。ROS 的底盘前进速度用 `Twist.linear.x`，ArmPi 运动学以 `arm_base` 的 Y 为前方，两套轴约定要按已有实测关系使用。近回波异常触发停车后，以停车和新观测作为下一次决策起点，避免连续补发。[实物前移与定位历史](抓球成功记录-2026-11-02.md)

### 4.3 从高位接近到合爪和抬升

1. **规划并验证高位段。** 以当前盘角、夹爪开度和真实六轴读数计算，先检查端点误差与整个扫掠路径。实物试验优先用已验证方向的单轴短步，最近采用的首段上限是 20 ticks。入口重测起点，按观测区间设固定目标，避免同一个名义目标从低一格的起点出发就超过最大行程。旧轨迹的动作回执只证明驱动接受。
2. **按停稳回读决定下一段。** 每段结束撤销许可，记录至少数次新鲜回读及最大目标误差。腕、肩、肘近期短步以 ±8 ticks 判定。肘轴超差或持续回偏时，如实记为未通过，由最新姿态重算；应优先解决回偏或采用固定肘轴方案，再推进需要精确对准的段。若 SSH 在可能发布后断开，先只读核对最新动作序号、关节和许可，再决定如何恢复。
3. **对准球与夹口。** 盘轴固定时，先量横向残差，再选开口或做低速对准。36 mm 球配约 42 mm 开口时每侧名义余量仅约 3 mm。球位噪声、指尖形状和负载都会吃掉一部分余量，实拍核对比单用算出的余量可靠。夹爪预张开后确认真实开度，并重新评估扫掠包络。
4. **短程下探。** 用每段后的新鲜图像确认球仍在地面原位，且两指接近球的两侧。检查地面和车身净空。接近接触时减小末端步长并复测。球被推走、定位消失、夹爪离球仍高或关节未到位时，先停稳重定位。历史成功姿态和工具点补偿随场地与姿态变化，用于理解方法。
5. **分开执行合爪和抬升。** 底盘静止，合爪段只开放夹爪许可。记录开口和停住的实际刻度，用画面确认红球位于指间。先做一段小幅抬升，确认球跟着指间离地并保持，再完成抬升和第 1 节的验收。保存动作前、指间接触、离地后的多帧画面及停稳数据。

### 4.4 夹爪的空载参考

| 两指内侧间距 | 现场对应的回读 | 36 mm 球居中时每侧名义余量 |
| --- | --- | --- |
| 50 mm | 155 ticks | 7 mm |
| 47 mm | 201 ticks | 5.5 mm |
| 约 42 mm | 246–247 ticks | 约 3 mm |

这三点测的是空载，支持该区间内刻度增加、开口缩小。尺量和编码器的时间未精确配对，用作预张开参考。夹球时的刻度需结合当时接触和抬升图像复核。[开口现场复测](SmolVLA离线验证.md#50-mm-夹爪开口现场复测2026-11-22)

## 5. 环境、接口和只读接手命令

工作区为 `D:\多车协作`。已核验设备运行 ROS 2 Jazzy，容器名为 `armpi_ros2_native`，本轮域号 42。最近核查镜像为 `armpi-ros2-migration:elbow-diagnostics-20261122-ec0697cbde9f`。换场地后按车上实际镜像复核。

Windows 仓库根目录可先运行只读设备清单。使用此电脑已登记的 SSH 主机密钥和本地凭证配置。

```powershell
$ArmPiAddress = '<已核验的 ArmPi 地址>'
python tools/ros2_preflight.py --host $ArmPiAddress --host-key-alias raspberrypi --config tools/console_supervisor.local.json
```

在已核验的小车宿主机上，进入容器并从下列只读命令中按需选择。带 `timeout` 的命令结束时直接返回终端。

```bash
docker exec -it armpi_ros2_native bash
source /opt/armpi_ros2/install/setup.bash
export ROS_DOMAIN_ID=42

timeout 8s ros2 topic echo /arm_controller/status --once
timeout 8s ros2 topic echo /chassis_control/status --once
ros2 topic info /cmd_vel -v
timeout 8s ros2 topic hz /rear_camera/image_raw/compressed
timeout 8s ros2 topic echo /rear_camera/camera_info --once
timeout 8s ros2 topic hz /scan
timeout 8s ros2 run tf2_ros tf2_echo arm_base rear_camera
```

| 接口 | 用途 |
| --- | --- |
| `/rear_camera/image_raw/compressed`、`/rear_camera/camera_info`、`/tf` | 图像、内参、带时间戳的实车坐标变换 |
| `/scan`、`/odom`、`/chassis_control/status` | 雷达、里程计、车轮输出与底盘健康 |
| `/arm_controller/status`、`/joint_states` | 末次指令结果、真实关节反馈、读数时效、运动许可和锁存状态 |
| `/arm_controller/command` | 受保护的机械臂或夹爪指令话题。动作链使用 ROS 2 单点轨迹，关节角与编码器转换见代码 |
| `/cmd_vel`、`/chassis_control/stop` | 底盘速度和停车，由受保护的试移流程管理 |
| `/arm_controller/read_elbow_diagnostics` | 两项许可均关闭且空闲时，通过现有控制器只读查询肘轴扭矩使能及端电压 |
| `/arm_controller/reset_estop` | 控制器的复位服务，有自己的健康与故障检查。与动作许可分别管理 |

ROS 关节 `joint1` 到 `joint5` 依次为转盘、肩、肘、腕俯仰、腕旋转，对应舵机 ID 6、5、4、3、2。夹爪 `r_joint` 对应 ID 1。肘轴角度转换有负号。串口查询 `0x05` 是位置、`0x09` 是温度，诊断超时时记录是哪一类以及舵机 ID。底盘与机械臂均确认空闲后，可以用下面的命令读取已有诊断服务。

```bash
timeout 8s ros2 service call /arm_controller/read_elbow_diagnostics std_srvs/srv/Trigger '{}'
```

[连接与主机密钥校验](../tools/ros2_preflight.py) · [机械臂控制器](../ros2_ws/src/armpi_ros2/armpi_ros2/arm_node.py) · [舵机映射及协议](../ros2_ws/src/armpi_ros2/armpi_ros2/servo_protocol.py) · [ROS 2 迁移记录](ROS2迁移实施与验收.md)

## 6. 接手时从哪里取文件

| 需要什么 | 从哪里取 |
| --- | --- |
| 只读采样脚本 | [最近一轮归档的 read_only_probe.py](../datasets/smolvla_probe/grasp_dwell_combined_20261123T072332Z/read_only_probe.py)。脚本只订阅话题，向标准输出写一条 `POSITION_REVIEW=` 开头的 JSON。宿主端解析并保存 `capture_report.json`，从 `selected_frames` 解码 JPEG。独立的 `image_only` 最新帧留作现场检查 |
| 红球识别与装配几何 | [红球检测](../ros2_ws/src/armpi_ros2/armpi_ros2/red_ball_detector.py)、[坐标变换](../tools/ball_in_arm_base.py)、[ArmPi 运动学](../ros2_ws/src/armpi_ros2/armpi_ros2/arm_kinematics.py)、[名义 URDF](../ros2_ws/src/robot_description/urdf/armpi_pro.urdf) |
| 运动许可、步长和停稳验收参考 | [受保护控制脚本源文件](../car_setup/ros2/arm_startup_pose_ros.py)、[本轮已执行肘轴保护脚本](../datasets/smolvla_probe/grasp_high_entry_preflight_20261123T063417Z/guarded_grasp_elbow_20261123T063548Z/executed_guard.py) |
| 实际前移如何停车 | [已执行底盘试移脚本](../datasets/smolvla_probe/single_forward_ball_approach_20261123T065328Z/executed_trial.py)、[结果及停车证据](../datasets/smolvla_probe/single_forward_ball_approach_20261123T065328Z/trial_result.json) |
| 历史失败与定位调试 | [完整离线及真机记录](SmolVLA离线验证.md)、[历史抓球记录](抓球成功记录-2026-11-02.md) |

归档的 `executed_guard.py` 和 `executed_trial.py` 含当时的球位时限、起点和现场假设，用来核对实际发令过程。接手时以受保护控制器和当次观测新建试验。上次把采样、路径筛查、曝光比对和额外等待组合起来的宿主脚本主要保存在对话执行会话中，未形成仓库入口。可从表中的只读脚本、控制脚本和第 4 节重新组合。Windows 中文路径下读 JPEG 用 `Path.read_bytes()` 配合 `cv2.imdecode`，已有 `cv2.imread` 读图失败记录。场景比对需要先确认曝光变化和真实现场变化，回放的画面阈值尚未完成验收。

`datasets/`、`tools/runtime/` 和本地连接配置由 Git 忽略。换电脑时需要带走必要的采样和运行脚本，并在新电脑单独配置认证。项目交接文件、临时照片、原始遥测和本地凭证的传递分别按仓库和现场权限处理。

## 7. ycw 上的 SmolVLA 进度

ycw 已用现有环境和缓存 `lerobot/smolvla_base` 做过离线推理。一次使用同步的真车图像和六关节数据，在 CPU 上推理约 4.25 秒，返回 `1 × 50 × 6`；另用存档红球画面做过颜色和文本对照。这些结果标记为 `executable: false`。本轮真实底盘、腕、肩、肘轴的动作来自第 1 节的视觉和 ROS 2 路线。[ycw 单次推理与对照](SmolVLA离线验证.md#ycw-的车端采样与单次推理2026-11-22) · [单次推理原始结果](../datasets/smolvla_probe/ycw_live_input_20261122T040735Z/ycw_cpu_inference.json)

真机抓球链稳定后，按以下顺序接入模型。

1. 确定源模型适用的数据域与训练时的字段顺序，复核 `observation.state` 和 `action` 的归一化及反归一化。
   ✅ **数据域已定：`so100-blue`**（50 步全量复核零越界，`so100` 排除，见 §7.1）；**归一化仍是跳过的，未解决**。
   真正卡住的是输入侧的关节坐标系（§7.3），不是统计量选择。
2. 核实 6 维输出的语义、单位、零点、方向和量级。现有 ROS 2 VLA 执行入口使用另一路 7 维动作表示，先设计明确转换。需要时用源机器人正算、任务坐标与 ArmPi 逆算来做物理映射，同时校准夹爪。
3. 用固定画面和同步反馈做可重复的离线回放，再用实时帧运行影子模式，保存候选动作、限位、预计末端轨迹和模型结果。校对底盘、机械臂、夹爪权限和停止条件。
4. 确认映射与保护流程后，从有界单段真机动作开始，逐项验收跟踪、净空和夹持。现有 `command_output_enabled` 为 `false`，应将适配和验证完成后再决定启用。
5. **前置硬条件（§7.3 得出）：先做 ArmPi↔SO100 的逐关节仿射标定（符号、零点、斜率）**，把状态拉回训练分布内，
   否则第 2~4 步做的动作转换没有可用的输入基础。

[待定适配配置](../tools/config/armpi_smolvla_adapter.pending.json) · [三组源统计](../tools/config/smolvla_source_stats/) · [离线探针](../tools/smolvla_probe.py) · [域审计工具](../tools/smolvla_domain_audit.py) · [ROS 2 VLA 配置](../ros2_ws/src/vla_controller/config/vla_controller.yaml)

### 7.1 归一化阻塞点已定位，选错域的代价已量化（2026-11-23）

**阻塞点不是缺统计，而是键名对不上。** `ycw_cpu_inference.json` 的 `normalization_audit` 写明：
checkpoint 里存的统计键是 `so100.buffer.action`、`so100-red.buffer.action`、`so100-blue.buffer.action`（数据集 buffer 形式），
而官方 LeRobot 归一化变换查找的是规范键 `observation.state` / `action`。键名不匹配时官方 transform **直接跳过该特征**，
所以那次推理输出的 6 维动作是**归一化空间的原值**（`output_matches_raw: true`、`normalization_verified: false`），
不是物理量。这也解释了三套统计为什么并存——它们分别对应 checkpoint 里的三个 buffer 键。

**选错域的代价（按 `raw = z*std + mean` 反归一化，单位：度 / 夹爪为百分比）：**

| 通道 | 对应关节 | 三套统计间最大差异 |
| --- | --- | --- |
| shoulder_pan | joint1 | 12.9° |
| shoulder_lift | joint2 | 18.8° |
| elbow_flex | joint3 | **47.3°** |
| wrist_flex | joint4 | 25.6° |
| wrist_roll | joint5 | **102.3°** |
| gripper | r_joint | 23.7 个百分点 |

结论有两层：
1. **选错域主要毁在 wrist_roll 和 elbow_flex**（102° / 47°），这两路错了动作完全不可用，不是"精度差一点"。
2. `so100-red` 与 `so100-blue` 互相高度一致（wrist_roll 均值 −103.9 vs −106.5，仅差 2.6°），`so100` 则明显偏离
   （wrist_roll −27.4，与两者差约 76°）。所以候选实际是「red/blue 一组」对「so100 一组」。

**已用真实模型输出做交叉验证（2026-11-23，ycw GPU）。** 在 ycw 上按 `tools/smolvla_probe.py infer --allow-unadapted-state`
跑通一次（`status: inference_completed`、`peak_cuda_bytes=1.26 GB`），并把 3 步 × 6 维原始输出按三套统计分别反归一化：

| 统计 | 越界通道 | 具体表现 |
| --- | --- | --- |
| `so100` | **5** | `elbow_flex` = **−280 / −265 / −189 度**（超 ±180），`gripper` = −0.4 / −0.3%（负百分比） |
| `so100-red` | 2 | 仅 `gripper` = −0.0%（浮点边界） |
| `so100-blue` | **0** | 肩 129~131°、肘 −17~+16°、腕俯 30~45°、腕转 −89~−115°、爪 0.1~0.3% |

**`so100` 可以排除**：它把肘关节反归一化成 −280 度，物理上不存在。`red` 与 `blue` 实质等价（`blue` 零越界），
在现有证据下 **`so100-blue` 是最自洽的一套**。这一步同时印证了 `output_matches_raw: true`——
`postprocessor_output_equal_model_output` 为真，官方 pre/post 变换确实被跳过，输出停留在归一化空间。

**这次验证的边界（不要当成"已选定域"）：**
- ~~只拿到 3 步~~ → 已补全 50 步，见下；
- 反归一化是**手工套公式**做的（`raw = z*std + mean`），不是官方 transform；
- `target_robot_mapping_verified`、`checkpoint_to_urdf_calibration_verified`、`absolute_zero_verified`
  在 `armpi_smolvla_adapter.pending.json` 里**仍然全是 false**，跨域标定一项都没解决。

**50 步全量复核（2026-11-23）。** `actions.json` 里本来就存了完整 `[1,50,6]`，之前只看 3 步是漏了。
按三套统计各反归一化 50 步（共 300 个通道值），判定"物理上是否可能存在"：

| 统计 | 越界步数 | 具体表现 |
| --- | --- | --- |
| `so100` | **17 / 300** | `elbow_flex` **15 步**落在 −280.1~11.5°（超 ±180），`gripper` 2 步为负（−0.4%） |
| `so100-red` | 2 / 300 | 仅 `gripper` 2 步为 −0.03% 量级的浮点边界 |
| `so100-blue` | **0 / 300** | 全 50 步全部落在 ±180° / 0~100% 内 |

`so100-blue` 全 50 步轨迹（度 / 百分比）与跨步最大跳变：

| 通道 | 对应关节 | 首值 | 末值 | 区间 | 净变化 | 最大跨步 |
| --- | --- | --- | --- | --- | --- | --- |
| shoulder_pan | joint1 | −0.27 | −10.13 | −10.26 ~ 26.91 | −9.9 | 7.51 |
| shoulder_lift | joint2 | 131.33 | 103.02 | 82.44 ~ 131.33 | −28.3 | 13.65 |
| elbow_flex | joint3 | −17.22 | 79.46 | −17.22 ~ 89.47 | +96.7 | 27.83 |
| wrist_flex | joint4 | 44.64 | 36.60 | 17.00 ~ 52.23 | −8.0 | 11.72 |
| wrist_roll | joint5 | −88.89 | −85.24 | −126.56 ~ −81.14 | +3.7 | 17.97 |
| gripper | r_joint | 0.05 | 16.30 | 0.05 ~ 24.25 | +16.3 | 16.39 |

**域的选择就此定案：`so100-blue`，`so100` 排除。** 这条只说明**输出侧**换算自洽；
输入侧仍然是坏的，见 §7.3。

### 7.2 相机数量不是阻塞点（2026-11-23 修正，源码级结论）

**此前我写过"`configured_cameras=3` 而只喂 1 路 → 视觉输入不足"，这条是错的，作废。**
SmolVLA 的 `prepare_images()`（`code/lerobot/src/lerobot/policies/smolvla/modeling_smolvla.py:385`）逻辑是：

```python
present_img_keys = [key for key in self.config.image_features if key in batch]
missing_img_keys = [key for key in self.config.image_features if key not in batch]
if len(present_img_keys) == 0:
    raise ValueError("All image features are missing from the batch. At least one expected.")
# ...只处理 present 的部分，各自 resize_with_pad 到 512x512、归一化到 [-1,1]...
for num_empty_cameras in range(len(missing_img_keys)):
    if num_empty_cameras >= self.config.empty_cameras:
        break                      # ← empty_cameras=0 时第一次就 break，一张空图都不补
    images.append(torch.ones_like(img) * -1); img_masks.append(torch.zeros_like(mask))
```

即 **`empty_cameras: 0` 的设计语义就是"你给几路就用几路，至少一路"**：缺失相机被整段丢弃，不进序列、
也不进 mask，只保留实际提供的那几路。所以：

- **单路腕部相机是原生支持的工作方式，不需要改 config、不需要复制图像凑数。**
- 这也与 ycw 上真机数据集的写法一致：`scripts/t07_convert_remote.py` 的 features 只有
  `observation.state(6)` + `action(6)` + `observation.images.camera1`，`t07_completion_audit_remote.py`
  断言读回形状为 `[2,6]` / `[2,50,6]` / `[2,3,480,640]`——**一路相机、六维关节**。
- `configs/t02_policy_config.json` 里那套 `camera1+camera2 / state 8 / action 7` 是 **LIBERO** 的配置
  （agentview + wrist），不是 Gazebo 抓球实验的；HANDOFF §6 明确写了"LIBERO 状态8维、动作7维"与
  "ArmPi 六维关节目标"是两套东西。拿它当 ArmPi 的相机数依据是引错了配置。

同一份源码还说明：喂 1 路时视觉 token 比 3 路少，这是**信息量差异**（视野缺一路），不是能否运行的结构问题。
若要补第二路，成本是加一路相机与标定，而不是改模型。

**下一步（要真正定域）：** ①把 `tools/smolvla_probe.py` 的 preview 改成落盘完整 50 步；
②确认源域（现有证据指向 `so100-blue`）后写进 `source_statistics_profile`；③做 §7 前面列出的六维↔七维动作映射与跨域标定。
相机这一项可以从待办里划掉。

### 7.3 输入侧才是真正过不去的坎：状态偏离训练分布 12~22 sigma（2026-11-23）

§7.1 只证明了输出侧换算自洽。输入侧的问题是另一个量级。SmolVLA 预训练用 **SO100 关节角（度）**，
而探针喂进去的是 **ArmPi 关节弧度原值**（`state_mapping: unadapted_armpi_joint_order`、
`embodiment_compatible: false`）。把那次真实输入换算成度，再对 `so100-blue` 的 `observation.state`
统计做 z 分数：

| 通道 | ArmPi 关节 | 输入值（度） | 源均值 | 源 std | z |
| --- | --- | --- | --- | --- | --- |
| shoulder_pan | joint1 | −4.08 | 1.39 | 13.82 | −0.40 |
| shoulder_lift | joint2 | 44.88 | 124.27 | 40.27 | −1.97 |
| elbow_flex | joint3 | −96.24 | 126.75 | 17.56 | **−12.70** |
| wrist_flex | joint4 | −72.00 | 63.72 | 23.59 | **−5.75** |
| wrist_roll | joint5 | −0.24 | −104.07 | 33.37 | **+3.11** |
| gripper | r_joint | −98.10 | 3.96 | 4.51 | **−22.64** |

**6 路里有 4 路落在 ±3σ 以外，最远 22.6σ（夹爪）、12.7σ（肘）。** 这意味着：给定这个状态，
模型不是在插值，而是在**外推**。外推出来的动作即使是自洽的（§7.1 那 50 步确实自洽），也不代表它对
ArmPi 有意义——SO100 从没在这个位姿上演示过任何东西。

还有一层更隐蔽的：因为规范化键名缺失（§7.1），官方 transform 被跳过，模型**实际收到的就是弧度原值**
（joint3 = −1.68），而训练时收到的是 MEAN_STD 后的 z 值（joint3 ≈ 0）。也就是说当前这次推理
**连单位都是错的**，只是这些弧度数字碰巧落在 z 值的常见数值区间里，所以没有报错。

**为什么不能靠"换算单位"一把过：** 光把弧度换成度，joint3 已经是 −12.7σ。真正的差距在于 ArmPi 在开机
姿态（joint3 = 907 tick，肘是**反方向**弯的），而 SO100 的 elbow_flex 均值是 **+126.75°**。两家的符号与零点
本来就不是一套。要把状态拉回分布内，必须做**逐关节的仿射映射（符号 + 零点 + 斜率）标定**，这正是
`target_robot_mapping_verified` / `absolute_zero_verified` 那两个 false 所代表的未完成工作。

**对"现在为什么不能用 VLA"的一句话回答：**
不是缺相机，不是缺显存，也不是模型跑不起来——是**预训练模型的输入口只认 SO100 的关节坐标系，
而 ArmPi 的六轴和它差了 12~22 个标准差；不先把两套关节坐标系标定对齐，模型输出的 50 步动作再自洽也不能下发。**

**复现方式**（两个子命令，纯 Python，不需要 GPU）：

```bash
python3 tools/smolvla_domain_audit.py domains \
  --actions artifacts/smolvla_domain_audit_20261123/actions.json \
  --out   artifacts/smolvla_domain_audit_20261123/domain_audit.json
python3 tools/smolvla_domain_audit.py state \
  --observation artifacts/smolvla_domain_audit_20261123/observation.json \
  --domain so100-blue \
  --out artifacts/smolvla_domain_audit_20261123/state_gap.json
python3 -B tools/test_smolvla_domain_audit.py     # 15 项，含"弧度必须先转度"的回归
```

证据：`artifacts/smolvla_domain_audit_20261123/`（`actions.json` 原样取自 ycw 的
`probe_20261123/run1.json/actions.json`，`observation.json` 取自 `probe_20261123/input1/`）。

本机（Windows）没有 `torch`/`lerobot`/本地 checkpoint，且设计上不装——环境在 ycw：
`/home/ycw/armpi_pro_ws/vla`（`.venv` 里 torch 2.11.0+cu130、lerobot 0.7.0、RTX 3090 24 GB），
模型缓存在 `cache/huggingface/hub/models--lerobot--smolvla_base`（1.8 GB）。
**ycw 无外网**，跑推理必须带 `HF_HOME=<缓存路径> HF_HUB_OFFLINE=1 TRANSFORMERS_OFFLINE=1`，
否则 huggingface_hub 会卡在版本检查上反复重试（实测卡了 10 分钟）。


