# ArmPi Pro ROS 2 核心硬件（静止接管，运动待验收）

**最新部署：ROS 2 原生开机配置已切换**。摄像头及视频服务不再依赖 ROS 1；旧容器已停止、开机自启动已禁用，核心异常退出自动重建且不恢复运动授权。
当前镜像 `armpi-ros2-migration:stage-zjt99z`，固定 ID 为 `sha256:55afa36547a0facf21c13d779a3284464862e93b6c24b4d6f18fcdab84b2a4f4`。
真实雷达、IMU、里程计、关节状态、视频均在线，但六舵机温度回读仍有新增超时，机械臂锁存未解除；新配置完整断电重启尚待验收。
详见 [原生开机切换验收](../docs/ROS2原生开机切换验收.md)。以下较早的状态段落是历史记录。

目标环境：ROS 2 Jazzy / Ubuntu 24.04 / ARM64 容器，保留 Debian 12 宿主。
本目录与 `armpi_pro/`（ROS 1）独立。2026-10-11 已切换四类核心硬件、控制台入口与开机服务。
底盘及机械臂动作当前锁定，自动建图/避障和视觉应用没有迁移完成；下面日期较早的段落是历史记录。

2026-10-14：整车核心已经退出，不再是历史 PID 的 SIGSTOP 状态。
雷达已通过独立 `armpi-ros2-lidar.service` 恢复，约 10 Hz，9091 WebSocket 可接收扫描。
只接入雷达，不恢复机械臂/底盘；异常退出自动重建传感器容器的测试通过。
独立服务未启用开机自启动，原整车启动保护未修改；整车故障仍待处理。
详见 [雷达独立恢复记录](../docs/雷达独立恢复-2026-10-14.md)。

2026-10-14 后续：在用户确认现场可低速验证后，关节 3 的越界恢复及保存姿态回位已部署。
核心与开机回位服务已恢复，实测五关节在允许误差内，25 秒观察无新增故障。
只对关节 3 有界向内引导，不放宽正常限位，底盘及夹爪仍禁用；全断电验收待进行。
详见 [机械臂开机回位修复](../docs/机械臂开机回位修复-2026-10-14.md)，以上暂停记录为历史状态。

此前实车镜像 `armpi-ros2-migration:stage-4xhjqf`，ID
`sha256:3704db56c46ebe9ade423cfef9f0e19ea0ac177e440411271105501d455acac6`，64 项测试通过。
该版本新增完整反馈、通信原始报文/计数、至少10秒健康回读后的人工复位、独立夹爪授权。
复位不授权动作、不续跑；开机仍锁定，温度/限位/单步保护未放宽。
2026-10-13 用户确认机械臂下垂/意外转动，程序现已 SIGSTOP 暂停，不能操作机械臂。
PID 暂停不保证物理保持力，重启会使暂停失效；现场安全未确认前不得恢复或重启。
补充接收线程故障检查的 `stage-h0wcqg`（65项通过）暂未部署，因实物运动异常停止验收。
原生 `/scan` 约 10 Hz、BMI270 约 50 Hz、编码器/里程计约 20 Hz、真实六舵机约 2 Hz。
`armpi-ros2.service` 已 enabled，禁止旧驱动抢占；尚未做整车断电重启验收。
Windows 入口为 `http://127.0.0.1:8000/`，本机 SSH 9090 转发至车端仅回环监听的 ROS 2 9091。
公开的 `tools/console_core_profile.json` 管理版本/端口，不包含密码。不要混用旧控制台的消息类型。

**当前阻塞：持续验收仍出现六舵机温度查询偶发无应答，机械臂故障/急停锁存。**
短时真实反馈在线不等于稳定控制可用。底盘和机械臂动作保持禁用，未自动复位或启动旧驱动。
持续接收、有限读取重试、20 ms 查询间隔与停止抢占修正已部署，但尚未消除此故障。
下一步需要确定控制板串口/舵机总线回读丢失原因，再做现场动作和整车重启验收。

**当前已通过隔离的软件验收，不是可直接替换 ROS 1 的整车版本。**
2026-10-10 已修正实测编码器轴映射，完成三向架空底盘/停轮复测和 ROS 2 雷达约 10 Hz 验证。
真实雷达 + 模拟静止里程计已产出 `/map` 和 `map -> base_link` TF，尚未验证移动建图质量。
真实 BMI270/CH341 约 50 Hz 读取及零编码器下的陀螺仪 yaw 融合观察已通过。
每次测试后恢复 ROS 1；没有切换控制台、自启动或机械臂，安装外参、落地方向/精度尚未验收。
原生 rclpy 底盘、编码器里程计、BMI270 接入和雷达/SLAM 启动文件已编写；
机械臂、视觉抓取、动态避障/自动探索、控制台和开机恢复尚待迁移。
没有验证过的驱动不能作为自动巡航的安全保障。

### 2026-10-11 四类硬件切换准备（仍未接管实车）

已新增原生 ROS 2 机械臂/夹爪节点、真实串口回读、限位/温度/单步保护，
启动不发送位置、回中、卸力或标定写入。夹爪保留旧驱动 180°/1000 脉冲比例，
其余五关节为 240°/1000，关节 3 方向反转。
六舵机实际位置/限位已读取；新驱动默认动作禁用，单步最多 100 脉冲，
使用有机器人时钟时间戳的单点 JointTrajectory，不实现 FollowJointTrajectory action。
雷达偏移已记录为 `[-0.01,+0.01,0.18] m`。

增量镜像 `armpi-ros2-migration:stage-rpgqzu`：
`sha256:a9627f4c7cdc325f95b0b127457f9fde35532e4c7d17521630a057e04f060e25`。
本地及 ARM64 镜像中 35 项测试通过。此前版本已完成无设备隔离运行及共享急停验证；
最终镜像的隔离复验、真实串口接管、控制台端到端验证和开机切换结果见实施记录。
不能把软件验收或界面准备当成实车迁移完成。

`web_console/ros2_console.html` 为独立的 ROS 2 手动控制页面，需对应 9091 SSH 转发；
不会自动替换旧控制台。界面仅做了语法检查，尚未完成真实浏览器/硬件端到端验收。
视觉抓取、自动探索、开机配置及旧控制台适配仍未迁移。

## 默认安全边界

- `dry_run=true`：不打开 I²C，不访问电机或编码器硬件；编码器数据固定为模拟零。
- `enable_motion=false`：即使收到 `/cmd_vel`，四轮输出仍为零。
- 传感器默认不启动，避免 ROS 1 和 ROS 2 同时抢串口/I²C。
- 临时 rosbridge 仅绑定 `127.0.0.1:9091`，不占用正在使用的 `9090`。
- 隔离测试使用 `ROS_DOMAIN_ID=42`、`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`。
- 速度限幅：平移 `0.08m/s`、角速度 `0.20rad/s`；命令超过 `0.35s` 未更新归零。
- `/chassis_control/stop` 立即归零、清除旧命令；`/safety/estop` 收到 `true` 后锁定，
  收到 `false` 不自动解除，必须明确调用 `/chassis_control/reset_estop`。
- I²C 异常锁定；软件无法在电机板掉电、总线断开或进程被强杀时保证物理制动，须保留物理急停。

## 本机离线测试

PowerShell（在仓库根目录运行）：

```powershell
$env:PYTHONPATH = "$PWD\ros2_ws\src\armpi_ros2"
& 'C:\Users\46361\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe' -m unittest discover -s ros2_ws/src/armpi_ros2/test -v
```

这些测试验证命令转换、轮速映射、限幅、编码器回绕、里程计弧线积分、
IMU 安装方向变换和命令看门狗，不替代 ROS 2 运行时或实车验收。

## 只读车端检查与隔离构建

已有 Windows 控制台 Python 环境包含 paramiko，使用历史可信主机密钥验证新 IP，
不会自动接受未知密钥、改写 known_hosts 或打印密码。

```powershell
$robotIp = '10.217.102.178'  # 换成当前小车 IP
$consolePython = 'C:\Users\46361\AppData\Local\Programs\Python312-embed\python.exe'
$consoleConfig = "$env:LOCALAPPDATA\ArmPiConsole\console_supervisor.json"
& $consolePython tools/ros2_preflight.py --host $robotIp --config $consoleConfig
& $consolePython tools/stage_ros2.py --host $robotIp --config $consoleConfig --build
```

构建工具每次创建全新的 `/home/pi/armpi-ros2-stage.XXXXXX`，输出对应镜像标签；
不会清理旧文件、停止 ROS 1、安装自启动或挂载硬件。要求构建前至少 5 GiB 可用。
需要小车能够正常访问 Ubuntu/ROS 软件源和 GitHub；Docker Hub 无法访问时可用下面的电脑中转。

2026-10-09 首次构建在拉取 `ros:jazzy-ros-base` 元数据时超时。
随后通过电脑下载官方 ARM64 镜像，验证 manifest、配置、10 个压缩分层及解压内容摘要，
再上传到小车导入为 `armpi-ros2-base:jazzy-arm64`。基础容器中的
`ROS_VERSION=2`、`ROS_DISTRO=jazzy` 和 `rclpy` 导入已验证。
这只证明基础运行时可用，不代表自定义驱动和整车迁移通过。

电脑中转（在仓库根目录）：

```powershell
& 'C:\Users\46361\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe' tools/download_ros2_base.py
# --base-archive 使用上一条命令实际输出的文件路径，不使用下面的占位符。
& $consolePython tools/stage_ros2.py --host $robotIp --config $consoleConfig --base-archive '<已校验的归档路径>' --build
```

归档和 `verification.json` 保存在被 Git 忽略的 `tools/runtime/ros2_images/` 中。
构建工具核对远端基础镜像配置摘要；相同镜像已导入时不会再次传输。
初次自定义构建目录为 `/home/pi/armpi-ros2-stage.8WEswk`，标签为
`armpi-ros2-migration:stage-8weswk`；构建/启动验收结果见实施记录。
所有旧临时目录均保留，不把旧目录当成最新源码。
该镜像已构建成功，车端 21 项测试及无硬件节点、TF、WebSocket 验收通过。
该次测试容器已停止，2026-10-09 未启用电机、雷达或机械臂，未切换开机服务。

2026-10-10 同目录更新 ROS 2 里程计后生成新镜像
`armpi-ros2-migration:stage-encoders1`，ID
`sha256:c06d04049a9650aef1044e5811887a4fb6e459871ae7bbedc8cba9b65addca08`。
车端 25 项测试通过，旧镜像保留。最新结果以实施记录的日期章节为准。

首次编译发现固定版本的 LD06 日志头文件把 `pthread.h` 注释掉，新编译器因此报错。
`patches/ldlidar_jazzy_pthread.patch` 只恢复该头文件引用；构建先执行 `git apply --check`，
不静默修改不匹配的驱动版本。没有改变雷达协议、串口或扫描方向。

构建成功后，在 Windows 运行隔离验收工具（镜像名使用工具实际输出）：

```powershell
& $consolePython tools/verify_ros2_stage.py --host $robotIp --config $consoleConfig --image 'armpi-ros2-migration:stage-8weswk'
```

工具创建唯一名称的测试容器，固定 `--network none`，没有 `--privileged`、
`--device` 或宿主挂载。容器内默认只运行模拟底盘、里程计和临时 rosbridge。
先核对 `dry_run=true`、`enable_motion=false`，再向该隔离网络里的 ROS 2 节点发送
模拟命令，检查输出仍为零、编码器/里程计频率、静态 TF，并只读订阅测试 WebSocket。
结束后只停止它刚创建的测试容器，保留日志，不自动重启；ROS 1 不受影响。
通过纯软件验收也不能省略下面的硬件验收。

## 硬件验收前必须完成

1. 核对车端最新驱动、脉冲标定、雷达安装外参和 IMU 安装方向；不能只按照片或旧注释推断。
2. 备份工作区、控制器配置、地图与启动服务，确保回退路径可用。
3. 车体支稳、车轮架空后，停止 ROS 1 对同一电机 I²C 板的占用，再只允许一个新驱动接入。
4. 独立验证前后左右/转向与编码器符号。迁移数学匹配现有源码，不代表现有实车符号一定正确。
5. 核对 CH341 主机内核驱动；把已验证 BMI270 库以只读方式提供到容器 `/opt/bmi270_py`，
   停止旧 BMI270 驱动后才允许 ROS 2 接入。它不是普通 USB 串口 IMU。
6. 验证雷达 `/dev/ld06` 固定名，不回退 `/dev/ttyUSB0`，防止碰到其他串口。
7. 最后单独验证安全控制器和自动探索，再允许落地运动。当前基线没有障碍物控制器。

## 与旧前端的重要不兼容

旧按钮发布 `chassis_control/SetTranslation`，单位 mm/s，X 向右、Y 向前；
新底盘接收 `geometry_msgs/msg/Twist` 的 `/cmd_vel`，单位 m/s，X 向前、Y 向左。
旧前端不能仅通过换 IP/端口就当成 ROS 2 控制台。
ROS 2 的标准消息/服务名称分别为 `包/msg/类型`、`包/srv/类型`；
时间字段为 `sec/nanosec`，不是 ROS 1 的 `secs/nsecs`。
动作组、夹爪、视觉服务也需要独立迁移，不可直接更改类型字符串后试动。

完整迁移范围和当天检查结果见 `../docs/ROS2迁移实施与验收.md`。
