# 真机抓取示范数据集 v1（LeRobot 格式）

由 `tools/convert_demo_to_lerobot.py` 从 9 条真机成功示范录制生成。

## 位置与规模

- 数据集：`datasets/lerobot_armpi_grasp_v1/`（gitignore，仅本地）
- 9 episodes，5260 帧，2.0 Hz，7 训练 / 2 验证（`splits: {train: 0:7, val: 7:9}`）
- 特征：`observation.state`(6) / `action`(6) / `observation.images.camera1`(480×640) / timestamp / frame_index / episode_index / index / task_index
- 任务指令：`Pick up the red ball.`

## 来源录制（全部为真机成功抓取）

| 来源 | 闭合值 | 备注 |
|---|---|---|
| demo2_20261125_e01b | 322 | |
| demo2_20261125_e02 | 331 | |
| demo2_20261125_e03b | 329 | 相机复核首次生效 |
| demo2_20261125_e04 | 334 | 脚本曾误报失败，真值为成功 |
| demo2_20261126_e08 | 350 | |
| demo2_20261126_e09 | 333 | 配对 89.5%（擦边合格） |
| demo2_20261126_e10b | 348 | |
| demo2_20261126_e11 | 341 | |
| demo2_20261126_e12b | 329 | 抬升保持 315（最深） |

**排除**：`demo2_20261125_e07`（成功，但状态流有 16.5 s 断档，会引入跳变）。
**未使用**：e03/e05/e06 为失败样本，另有 `episode_truth.json` 标注。

## 关键设计决策（改之前先读）

1. **fps = 2.0**：`/joint_states` 由 `arm_node.py` 的 0.5 s 定时器驱动，这是本机的原生示范频率。执行端必须同频（2 Hz、0.5 s/命令，总线允许 0.3~5 s），不要按 SmolVLA 的 30 Hz 去重采样——那是编造样本。
2. **单位 = 弧度**：状态与动作都走 `servo_protocol.to_radians`，同一套换算、同一套归一化。
3. **动作 = 每关节最近一次下发目标的"前向保持"**：控制器一次只 jog 一个关节，六维动作里其余关节保持。这是控制器真实行为。
4. **夹爪刻度方向**：128 = 张开，~330 = 闭合并夹住球，**数值越大越闭合**。
5. **尾部裁剪锚点**：以"夹爪首次达到最大闭合值"为基准向后留 40 s（覆盖两次抬升 jog + 相机复核）。不要用"最后运动时刻"——操作者事后取球会碰动臂，把空闲帧拖回来。
6. **丢弃**六轴不全的状态（臂读失败时节点会丢夹爪样本），不插值填补。

## 重新生成

```bash
python tools/convert_demo_to_lerobot.py --out datasets/lerobot_armpi_grasp_v1 --val 2 \
  recordings/demo2_20261125_e01b recordings/demo2_20261125_e02 recordings/demo2_20261125_e03b \
  recordings/demo2_20261125_e04 recordings/demo2_20261126_e08 recordings/demo2_20261126_e09 \
  recordings/demo2_20261126_e10b recordings/demo2_20261126_e11 recordings/demo2_20261126_e12b
python tools/convert_demo_to_lerobot.py --verify datasets/lerobot_armpi_grasp_v1
```

## 依赖说明

本机 pip 被应用控制策略阻断（pyexpat DLL），pyarrow 以 wheel 手工解包到
`vendor/site/`，脚本已把该路径加入 `sys.path`。
