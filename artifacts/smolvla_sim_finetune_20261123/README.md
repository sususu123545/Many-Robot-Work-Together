# 仿真微调结果（2026-11-23）

在 ycw 上用**已存在**的 Gazebo T10 数据集跑完一次 SmolVLA 微调。这是本项目第一次非零训练更新
（此前 T10 各文档均记「训练更新为 0」）。

## 权重为什么不在仓库里

`model.safetensors` = **1,197,789,256 字节（1.14 GiB）**。GitHub 对普通文件有 **100 MB 硬限制**，
push 会直接被拒；Git LFS 免费额度 1 GiB 也装不下。所以权重只存在于 ycw：

```
/home/ycw/armpi_pro_ws/vla/logs/t10/runs/20261123T104500Z_full3600_train/
  full3600_b8/checkpoints/003600/pretrained_model/model.safetensors

sha256  d8035fa9d429370954a4fb027cb49721589c27cbaf7da629826c8a9bb548e79d
```

要进 GitHub 只能用 **Release 附件**（单文件上限 2 GB），需要网页拖拽或 API token。

## 这里有什么

`checkpoint_003600/` 是完整检查点**除权重以外**的全部 9 个文件（3.4 MB），含归一化统计：

| 文件 | 作用 |
| --- | --- |
| `config.json` | 策略结构（state[6] / camera1[3,480,640] → action[6]，`max_action_dim=32`） |
| `train_config.json` | 训练超参（seed 1000、batch 8、3600 步） |
| `policy_preprocessor.json` + `..._normalizer_processor.safetensors` | 输入归一化统计（本数据集自身的 mean/std） |
| `policy_postprocessor.json` + `..._unnormalizer_processor.safetensors` | 输出反归一化统计 |
| `tokenizer/*` | 语言指令分词器 |

**把 `model.safetensors` 放进这个目录，就是一个可直接 `from_pretrained` 的检查点。**
注意归一化统计来自本数据集，不是 SO100 的那三套。

## 复现（约 12 分钟）

前提：`/home/ycw/armpi_pro_ws/vla/.venv`，数据集
`logs/t07/runs/20261121T120535Z_7612fc4e_supplement/lerobot_gazebo_t10_2_camera_corrected`。

```bash
cd /home/ycw/armpi_pro_ws/vla
export HF_HOME=$PWD/cache/huggingface HF_HUB_OFFLINE=1 TRANSFORMERS_OFFLINE=1
export PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True

.venv/bin/lerobot-train \
  --dataset.repo_id=local/armpi_t07_sim \
  --dataset.root=$PWD/logs/t07/runs/20261121T120535Z_7612fc4e_supplement/lerobot_gazebo_t10_2_camera_corrected \
  --dataset.episodes="[0,1,2,4,5,6,7,8,9,10,11,12,14,15,16,17,18,19]" \
  --policy.type=smolvla \
  --policy.pretrained_path=$PWD/cache/huggingface/hub/models--lerobot--smolvla_base/snapshots/5e8d12a6e2975b0e5e5fce7c8caf47c371d257b6 \
  --policy.device=cuda --policy.push_to_hub=false \
  --output_dir=<新目录>/full3600_b8 \
  --batch_size=8 --steps=3600 --log_freq=100 --save_freq=1800 \
  --num_workers=0 --seed=1000
```

⚠️ **batch 不能上 16**：GPU 上常驻 `llama-server`（Ternary-Bonsai-2-27B）占 20.4 GB，
batch 16 实测 `CUDA out of memory`。要用更大批次必须先停那个服务。

## 复现是否成功：按这三个数对

| 指标 | 期望值 |
| --- | --- |
| 最终训练 loss（step 3600） | **0.045** |
| 留出集归一化 L1（回合 3、13，64 帧） | **0.1034** |
| 「原地不动」基线 L1 | 0.4047（比值 0.2554） |

训练 loss 曲线：`0.374 (100) → 0.156 (500) → 0.126 (900) → 0.053 (2000) → 0.045 (3600)`。
完整曲线见 `train_summary.json`（注意 1000 步以上的 `step` 是 lerobot 的 `NK` 四舍五入值，loss 精确）。

评估命令：

```bash
.venv/bin/python scripts/eval_smolvla_offline.py \
  --checkpoint <run>/full3600_b8/checkpoints/003600/pretrained_model \
  --dataset-root <...>/lerobot_gazebo_t10_2_camera_corrected \
  --episodes 3,13 --samples 64 --out eval_3600.json
```

脚本在 `tools/eval_smolvla_offline.py`（本地），需先上传到 ycw 的 `scripts/`。
它**内建「原地不动」基线**——没有这个基线，平滑演示上 loss 降低说明不了任何事。

## 训练数据

`lerobot_gazebo_t10_2_camera_corrected`：20 回合 / 8,557 帧 / 10 fps / 6 维关节 / 单相机 480×640 /
两条语言指令（"把粉色圆柱放到左/右目标区"）。按官方 `pilot_episode_split.csv`：
**18 条训练（7,707 帧）**，**回合 3、13 留出验证**（G03 布局，左+右）。

## 结果的边界（别当成"能干活了"）

- **离线回放，不是闭环**：没有 Gazebo rollout，**没有任务成功率**。超过"不动"基线只证明模型在
  预测运动，不证明能把东西放到目标区。
- 仿真 **10 fps**，真机总线只有 **3.33 Hz**（一条命令 0.3~5 s）。上真机必须按 3.33 fps 重新采/训。
- 任务是**放粉色圆柱**，不是**抓红球**；仿真相机也不等于真机手眼与畸变。
- `joint5` 全程 std = 0，这个数据集实际只有 5 个自由度在动。
- 训练用了 `train_expert_only`（可训练 1.0 亿 / 总 4.5 亿），视觉编码器冻结。

## 相关文件

- `train_summary.json` — 训练 loss 曲线（36 个点）
- `eval_1800.json` / `eval_3600.json` — 两个检查点的留出集评估
- `docs/ArmPi红球抓取交接.md` §7.7 — 中文说明与上下文
