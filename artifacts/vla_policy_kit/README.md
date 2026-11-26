# 真机测试工具包（策略服务侧）

这套东西用来把你的 SmolVLA 微调接到真机上。机器人侧我们已经写好并在本机跑着，
你只需要在这里起一个 HTTP 服务。

## 三步

### 1. 先用 mock 模式把协议跑通（不需要 checkpoint）

```bash
python policy_server_example.py --mock --port 8000
```

另开一个终端自检：

```bash
python check_server.py --endpoint http://127.0.0.1:8000/act
```

`check_server.py` 会用**真实观测**（`samples/sample_observation.json`，取自训练集
第 0 条 episode 的下降中段帧）去打你的服务，并按**机器人侧完全相同的规则**校验
返回：维度、有限性、control_dt、output_kind、动作块长度。全部 OK 才算协议通了。

### 2. 接上 checkpoint（改 3 处，都标了 `ADAPT`）

```bash
python policy_server_example.py --ckpt <你的 pretrained_model 目录> --port 8000
```

`policy_server_example.py` 里三处 `# ADAPT`：

1. **加载模型**：`SmolVLAPolicy.from_pretrained(...)`（按你训练脚本的实际调用改）；
2. **构造 batch**：`observation.state`(1×6) + `observation.images.camera1`(1×3×480×640，
   0~1) + `task` —— 键名必须和数据集完全一致；
3. **出动作**：`predict_action_chunk(...)` → 转成 list，**确认输出已在物理弧度**
   （归一化应当已经被 checkpoint 里的 preprocessor 消化掉）。

接上之后**先别删 mock**：用 `--ckpt` 起服务，再跑一次 `check_server.py`，看动作
数值是否落在合理范围（|值| > 4 rad 基本就是没反归一化，自检会直接报出来）。

### 3. 告诉我们你的地址

``
python check_server.py --endpoint http://<Tailscale IP>:8000/act --rounds 5
``

把 Tailscale IP 和端口发过来，我们这边跑影子模式（臂不动，只记录你的输出）。

## 关于延迟

自检会打印每轮往返毫秒数。我们的预算是 **500 ms**（2 Hz），其中推理占约 180 ms、
网络约 40 ms。超过 500 ms 的块我们会丢掉重发，不排队——所以看到 600 ms 以上
要查一下是不是首次推理的预热。

## 关于 joint1/joint2/joint3

你反馈这三维比"照抄当前状态"还差。真机第一轮我们会跑**影子模式**量化它们在
真实观测下的抖动幅度。如果确认是噪声，会切成**混合控制**：底座/肩/肘走我们
已验证的脚本下降，**策略只接管 joint4 + r_joint**。到时候会向你提一个"只回这两
维、其余原样返回当前状态"的变体请求——提前说一声，你那边只要改一行。

## 文件清单

| 文件 | 作用 |
|---|---|
| `CONTRACT.md` | 协议全文（请求/响应字段、我们做的校验、延迟预算） |
| `policy_server_example.py` | 可直接运行的服务骨架（`--mock` 免模型） |
| `check_server.py` | 协议自检（与我们机器人侧同一套规则） |
| `samples/sample_observation.json` | 真实观测（6 维状态 + base64 JPEG） |
| `samples/sample_frame.jpg` | 同一帧的图片，确认视角用 |
