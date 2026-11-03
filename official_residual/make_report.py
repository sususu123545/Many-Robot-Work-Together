"""Render measured evaluation traces and an honest Chinese report."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

p=argparse.ArgumentParser();p.add_argument('run',type=Path);a=p.parse_args();root=a.run
report=json.loads((root/'result.json').read_text());manifest=json.loads((root/'manifest.json').read_text())
rows=[json.loads(x) for x in (root/'live-telemetry.jsonl').read_text().splitlines()]
episodes=sorted(set(x['episode'] for x in rows));groups={}
for name,ids in [('Official baseline',episodes[:2]),('Trained residual',episodes[-2:])]:
    groups[name]=[[x for x in rows if x['episode']==ep] for ep in ids]
fig,axes=plt.subplots(3,1,figsize=(11,8),sharex=True,layout='constrained')
colors=['#64748b','#007f75']
for (name,traces),color in zip(groups.items(),colors):
    t=np.arange(len(traces[0]))*.05
    for ax,key in zip(axes,['actual_v','tilt_deg','input_forward']):
        array=np.array([[x[key] for x in trace] for trace in traces])
        ax.plot(t,array.mean(axis=0),color=color,label=name,lw=1.6)
        ax.fill_between(t,array.min(axis=0),array.max(axis=0),color=color,alpha=.14)
axes[0].plot(t,[x['desired_v'] for x in groups['Official baseline'][0]],'--',color='#d97706',label='Requested speed',lw=1.3)
for ax,label in zip(axes,['Forward speed (m/s)','Body tilt (deg)','Forward input (normalized)']):
    ax.set_ylabel(label);ax.grid(alpha=.2)
axes[0].legend(loc='lower left',ncol=3,fontsize=9);axes[-1].set_xlabel('Time in evaluation episode (s)')
fig.suptitle('ZSL1 + frozen official controller: live MATRiX evaluation\nTwo episodes per condition; shaded range shows the two measurements',fontsize=12)
fig.savefig(root/'evaluation.png',dpi=160);plt.close(fig)
b=report['baseline'];r=report['trained'];improvement=(1-r['speed_rmse_m_s']/b['speed_rmse_m_s'])*100
official_unchanged=hashlib.sha256(Path(manifest['baseline_model']).read_bytes()).hexdigest()==manifest['baseline_sha256']
report['official_model_hash_unchanged']=official_unchanged
(root/'result.json').write_text(json.dumps(report,indent=2))
result='达到预设改善标准' if report['improvement_verified'] else '未达到预设改善标准，保留原官方控制作为默认'
text=f'''# ZSL1 官方步态修正层训练结果

完成 {report['timesteps']:,} 步 PPO 训练。训练数据直接来自 MATRiX 中的 ZSL1。{result}。

| 指标 | 官方基线 | 加上训练后的修正层 |
|---|---:|---:|
| 速度均方根误差（米/秒） | {b['speed_rmse_m_s']:.4f} | {r['speed_rmse_m_s']:.4f} |
| 稳态速度均方根误差（米/秒） | {b['steady_speed_rmse_m_s']:.4f} | {r['steady_speed_rmse_m_s']:.4f} |
| 航向均方根误差（度） | {np.degrees(b['heading_rmse_rad']):.3f} | {np.degrees(r['heading_rmse_rad']):.3f} |
| 两次测试中最大机身倾角（度） | {max(x['max_tilt_deg'] for x in b['episodes']):.2f} | {max(x['max_tilt_deg'] for x in r['episodes']):.2f} |

速度误差变化：{improvement:.1f}%（正数表示降低）。每种条件测试两轮，每轮 20 秒，目标速度为停止或正负 0.50 米/秒。图中的阴影是两次测量的范围，不能代替大量重复试验的置信区间。

官方控制器权重没有训练或解密，其部署文件 SHA-256 校验{'一致' if official_unchanged else '不一致，需检查'}。新增权重保存为 `final.zip`，适用于本项目的 `live_env.py` 输入和动作定义。默认桌面控制仍使用原官方控制器。

![测试曲线](evaluation.png)

本次只验证平坦场景与上述速度。复杂地形和实机表现仍需单独测试。
'''
(root/'训练结果.md').write_text(text)
print(json.dumps({'steps':report['timesteps'],'speed_rmse_before':b['speed_rmse_m_s'],'speed_rmse_after':r['speed_rmse_m_s'],'improvement_percent':improvement,'passed':report['improvement_verified'],'official_hash_unchanged':official_unchanged},indent=2))
