"""Measure real rollout throughput before increasing environment count."""
import json
from pathlib import Path
import time
import numpy as np
import psutil
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import SubprocVecEnv
from train_terrain import factory

if __name__=='__main__':
    torch.set_num_threads(1);torch.set_num_interop_threads(1)
    model=PPO.load('runs/ppo_terrain_v1/interrupted.zip',device='cpu');results=[]
    for count in [16,24,32]:
        env=SubprocVecEnv([factory(i) for i in range(count)],start_method='fork');obs=env.reset()
        try:
            for _ in range(64):obs,_,_,_=env.step(model.predict(obs,deterministic=True)[0])
            started=time.monotonic();minimum=psutil.virtual_memory().available
            for i in range(1024):
                obs,_,_,_=env.step(model.predict(obs,deterministic=True)[0])
                if i%64==0:
                    minimum=min(minimum,psutil.virtual_memory().available)
                    if minimum<3*1024**3:raise RuntimeError('Available RAM below benchmark reserve')
            elapsed=time.monotonic()-started
            row={'envs':count,'samples_per_second':count*1024/elapsed,'elapsed_s':elapsed,'min_available_ram_gib':minimum/1024**3}
            results.append(row);print(json.dumps(row),flush=True)
        finally:env.close()
    best=max(results,key=lambda r:r['samples_per_second'])
    recommended=best['envs'] if best['samples_per_second']>results[0]['samples_per_second']*1.05 else 16
    report={'measurements':results,'recommended_envs':recommended,'criterion':'highest throughput with at least 5% gain over 16 workers; retain 3 GiB available RAM'}
    Path('runs/parallel-benchmark.json').write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True)
