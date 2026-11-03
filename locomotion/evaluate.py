"""Report held-out velocity tracking, falls and distance; optional standalone video."""
import argparse
import json
from pathlib import Path
import numpy as np
from env import ZSL1Env


def evaluate(policy=None, seeds=(101,102,103), randomize=False, speed=.25, video=None):
    env=ZSL1Env(randomize=randomize,speed=speed)
    if video:
        import cv2
        writer=cv2.VideoWriter(str(video),cv2.VideoWriter_fourcc(*'mp4v'),25,(1280,720))
    episodes=[]
    for seed in seeds:
        obs,_=env.reset(seed=seed);rewards=0;rows=[]
        for step in range(1000):
            action=np.zeros(12) if policy is None else policy.predict(obs,deterministic=True)[0]
            obs,r,term,trunc,info=env.step(action);rewards+=r
            if step>=100:rows.append(info)
            if video and seed==seeds[0] and step%2==0:
                writer.write(env.render()[:,:,::-1])
            if term or trunc:break
        episodes.append({'seed':seed,'seconds':(step+1)*.02,'fell':term,'return':rewards,
                         'distance_m':info['distance'],'mean_speed_m_s':float(np.mean([r['forward_speed'] for r in rows])) if rows else 0,
                         'mean_tracking':float(np.mean([r['tracking'] for r in rows])) if rows else 0,
                         'max_tilt_deg':float(np.rad2deg(np.arccos(np.clip(min([r['upright'] for r in rows] or [1]),-1,1)))),
                         'min_height_m':min([r['height'] for r in rows] or [info['height']])})
    if video:writer.release()
    env.close()
    return {'randomized':randomize,'requested_speed_m_s':speed,'episodes':episodes,
            'survival_rate':float(np.mean([not e['fell'] for e in episodes])),
            'mean_return':float(np.mean([e['return'] for e in episodes])),
            'mean_tracking':float(np.mean([e['mean_tracking'] for e in episodes])),
            'mean_speed_m_s':float(np.mean([e['mean_speed_m_s'] for e in episodes]))}


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--model');p.add_argument('--randomize',action='store_true');p.add_argument('--video');p.add_argument('--out',default='evaluation.json');a=p.parse_args()
    policy=None
    if a.model:
        import torch
        from stable_baselines3 import PPO
        torch.set_num_threads(1);policy=PPO.load(a.model,device='cpu')
    report=evaluate(policy,randomize=a.randomize,video=a.video)
    Path(a.out).write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2),flush=True)
