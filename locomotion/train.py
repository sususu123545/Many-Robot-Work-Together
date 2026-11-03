"""PPO residual locomotion with snapshots and evaluation independent of training seeds."""
import argparse
import json
import os
from pathlib import Path
import time
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.vec_env import SubprocVecEnv,VecMonitor
from env import ZSL1Env
from evaluate import evaluate


class CheckpointEvaluation(BaseCallback):
    def __init__(self, root, every=98304):
        super().__init__();self.root=root;self.every=every;self.next=every;self.best=-float('inf');self.started=time.time()
    def _on_step(self):
        if self.num_timesteps<self.next:return True
        self.next+=self.every
        self.model.save(self.root/'latest')
        report=evaluate(self.model,seeds=(201,202),randomize=False)
        report.update(timesteps=self.num_timesteps,elapsed_s=time.time()-self.started)
        with (self.root/'evaluations.jsonl').open('a') as f:f.write(json.dumps(report)+'\n')
        score=report['mean_return']
        if score>self.best:
            self.best=score;self.model.save(self.root/'best');(self.root/'best-evaluation.json').write_text(json.dumps(report,indent=2))
        print('EVALUATION '+json.dumps(report),flush=True)
        (self.root/'progress.json').write_text(json.dumps({'timesteps':self.num_timesteps,'target':self.model._total_timesteps,'elapsed_s':time.time()-self.started,'best_return':self.best},indent=2))
        return True


def make_env(rank):
    def factory():
        e=ZSL1Env(randomize=True);e.reset(seed=17+rank);return e
    return factory


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--steps',type=int,default=1500000);p.add_argument('--envs',type=int,default=12);p.add_argument('--run',default='runs/ppo_flat_v1');p.add_argument('--resume');a=p.parse_args()
    torch.set_num_threads(1);torch.set_num_interop_threads(1)
    root=Path(a.run).resolve();root.mkdir(parents=True,exist_ok=True)
    config={'steps':a.steps,'envs':a.envs,'seed':17,'device':'cpu','algorithm':'PPO residual over analytic trot','action_scale_rad':.15,'policy_hz':50,'physics_hz':500,'randomization':True,'resume':a.resume}
    (root/'training-config.json').write_text(json.dumps(config,indent=2))
    print('CONFIG '+json.dumps(config),flush=True)
    baseline=evaluate();(root/'baseline-evaluation.json').write_text(json.dumps(baseline,indent=2));print('BASELINE '+json.dumps(baseline),flush=True)
    env=VecMonitor(SubprocVecEnv([make_env(i) for i in range(a.envs)],start_method='fork'),filename=str(root/'monitor.csv'))
    if a.resume:model=PPO.load(a.resume,env=env,device='cpu')
    else:model=PPO('MlpPolicy',env,learning_rate=2e-4,n_steps=256,batch_size=1024,n_epochs=5,gamma=.99,gae_lambda=.95,clip_range=.2,ent_coef=.002,target_kl=.025,policy_kwargs={'net_arch':dict(pi=[128,128],vf=[128,128]),'log_std_init':-1.8},seed=17,device='cpu',verbose=1,tensorboard_log=str(root/'tensorboard'))
    try:
        model.learn(total_timesteps=a.steps,callback=CheckpointEvaluation(root),reset_num_timesteps=not bool(a.resume),tb_log_name='ZSL1')
        model.save(root/'final')
        report=evaluate(model,seeds=(301,302,303),randomize=False)
        robust=evaluate(model,seeds=(401,402,403,404,405),randomize=True)
        (root/'final-evaluation.json').write_text(json.dumps({'flat':report,'randomized':robust},indent=2))
        (root/'status.json').write_text(json.dumps({'status':'completed','timesteps':model.num_timesteps,'evaluation':'final-evaluation.json'},indent=2))
        print('FINAL '+json.dumps({'flat':report,'randomized':robust}),flush=True)
    except BaseException as e:
        model.save(root/'interrupted');(root/'status.json').write_text(json.dumps({'status':'interrupted','error':str(e),'timesteps':model.num_timesteps},indent=2));raise
    finally:env.close()
