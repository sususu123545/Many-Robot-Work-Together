"""PPO training on actual MATRiX rollouts; never decrypts or updates official weights."""
import argparse
import fcntl
import hashlib
import json
from pathlib import Path
import signal
import subprocess
import time
import numpy as np
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.monitor import Monitor
from live_env import OfficialResidualEnv,ROOT


class Progress(BaseCallback):
    def __init__(self,root):super().__init__();self.root=root;self.started=time.time();self.last=0
    def _on_step(self):
        if self.num_timesteps-self.last>=512:
            self.last=self.num_timesteps;self.model.save(self.root/f'checkpoint-{self.num_timesteps}')
            self.model.save(self.root/'latest')
            p={'steps':self.num_timesteps,'target':self.model._total_timesteps,'elapsed_s':time.time()-self.started,'status':'training'}
            (self.root/'progress.json').write_text(json.dumps(p,indent=2));print('PROGRESS '+json.dumps(p),flush=True)
        return True


def evaluate(env,model=None,seeds=(101,102)):
    env.randomize=False;reports=[]
    for seed in seeds:
        obs,_=env.reset(seed=seed)
        while True:
            action=np.zeros(2) if model is None else model.predict(obs,deterministic=True)[0]
            obs,_,_,done,_=env.step(action)
            if done:break
        reports.append(env.metrics())
    result={k:float(np.mean([r[k] for r in reports])) for k in reports[0]};result['episodes']=reports
    return result


def gateway(mode):
    script='/mnt/llm/matrix_install/official-zsl1-control.py'
    subprocess.run(['/usr/bin/python3',script,mode],check=True)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--steps',type=int,default=8192);p.add_argument('--run',default='runs/official_live_v1');p.add_argument('--resume');a=p.parse_args()
    torch.set_num_threads(1);torch.set_num_interop_threads(1)
    control_lock=open(ROOT/'.official-control.lock','a')
    try:fcntl.flock(control_lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:raise SystemExit('Another residual training/evaluation process owns control')
    root=(ROOT/a.run).resolve();root.mkdir(parents=True,exist_ok=True)
    original=Path('/mnt/llm/matrix_install/runtime/MATRiX_v1.1.2/UeSim/Plugins/MatrixMotionControl/Source/ThirdParty/MatrixMC/models/xg/policy_mix_walk')
    manifest={'method':'high-level PPO residual on frozen official XGB controller','official_weights_updated':False,
        'data_source':'live MATRiX rt/odom, 20 Hz rollouts','baseline_model':str(original),'baseline_sha256':hashlib.sha256(original.read_bytes()).hexdigest(),
        'residual_action':'forward normalized input +/-0.10, yaw input +/-0.10','seed':19,'timesteps':a.steps}
    (root/'manifest.json').write_text(json.dumps(manifest,indent=2))
    (root/'progress.json').write_text(json.dumps({'status':'baseline_evaluation','target':a.steps},indent=2))
    pidfile=Path('/mnt/llm/matrix_install/official-input.pid')
    if pidfile.exists():
        pid=int(pidfile.read_text())
        try:
            cmd=Path(f'/proc/{pid}/cmdline').read_bytes()
            if b'official-zsl1-control.py' in cmd and b'daemon' in cmd:subprocess.run(['kill','-TERM',str(pid)],check=True);time.sleep(.15)
        except FileNotFoundError:pass
    env=None;model=None
    def interrupt(*_):raise KeyboardInterrupt('User stopped training')
    signal.signal(signal.SIGTERM,interrupt);signal.signal(signal.SIGINT,interrupt)
    try:
        env=OfficialResidualEnv(log_path=root/'live-telemetry.jsonl')
        baseline=evaluate(env);(root/'baseline.json').write_text(json.dumps(baseline,indent=2));print('BASELINE '+json.dumps(baseline),flush=True)
        if baseline['moving_mean_abs_speed_m_s']<.20:raise RuntimeError('Baseline motion was not verified; training rejected')
        env.randomize=True;mon=Monitor(env,filename=str(root/'monitor.csv'))
        if a.resume:model=PPO.load(a.resume,env=mon,device='cpu')
        else:
            model=PPO('MlpPolicy',mon,n_steps=128,batch_size=64,n_epochs=6,learning_rate=3e-4,
                gamma=.98,gae_lambda=.95,ent_coef=0.,target_kl=.03,policy_kwargs={'net_arch':dict(pi=[64,64],vf=[64,64]),'log_std_init':-1.5},seed=19,device='cpu',verbose=1,tensorboard_log=str(root/'tensorboard'))
            with torch.no_grad():model.policy.action_net.weight.zero_();model.policy.action_net.bias.zero_()
        model.learn(total_timesteps=a.steps,callback=Progress(root),reset_num_timesteps=not bool(a.resume))
        model.save(root/'final')
        result=evaluate(env,model,seeds=(201,202));(root/'trained-evaluation.json').write_text(json.dumps(result,indent=2))
        passed=result['speed_rmse_m_s']<baseline['speed_rmse_m_s']*.95 and result['heading_rmse_rad']<=baseline['heading_rmse_rad']*1.15+.005 and result['max_tilt_deg']<15
        report={'status':'completed','timesteps':model.num_timesteps,'method':manifest['method'],'official_weights_updated':False,'baseline':baseline,'trained':result,'improvement_verified':bool(passed),'enabled_by_default':False,'residual_checkpoint':str(root/'final.zip')}
        (root/'result.json').write_text(json.dumps(report,indent=2));(root/'progress.json').write_text(json.dumps({'status':'completed','steps':model.num_timesteps,'improvement_verified':bool(passed)},indent=2));print('RESULT '+json.dumps(report),flush=True)
    except BaseException as e:
        if model:model.save(root/'interrupted')
        interrupted={'status':'interrupted','error':str(e),'steps':model.num_timesteps if model else 0}
        (root/'status.json').write_text(json.dumps(interrupted,indent=2));(root/'progress.json').write_text(json.dumps(interrupted,indent=2));raise
    finally:
        if env:env.close()
        gateway('neutral')
        subprocess.Popen(['/usr/bin/python3','/mnt/llm/matrix_install/official-zsl1-control.py','daemon'],stdout=open(root/'gateway.log','a'),stderr=subprocess.STDOUT,start_new_session=True)
