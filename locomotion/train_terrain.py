"""Fine-tune our existing ZSL1 policy across independent collidable terrains."""
import argparse
import json
from pathlib import Path
import signal
import time
import numpy as np
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.vec_env import SubprocVecEnv,VecMonitor
from terrain_env import TerrainEnv,KINDS


def evaluate(model,levels=(0,),seeds=(901,902),randomize=True):
    reports=[]
    for kind in KINDS:
        for level in ((0,) if kind=='flat' else levels):
            env=TerrainEnv(kind=kind,level=level,curriculum=False,randomize=randomize)
            for seed in seeds:
                obs,_=env.reset(seed=seed);error=[];distance=0.;ended=False
                while not ended:
                    action=model.predict(obs,deterministic=True)[0]
                    obs,_,failed,done,info=env.step(action);ended=failed or done
                    error.append((info['forward_speed']-env.command[0])**2);distance=info['distance']
                reports.append({'terrain':kind,'level':level,'seed':seed,'success':bool(info['is_success']),
                    'distance_m':distance,'seconds':env.steps*.02,'speed_rmse_m_s':float(np.sqrt(np.mean(error))),
                    'max_tilt_deg':env.episode_tilt,'fell':bool(failed)})
            env.close()
    return {'episodes':reports,'success_rate':float(np.mean([r['success'] for r in reports])),
        'speed_rmse_m_s':float(np.mean([r['speed_rmse_m_s'] for r in reports]))}


class Progress(BaseCallback):
    def __init__(self,root,start,target):
        super().__init__();self.root=root;self.start=start;self.target=target;self.last_snapshot=0;self.last_save=0;self.began=time.time()
    def _on_step(self):
        trained=self.num_timesteps-self.start
        if trained-self.last_snapshot>=4096:
            self.last_snapshot=trained
            states=self.training_env.env_method('snapshot')
            p={'status':'training','additional_steps':trained,'target_additional_steps':self.target,
                'total_steps':self.num_timesteps,'envs':len(states),'elapsed_s':time.time()-self.began,
                'source':'our PPO flat policy; official network weights are not used',
                'curriculum':[{'rank':s['rank'],'terrain':s['terrain'],'level':s['level'],'episodes':s['episodes'],'successes':s['successes']} for s in states]}
            for name,data in [('progress.json',p),('live-states.json',{'wall':time.time(),'status':'training','states':states})]:
                tmp=self.root/(name+'.tmp');tmp.write_text(json.dumps(data));tmp.replace(self.root/name)
        if trained-self.last_save>=98304:
            self.last_save=trained;self.model.save(self.root/'latest');self.model.save(self.root/f'checkpoint-{trained}')
            print('CHECKPOINT '+json.dumps({'additional_steps':trained,'total_steps':self.num_timesteps}),flush=True)
        return True


def factory(rank,level=0):
    def make():
        e=TerrainEnv(rank=rank,level=level);e.reset(seed=33+rank);return e
    return make


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--steps',type=int,default=2000000);p.add_argument('--envs',type=int,default=16)
    p.add_argument('--run',default='runs/ppo_terrain_v1');p.add_argument('--resume',default='runs/ppo_flat_v1/final.zip');a=p.parse_args()
    torch.set_num_threads(1);torch.set_num_interop_threads(1)
    root=Path(a.run).resolve();root.mkdir(parents=True,exist_ok=True)
    config={'method':'PPO joint residual over analytic trot, resumed from our learned flat policy',
        'official_weights_updated':False,'resume':str(Path(a.resume).resolve()),'envs':a.envs,'additional_steps':a.steps,
        'terrains':KINDS,'stairs_risers_m':[.02,.04,.06,.08,.10],'slope_degrees':[3,5,7,10,12],
        'rough_heights_m':[.01,.02,.03,.04,.05],'curriculum':'increase after 3 consecutive 6.2 m successes; decrease after 3 failures',
        'policy_hz':50,'physics_hz':500,'command_m_s':[.23,.32],'observation_dim':50,'action_dim':12,'seed':33}
    (root/'training-config.json').write_text(json.dumps(config,indent=2))
    (root/'progress.json').write_text(json.dumps({'status':'baseline_evaluation','envs':a.envs},indent=2))
    model=PPO.load(a.resume,device='cpu');start=model.num_timesteps
    baseline=evaluate(model);(root/'baseline-evaluation.json').write_text(json.dumps(baseline,indent=2));print('BASELINE '+json.dumps(baseline),flush=True)
    previous_progress=Path(a.resume).parent/'live-states.json'
    previous_states=json.loads(previous_progress.read_text())['states'] if previous_progress.exists() else []
    levels=[previous_states[i%len(previous_states)]['level'] if previous_states else 0 for i in range(a.envs)]
    config['initial_curriculum_levels']=levels;(root/'training-config.json').write_text(json.dumps(config,indent=2))
    env=VecMonitor(SubprocVecEnv([factory(i,levels[i]) for i in range(a.envs)],start_method='fork'),filename=str(root/'monitor.csv'))
    model=PPO.load(a.resume,env=env,device='cpu');model.learning_rate=1e-4;model.lr_schedule=lambda _:1e-4;model.ent_coef=.001
    model.tensorboard_log=str(root/'tensorboard')
    def stop(*_):raise KeyboardInterrupt('Training stopped')
    signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop)
    interrupted=False
    try:
        model.learn(total_timesteps=a.steps,callback=Progress(root,start,a.steps),reset_num_timesteps=False,tb_log_name='ZSL1_terrain')
        model.save(root/'final')
        (root/'progress.json').write_text(json.dumps({'status':'final_evaluation','additional_steps':model.num_timesteps-start,'envs':a.envs}))
        result=evaluate(model,levels=(0,1,2,3,4));(root/'final-evaluation.json').write_text(json.dumps(result,indent=2))
        original=evaluate(PPO.load('runs/ppo_flat_v1/final.zip',device='cpu'))
        (root/'original-flat-policy-evaluation.json').write_text(json.dumps(original,indent=2))
        status={'status':'completed','additional_steps':model.num_timesteps-start,'total_steps':model.num_timesteps,'envs':a.envs,
            'baseline_level0':baseline,'evaluation':result,'official_weights_updated':False,'checkpoint':str(root/'final.zip')}
        (root/'result.json').write_text(json.dumps(status,indent=2));(root/'progress.json').write_text(json.dumps({k:v for k,v in status.items() if k not in ['baseline_level0','evaluation']},indent=2))
        print('FINAL '+json.dumps(status),flush=True)
    except BaseException as error:
        interrupted=True
        model.save(root/'interrupted');(root/'progress.json').write_text(json.dumps({'status':'interrupted','error':str(error),'additional_steps':model.num_timesteps-start,'total_steps':model.num_timesteps,'envs':a.envs},indent=2));raise
    finally:
        if interrupted:
            # A signal can interrupt step_wait after only some pipes were consumed.
            # Do not call its normal close path, which could wait on an empty pipe.
            raw=env.venv
            for process in raw.processes:process.terminate()
            for process in raw.processes:
                process.join(timeout=2)
                if process.is_alive():process.kill();process.join(timeout=2)
            for remote in raw.remotes:remote.close()
            raw.closed=True
        else:env.close()
