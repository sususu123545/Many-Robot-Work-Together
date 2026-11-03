"""Independent ZSL1 terrain environments with a measured success curriculum."""
from pathlib import Path
import numpy as np
from env import ZSL1Env,MODEL

TERRAINS=MODEL.parent/'zsl1_terrains'
KINDS=['flat','stairs','slope','rough']


class TerrainEnv(ZSL1Env):
    def __init__(self,rank=0,kind=None,level=0,curriculum=True,randomize=True):
        self.rank=rank;self.kind=kind or KINDS[rank%4];self.level=level;self.curriculum=curriculum
        self.loaded_level=level;self.success_streak=0;self.failure_streak=0
        self.episodes=0;self.successes=0;self.last_success=False
        super().__init__(randomize=randomize,model_path=self.scene())
    def scene(self):return TERRAINS/('flat.xml' if self.kind=='flat' else f'{self.kind}-{self.level}.xml')
    def reset(self,*,seed=None,options=None):
        if self.loaded_level!=self.level:
            if self.renderer:self.renderer.close();self.renderer=None
            self._load_model(self.scene());self.loaded_level=self.level
        obs,info=super().reset(seed=seed,options=options)
        self.command[0]=float(self.np_random.uniform(.23,.32)) if self.randomize else .27
        self.episode_tilt=0.;self.episode_contacts=0
        return self._obs(),info
    def step(self,action):
        obs,reward,terminated,_,info=super().step(action)
        self.episode_tilt=max(self.episode_tilt,float(np.degrees(np.arccos(np.clip(info['upright'],-1,1)))))
        self.episode_contacts+=info['bad_contacts']
        success=not terminated and info['distance']>=6.2
        truncated=bool(success or self.steps>=2000)
        if success:reward+=8.
        info.update(terrain=self.kind,level=self.loaded_level,is_success=bool(success))
        if terminated or truncated:
            self.episodes+=1;self.successes+=int(success);self.last_success=bool(success)
            if success:self.success_streak+=1;self.failure_streak=0
            else:self.failure_streak+=1;self.success_streak=0
            if self.curriculum and self.kind!='flat':
                if self.success_streak>=3:self.level=min(4,self.level+1);self.success_streak=0
                elif self.failure_streak>=3:self.level=max(0,self.level-1);self.failure_streak=0
        return obs,reward,terminated,truncated,info
    def snapshot(self):
        return {'rank':self.rank,'terrain':self.kind,'level':self.loaded_level,'next_level':self.level,'scene':str(self.scene()),
            'qpos':self.data.qpos.tolist(),'time':float(self.data.time),'steps':self.steps,'episodes':self.episodes,
            'successes':self.successes,'command':self.command.tolist(),'last_success':self.last_success}
