"""Flat ground, velocity-commanded residual gait learning for the custom ZSL1."""
import json
from pathlib import Path
import gymnasium as gym
from gymnasium import spaces
import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parent
MODEL = ROOT.parent/'models'/'zsl1'
HOME = np.tile([0., .8, -1.5], 4)
PHASES = np.array([0., .5, .5, 0.])
DT = .02


def reference(t, speed=.25, frequency=1.7, clearance=.045, height=.30):
    """Trot with an analytic foot path; the learned policy supplies joint residuals."""
    blend = np.clip((t-.5)/1., 0., 1.)
    phase = ((max(0., t-.5)*frequency) + PHASES) % 1.
    duty = .60
    stride = speed/frequency*duty
    stance = phase < duty
    u = (phase-duty)/(1-duty)
    x = np.where(stance, stride*(.5-phase/duty), stride*(-.5+u))
    z = -height + np.where(stance, 0., clearance*np.sin(np.pi*u))
    l1, l2 = .2, .2136600018
    knee = -np.arccos(np.clip((x*x+z*z-l1*l1-l2*l2)/(2*l1*l2), -.999, .999))
    hip = np.arctan2(-x, -z)-np.arctan2(l2*np.sin(knee), l1+l2*np.cos(knee))
    q = np.stack([np.zeros(4), hip, knee], axis=-1).reshape(-1)
    return HOME+blend*(q-HOME), phase


def observation(q, qd, quat, vworld, gyro, command, previous, t, contacts):
    rot = np.empty(9); mujoco.mju_quat2Mat(rot, np.asarray(quat, dtype=np.float64))
    rot = rot.reshape(3,3)
    phase = max(0., t-.5)*1.7*2*np.pi
    return np.clip(np.concatenate([rot.T@vworld*2, np.asarray(gyro)*.25,
                    rot.T@np.array([0.,0.,-1.]), command, np.asarray(q)-HOME,
                    np.asarray(qd)*.05, previous, [np.sin(phase),np.cos(phase)]]), -10, 10).astype(np.float32)


class ZSL1Env(gym.Env):
    metadata = {'render_modes':['rgb_array'], 'render_fps':50}
    def __init__(self, randomize=True, speed=.25, render_mode=None, model_path=None):
        self.action_space = spaces.Box(-1.,1.,(12,),np.float32)
        self.observation_space = spaces.Box(-10.,10.,(50,),np.float32)
        self.randomize = randomize; self.speed = speed; self.render_mode = render_mode
        self.renderer = None; self.previous = np.zeros(12); self.filtered_tau = np.zeros(12)
        self.contact_bits = np.zeros(4)
        self.command = np.array([speed,0.,0.]);self.strength=1.;self.lag=.004
        self._load_model(model_path or MODEL/'train_scene.xml')
    def _load_model(self,path):
        self.model = mujoco.MjModel.from_xml_path(str(path));self.data = mujoco.MjData(self.model)
        self.nominal_mass = self.model.body_mass.copy();self.nominal_inertia = self.model.body_inertia.copy()
        self.foot_ids = [mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_GEOM,f'{leg}_foot_collision') for leg in ['FL','FR','RL','RR']]
        self.foot_bodies = self.model.geom_bodyid[self.foot_ids]
        self.floor = mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_GEOM,'floor')
        self.terrain_ids=set(np.flatnonzero((self.model.geom_bodyid==0)&(self.model.geom_contype>0)).tolist())
        self.ground_groups=np.zeros(6,dtype=np.uint8)
        for g in self.terrain_ids:self.ground_groups[self.model.geom_group[g]]=1
    def ground_height(self,x,y):
        geom=np.array([-1],dtype=np.int32)
        distance=mujoco.mj_ray(self.model,self.data,np.array([x,y,5.]),np.array([0.,0.,-1.]),self.ground_groups,1,-1,geom)
        return 5.-distance if distance>=0 else 0.
    def _contacts(self):
        self.contact_bits[:] = 0
        bad = 0
        for c in self.data.contact:
            pair = [int(c.geom1),int(c.geom2)]
            for i,f in enumerate(self.foot_ids):
                if f in pair and any(g in self.terrain_ids for g in pair):self.contact_bits[i]=1
            if any(g in self.terrain_ids for g in pair) and not any(f in pair for f in self.foot_ids):bad+=1
            if all(self.model.geom_bodyid[g]>0 for g in pair):bad+=1
        return bad
    def _obs(self):
        return observation(self.data.qpos[7:],self.data.qvel[6:],self.data.qpos[3:7],
                           self.data.qvel[:3],self.data.qvel[3:6],self.command,self.previous,
                           self.data.time,self.contact_bits)
    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        mujoco.mj_resetData(self.model,self.data)
        self.model.body_mass[:]=self.nominal_mass;self.model.body_inertia[:]=self.nominal_inertia
        if self.randomize:
            scale=self.np_random.uniform(.9,1.1)
            self.model.body_mass[1:]*=scale;self.model.body_inertia[1:]*=scale
            self.model.geom_friction[:,0]=self.np_random.uniform(.5,1.1)
            self.strength=self.np_random.uniform(.85,1.);self.lag=self.np_random.uniform(.002,.006)
            self.command[0]=self.np_random.uniform(.15,.35)
        else:self.command[0]=self.speed;self.strength=1.;self.lag=.004
        mujoco.mj_setConst(self.model,self.data)
        # mj_setConst uses qpos as scratch; restore the model's home configuration afterwards.
        mujoco.mj_resetData(self.model,self.data)
        if self.randomize:self.data.qpos[7:]+=self.np_random.uniform(-.015,.015,12)
        self.data.qpos[2]=.34
        self.previous[:]=0;self.filtered_tau[:]=0
        mujoco.mj_forward(self.model,self.data);self._contacts()
        self.start_x=float(self.data.qpos[0]);self.steps=0
        return self._obs(),{}
    def step(self, action):
        action=np.clip(np.asarray(action,dtype=float),-1,1)
        if not np.all(np.isfinite(action)):raise ValueError('Nonfinite action')
        ref,phase=reference(self.data.time,self.command[0])
        target=ref+.15*action
        limits=self.model.jnt_range[1:]
        target=np.clip(target,limits[:,0]+.01,limits[:,1]-.01)
        tau2=0.
        for _ in range(10):
            tau=np.clip(70*(target-self.data.qpos[7:])-2*self.data.qvel[6:],-28*self.strength,28*self.strength)
            # Source USD maxJointVelocity is 1604.28 deg/s = 28 rad/s. Suppress drive into overspeed.
            tau=np.where((np.abs(self.data.qvel[6:])>28)&(tau*self.data.qvel[6:]>0),0,tau)
            self.filtered_tau+=(tau-self.filtered_tau)*(.002/(self.lag+.002))
            self.data.ctrl[:]=self.filtered_tau;tau2+=np.mean(self.filtered_tau**2)
            mujoco.mj_step(self.model,self.data)
        self.steps+=1;bad=self._contacts()
        rot=self.data.xmat[1].reshape(3,3);vel=rot.T@self.data.qvel[:3]
        height=self.data.qpos[2]-self.ground_height(*self.data.qpos[:2]);upright=rot[2,2]
        tracking=np.exp(-np.sum((vel[:2]-self.command[:2])**2)/.04)
        yaw=np.exp(-(self.data.qvel[5]-self.command[2])**2/.25)
        reward=1.5*tracking+.3*yaw+.3*upright-.8*(height-.33)**2*10
        reward-=.15*np.sum(self.data.qvel[3:5]**2)+.5*self.data.qvel[2]**2
        reward-=.00025*tau2/10+.015*np.sum((action-self.previous)**2)+.01*np.sum(action**2)+.25*bad
        terminated=bool(height<.19 or upright<.65 or not np.all(np.isfinite(self.data.qpos)))
        if terminated:reward-=5
        truncated=self.steps>=1000
        self.previous=action.copy()
        info={'forward_speed':float(vel[0]),'height':float(height),'distance':float(self.data.qpos[0]-self.start_x),
              'tracking':float(tracking),'bad_contacts':bad,'upright':float(upright)}
        return self._obs(),float(reward),terminated,truncated,info
    def render(self):
        if self.renderer is None:self.renderer=mujoco.Renderer(self.model,720,1280)
        self.renderer.update_scene(self.data,camera='follow')
        return self.renderer.render()
    def close(self):
        if self.renderer:self.renderer.close();self.renderer=None
