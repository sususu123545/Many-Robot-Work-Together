"""Train a high-level residual over the frozen official XGB runtime, in live MATRiX."""
from collections import deque
import json
import math
from pathlib import Path
import socket
import subprocess
import threading
import time
import gymnasium as gym
from gymnasium import spaces
import numpy as np
import zenoh
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message

ROOT=Path(__file__).resolve().parent


def wrap(x):return (x+math.pi)%(2*math.pi)-math.pi


class OfficialResidualEnv(gym.Env):
    """Only writes localhost high-level commands; official joint policy stays frozen."""
    def __init__(self, log_path=None, horizon=20., randomize=True):
        self.action_space=spaces.Box(-1.,1.,(2,),np.float32)
        self.observation_space=spaces.Box(-10.,10.,(9,),np.float32)
        self.horizon=horizon;self.randomize=randomize;self.dt=.05
        self.lock=threading.Lock();self.frames=deque(maxlen=30)
        self.running=True;self.command={'forward':0.,'lateral':0.,'yaw':0.}
        self.command_time=0.;self.action_until=0.;self.pending_action=None
        self.socket=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.sender=threading.Thread(target=self._send_loop,daemon=True);self.sender.start()
        c=zenoh.Config();c.insert_json5('mode','"client"');c.insert_json5('connect/endpoints','["tcp/127.0.0.1:7447"]');c.insert_json5('scouting/multicast/enabled','false')
        self.session=zenoh.open(c);self.sub=self.session.declare_subscriber('rt/odom',self._receive)
        self.file=open(log_path,'w',buffering=1) if log_path else None
        self.previous=np.zeros(2);self.filtered=np.zeros(2);self.integral=0.
        self.center=None;self.heading=None;self.origin_heading=None;self.speed=.50;self.step_index=0;self.episode=0;self.last_desired=0.
        self.rows=[];self.next_tick=time.monotonic();self.started=False
        deadline=time.monotonic()+8
        while not self.frames:
            if time.monotonic()>deadline:raise RuntimeError('No rt/odom from MATRiX')
            time.sleep(.02)
    def _receive(self,sample):
        try:
            m=deserialize_message(sample.payload.to_bytes(),Odometry);p=m.pose.pose.position;q=m.pose.pose.orientation
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            tilt=2*math.acos(min(1.,math.hypot(q.w,q.z)))
            row=(time.monotonic(),m.header.stamp.sec+m.header.stamp.nanosec*1e-9,p.x,p.y,p.z,yaw,tilt)
            if not all(math.isfinite(v) for v in row):return
            with self.lock:self.frames.append(row)
        except Exception as e:
            print('ODOM_DECODE_ERROR',str(e),flush=True)
    def _send_loop(self):
        while self.running:
            now=time.monotonic()
            with self.lock:
                cmd=dict(self.command) if now-self.command_time<.18 else {'forward':0.,'lateral':0.,'yaw':0.}
                if now<self.action_until:cmd['action']=self.pending_action
            self.socket.sendto(json.dumps(cmd).encode(),('127.0.0.1',7447));time.sleep(.025)
    def set_command(self,forward=0.,yaw=0.,action=None):
        with self.lock:
            self.command={'forward':float(np.clip(forward,-.30,.30)),'lateral':0.,'yaw':float(np.clip(yaw,-.10,.10))}
            self.command_time=time.monotonic()
            if action:self.pending_action=action;self.action_until=time.monotonic()+.09
    def state(self):
        with self.lock:r=list(self.frames)
        if not r or time.monotonic()-r[-1][0]>.2:
            self.set_command();raise RuntimeError('MATRiX odometry is stale; stopped')
        cur=r[-1];old=min(r,key=lambda x:abs(x[1]-(cur[1]-.10)));dt=cur[1]-old[1]
        if dt<=.001:vx=vy=w=0.
        else:vx=(cur[2]-old[2])/dt;vy=(cur[3]-old[3])/dt;w=wrap(cur[5]-old[5])/dt
        v=vx*math.cos(cur[5])+vy*math.sin(cur[5])
        if cur[4]<.24 or cur[6]>math.radians(25):
            self.set_command(action='stand');raise RuntimeError('Pose safety boundary reached')
        return {'x':cur[2],'y':cur[3],'height':cur[4],'yaw':cur[5],'tilt':cur[6],'v':v,'w':w}
    def desired(self,t=None):
        t=self.step_index*self.dt if t is None else t
        phase=t%10
        return 0. if phase<2 or 5<=phase<7 else (self.speed if phase<5 else -self.speed)
    def _obs(self,st):
        return np.clip(np.array([self.desired()/.5,st['v']/.5,wrap(st['yaw']-self.heading)/.3,
            st['w']/.5,self.integral/.3,*self.previous,(st['height']-.34)/.1,st['tilt']/.2]),-10,10).astype(np.float32)
    def _wait(self,seconds,forward=0.,yaw=0.,action=None):
        deadline=time.monotonic()+seconds;self.set_command(forward,yaw,action)
        while time.monotonic()<deadline:
            self.set_command(forward,yaw);time.sleep(.025)
    def reset(self,*,seed=None,options=None):
        super().reset(seed=seed)
        # Reload the same native scene so every rollout starts at the same pose.
        self._wait(.25,action='stand')
        def console(command):
            subprocess.run(['xdotool','key','--window','62914650','grave'],check=True)
            subprocess.run(['xdotool','type','--window','62914650','--clearmodifiers',command],check=True)
            subprocess.run(['xdotool','key','--window','62914650','Return'],check=True)
        console('open /Game/Maps/IROSFlatWorld')
        self._wait(3.5)
        console('set MatrixMotionControlActor GamepadInput Udp')
        self._wait(.8,action='stand')
        deadline=time.monotonic()+12
        while True:
            try:
                st=self.state()
                if math.hypot(st['x'],st['y'])<.15:break
            except RuntimeError:pass
            if time.monotonic()>deadline:raise RuntimeError('Native scene reset was not verified')
            self._wait(.1)
        self.center=np.array([st['x'],st['y']]);self.heading=st['yaw'];self.started=True
        self.previous[:]=0;self.filtered[:]=0;self.integral=0.;self.step_index=0;self.rows=[];self.episode+=1;self.last_desired=0.
        self.speed=float(self.np_random.uniform(.45,.55)) if self.randomize else .50
        self.next_tick=time.monotonic();self.started_at=time.monotonic()
        return self._obs(self.state()),{}
    def step(self,action):
        st=self.state();desired=self.desired();action=np.clip(np.asarray(action,dtype=float),-1,1)
        if not np.all(np.isfinite(action)):raise ValueError('Nonfinite residual action')
        self.filtered=.65*self.filtered+.35*action
        # Official axis scale is forward 2 m/s and yaw 1 rad/s per normalized input.
        u=np.clip(desired/2.+.10*self.filtered[0],-.30,.30)
        yaw=np.clip(.10*self.filtered[1],-.10,.10)
        transition=None
        if desired!=self.last_desired:transition='walk' if desired else 'stand'
        self.set_command(u,yaw,action=transition);self.last_desired=desired
        self.next_tick=max(self.next_tick+self.dt,time.monotonic()+.001)
        time.sleep(max(0,self.next_tick-time.monotonic()))
        after=self.state();error=after['v']-desired;heading=wrap(after['yaw']-self.heading)
        self.integral=float(np.clip(self.integral-error*self.dt,-.5,.5))
        reward=1.-20*error**2-8*heading**2-.1*after['w']**2-.01*np.sum(action**2)-.02*np.sum((action-self.previous)**2)
        row={'step':self.step_index,'episode':self.episode,'desired_v':desired,'actual_v':after['v'],
             'error_v':error,'heading_error':heading,'tilt_deg':math.degrees(after['tilt']),
             'height':after['height'],'input_forward':float(u),'input_yaw':float(yaw),'action':action.tolist(),
             'position':[after['x'],after['y']],'reward':float(reward),'wall':time.time()}
        self.rows.append(row)
        if self.file:self.file.write(json.dumps(row)+'\n')
        self.previous=action.copy();self.step_index+=1
        distance=np.linalg.norm(np.array([after['x'],after['y']])-self.center)
        if distance>4.0:
            self.set_command(action='stand');raise RuntimeError('Flat-area workspace boundary reached; stopped')
        trunc=self.step_index*self.dt>=self.horizon
        if trunc:self.set_command()
        return self._obs(after),float(reward),False,trunc,{'telemetry':row}
    def metrics(self):
        r=self.rows
        steady=[x for x in r if min(abs((x['step']*.05)%10-v) for v in [0,2,5,7,10])>.7]
        def rmse(key,values):return float(np.sqrt(np.mean([x[key]**2 for x in values]))) if values else None
        return {'steps':len(r),'seconds':len(r)*self.dt,'speed_rmse_m_s':rmse('error_v',r),
                'steady_speed_rmse_m_s':rmse('error_v',steady),'heading_rmse_rad':rmse('heading_error',r),
                'max_tilt_deg':max(x['tilt_deg'] for x in r),'min_height_m':min(x['height'] for x in r),
                'moving_mean_abs_speed_m_s':float(np.mean([abs(x['actual_v']) for x in r if x['desired_v']]))}
    def close(self):
        if not self.running:return
        self._wait(.25,action='stand');self.running=False;self.sender.join(timeout=1)
        self.session.close();self.socket.close()
        if self.file:self.file.close()
