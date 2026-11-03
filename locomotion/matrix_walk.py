"""Replay a validated ZSL1 residual policy in local MATRiX over Zenoh, never on hardware."""
import argparse
import hashlib
import json
from pathlib import Path
import signal
import sys
import threading
import time
import numpy as np
import torch
import zenoh
from stable_baselines3 import PPO
from env import HOME,MODEL,ROOT,observation,reference

sys.path.insert(0,str(ROOT.parent/'matrix_control'))
from protocol import RobotState,command


def main():
    p=argparse.ArgumentParser();p.add_argument('--model',required=True);p.add_argument('--seconds',type=float,default=30.);p.add_argument('--speed',type=float,default=.25);p.add_argument('--allow-unvalidated',action='store_true');a=p.parse_args()
    if not 0<=a.speed<=.35 or not 0<a.seconds<=300:p.error('speed 0..0.35 m/s; duration 0..300 s')
    checkpoint=Path(a.model).resolve();report_path=checkpoint.parent/'deployment-validation.json'
    if not a.allow_unvalidated:
        report=json.loads(report_path.read_text())
        if not report.get('passed') or report.get('checkpoint_sha256')!=hashlib.sha256(checkpoint.read_bytes()).hexdigest():
            raise RuntimeError('Checkpoint has no matching passed deployment evaluation')
    config=json.loads(Path('/mnt/llm/matrix_install/runtime/MATRiX_v1.1.2/UeSim/Content/model/config/config.json').read_text())
    if config['robot'].get('inside_mc',True):raise RuntimeError('Disable built-in control before using the learned controller')
    if Path(config['robot']['mujoco_model']).resolve()!=MODEL/'zsl1_calibrated.xml':raise RuntimeError('MATRiX must load zsl1_calibrated.xml before walking')
    torch.set_num_threads(1);torch.set_num_interop_threads(1)
    policy=PPO.load(checkpoint,device='cpu');print('POLICY_LOADED',str(checkpoint),flush=True)
    c=zenoh.Config();c.insert_json5('mode','"client"');c.insert_json5('connect/endpoints','["tcp/127.0.0.1:7447"]');c.insert_json5('scouting/multicast/enabled','false')
    latest=None;sequence=0;lock=threading.Lock();running=True
    def stop(*_):
        nonlocal running
        running=False
    signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop)
    def cb(sample):
        nonlocal latest,sequence
        try:
            st=RobotState.FromString(sample.payload.to_bytes())
            q=np.array([st.q_abad,st.q_hip,st.q_knee]).T.reshape(-1)
            qd=np.array([st.qd_abad,st.qd_hip,st.qd_knee]).T.reshape(-1)
            if q.shape!=(12,) or qd.shape!=(12,) or not np.all(np.isfinite(np.r_[q,qd,st.quat,st.position,st.gyro,st.v_world])):return
            with lock:
                sequence+=1;latest=(time.monotonic(),sequence,st,q,qd)
        except Exception as e:print('STATE_ERROR',str(e),flush=True)
    session=None
    while running and session is None:
        try:session=zenoh.open(c)
        except zenoh.ZError:time.sleep(.5)
    if session is None:return
    sub=session.declare_subscriber('mujoco/state',cb);pub=session.declare_publisher('mujoco/cmd')
    msg=command(kp=0,kd=0);damping=command(kp=0,kd=2).SerializeToString()
    previous=np.zeros(12);filtered=np.zeros(12);action=np.zeros(12)
    seen=0;first=None;target=HOME.copy();last_policy=-1.;log_next=0.;rows=[];start_pos=None
    logpath=ROOT/'matrix-walk-telemetry.jsonl'
    print('WAITING_FOR_SIMULATION_STATE',flush=True);waiting_since=time.monotonic()
    try:
        with logpath.open('w') as log:
            while running:
                now=time.monotonic()
                with lock:cur=latest
                if cur is None:
                    if now-waiting_since>20:raise RuntimeError('No simulator state within 20 seconds')
                    time.sleep(.005);continue
                if now-cur[0]>.1:pub.put(damping);time.sleep(.005);continue
                if cur[1]==seen:time.sleep(.0003);continue
                seen=cur[1];st,q,qd=cur[2:]
                quat=np.asarray(st.quat);pos=np.asarray(st.position)
                tilt=2*np.arccos(min(1.,np.hypot(quat[0],quat[3])))
                if tilt>np.deg2rad(35) or pos[2]<.18:raise RuntimeError('Fall boundary reached; damping requested')
                if first is None:first=now;start_pos=pos.copy()
                t=now-first
                if t>a.seconds:break
                if t-last_policy>=.0199:
                    obs=observation(q,qd,quat,np.asarray(st.v_world),np.asarray(st.gyro),np.array([a.speed,0,0]),previous,t,np.zeros(4))
                    action=np.clip(policy.predict(obs,deterministic=True)[0],-1,1)
                    ref,_=reference(t,a.speed);target=np.clip(ref+.15*action,np.tile([-.4787,-.339,-2.713],4),np.tile([.4787,2.957,-.612],4))
                    previous=action.copy();last_policy=t
                tau=np.clip(70*(target-q)-2*qd,-28,28)
                tau=np.where((np.abs(qd)>28)&(tau*qd>0),0,tau)
                filtered+=(tau-filtered)*(.002/(.004+.002))
                for i,j in enumerate(['abad','hip','knee']):
                    field=getattr(msg,'tau_'+j+'_ff');del field[:];field.extend(filtered.reshape(4,3)[:,i])
                pub.put(msg.SerializeToString())
                if t>=log_next:
                    row={'simulation_s':t,'state_seq':seen,'position':pos.tolist(),'distance_x_m':float(pos[0]-start_pos[0]),'velocity_world':list(st.v_world),'tilt_deg':float(np.rad2deg(tilt)),'max_tau_nm':float(np.max(np.abs(filtered)))}
                    rows.append(row);log.write(json.dumps(row)+'\n');log.flush();print(json.dumps(row),flush=True);log_next+=.2
    finally:
        # Return to the established stand posture, then keep its PD loop inside MATRiX.
        # On a fall use damping instead of attempting a standing recovery.
        with lock:cur=latest
        can_stand=cur is not None and time.monotonic()-cur[0]<.1 and cur[2].position[2]>.22 and np.hypot(cur[2].quat[0],cur[2].quat[3])>np.cos(np.deg2rad(15)/2)
        stopmsg=command(kp=70,kd=2).SerializeToString() if can_stand else damping
        for _ in range(20):pub.put(stopmsg);time.sleep(.002)
        session.close()
        (ROOT/'matrix-walk-summary.json').write_text(json.dumps({'seconds':rows[-1]['simulation_s'] if rows else 0,'distance_x_m':rows[-1]['distance_x_m'] if rows else 0,'samples':len(rows),'returned_to_stand':bool(can_stand),'checkpoint':str(checkpoint)},indent=2))


if __name__=='__main__':main()
