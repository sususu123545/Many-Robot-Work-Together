"""Local MATRiX validation of the untouched official joint controller."""
import argparse
import json
import math
from pathlib import Path
import socket
import subprocess
import threading
import time
import zenoh
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message

ROOT=Path(__file__).resolve().parent
CONFIG=Path('/mnt/llm/matrix_install/runtime/MATRiX_v1.1.2/UeSim/Content/model/config/config.json')
GATEWAY=Path('/mnt/llm/matrix_install/official-zsl1-control.py')
p=argparse.ArgumentParser();p.add_argument('--name',default='official-zsl1-flat');p.add_argument('--forward-seconds',type=float,default=3.);p.add_argument('--reverse',action='store_true');p.add_argument('--required-distance',type=float,default=.7);p.add_argument('--action',choices=['walk','jump','front_jump','stand_up'],default='walk');p.add_argument('--heading-hold',action='store_true');p.add_argument('--forward-input',type=float,default=.25);a=p.parse_args()
config=json.loads(CONFIG.read_text())['robot']
assert config['inside_mc'],'The test requires the official embedded controller'
pidfile=GATEWAY.parent/'official-input.pid'
if pidfile.exists():
    pid=int(pidfile.read_text())
    try:
        command=Path(f'/proc/{pid}/cmdline').read_bytes()
        if b'official-zsl1-control.py' in command and b'daemon' in command:subprocess.run(['kill','-TERM',str(pid)],check=True);time.sleep(.15)
    except FileNotFoundError:pass
rows=[];phase='connect';lock=threading.Lock();started=time.monotonic();error=None
def receive(sample):
    try:
        m=deserialize_message(sample.payload.to_bytes(),Odometry);p=m.pose.pose.position;q=m.pose.pose.orientation;v=m.twist.twist.linear
        r={'wall':time.monotonic()-started,'phase':phase,'position':[p.x,p.y,p.z],
            'yaw':math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),
            'tilt_deg':math.degrees(2*math.acos(min(1,math.hypot(q.w,q.z)))),
            'velocity':[v.x,v.y,v.z]}
        with lock:rows.append(r)
    except Exception as exc:print('ODOM_ERROR',str(exc),flush=True)
c=zenoh.Config();c.insert_json5('mode','"client"');c.insert_json5('connect/endpoints','["tcp/127.0.0.1:7447"]')
c.insert_json5('scouting/multicast/enabled','false');session=zenoh.open(c);subscriber=session.declare_subscriber('rt/odom',receive)
sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
def hold(name,seconds,forward=0.,action=None):
    global phase
    phase=name;start=time.monotonic();special_sent=False
    while time.monotonic()-start<seconds:
        cmd={'forward':forward,'lateral':0.,'yaw':0.}
        if action and time.monotonic()-start<.1:cmd['action']=action
        with lock:latest=rows[-1] if rows else None
        if latest and a.heading_hold and name=='forward' and a.action=='walk':
            first=rows[0];yaw_error=math.atan2(math.sin(latest['yaw']-first['yaw']),math.cos(latest['yaw']-first['yaw']))
            cross=-(latest['position'][0]-first['position'][0])*math.sin(first['yaw'])+(latest['position'][1]-first['position'][1])*math.cos(first['yaw'])
            cmd['yaw']=max(-.15,min(.15,-.8*yaw_error-.5*cross))
        if a.action=='stand_up' and name in ['wait','initial']:pass
        elif action in ['jump','front_jump','stand_up']:
            # Speed-field packets implicitly request Walk and cancel a special action.
            if not special_sent:sock.sendto(json.dumps({'action':action}).encode(),('127.0.0.1',7447));special_sent=True
        else:sock.sendto(json.dumps(cmd).encode(),('127.0.0.1',7447))
        if latest and a.action!='stand_up' and (latest['tilt_deg']>45 or latest['position'][2]<.17):raise RuntimeError('Fall detected in simulation')
        if latest and time.monotonic()-started-latest['wall']>.3:raise RuntimeError('Odometry became stale')
        time.sleep(.05)
    print('PHASE_DONE',name,flush=True)
try:
    deadline=time.monotonic()+15
    while not rows and time.monotonic()<deadline:hold('wait',.2)
    if not rows:raise RuntimeError('No odometry within 15 seconds; robot was not spawned')
    if a.action=='stand_up':hold('initial',1.)
    else:hold('stand',3,action='stand')
    hold('forward',a.forward_seconds,forward=a.forward_input if a.action=='walk' else 0.,action=a.action)
    hold('stop',3,action='stand')
    if a.reverse:
        hold('reverse',3,forward=-.20,action='walk');hold('stop_final',4,action='stand')
except BaseException as exc:error=str(exc)
finally:
    sock.sendto(json.dumps({'forward':0.,'lateral':0.,'yaw':0.,'action':'stand'}).encode(),('127.0.0.1',7447));sock.close();session.close()
    subprocess.run(['/usr/bin/python3',str(GATEWAY),'neutral'],check=True)
    subprocess.Popen(['/usr/bin/python3',str(GATEWAY),'daemon'],stdout=subprocess.DEVNULL,stderr=subprocess.STDOUT,start_new_session=True)
    stages={}
    for name in ['initial','stand','forward','stop','reverse','stop_final']:
        r=[x for x in rows if x['phase']==name]
        if len(r)<5:continue
        first,last=r[0],r[-1];dx=last['position'][0]-first['position'][0];dy=last['position'][1]-first['position'][1]
        tail=[x for x in r if x['wall']>last['wall']-.75]
        stages[name]={'seconds':last['wall']-first['wall'],'forward_distance_m':dx*math.cos(first['yaw'])+dy*math.sin(first['yaw']),
            'max_lateral_deviation_m':max(abs(-(x['position'][0]-first['position'][0])*math.sin(first['yaw'])+(x['position'][1]-first['position'][1])*math.cos(first['yaw'])) for x in r),
            'max_tilt_deg':max(x['tilt_deg'] for x in r),'min_height_m':min(x['position'][2] for x in r),
            'max_height_m':max(x['position'][2] for x in r),'final_tilt_deg':sum(x['tilt_deg'] for x in tail)/len(tail),
            'final_speed_m_s':sum(math.hypot(*x['velocity'][:2]) for x in tail)/len(tail)}
    stopped=stages.get('stop_final',stages.get('stop',{}))
    passed=not error and stages.get('forward',{}).get('forward_distance_m',0)>a.required_distance and stopped.get('final_speed_m_s',1)<.05
    passed=passed and all(s['max_tilt_deg']<35 and s['max_lateral_deviation_m']<.7 for s in stages.values())
    if a.reverse:passed=passed and stages.get('reverse',{}).get('forward_distance_m',0)<-.4
    if a.action in ['jump','front_jump']:
        baseline=stages.get('stand',{}).get('max_height_m',1.)
        rise=stages.get('forward',{}).get('max_height_m',0)-baseline
        passed=not error and rise>.04 and stopped.get('final_tilt_deg',90)<15 and stopped.get('final_speed_m_s',1)<.05
        if a.action=='front_jump':passed=passed and stages.get('forward',{}).get('forward_distance_m',0)>.15
    if a.action=='stand_up':
        initial=max((x['tilt_deg'] for x in rows if x['phase'] in ['wait','initial']),default=0.)
        passed=not error and initial>=50 and stages.get('forward',{}).get('final_tilt_deg',90)<15 and stopped.get('min_height_m',0)>.25
        if initial<50:error='Initial fallen pose was not verified; recovery capability remains unverified'
    report={'name':a.name,'controller':'unchanged official embedded XGB controller','model':config['mujoco_model'],
        'phases':stages,'action':a.action,'heading_hold':a.heading_hold,'forward_input':a.forward_input,'passed':bool(passed),'error':error,'simulation_only':True}
    out=ROOT/'results';out.mkdir(exist_ok=True)
    (out/f'{a.name}.json').write_text(json.dumps(report,indent=2));(out/f'{a.name}-telemetry.json').write_text(json.dumps(rows))
    print(json.dumps(report,indent=2),flush=True)
if error:raise SystemExit(error)
