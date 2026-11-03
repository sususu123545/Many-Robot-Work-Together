"""Launch one verified local MATRiX fixture and run the official gait test."""
import argparse
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import time
import threading
import psutil
from fixtures import build,SOURCE

ROOT=Path(__file__).resolve().parent;INSTALL=Path('/mnt/llm/matrix_install')
CONFIG=INSTALL/'runtime/MATRiX_v1.1.2/UeSim/Content/model/config/config.json'
p=argparse.ArgumentParser();p.add_argument('case',choices=['stairs12','slope20','gap15','flat','recover_side']);p.add_argument('--action',choices=['walk','jump','front_jump','stand_up'],default='walk');p.add_argument('--heading-hold',action='store_true');p.add_argument('--forward-input',type=float,default=.25);a=p.parse_args()
model=SOURCE if a.case=='flat' else Path(build(a.case)['model'])
subprocess.run(['/usr/bin/python3',str(INSTALL/'official-zsl1-control.py'),'neutral'],check=True)
passive_end=threading.Event();passive_thread=None
if a.case=='recover_side':
    pidfile=INSTALL/'official-input.pid'
    if pidfile.exists():
        pid=int(pidfile.read_text())
        try:
            if b'official-zsl1-control.py' in Path(f'/proc/{pid}/cmdline').read_bytes():psutil.Process(pid).terminate()
        except (FileNotFoundError,psutil.NoSuchProcess):pass
    def keep_passive():
        with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as udp:
            while not passive_end.is_set():udp.sendto(b'{"action":"passive"}',('127.0.0.1',7447));time.sleep(.025)
    passive_thread=threading.Thread(target=keep_passive,daemon=True);passive_thread.start()
for process in psutil.process_iter(['pid','cmdline']):
    cmd=process.info['cmdline'] or []
    if cmd and cmd[0].endswith('/UeSim/Binaries/Linux/UeSim'):
        process.terminate()
        try:process.wait(timeout=12)
        except psutil.TimeoutExpired:raise RuntimeError('Previous simulator did not stop; refusing duplicate')
config=json.loads(CONFIG.read_text());config['robot']['mujoco_model']=str(model);config['robot']['inside_mc']=True
CONFIG.write_text(json.dumps(config,indent=2))
log=open(ROOT/f'launch-{a.case}.log','w')
process=subprocess.Popen([str(INSTALL/'start-matrix.sh')],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
deadline=time.monotonic()+45;window=None;native_pid=None
while time.monotonic()<deadline:
    for child in psutil.process_iter(['pid','cmdline']):
        cmd=child.info['cmdline'] or []
        if cmd and cmd[0].endswith('/UeSim/Binaries/Linux/UeSim'):
            native_pid=child.pid
            result=subprocess.run(['xdotool','search','--pid',str(native_pid)],capture_output=True,text=True)
            if result.returncode==0:
                window=result.stdout.splitlines()[-1];break
    if window:break
    if process.poll() is not None:raise RuntimeError('Simulator launcher exited before window creation')
    time.sleep(.5)
if not window:raise RuntimeError('No simulator window')
(INSTALL/'matrix.pid').write_text(str(native_pid)+'\n')
os.environ['DISPLAY']=':1';os.environ['XAUTHORITY']='/run/user/1000/gdm/Xauthority'
subprocess.run(['xdotool','windowactivate','--sync',window],check=True)
time.sleep(4)
def click(x,y):
    subprocess.run(['xdotool','mousemove','--window',window,str(x),str(y),'click','1'],check=True)
click(1184,683);time.sleep(1.5);click(1210,680);time.sleep(1.5);click(1200,35)
deadline=time.monotonic()+35
while True:
    try:
        with socket.create_connection(('127.0.0.1',7447),timeout=.3):break
    except OSError:
        if time.monotonic()>deadline:raise RuntimeError('Simulation did not start; check launcher')
        time.sleep(.3)
time.sleep(2)
for text in ['set MatrixMotionControlActor GamepadInput Udp','set MatrixMotionControlActor GamepadDeadZone 0.02']:
    subprocess.run(['xdotool','key','--window',window,'grave'],check=True)
    subprocess.run(['xdotool','type','--window',window,'--clearmodifiers',text],check=True)
    subprocess.run(['xdotool','key','--window',window,'Return'],check=True)
if json.loads(CONFIG.read_text())['robot']['mujoco_model']!=str(model):raise RuntimeError('Launcher changed the fixture model; refusing mislabeled test')
cmd=[sys.executable,str(ROOT/'test_official_motion.py'),'--name',f'official-{a.case}'+('' if a.action=='walk' else '-'+a.action),'--action',a.action]
cmd+=['--forward-input',str(a.forward_input)]
if a.heading_hold:cmd+=['--heading-hold']
if a.action!='walk':cmd+=['--forward-seconds','8' if a.action=='stand_up' else '5']
elif a.case=='flat':cmd+=['--reverse']
elif a.case=='gap15':cmd+=['--forward-seconds',str(4.6/(2*a.forward_input)),'--required-distance','4.3']
else:cmd+=['--forward-seconds','8','--required-distance','3.5']
passive_end.set()
if passive_thread:passive_thread.join(timeout=1)
raise SystemExit(subprocess.call(cmd))
