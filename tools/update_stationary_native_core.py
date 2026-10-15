"""Apply verified read-query retry image while keeping all motion disabled."""
import argparse
import json
import re
import shlex
import sys
import time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from ros2_preflight import connect
from stage_ros2 import execute
from bench_ros2_forward import ROS2_SETUP,run_script
from switch_ros2_core import PROBE

CAPTURE='''
import json,time,math,rclpy
from rclpy.node import Node
from std_msgs.msg import String,Int32MultiArray
from nav_msgs.msg import Odometry
rclpy.init();n=Node('stationary_native_update_guard');base=[];arm=[];enc=[];odom=[]
n.create_subscription(String,'/chassis_control/status',lambda m:base.append(json.loads(m.data)),10)
n.create_subscription(String,'/arm_controller/status',lambda m:arm.append(json.loads(m.data)),10)
n.create_subscription(Int32MultiArray,'/chassis_control/encoder_counts',lambda m:enc.append(list(m.data)),10)
n.create_subscription(Odometry,'/odom',lambda m:odom.append(m),10)
deadline=time.monotonic()+10
while time.monotonic()<deadline:
 rclpy.spin_once(n,timeout_sec=.05)
 if len(base)>2 and len(arm)>5 and len(enc)>40 and odom and arm[-1]['feedback_real']:break
assert len(base)>2 and len(arm)>5 and len(enc)>40 and odom and arm[-1]['feedback_real']
assert all(not s['enable_motion'] and not s['dry_run'] and s['wheel_command']==[0,0,0,0] and not s['hardware_fault'] for s in base)
assert all(not s['enable_arm_motion'] and not s['dry_run'] for s in arm)
assert all(not s.get('enable_gripper_motion',False) for s in arm)
assert all(e==enc[0] for e in enc),'Not stationary'
readings=[a['joints'] for a in arm if a['feedback_real']]
assert all(all(abs(a[k]['raw']-readings[0][k]['raw'])<=3 for k in readings[0]) for a in readings)
assert all(a['temperature']<65 for a in readings[-1].values())
assert len(readings[-1])==6 and all(0<=time.monotonic()-a['time']<1.5 for a in readings[-1].values())
o=odom[-1];p=o.pose.pose.position;q=o.pose.pose.orientation
seed={'encoder_counts':enc[-1],'old_odom_pose':[p.x,p.y,2*math.atan2(q.z,q.w)],
 'arm_raw':{str({'joint1':6,'joint2':5,'joint3':4,'joint4':3,'joint5':2,'r_joint':1}[k]):v['raw'] for k,v in readings[-1].items()},
 'arm_status_before_update':arm[-1]}
open('/tmp/ros2-update-seed.json','w').write(json.dumps(seed))
print(json.dumps({'safe_stationary_update':True,'read_fault':arm[-1]['fault'],'seed':seed}))
n.destroy_node();rclpy.shutdown()
'''

UPDATE='''
import importlib.util,pathlib,json,subprocess,shutil,sys
s=importlib.util.spec_from_file_location('core','/opt/armpi-ros2/run_native_core.py');m=importlib.util.module_from_spec(s);s.loader.exec_module(m)
assert not m.legacy_processes()
record=json.load(open('/run/armpi-ros2/container.json'))
assert subprocess.check_output(['docker','inspect',record['name'],'--format','{{.State.Running}}']).decode().strip()=='false'
assert not any(m.owners(record['devices']).values()),'Hardware is still owned'
cfg_path=pathlib.Path('/etc/armpi-ros2/core.json');cfg=json.loads(cfg_path.read_text());backup=pathlib.Path(sys.argv[1])
assert cfg['image_id']==sys.argv[4], 'Previous runtime changed; inspect before updating'
shutil.copy2(cfg_path,backup/'previous-core.json')
cfg['image']=sys.argv[2];cfg['image_id']=sys.argv[3]
assert subprocess.check_output(['docker','image','inspect',cfg['image'],'--format','{{.Id}}']).decode().strip()==cfg['image_id']
if len(sys.argv)>5:
 runner=pathlib.Path('/opt/armpi-ros2/run_native_core.py');old=runner.read_text();new=pathlib.Path(sys.argv[5]).read_text()
 expected=old.replace('enable_arm_motion:=false arm:=true','enable_arm_motion:=false enable_gripper_motion:=false arm:=true')
 expected=expected.replace("'arm_motion_enabled': False}))", "'arm_motion_enabled': False,\\n            'gripper_motion_enabled': False}))")
 assert new==expected,'Runner differs beyond explicit gripper-off startup; inspect before changing'
 compile(new,str(runner),'exec')
 shutil.copy2(runner,backup/'previous-runner.py');runner.write_text(new)
cfg_path.write_text(json.dumps(cfg))
print('Updated pinned image; motion remains disabled')
'''

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--host',required=True);p.add_argument('--config',required=True)
 p.add_argument('--image',required=True);p.add_argument('--image-id',required=True);p.add_argument('--previous-image-id',required=True)
 p.add_argument('--diagnostic-profile',action='store_true',help='Verify diagnostic interfaces while keeping startup emergency latched')
 a=p.parse_args();sys.stdout.reconfigure(encoding='utf-8');c=connect(a.host,config_path=a.config)
 stopped=False
 try:
  assert re.fullmatch(r'armpi-ros2-migration:stage-[a-z0-9]+',a.image)
  assert all(re.fullmatch(r'sha256:[a-f0-9]{64}',v) for v in (a.image_id,a.previous_image_id))
  assert execute(c,'docker image inspect '+shlex.quote(a.image)+" --format '{{.Id}}'")==a.image_id
  assert json.loads(execute(c,'sudo -n cat /etc/armpi-ros2/core.json'))['image_id']==a.previous_image_id
  r=json.loads(execute(c,'sudo -n cat /run/armpi-ros2/container.json'))
  run_script(c,r['name'],ROS2_SETUP,CAPTURE)
  backup=execute(c,'mktemp -d /home/pi/armpi-ros2-update.XXXXXX')
  assert re.fullmatch('/home/pi/armpi-ros2-update.[A-Za-z0-9]+',backup)
  execute(c,'sudo -n docker cp '+r['name']+':/tmp/ros2-update-seed.json '+shlex.quote(backup+'/seed.json'))
  runner_arg=''
  if a.diagnostic_profile:
   with c.open_sftp() as sftp:sftp.put(str(Path(__file__).resolve().parents[1]/'car_setup/ros2/run_native_core.py'),backup+'/new-runner.py')
   runner_arg=' '+shlex.quote(backup+'/new-runner.py')
  execute(c,'sudo -n systemctl stop armpi-ros2');stopped=True
  print(execute(c,'sudo -n python3 -c '+shlex.quote(UPDATE)+' '+shlex.quote(backup)+' '+shlex.quote(a.image)+' '+a.image_id+' '+a.previous_image_id+runner_arg),flush=True)
  execute(c,'sudo -n cp '+shlex.quote(backup+'/seed.json')+' /run/armpi-ros2/cutover-seed.json && sudo -n systemctl start armpi-ros2')
  time.sleep(3)
  current=json.loads(execute(c,'sudo -n cat /run/armpi-ros2/container.json'))
  baseline=json.loads(execute(c,'sudo -n cat /run/armpi-ros2/cutover-seed.json'))
  run_script(c,current['name'],ROS2_SETUP,'BASELINE='+repr(baseline)+'\nDIAGNOSTIC_ARM='+repr(a.diagnostic_profile)+'\n'+PROBE)
  print('STATIONARY_IMAGE_INTERFACE_VERIFIED='+current['name']+' BACKUP='+backup,flush=True)
 except Exception:
  if stopped:
   try:print(execute(c,'sudo -n systemctl stop armpi-ros2'),flush=True)
   except Exception:print('Stop outcome uncertain; keep physical emergency stop available')
  raise
 finally:c.close()

if __name__=='__main__':main()
