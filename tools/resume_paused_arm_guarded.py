"""Resume ONLY the verified paused arm, with an independent re-pause watchdog.

Requires physical arm support. Never resets protection, grants motion, publishes
targets, opens UART, or restarts the robot/service. Failed feedback re-pauses it.
"""
import argparse
import shlex
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ros2_preflight import connect
from diagnose_lidar_ssh import check


MONITOR = r'''
import json,time,rclpy
from rclpy.node import Node
from std_msgs.msg import String,Int32MultiArray
rclpy.init();n=Node('supported_arm_resume_readonly');arm=[];base=[];enc=[]
n.create_subscription(String,'/arm_controller/status',lambda m:arm.append(json.loads(m.data)),10)
n.create_subscription(String,'/chassis_control/status',lambda m:base.append(json.loads(m.data)),10)
n.create_subscription(Int32MultiArray,'/chassis_control/encoder_counts',lambda m:enc.append(list(m.data)),10)
end=time.monotonic()+8;reason=None
while time.monotonic()<end:
 rclpy.spin_once(n,timeout_sec=.04)
 if arm:
  s=arm[-1]
  if s['enable_arm_motion'] or s['enable_gripper_motion'] or not s['estop']:reason='Motion gate or latch changed';break
  if s['current_read_error'] or not s['feedback_complete']:reason='Physical feedback is still invalid: '+str(s['current_read_error']);break
 if base and (base[-1]['enable_motion'] or base[-1]['wheel_command']!=[0,0,0,0]):reason='Chassis motion gate changed';break
 if enc and any(e!=enc[0] for e in enc):reason='Wheel encoders changed';break
 if len(arm)>1:
  first,last=arm[0]['joints'],arm[-1]['joints']
  if any(abs(last[k]['raw']-first[k]['raw'])>3 for k in first):reason='Uncommanded arm feedback changed';break
if not reason and (len(arm)<10 or not base or not enc):reason='Insufficient fresh telemetry'
s=arm[-1] if arm else {}
print(json.dumps({'safe_observation':reason is None,'reason':reason,'arm_samples':len(arm),
 'health_state':s.get('health_state'),'current_read_error':s.get('current_read_error'),
 'fault_count':s.get('fault_count'),'feedback_complete':s.get('feedback_complete'),
 'positions':{k:v['raw'] for k,v in s.get('joints',{}).items()},
 'temperatures':{k:v['temperature'] for k,v in s.get('joints',{}).items()},
 'encoder_counts':enc[-1] if enc else None,'no_target_grant_or_reset_sent':True}))
n.destroy_node();rclpy.shutdown()
'''

GUARD = r'''
import os,pathlib,select,signal,sys
pid=int(sys.argv[1]);start=sys.argv[2];container=sys.argv[3]
ready,_,_=select.select([sys.stdin],[],[],18)
ack=os.read(sys.stdin.fileno(),16) if ready else b''
if ack==b'VERIFIED\n':sys.exit(0)
p=pathlib.Path('/proc')/str(pid)
try:
 assert (p/'stat').read_text().rsplit(')',1)[1].split()[19]==start
 assert container in (p/'cgroup').read_text()
 assert b'/opt/armpi_ros2/install/armpi_ros2/lib/armpi_ros2/arm' in (p/'cmdline').read_bytes().split(bytes([0]))
 os.kill(pid,signal.SIGSTOP)
except (OSError,AssertionError):pass
'''

RESUME = r'''
import json,os,pathlib,shlex,signal,subprocess,uuid
root=pathlib.Path('/run/armpi-ros2');receipt=root/'arm-paused.json'
r=json.loads(receipt.read_text());m=json.loads((root/'container.json').read_text())
c=json.loads(subprocess.check_output(['docker','inspect',m['name']],text=True,timeout=5))[0]
pid=r['owner_pid'];p=pathlib.Path('/proc')/str(pid)
def verify():
 assert r['container']==m['name'] and r['container_id']==c['Id'] and c['Id'] in (p/'cgroup').read_text()
 assert c['State']['Running'] and c['Image']=='sha256:3704db56c46ebe9ade423cfef9f0e19ea0ac177e440411271105501d455acac6'
 args=[a.decode() for a in (p/'cmdline').read_bytes().split(bytes([0])) if a]
 assert '/opt/armpi_ros2/install/armpi_ros2/lib/armpi_ros2/arm' in args
 assert all(m.get(k) is False for k in ('motion_enabled','arm_motion_enabled','gripper_motion_enabled'))
 for i,a in enumerate(args[:-1]):
  if a=='--params-file':
   text=(p/'root'/args[i+1].lstrip('/')).read_text()
   assert 'enable_arm_motion: true' not in text and 'enable_gripper_motion: true' not in text
 assert next(v for v in (p/'status').read_text().splitlines() if v.startswith('State:')).split()[1]=='T'
 return (p/'stat').read_text().rsplit(')',1)[1].split()[19]
start=verify();guard=subprocess.Popen(['/usr/bin/python3','-c',GUARD,str(pid),start,c['Id']],
 stdin=subprocess.PIPE,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,start_new_session=True)
healthy=False;resumed=False;archived=None
try:
 assert verify()==start
 os.kill(pid,signal.SIGCONT);resumed=True
 print('ARM_SIGCONT_SENT='+str(pid),flush=True)
 setup='source /opt/armpi_ros2/install/setup.bash && python3 -c '+shlex.quote(MONITOR)
 result=subprocess.run(['docker','exec',m['name'],'bash','-lc',setup],capture_output=True,text=True,timeout=14)
 if result.returncode:raise RuntimeError(result.stderr[-2000:] or result.stdout[-2000:])
 observed=json.loads(result.stdout.strip().splitlines()[-1]);print(json.dumps(observed),flush=True)
 if observed['safe_observation']:
  assert (p/'stat').read_text().rsplit(')',1)[1].split()[19]==start
  assert next(v for v in (p/'status').read_text().splitlines() if v.startswith('State:')).split()[1]!='T'
  archived=root/('arm-pause-resolved-'+uuid.uuid4().hex[:12]+'.json')
  os.rename(receipt,archived)
  guard.stdin.write(b'VERIFIED\n');guard.stdin.flush()
  healthy=True
finally:
 if not healthy and resumed:
  assert (p/'stat').read_text().rsplit(')',1)[1].split()[19]==start and c['Id'] in (p/'cgroup').read_text()
  os.kill(pid,signal.SIGSTOP)
  if archived is not None and archived.exists() and not receipt.exists():os.rename(archived,receipt)
 guard.stdin.close();guard.wait(timeout=3)
 state=next(v for v in (p/'status').read_text().splitlines() if v.startswith('State:'))
 print(json.dumps({'final_process_state':state,'paused_again':not healthy,
                   'protection_not_reset':True,'motion_not_authorized':True}),flush=True)
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', required=True)
    parser.add_argument('--config', required=True)
    parser.add_argument('--confirm-arm-supported', action='store_true', required=True)
    args = parser.parse_args()
    sys.stdout.reconfigure(encoding='utf-8')
    source = 'MONITOR=' + repr(MONITOR) + '\nGUARD=' + repr(GUARD) + '\n' + RESUME
    client = connect(args.host, config_path=args.config)
    try:
        check(client, 'supported_arm_guarded_resume', 'sudo -n python3 -c ' + shlex.quote(source), seconds=24)
    finally:
        client.close()


if __name__ == '__main__':
    main()
