"""Two bounded read-only endurance windows; no publish or service calls."""
import argparse
import json
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from ros2_preflight import connect
from stage_ros2 import execute
from bench_ros2_forward import ROS2_SETUP,run_script

WATCH='''
import rclpy,json,time,math
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String,Int32MultiArray
from sensor_msgs.msg import Imu,LaserScan,JointState
from nav_msgs.msg import Odometry
rclpy.init();n=Node('native_core_live_readonly');rows={k:[] for k in ['base','arm','enc','scan','imu','joints','odom']}
def save(k,decode=False):return lambda m:rows[k].append((time.monotonic(),json.loads(m.data) if decode else m))
for k,topic,typ in [('base','/chassis_control/status',String),('arm','/arm_controller/status',String),('enc','/chassis_control/encoder_counts',Int32MultiArray),('scan','/scan',LaserScan),('imu','/bmi270/imu_raw',Imu),('joints','/joint_states',JointState),('odom','/odom',Odometry)]:
 n.create_subscription(typ,topic,save(k,k in ('base','arm')),qos_profile_sensor_data if k in ('scan','imu') else 10)
end=time.monotonic()+25
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.03)
assert all(rows.values())
for _,s in rows['base']:
 assert not s['dry_run'] and not s['enable_motion'] and not s['hardware_fault'] and not s['estop'] and s['wheel_command']==[0,0,0,0],s
for _,s in rows['arm']:
 assert not s['dry_run'] and not s['enable_arm_motion'] and s['feedback_real'] and not s['fault'] and not s['estop'],s
 assert all(v['temperature']<65 for v in s['joints'].values())
enc=[list(m.data) for _,m in rows['enc']]
assert len(enc)>350 and all(e==enc[0] for e in enc)
assert len(rows['scan'])>170 and len(rows['imu'])>800 and len(rows['arm'])>35
now=time.monotonic();assert all(now-v[-1][0]<1.5 for v in rows.values()),'Stale telemetry'
arm=[s['joints'] for _,s in rows['arm']]
assert all(abs(a[k]['raw']-arm[0][k]['raw'])<=3 for a in arm for k in arm[0])
rates={k:round((len(v)-1)/(v[-1][0]-v[0][0]),2) for k,v in rows.items()}
print(json.dumps({'steady_seconds':25,'healthy_native_core':True,'motion_disabled':True,'rates_hz':rates,'encoder_counts':enc[0],'arm_raw':{k:v['raw'] for k,v in arm[-1].items()}}))
n.destroy_node();rclpy.shutdown()
'''

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--host',required=True);p.add_argument('--config',required=True)
 a=p.parse_args();sys.stdout.reconfigure(encoding='utf-8');c=connect(a.host,config_path=a.config)
 try:
  r=json.loads(execute(c,'sudo -n cat /run/armpi-ros2/container.json'))
  for i in range(2):
   print('READ_ONLY_ENDURANCE_WINDOW='+str(i+1),flush=True)
   run_script(c,r['name'],ROS2_SETUP,WATCH)
  print(execute(c,'systemctl is-active armpi-ros2 && docker inspect '+r['name']+" --format 'Running={{.State.Running}} Started={{.State.StartedAt}}'"),flush=True)
 finally:c.close()

if __name__=='__main__':main()
