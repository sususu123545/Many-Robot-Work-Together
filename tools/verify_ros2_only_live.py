"""Read-only ROS2-only runtime acceptance. Never publish or call services."""
import argparse
import json
from pathlib import Path
import shlex
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ros2_preflight import connect
from install_ros2_only_boot import execute

WATCH = r'''
import json,time,urllib.request
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String,Int32MultiArray
from sensor_msgs.msg import Imu,LaserScan,JointState,Image
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
rclpy.init();n=Node('ros2_only_readonly_acceptance')
rows={k:[] for k in ['base','arm','enc','scan','imu','joints','odom','camera','tf']}
def save(k):
 def callback(m):
  if k in ('base','arm'):v=json.loads(m.data)
  elif k=='enc':v=list(m.data)
  elif k=='camera':v={'width':m.width,'height':m.height,'encoding':m.encoding}
  elif k=='scan':v={'ranges':len(m.ranges),'frame':m.header.frame_id}
  elif k=='tf':v=[(t.header.frame_id,t.child_frame_id) for t in m.transforms]
  else:v=None
  rows[k].append((time.monotonic(),v))
 return callback
for k,topic,typ in [('base','/chassis_control/status',String),('arm','/arm_controller/status',String),
 ('enc','/chassis_control/encoder_counts',Int32MultiArray),('scan','/scan',LaserScan),
 ('imu','/bmi270/imu_raw',Imu),('joints','/joint_states',JointState),('odom','/odom',Odometry),
 ('camera','/rear_camera/image_raw',Image),('tf','/tf',TFMessage)]:
 n.create_subscription(typ,topic,save(k),qos_profile_sensor_data if k in ('scan','imu','camera') else 10)
end=time.monotonic()+25
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.02)
now=time.monotonic()
rates={k:{'messages':len(v),'hz':round((len(v)-1)/(v[-1][0]-v[0][0]),2) if len(v)>1 else 0,
 'age':round(now-v[-1][0],2) if v else None} for k,v in rows.items()}
print('TELEMETRY='+json.dumps(rates),flush=True)
for k in ('base','arm','scan','camera','tf'):
 value=rows[k][-1][1] if rows[k] else None
 if k=='arm' and value:
  value=dict(value);value.pop('transport',None)
 print(k.upper()+'='+json.dumps(value),flush=True)
failures=[]
warnings=[]
for k,v in rows.items():
 if not v or now-v[-1][0]>2:failures.append(k+' missing/stale')
for _,s in rows['base']:
 if s.get('dry_run') or s.get('enable_motion') or s.get('hardware_fault') or s.get('wheel_command')!=[0,0,0,0]:
  failures.append('base unsafe/fault');break
for _,s in rows['arm']:
 if s.get('dry_run') or s.get('enable_arm_motion') or s.get('enable_gripper_motion') or not s.get('feedback_real') or not s.get('feedback_complete') or s.get('current_read_error'):
  failures.append('arm unsafe/fault');break
if rows['arm']:
 first=rows['arm'][0][1];last=rows['arm'][-1][1]
 if last.get('fault_count',0)!=first.get('fault_count',0):failures.append('new arm fault during observation')
 if last.get('fault'):warnings.append('arm historical fault remains latched; no reset performed')
enc=[v for _,v in rows['enc']]
if enc and any(v!=enc[0] for v in enc):failures.append('encoders changed')
try:
 url='http://127.0.0.1:8080/snapshot?topic=/rear_camera/image_raw&qos_profile=sensor_data'
 with urllib.request.urlopen(url,timeout=15) as response:
  body=response.read(2000000)
  assert response.status==200 and body[:2]==b'\xff\xd8' and body[-2:]==b'\xff\xd9'
  print('JPEG_SNAPSHOT_BYTES='+str(len(body)),flush=True)
except Exception as error:failures.append('snapshot: '+str(error))
print('READ_ONLY_FAILURES='+json.dumps(sorted(set(failures))),flush=True)
print('WARNINGS='+json.dumps(warnings),flush=True)
print('ARM_MOTION_READY=False (intentionally no reset or motion grant)',flush=True)
n.destroy_node();rclpy.shutdown()
assert not failures,failures
'''


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host',required=True)
    parser.add_argument('--config',required=True)
    args=parser.parse_args()
    sys.stdout.reconfigure(encoding='utf-8')
    client=connect(args.host,config_path=args.config)
    try:
        runtime=json.loads(execute(client,'sudo -n cat /run/armpi-ros2/container.json'))
        name=runtime['name']
        assert name.startswith('armpi_ros2_native_')
        command=('source /opt/armpi_ros2/install/setup.bash && export ROS_DOMAIN_ID=42 '
                 'ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST && python3 -u -c '+shlex.quote(WATCH))
        check=r'''systemctl show armpi-ros2 -p ActiveState -p UnitFileState -p Restart -p NRestarts
systemctl is-enabled armpi-startup-pose
docker inspect armpi_pro --format 'ROS1_RUNNING={{.State.Running}} RESTART={{.HostConfig.RestartPolicy.Name}}'
for unit in start_node ld06 odom ch341-bmi270 auto-mapping slam-control; do systemctl show "$unit" -p Id -p UnitFileState -p ActiveState; done
ss -ltnp | awk 'NR==1 || /:11311 |:9090 |:9091 |:8080 /'
pgrep -af 'roscore|rosmaster|/opt/ros/noetic' || true'''
        print(execute(client,check),flush=True)
        print(execute(client,'docker exec '+name+' bash -lc '+shlex.quote(command),seconds=65),flush=True)
    finally:
        client.close()


if __name__=='__main__':main()
