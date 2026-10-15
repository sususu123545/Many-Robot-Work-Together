"""Explicit stationary ROS2 cutover, with backed-up ROS1 boot configuration.

No nonzero velocity, arm trajectory, torque unload or automatic recentering.
On a failure after retiring ROS1, stop only our native service and KEEP motion
disabled. Restore ROS1 only through a separately reviewed stationary rollback.
"""
import argparse
import json
import re
import shlex
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ros2_preflight import connect
from stage_ros2 import execute
from bench_ros2_forward import ROS1_SETUP, ROS2_SETUP, run_script
from inspect_ros2_cutover import READINESS

IMAGE = 'armpi-ros2-migration:stage-uzulgo'
IMAGE_ID = 'sha256:1290124e5645fc7d5114520de12767a0a61b76feb42c0a54ecd4d8571607cf3b'
LIBRARY = '/home/pi/armpi-ros2-imu-library.LvHsqp'
LIBRARY_SHA = 'c849d1fe9204515d2eed063198d84ee97f08df8bd2c3b1a3f0ebef17fe0d7cb0'
# See constant used by actual source verification below; avoid copying binaries.

INSTALL = r'''
import json,pathlib,shutil,subprocess,sys,hashlib
stage=pathlib.Path(sys.argv[1]);cfg=json.loads((stage/'core.json').read_text())
root=pathlib.Path('/etc/armpi-ros2');dest=pathlib.Path('/opt/armpi-ros2')
assert not root.exists() and not dest.exists(), 'A prior native installation exists; inspect before retry'
def run(args):
 p=subprocess.run(args,capture_output=True,text=True,timeout=30)
 assert p.returncode==0,(args,p.stderr)
 return p.stdout.strip()
assert run(['docker','image','inspect',cfg['image'],'--format','{{.Id}}'])==cfg['image_id']
backup=stage/'backup';backup.mkdir()
wrapper=pathlib.Path('/home/pi/armpi_pro/start_node.sh')
old=wrapper.read_bytes();assert b'docker exec -u ubuntu' in old and b'start_node.sh' in old
(backup/'start_node.sh').write_bytes(old)
units=['ld06','odom','ch341-bmi270','auto-mapping','slam-control']
states={}
for u in units:
 for action in ('is-enabled','is-active'):
  p=subprocess.run(['systemctl',action,u],capture_output=True,text=True)
  states.setdefault(u,{})[action]=p.stdout.strip()
 p=subprocess.run(['systemctl','cat',u],capture_output=True,text=True)
 assert p.returncode==0
 (backup/(u+'.service.txt')).write_text(p.stdout)
 drop=pathlib.Path('/etc/systemd/system')/(u+'.service.d')/'90-ros2-core.conf'
 assert not drop.exists(),'Existing cutover drop-in: '+str(drop)
(backup/'service-states.json').write_text(json.dumps(states,indent=2))
# Copy full controller/calibration configuration before changing any boot state.
for name,path in [('servo_config','/home/ubuntu/armpi_pro/src/hiwonder_servo_controllers'),
                  ('legacy_bringup','/home/ubuntu/armpi_pro/src/armpi_pro_bringup')]:
 run(['docker','cp','armpi_pro:'+path,str(backup/name)])
root.mkdir();dest.mkdir()
shutil.copy2(stage/'core.json',root/'core.json')
shutil.copy2(stage/'run_native_core.py',dest/'run_native_core.py')
shutil.copy2(stage/'armpi-ros2.service','/etc/systemd/system/armpi-ros2.service')
run(['docker','cp',str(stage/'legacy_sidecar.sh'),'armpi_pro:/home/ubuntu/armpi_ros2_legacy_sidecar.sh'])
run(['docker','cp',str(stage/'legacy_sidecar.launch'),'armpi_pro:/home/ubuntu/armpi_ros2_legacy_sidecar.launch'])
prefix=b"""#!/bin/bash
if [ -f /etc/armpi-ros2/active ]; then
  exec docker exec -u ubuntu -w /home/ubuntu armpi_pro bash /home/ubuntu/armpi_ros2_legacy_sidecar.sh
fi
"""
# Keep original fallback bytes, including xhost and source_env invocation.
wrapper.write_bytes(prefix+old.split(b'\n',1)[1])
for u in units:
 drop=pathlib.Path('/etc/systemd/system')/(u+'.service.d')/'90-ros2-core.conf'
 drop.parent.mkdir(exist_ok=True)
 drop.write_text('[Unit]\nConditionPathExists=!/etc/armpi-ros2/active\n')
run(['systemctl','daemon-reload'])
print(json.dumps({'backup':str(backup),'boot_prepared':True,'native_not_started':True}))
'''

RETIRE = '''
import json,time,rospy,rosnode
from std_msgs.msg import Empty,Int32MultiArray
from nav_msgs.msg import Odometry
rospy.init_node('ros2_core_retire_legacy',anonymous=True)
p=rospy.Publisher('/chassis_control/stop',Empty,queue_size=1)
deadline=time.monotonic()+4
while p.get_num_connections()<1 and time.monotonic()<deadline:time.sleep(.05)
assert p.get_num_connections()>=1
for _ in range(3):p.publish(Empty());time.sleep(.1)
a=rospy.wait_for_message('/chassis_control/encoder_counts',Int32MultiArray,timeout=3)
time.sleep(.5);b=rospy.wait_for_message('/chassis_control/encoder_counts',Int32MultiArray,timeout=3)
assert list(a.data)==list(b.data)
o=rospy.wait_for_message('/odom',Odometry,timeout=4)
import math
seed={'encoder_counts':list(b.data),'old_odom_pose':[o.pose.pose.position.x,o.pose.pose.position.y,2*math.atan2(o.pose.pose.orientation.z,o.pose.pose.orientation.w)]}
from ros_robot_controller.srv import GetBusServoState,GetBusServoStateRequest
from ros_robot_controller.msg import GetBusServoCmd
req=GetBusServoStateRequest()
for i in range(1,7):
 cmd=GetBusServoCmd();cmd.id=i;cmd.get_position=1;cmd.get_temperature=1;req.cmd.append(cmd)
reply=rospy.ServiceProxy('/ros_robot_controller/bus_servo/get_state',GetBusServoState)(req)
assert reply.success and len(reply.state)==6
seed['arm_raw']={str(i+1):s.position[0] for i,s in enumerate(reply.state)}
assert all(s.temperature[0]<65 for s in reply.state)
open('/tmp/ros2-cutover-seed.json','w').write(json.dumps(seed))
nodes=rosnode.get_node_names()
# Retire command-consuming manager BEFORE releasing its board serial owner.
for name in ['/auto_mapping_control','/slam_control','/chassis_control','/mecanum_odom',
             '/joint_states_publisher','/bmi270_imu','/LD06','/base_to_laser','/base_to_imu',
             '/hiwonder_servo_manager','/ros_robot_controller']:
 if name in rosnode.get_node_names():
  succeeded,failed=rosnode.kill_nodes([name]);assert not failed,(name,failed)
print(json.dumps({'legacy_core_retired':True,'seed':seed}))
'''

PROBE = '''
import json,time,math,rclpy,tf2_ros
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String,Int32MultiArray
from sensor_msgs.msg import Imu,LaserScan,JointState
from nav_msgs.msg import Odometry
rclpy.init();n=Node('native_whole_core_acceptance');buf=tf2_ros.Buffer();tf=tf2_ros.TransformListener(buf,n)
diagnostic_arm=globals().get('DIAGNOSTIC_ARM',False)
streams={k:[] for k in ['base','arm','enc','scan','imu','joints','odom']}
def store(k,decode=False):
 return lambda m:streams[k].append((time.monotonic(),json.loads(m.data) if decode else m))
n.create_subscription(String,'/chassis_control/status',store('base',True),10)
n.create_subscription(String,'/arm_controller/status',store('arm',True),10)
n.create_subscription(Int32MultiArray,'/chassis_control/encoder_counts',store('enc'),10)
n.create_subscription(LaserScan,'/scan',store('scan'),qos_profile_sensor_data)
n.create_subscription(Imu,'/bmi270/imu_raw',store('imu'),qos_profile_sensor_data)
n.create_subscription(JointState,'/joint_states',store('joints'),10)
n.create_subscription(Odometry,'/odom',store('odom'),10)
deadline=time.monotonic()+20
while time.monotonic()<deadline:
 rclpy.spin_once(n,timeout_sec=.03)
 if len(streams['scan'])>50 and len(streams['imu'])>200 and len(streams['joints'])>6 and len(streams['base'])>3:break
assert all(streams.values()), {k:len(v) for k,v in streams.items()}
for _,s in streams['base']:
 assert not s['dry_run'] and not s['enable_motion'] and not s['hardware_fault'] and s['wheel_command']==[0,0,0,0],s
for _,s in streams['arm']:
 assert not s['dry_run'] and not s['enable_arm_motion'],s
 if diagnostic_arm:
  assert s['enable_gripper_motion'] is False and s['estop'] is True,s
  assert s['transport'] is not None and 'current_read_error' in s and 'last_fault' in s
 else:
  assert s['feedback_real'] and not s['fault'] and not s['estop'],s
  assert len(s['joints'])==6
if diagnostic_arm:
 assert sum(s['feedback_complete'] and len(s['joints'])==6 for _,s in streams['arm'])>=2,'No complete physical arm feedback'
enc=[list(e.data) for _,e in streams['enc']]
assert all(e==enc[0] for e in enc),'Encoders changed during stationary cutover'
assert enc[0]==BASELINE['encoder_counts'],'Physical wheels changed since retirement'
assert all(abs(v['raw']-BASELINE['arm_raw'][str({'joint1':6,'joint2':5,'joint3':4,'joint4':3,'joint5':2,'r_joint':1}[k])])<=5 for k,v in streams['arm'][-1][1]['joints'].items()),'Physical arm moved during cutover'
for k,low,high in [('scan',7,13),('imu',20,80),('enc',10,30)]:
 rows=streams[k];rate=(len(rows)-1)/(rows[-1][0]-rows[0][0]);assert low<rate<high,(k,rate)
last=streams['imu'][-1][1]
assert last.header.frame_id=='imu_link' and 7<math.sqrt(sum(getattr(last.linear_acceleration,a)**2 for a in 'xyz'))<12
subs=[i.node_name for i in n.get_subscriptions_info_by_topic('/bmi270/imu_raw')]
assert 'mecanum_odom' in subs,'Native odometry not using IMU'
t=buf.lookup_transform('base_link','base_laser',rclpy.time.Time()).transform.translation
assert abs(t.x+.01)<1e-8 and abs(t.y-.01)<1e-8
q=buf.lookup_transform('base_link','imu_link',rclpy.time.Time()).transform.rotation
assert abs(q.z-math.sqrt(.5))<1e-8 and abs(q.w-math.sqrt(.5))<1e-8
o=streams['odom'][0][1];p=o.pose.pose.position;q=o.pose.pose.orientation
assert math.hypot(p.x-BASELINE['old_odom_pose'][0],p.y-BASELINE['old_odom_pose'][1])<.001
assert abs(2*math.atan2(q.z,q.w)-BASELINE['old_odom_pose'][2])<.02,'Odometry heading continuity lost'
rates={k:round((len(v)-1)/(v[-1][0]-v[0][0]),2) for k,v in streams.items() if len(v)>1}
print(json.dumps({'native_ros2_interfaces_verified':True,'native_ros2_four_hardware_ready':not diagnostic_arm,
 'diagnostic_profile':diagnostic_arm,'arm_health_state':streams['arm'][-1][1].get('health_state'),
 'motion_disabled':True,'rates_hz':rates,'encoder_counts':enc[0],
 'initial_observed_odom':[p.x,p.y,2*math.atan2(q.z,q.w)],'real_arm':streams['arm'][-1][1]['joints'],'imu_odom_subscriber':True}))
n.destroy_node();rclpy.shutdown()
'''

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--host',required=True);p.add_argument('--config',required=True)
    p.add_argument('--apply-stationary-cutover',action='store_true',required=True)
    args=p.parse_args();sys.stdout.reconfigure(encoding='utf-8')
    c=connect(args.host,config_path=args.config)
    retired=False
    try:
        assert execute(c,'docker image inspect '+IMAGE+" --format '{{.Id}}'")==IMAGE_ID
        run_script(c,'armpi_pro',ROS1_SETUP,READINESS)
        # Verify native runtime library BEFORE touching old boot configuration.
        from verify_ros2_imu import LIBRARY_DIGEST
        digest=execute(c,'python3 -c '+shlex.quote('ROOT='+repr(LIBRARY)+'\n'+LIBRARY_DIGEST))
        assert digest==LIBRARY_SHA
        stage=execute(c,'mktemp -d /home/pi/armpi-ros2-cutover.XXXXXX')
        assert re.fullmatch('/home/pi/armpi-ros2-cutover.[A-Za-z0-9]+',stage)
        src=Path(__file__).resolve().parents[1]/'car_setup'/'ros2'
        cfg={'image':IMAGE,'image_id':IMAGE_ID,'bmi_library':LIBRARY,'bmi_library_sha256':digest}
        with c.open_sftp() as sftp:
            for name in ('run_native_core.py','armpi-ros2.service','legacy_sidecar.sh','legacy_sidecar.launch'):
                sftp.put(str(src/name),stage+'/'+name)
            with sftp.open(stage+'/core.json','w') as f:f.write(json.dumps(cfg))
        print(execute(c,'sudo -n python3 -c '+shlex.quote(INSTALL)+' '+shlex.quote(stage)),flush=True)
        # Prevent legacy watchdogs respawning a driver during ownership transfer.
        print(execute(c,'sudo -n systemctl stop ld06 odom ch341-bmi270 auto-mapping slam-control'),flush=True)
        retired=True
        run_script(c,'armpi_pro',ROS1_SETUP,RETIRE)
        print(execute(c,'sudo -n install -d /run/armpi-ros2 && sudo -n docker cp armpi_pro:/tmp/ros2-cutover-seed.json /run/armpi-ros2/cutover-seed.json'),flush=True)
        # No old driver may survive, even if docker-exec service stop left it orphaned.
        assertion="import importlib.util; s=importlib.util.spec_from_file_location('core','/opt/armpi-ros2/run_native_core.py'); m=importlib.util.module_from_spec(s);s.loader.exec_module(m);assert not m.legacy_processes(),m.legacy_processes()"
        for attempt in range(20):
            try:
                execute(c,'sudo -n python3 -c '+shlex.quote(assertion));break
            except RuntimeError:
                if attempt==19:raise
                import time
                time.sleep(.5)
        execute(c,'sudo -n touch /etc/armpi-ros2/active && sudo -n systemctl disable ld06 odom ch341-bmi270 auto-mapping slam-control && sudo -n systemctl enable armpi-ros2 && sudo -n systemctl start armpi-ros2')
        # Wait only for our named container, never assume systemd active is readiness.
        import time
        name=None
        for _ in range(20):
            try:
                record=json.loads(execute(c,'sudo -n cat /run/armpi-ros2/container.json'));name=record['name'];break
            except RuntimeError:time.sleep(.5)
        assert name and re.fullmatch('armpi_ros2_native_[0-9a-f]{12}',name),'Native container not created; inspect unit journal'
        baseline=json.loads(execute(c,'sudo -n cat /run/armpi-ros2/cutover-seed.json'))
        run_script(c,name,ROS2_SETUP,'BASELINE='+repr(baseline)+'\n'+PROBE)
        print(execute(c,'sudo -n systemctl is-active armpi-ros2 && sudo -n systemctl is-enabled armpi-ros2 && sudo -n cat /run/armpi-ros2/container.json'),flush=True)
        execute(c,'sudo -n mv /run/armpi-ros2/cutover-seed.json '+shlex.quote(stage+'/backup/cutover-seed.applied.json'))
        print('CUTOVER_VERIFIED_BACKUP='+stage+'/backup',flush=True)
    except Exception:
        if retired:
            try:print(execute(c,'sudo -n systemctl stop armpi-ros2; sudo -n journalctl -u armpi-ros2 -n 35 --no-pager'),flush=True)
            except Exception as e:print('Unable to confirm native stop; physical stop required: '+str(e),flush=True)
        raise
    finally:c.close()

if __name__=='__main__':main()
