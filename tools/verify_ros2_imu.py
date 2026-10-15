"""Stationary native ROS 2 BMI270 test, restoring the ROS 1 sensor in finally.

Only maps the discovered CH341 I2C device; never maps motors or arm serial.
Copies the existing pure-Python sensor library to a new directory and mounts
it read-only. No firmware calibration, TF, boot service or old source edits.
"""
import argparse
import json
import re
import shlex
import sys
import time
import uuid
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ros2_preflight import connect
from stage_ros2 import execute
from bench_ros2_forward import ROS1_SETUP, ROS2_SETUP, run_script


DISCOVER = '''
import json,os,pathlib
found=[]
for p in pathlib.Path('/sys/class/i2c-adapter').glob('i2c-*/name'):
 if 'i2c-ch341-usb' in p.read_text().lower():
  device='/dev/'+p.parent.name
  if os.path.exists(device): found.append(device)
assert len(found)==1, 'Expected exactly one CH341 adapter'
print(json.dumps(found[0]))
'''

OWNERS = '''
import json,os,pathlib,stat
target=os.stat(DEVICE)
assert stat.S_ISCHR(target.st_mode)
owners=[]
for process in pathlib.Path('/proc').glob('[0-9]*'):
 try:
  for fd in (process/'fd').iterdir():
   try:
    s=fd.stat()
    if stat.S_ISCHR(s.st_mode) and s.st_rdev==target.st_rdev:
     args=(process/'cmdline').read_bytes().split(b'\\x00')
     owners.append({'pid':int(process.name),'is_old_imu':any(a.endswith(b'/bmi270_imu_node.py') for a in args)})
     break
   except (FileNotFoundError,ProcessLookupError): pass
 except (FileNotFoundError,ProcessLookupError): pass
print(json.dumps(owners))
'''

LIBRARY_DIGEST = '''
import hashlib,pathlib
root=pathlib.Path(ROOT)
files=sorted(p for p in root.rglob('*') if p.is_file() and p.suffix=='.py' and '__pycache__' not in p.parts)
assert files and (root/'bmi270/config_file.py').is_file()
h=hashlib.sha256()
for p in files:
 h.update(p.relative_to(root).as_posix().encode()+b'\\0')
 h.update(p.read_bytes()+b'\\0')
print(h.hexdigest())
'''

READINESS = '''
import json,time,rospy
from std_msgs.msg import String,Int32MultiArray
from sensor_msgs.msg import Imu
rospy.init_node('migration_imu_readiness',anonymous=True)
s=json.loads(rospy.wait_for_message('/auto_mapping/status',String,timeout=5).data)
assert s['state']=='IDLE' and s['slam_running'] is False
a=rospy.wait_for_message('/chassis_control/encoder_counts',Int32MultiArray,timeout=4)
time.sleep(.4)
b=rospy.wait_for_message('/chassis_control/encoder_counts',Int32MultiArray,timeout=4)
assert list(a.data)==list(b.data), 'Wheels are not stationary'
m=rospy.wait_for_message('/bmi270/imu_raw',Imu,timeout=4)
assert max(abs(v) for v in (m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z))<.02, 'IMU indicates motion'
print(json.dumps({'mapping_state':s['state'],'encoder_counts_stable':True}))
'''

STOP_OLD = '''
import rosnode,rospy
rospy.init_node('migration_imu_retire_old',anonymous=True)
if '/bmi270_imu' in rosnode.get_node_names():
 succeeded,failed=rosnode.kill_nodes(['/bmi270_imu'])
 assert succeeded==['/bmi270_imu'] and not failed
print('OLD_IMU_NODE_STOPPED')
'''

ROS1_IMU = '''
import json,math,time,rospy
from sensor_msgs.msg import Imu
rospy.init_node('migration_imu_ros1_readonly',anonymous=True)
messages=[]
sub=rospy.Subscriber('/bmi270/imu_raw',Imu,lambda m:messages.append((time.monotonic(),m)),queue_size=100)
deadline=time.monotonic()+8
while len(messages)<50 and time.monotonic()<deadline: time.sleep(.02)
assert len(messages)>=50, 'ROS1 IMU did not recover'
last=messages[-1][1]
assert last.header.frame_id=='imu_link'
rate=(len(messages)-1)/(messages[-1][0]-messages[0][0])
assert 20<rate<80
assert all(math.isfinite(v) for v in (last.linear_acceleration.x,last.linear_acceleration.y,last.linear_acceleration.z,last.angular_velocity.z))
print(json.dumps({'ROS1_imu_ok':True,'samples':len(messages),'rate_hz':rate,'frame':last.header.frame_id}))
'''

ROS2_IMU = '''
import json,math,time,rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import String,Int32MultiArray
rclpy.init()
node=Node('migration_imu_ros2_readonly')
messages,odom,status,encoders=[],[],[],[]
node.create_subscription(Imu,'/bmi270/imu_raw',lambda m:messages.append((time.monotonic(),m)),qos_profile_sensor_data)
node.create_subscription(Odometry,'/odom',lambda m:odom.append(m),10)
node.create_subscription(String,'/chassis_control/status',lambda m:status.append(json.loads(m.data)),10)
node.create_subscription(Int32MultiArray,'/chassis_control/encoder_counts',lambda m:encoders.append(list(m.data)),10)
try:
 deadline=time.monotonic()+20
 while time.monotonic()<deadline:
  rclpy.spin_once(node,timeout_sec=.05)
  if status:
   assert status[-1]['dry_run'] is True and status[-1]['enable_motion'] is False
   assert status[-1]['wheel_command']==[0,0,0,0] and not status[-1]['hardware_fault']
  if len(messages)>=200 and len(odom)>=50 and status: break
 assert len(messages)>=200 and len(odom)>=50 and status and encoders, 'Native IMU/odom not ready'
 assert all(e==[0,0,0,0] for e in encoders), 'Only simulated encoders allowed'
 stamps=[];acc=[];gyro=[]
 for _,m in messages:
  assert m.header.frame_id=='imu_link' and m.orientation_covariance[0]==-1
  a=[m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z]
  g=[m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z]
  assert all(math.isfinite(v) for v in a+g)
  assert 7<math.sqrt(sum(v*v for v in a))<12 and max(abs(v) for v in g)<.02, 'Unexpected stationary SI readings'
  acc.append(a);gyro.append(g)
  stamps.append(m.header.stamp.sec*10**9+m.header.stamp.nanosec)
 assert min(stamps)>0 and all(b>a for a,b in zip(stamps,stamps[1:]))
 rate=(len(messages)-1)/(messages[-1][0]-messages[0][0])
 assert 20<rate<80
 names=[i.node_name for i in node.get_subscriptions_info_by_topic('/bmi270/imu_raw')]
 assert 'mecanum_odom' in names, 'No native IMU odometry subscriber'
 assert all(o.pose.pose.position.x==0 and o.pose.pose.position.y==0 for o in odom)
 # With zero encoder yaw and this test's yaw-only mount, nonzero odom yaw
 # must come from actual gyro data. Match past stamped samples (DDS order
 # across topics is not guaranteed); do not mistake a subscription for fusion.
 pairs=[]
 for o in odom:
  stamp=o.header.stamp.sec*10**9+o.header.stamp.nanosec
  previous=[i for i,s in enumerate(stamps) if 0<=stamp-s<=250000000]
  if previous: pairs.append((o.twist.twist.angular.z,.7*gyro[previous[-1]][2]))
 assert len(pairs)>=20 and max(abs(a) for a,b in pairs)>1e-6, 'No gyro-assisted yaw updates'
 assert max(abs(a-b) for a,b in pairs)<.003, 'IMU yaw fusion inconsistent'
 mean=lambda rows:[sum(r[i] for r in rows)/len(rows) for i in range(3)]
 print(json.dumps({'ROS2_imu_ok':True,'samples':len(messages),'rate_hz':rate,'frame':'imu_link',
                   'accel_mean_m_s2':mean(acc),'gyro_mean_rad_s':mean(gyro),
                   'native_odom_subscribes':True,'zero_encoder_gyro_fusion_observed':True,
                   'max_fusion_error_rad_s':max(abs(a-b) for a,b in pairs),
                   'mount_yaw_not_physically_verified':True,'physical_rotation_accuracy_not_validated':True}),flush=True)
finally:
 node.destroy_node();rclpy.shutdown()
'''


def owners(client, device):
    return json.loads(execute(client, 'sudo -n python3 -c ' + shlex.quote('DEVICE=' + repr(device) + '\n' + OWNERS)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', required=True)
    parser.add_argument('--config', required=True)
    parser.add_argument('--image', required=True)
    parser.add_argument('--expected-image-id', required=True)
    args = parser.parse_args()
    if (not re.fullmatch(r'armpi-ros2-migration:stage-[a-zA-Z0-9]+', args.image)
            or not re.fullmatch(r'sha256:[0-9a-f]{64}', args.expected_image_id)):
        parser.error('Use the verified migration image tag and exact image ID')
    client = connect(args.host, config_path=args.config)
    name = 'armpi_ros2_imu_' + uuid.uuid4().hex[:12]
    old_may_be_stopped, created, restore_allowed = False, False, True
    device = None
    try:
        assert execute(client, 'docker image inspect ' + shlex.quote(args.image) + " --format '{{.Id}}'") == args.expected_image_id
        device = json.loads(execute(client, 'python3 -c ' + shlex.quote(DISCOVER)))
        assert re.fullmatch(r'/dev/i2c-[0-9]+', device) and device != '/dev/i2c-1'
        assert execute(client, 'systemctl is-active ch341-bmi270') == 'active'
        current = owners(client, device)
        assert len(current) == 1 and current[0]['is_old_imu'], 'Unexpected CH341 device owner'
        run_script(client, 'armpi_pro', ROS1_SETUP, READINESS)
        run_script(client, 'armpi_pro', ROS1_SETUP, ROS1_IMU)
        library = execute(client, 'mktemp -d /home/pi/armpi-ros2-imu-library.XXXXXX')
        assert re.fullmatch(r'/home/pi/armpi-ros2-imu-library\.[a-zA-Z0-9]+', library)
        root = '/home/ubuntu/share/tmp/bmi270_py'
        source_digest = execute(client, 'docker exec armpi_pro python3 -c ' + shlex.quote('ROOT=' + repr(root) + '\n' + LIBRARY_DIGEST))
        execute(client, 'docker cp armpi_pro:' + root + '/. ' + shlex.quote(library))
        assert execute(client, 'python3 -c ' + shlex.quote('ROOT=' + repr(library) + '\n' + LIBRARY_DIGEST)) == source_digest
        print('COPIED_LIBRARY=' + library + ' PYTHON_SOURCE_SHA256=' + source_digest, flush=True)
        # Validate Python 3.12 dependencies before pausing the working ROS 1 sensor.
        dep_name = name + '_import'
        print(execute(client, 'docker run --network none --name ' + dep_name +
                      ' --mount ' + shlex.quote('type=bind,src=' + library + ',dst=/opt/bmi270_py,readonly') +
                      ' ' + shlex.quote(args.image) + ' python3 -c ' + shlex.quote(
                          "import sys;sys.path.insert(0,'/opt/bmi270_py');import numpy,smbus2,bmi270.BMI270;print('PYTHON312_IMU_IMPORT=PASS')")), flush=True)
        old_may_be_stopped = True
        execute(client, 'sudo -n systemctl stop ch341-bmi270')
        run_script(client, 'armpi_pro', ROS1_SETUP, STOP_OLD)
        deadline = time.monotonic() + 6
        while owners(client, device) and time.monotonic() < deadline: time.sleep(.2)
        assert owners(client, device) == [], 'Old IMU still owns CH341; refuse concurrent initialization'
        launch = (ROS2_SETUP + 'exec timeout --signal=INT --kill-after=5 90 '
                  'ros2 launch armpi_ros2 base.launch.py dry_run:=true enable_motion:=false '
                  'imu:=true lidar:=false rosbridge:=false')
        created = True
        print(execute(client, 'docker run -d --network none --stop-signal SIGINT --device ' +
                      shlex.quote(device + ':' + device) + ' --mount ' +
                      shlex.quote('type=bind,src=' + library + ',dst=/opt/bmi270_py,readonly') +
                      ' --name ' + name + ' ' + shlex.quote(args.image) + ' bash -lc ' + shlex.quote(launch)), flush=True)
        print(execute(client, 'docker inspect ' + name +
                      " --format 'Devices={{json .HostConfig.Devices}} Mounts={{json .Mounts}} Network={{.HostConfig.NetworkMode}}'"), flush=True)
        run_script(client, name, ROS2_SETUP, ROS2_IMU)
        print('STATIONARY_NATIVE_ROS2_IMU_VERIFICATION=PASS', flush=True)
    finally:
        try:
            if created and name in execute(client, "docker ps -a --format '{{.Names}}'").splitlines():
                execute(client, 'docker stop -t 15 ' + name)
                restore_allowed = execute(client, 'docker inspect ' + name + " --format '{{.State.Running}}'") == 'false'
                print('STOPPED_IMU_CONTAINER=' + name, flush=True)
            if old_may_be_stopped:
                if not restore_allowed or owners(client, device):
                    raise RuntimeError('CH341 owner uncertain; do not start the old sensor')
                execute(client, 'sudo -n systemctl start ch341-bmi270')
                run_script(client, 'armpi_pro', ROS1_SETUP, ROS1_IMU)
                assert execute(client, 'systemctl is-active ch341-bmi270') == 'active'
                print('ROS1_IMU_RESTORED=true', flush=True)
        finally:
            client.close()


if __name__ == '__main__':
    main()
