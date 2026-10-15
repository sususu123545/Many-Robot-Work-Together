"""Bounded stationary LD06 test; restore the original ROS 1 lidar in finally.

No motor/arm device, no autonomous task, no network exposure, no boot changes.
Requires trusted SSH and a verified migration image. Never shares the serial
port between ROS 1 and ROS 2. Stopped test containers retain their logs.
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


OWNERS = '''
import json,os,pathlib,stat
target=os.stat('/dev/ld06')
assert stat.S_ISCHR(target.st_mode), 'LD06 must resolve to a character device'
owners=[]
for process in pathlib.Path('/proc').glob('[0-9]*'):
    try:
        for fd in (process/'fd').iterdir():
            try:
                value=fd.stat()
                if stat.S_ISCHR(value.st_mode) and value.st_rdev==target.st_rdev:
                    args=(process/'cmdline').read_bytes().split(b'\\x00')
                    owners.append({'pid':int(process.name),'program':os.path.basename(args[0].decode())})
                    break
            except (FileNotFoundError,ProcessLookupError):
                pass
    except (FileNotFoundError,ProcessLookupError):
        pass
print(json.dumps(owners))
'''

READINESS = '''
import json,rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
rospy.init_node('migration_lidar_readiness',anonymous=True)
s=json.loads(rospy.wait_for_message('/auto_mapping/status',String,timeout=5).data)
o=rospy.wait_for_message('/odom',Odometry,timeout=5)
v=o.twist.twist
assert s['state']=='IDLE' and s['slam_running'] is False, 'Active autonomous/SLAM task'
assert abs(v.linear.x)+abs(v.linear.y)+abs(v.angular.z)<1e-6, 'Chassis must be stationary'
print(json.dumps({'mapping_state':s['state'],'chassis_stationary':True}))
'''

STOP_OLD = '''
import rosnode,rospy
rospy.init_node('migration_lidar_retire_old',anonymous=True)
if '/LD06' in rosnode.get_node_names():
    succeeded,failed=rosnode.kill_nodes(['/LD06'])
    assert succeeded==['/LD06'] and not failed
print('OLD_LD06_NODE_STOPPED')
'''

ROS1_SCAN = '''
import json,math,time,rospy
from sensor_msgs.msg import LaserScan
rospy.init_node('migration_lidar_ros1_readonly',anonymous=True)
messages=[]
def receive(m): messages.append((time.monotonic(),m))
subscriber=rospy.Subscriber('/scan',LaserScan,receive,queue_size=20)
deadline=time.monotonic()+12
while len(messages)<20 and time.monotonic()<deadline: time.sleep(.02)
assert len(messages)>=20, 'ROS1 /scan is not publishing'
last=messages[-1][1]
valid=sum(math.isfinite(r) and last.range_min<=r<=last.range_max for r in last.ranges)
assert last.header.frame_id=='base_laser' and valid>10
rate=(len(messages)-1)/(messages[-1][0]-messages[0][0])
assert rate>5, 'ROS1 lidar rate unexpectedly low'
print(json.dumps({'ROS1_scan_ok':True,'samples':len(messages),'rate_hz':rate,
                  'ranges':len(last.ranges),'valid_ranges':valid,'frame':last.header.frame_id}))
'''

ROS2_SCAN = '''
import json,math,time,rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
rclpy.init()
node=Node('migration_lidar_ros2_readonly')
messages=[]
node.create_subscription(LaserScan,'/scan',lambda m:messages.append((time.monotonic(),m)),qos_profile_sensor_data)
try:
 deadline=time.monotonic()+15
 while len(messages)<40 and time.monotonic()<deadline: rclpy.spin_once(node,timeout_sec=.1)
 assert len(messages)>=40, 'ROS2 /scan is not publishing'
 last=messages[-1][1]
 valid_counts=[]
 stamps=[]
 for _,m in messages:
  assert m.header.frame_id=='base_laser' and len(m.ranges)>100
  assert m.range_max>m.range_min>=0 and m.angle_increment>0
  assert 6.0<m.angle_max-m.angle_min<6.4, 'Not a complete scan'
  valid_counts.append(sum(math.isfinite(r) and m.range_min<=r<=m.range_max for r in m.ranges))
  stamps.append(m.header.stamp.sec*10**9+m.header.stamp.nanosec)
 assert min(valid_counts)>10 and min(stamps)>0
 assert all(b>a for a,b in zip(stamps,stamps[1:])), 'Scan timestamps not increasing'
 rate=(len(messages)-1)/(messages[-1][0]-messages[0][0])
 assert 5<rate<20, 'Unexpected LD06 scan rate'
 print(json.dumps({'ROS2_scan_ok':True,'samples':len(messages),'rate_hz':rate,
                   'ranges':len(last.ranges),'min_valid_ranges':min(valid_counts),
                   'frame':last.header.frame_id,'timestamps_increasing':True,
                   'angular_direction_not_physically_verified':True}),flush=True)
finally:
 node.destroy_node();rclpy.shutdown()
'''

ROS2_MAPPING = '''
import json,time,rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile,DurabilityPolicy
from nav_msgs.msg import OccupancyGrid,Odometry
from std_msgs.msg import String
from tf2_ros import Buffer,TransformListener,TransformException
rclpy.init()
node=Node('migration_stationary_mapping_readonly')
maps,status,odoms=[],[],[]
buffer=Buffer()
listener=TransformListener(buffer,node)
node.create_subscription(OccupancyGrid,'/map',lambda m:maps.append(m),
                         QoSProfile(depth=5,durability=DurabilityPolicy.TRANSIENT_LOCAL))
node.create_subscription(String,'/chassis_control/status',lambda m:status.append(json.loads(m.data)),10)
node.create_subscription(Odometry,'/odom',lambda m:odoms.append(m),10)
transform=None
try:
 deadline=time.monotonic()+25
 while time.monotonic()<deadline:
  rclpy.spin_once(node,timeout_sec=.1)
  if status:
   assert status[-1]['dry_run'] is True and status[-1]['enable_motion'] is False
   assert status[-1]['wheel_command']==[0,0,0,0] and not status[-1]['hardware_fault']
  try: transform=buffer.lookup_transform('map','base_link',Time())
  except TransformException: transform=None
  if len(maps)>=2 and transform is not None and len(odoms)>=20 and status: break
 assert len(maps)>=2 and transform is not None and status and len(odoms)>=20, 'Map/TF chain not ready'
 last=maps[-1]
 assert last.header.frame_id=='map' and last.info.width>0 and last.info.height>0
 assert len(last.data)==last.info.width*last.info.height and last.info.resolution>0
 known=sum(v>=0 for v in last.data); occupied=sum(v>=50 for v in last.data)
 assert known>20 and occupied>0, 'No mapped lidar returns'
 assert all(abs(m.twist.twist.linear.x)+abs(m.twist.twist.linear.y)+abs(m.twist.twist.angular.z)==0 for m in odoms)
 assert all(m.pose.pose.position.x==0 and m.pose.pose.position.y==0 for m in odoms)
 print(json.dumps({'ROS2_stationary_mapping_ok':True,'map_messages':len(maps),
                   'map_width':last.info.width,'map_height':last.info.height,
                   'resolution':last.info.resolution,'known_cells':known,'occupied_cells':occupied,
                   'tf_map_to_base_link':True,'motor_device_not_mapped':True,
                   'odometry_simulated_zero':True,'physical_mapping_quality_not_validated':True}),flush=True)
finally:
 node.destroy_node();rclpy.shutdown()
'''


def owners(client):
    return json.loads(execute(client, 'sudo -n python3 -c ' + shlex.quote(OWNERS)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', required=True)
    parser.add_argument('--config', required=True)
    parser.add_argument('--image', required=True)
    parser.add_argument('--expected-image-id', required=True)
    parser.add_argument('--mapping-smoke', action='store_true',
                        help='Also test stationary SLAM with real scan and simulated zero odometry')
    args = parser.parse_args()
    if (not re.fullmatch(r'armpi-ros2-migration:stage-[a-zA-Z0-9]+', args.image)
            or not re.fullmatch(r'sha256:[0-9a-f]{64}', args.expected_image_id)):
        parser.error('Use the verified migration image tag and exact image ID')
    client = connect(args.host, config_path=args.config)
    name = 'armpi_ros2_lidar_' + uuid.uuid4().hex[:12]
    old_may_be_stopped, created, restore_allowed = False, False, True
    try:
        assert execute(client, 'docker image inspect ' + shlex.quote(args.image) +
                       " --format '{{.Id}}'") == args.expected_image_id
        assert execute(client, 'systemctl is-active ld06') == 'active'
        current = owners(client)
        assert len(current) == 1 and current[0]['program'] == 'ldlidar_stl_ros_node', 'Unexpected serial owner'
        run_script(client, 'armpi_pro', ROS1_SETUP, READINESS)
        run_script(client, 'armpi_pro', ROS1_SETUP, ROS1_SCAN)
        old_may_be_stopped = True
        execute(client, 'sudo -n systemctl stop ld06')
        run_script(client, 'armpi_pro', ROS1_SETUP, STOP_OLD)
        deadline = time.monotonic() + 6
        while owners(client) and time.monotonic() < deadline:
            time.sleep(.2)
        assert owners(client) == [], 'Serial still occupied; do not start another driver'
        if args.mapping_smoke:
            launch = (ROS2_SETUP + 'exec timeout --signal=INT --kill-after=5 90 '
                      'ros2 launch armpi_ros2 base.launch.py '
                      'dry_run:=true enable_motion:=false lidar:=true imu:=false rosbridge:=false')
        else:
            launch = (ROS2_SETUP + 'exec timeout --signal=INT --kill-after=5 90 '
                      'ros2 run ldlidar_stl_ros2 ldlidar_stl_ros2_node --ros-args '
                      '-r __node:=LD06 -p product_name:=LDLiDAR_LD06 -p topic_name:=/scan '
                      '-p frame_id:=base_laser -p port_name:=/dev/ld06 -p port_baudrate:=230400 '
                      '-p laser_scan_dir:=true -p enable_angle_crop_func:=false '
                      '-p angle_crop_min:=135.0 -p angle_crop_max:=225.0')
        created = True
        print(execute(client, 'docker run -d --network none --stop-signal SIGINT '
                      '--device /dev/ld06:/dev/ld06 --name ' + name + ' ' + shlex.quote(args.image) +
                      ' bash -lc ' + shlex.quote(launch)), flush=True)
        print(execute(client, 'docker inspect ' + name +
                      " --format 'Devices={{json .HostConfig.Devices}} Mounts={{json .Mounts}} Network={{.HostConfig.NetworkMode}}'"), flush=True)
        run_script(client, name, ROS2_SETUP, ROS2_SCAN)
        if args.mapping_smoke:
            execute(client, 'docker exec -d ' + name + ' bash -lc ' + shlex.quote(
                ROS2_SETUP + 'exec timeout --signal=INT --kill-after=5 60 '
                'ros2 launch armpi_ros2 mapping.launch.py'))
            run_script(client, name, ROS2_SETUP, ROS2_MAPPING)
        print('STATIONARY_ROS2_LIDAR_VERIFICATION=PASS', flush=True)
    finally:
        try:
            if created and name in execute(client, "docker ps -a --format '{{.Names}}'").splitlines():
                execute(client, 'docker stop -t 15 ' + name)
                restore_allowed = execute(client, 'docker inspect ' + name + " --format '{{.State.Running}}'") == 'false'
                print('STOPPED_LIDAR_CONTAINER=' + name, flush=True)
            if old_may_be_stopped:
                if not restore_allowed or owners(client):
                    raise RuntimeError('Serial ownership uncertain; do not start the old lidar')
                execute(client, 'sudo -n systemctl start ld06')
                run_script(client, 'armpi_pro', ROS1_SETUP, ROS1_SCAN)
                assert execute(client, 'systemctl is-active ld06') == 'active'
                print('ROS1_LIDAR_RESTORED=true', flush=True)
        finally:
            client.close()


if __name__ == '__main__':
    main()
