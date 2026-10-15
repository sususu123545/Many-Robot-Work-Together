"""Apply only the measured ROS1 lidar XY offset, with backup and verification.

No wheel/arm commands, no lidar driver restart, no safety/filter edits.
The stop/start operations refer only to the static base_to_laser TF publisher.
"""
import argparse
import difflib
import hashlib
import shlex
import sys
import uuid
import xml.etree.ElementTree as ET
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ros2_preflight import connect
from bench_ros2_forward import ROS1_SETUP, run_script


PATH = '/home/ubuntu/ld06_ws/src/ldlidar_stl_ros/launch/ld06.launch'
OLD = '0.0 0.0 0.18 0 0.0 0.0 base_link base_laser 50'
NEW = '-0.01 0.01 0.18 0 0.0 0.0 base_link base_laser 50'


def exact_patch(original, updated):
    lines = difflib.unified_diff(original.splitlines(True), updated.splitlines(True),
                                 fromfile=PATH, tofile=PATH)
    return ''.join(line if line.endswith('\n') else line + '\n\\ No newline at end of file\n'
                   for line in lines)


STOP = r'''
import time,json,pathlib,rospy,rosnode
from std_msgs.msg import String,Int32MultiArray
rospy.init_node('lidar_translation_tf_replace_only',anonymous=True,disable_signals=True)
s=json.loads(rospy.wait_for_message('/auto_mapping/status',String,timeout=3).data)
assert s['state']=='IDLE' and not s['slam_running']
a=list(rospy.wait_for_message('/chassis_control/encoder_counts',Int32MultiArray,timeout=3).data)
time.sleep(.4)
b=list(rospy.wait_for_message('/chassis_control/encoder_counts',Int32MultiArray,timeout=3).data)
assert max(abs(x-y) for x,y in zip(a,b))<=2
ok,bad=rosnode.kill_nodes(['/base_to_laser'])
assert ok==['/base_to_laser'] and not bad
for _ in range(30):
 found=[]
 for p in pathlib.Path('/proc').glob('[0-9]*/cmdline'):
  try:
   args=p.read_bytes().split(b'\x00')
   if b'__name:=base_to_laser' in args and any(a.endswith(b'/static_transform_publisher') for a in args):found.append(str(p))
  except OSError:pass
 if not found:break
 time.sleep(.05)
assert not found,'Old TF still running'
print('ONLY_BASE_TO_LASER_RETIRED=true')
'''

VERIFY = r'''
import time,json,math,rospy,tf2_ros,rosnode
from std_msgs.msg import Int32MultiArray,String
from sensor_msgs.msg import LaserScan,Imu
from tf2_msgs.msg import TFMessage
rospy.init_node('lidar_translation_postcheck',anonymous=True,disable_signals=True)
b=tf2_ros.Buffer();l=tf2_ros.TransformListener(b)
enc=[];scan=[];imu=[];status=[];pubs=set()
def cb(m):
 for t in m.transforms:
  if t.child_frame_id=='base_laser':pubs.add(m._connection_header.get('callerid'))
subs=[rospy.Subscriber('/tf',TFMessage,cb,queue_size=100),rospy.Subscriber('/tf_static',TFMessage,cb,queue_size=100),rospy.Subscriber('/chassis_control/encoder_counts',Int32MultiArray,lambda m:enc.append(list(m.data)),queue_size=100),rospy.Subscriber('/scan',LaserScan,lambda m:scan.append((time.monotonic(),m)),queue_size=100),rospy.Subscriber('/bmi270/imu_raw',Imu,lambda m:imu.append(m),queue_size=200),rospy.Subscriber('/auto_mapping/status',String,lambda m:status.append(json.loads(m.data)),queue_size=30)]
time.sleep(3)
for s in subs:s.unregister()
assert len(enc)>40 and len(scan)>20 and len(imu)>120
assert max(abs(e[i]-enc[0][i]) for e in enc for i in range(4))<=2
assert status[-1]['state']=='IDLE' and not status[-1]['slam_running']
assert pubs=={'/base_to_laser'}
t=b.lookup_transform('base_link','base_laser',rospy.Time(0),rospy.Duration(2)).transform
xyz=[t.translation.x,t.translation.y,t.translation.z];q=[t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w]
assert max(abs(a-v) for a,v in zip(xyz,[-.01,.01,.18]))<1e-9
assert max(abs(a-v) for a,v in zip(q,[0,0,0,1]))<1e-9
# Candidates only: do not delete any points or publish a filtered scan.
inside=outside=0;outside_bins={}
for _,m in scan:
 for i,r in enumerate(m.ranges):
  if not(math.isfinite(r) and m.range_min<=r<=m.range_max and r<.25):continue
  a=m.angle_min+i*m.angle_increment;x=-.01+r*math.cos(a);y=.01+r*math.sin(a)
  if abs(x)<=.15 and abs(y)<=.125:inside+=1
  else:
   outside+=1;deg=math.degrees(math.atan2(math.sin(a),math.cos(a)));key=str(int(math.floor(deg/10))*10)
   outside_bins[key]=outside_bins.get(key,0)+1
rate=(len(scan)-1)/(scan[-1][0]-scan[0][0])
assert 8<rate<12
names=rosnode.get_node_names()
assert all(n in names for n in ['/LD06','/chassis_control','/mecanum_odom','/bmi270_imu','/hiwonder_servo_manager'])
print(json.dumps({'TF_verified':True,'lidar_xyz':xyz,'lidar_quaternion':q,'TF_publishers':sorted(pubs),'scan_hz':rate,'imu_messages':len(imu),'encoders_stable':True,'mapping_state':status[-1]['state'],'scan_untouched':True,'body_projection_candidates':inside,'near_points_outside_body_projection':outside,'outside_near_bins_deg':outside_bins,'lidar_chassis_odom_imu_servo_nodes_preserved':True},indent=2))
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', required=True)
    parser.add_argument('--config', required=True)
    args = parser.parse_args()
    sys.stdout.reconfigure(encoding='utf-8')
    client = connect(args.host, config_path=args.config)
    backup = PATH + '.xy-backup-20261011-' + uuid.uuid4().hex[:8]
    changed = retired = False

    def call(cmd, data=None):
        stdin, out, err = client.exec_command(cmd, timeout=20)
        if data is not None:
            stdin.write(data)
            stdin.flush()
            stdin.channel.shutdown_write()
        output, errors = out.read().decode(), err.read().decode()
        if out.channel.recv_exit_status() != 0:
            raise RuntimeError(errors or output)
        return output

    def start_tf(arguments):
        call('docker exec -d armpi_pro bash -lc ' + shlex.quote(
            ROS1_SETUP + 'exec rosrun tf static_transform_publisher ' + arguments + ' __name:=base_to_laser'))

    try:
        text = call('docker exec armpi_pro ' + shlex.join(['sed', '-n', '1,160p', PATH]))
        assert text.count('args="' + OLD + '"') == 1, 'Unexpected launch args: refuse overwrite'
        node = ET.fromstring(text).find("node[@name='base_to_laser']")
        assert node.get('required', 'false') == 'false' and node.get('respawn', 'false') == 'false'
        updated = text.replace('args="' + OLD + '"', 'args="' + NEW + '"', 1)
        patch = exact_patch(text, updated)
        print(call('docker exec -i armpi_pro patch --dry-run --forward --fuzz=0 ' + shlex.quote(PATH), patch), flush=True)
        call('docker exec armpi_pro ' + shlex.join(['cp', '-p', PATH, backup]))
        changed = True
        print(call('docker exec -i armpi_pro patch --forward --fuzz=0 ' + shlex.quote(PATH), patch), flush=True)
        assert call('docker exec armpi_pro ' + shlex.join(['sed', '-n', '1,160p', PATH])) == updated
        print('BACKUP=' + backup + ' ORIGINAL_SHA256=' + hashlib.sha256(text.encode()).hexdigest(), flush=True)
        retired = True
        run_script(client, 'armpi_pro', ROS1_SETUP, STOP)
        start_tf(NEW)
        run_script(client, 'armpi_pro', ROS1_SETUP, VERIFY)
        print('MOTION_COMMANDS_SENT=0; PRODUCTION_FILTERS_UNCHANGED=true', flush=True)
    except Exception:
        if changed:
            call('docker exec armpi_pro ' + shlex.join(['cp', '-p', backup, PATH]))
        if retired:
            try:
                run_script(client, 'armpi_pro', ROS1_SETUP, STOP)
            except Exception:
                names = call('docker exec armpi_pro bash -lc ' + shlex.quote(ROS1_SETUP + 'rosnode list'))
                if '/base_to_laser' in names.splitlines():
                    raise RuntimeError('TF ownership uncertain; refuse duplicate publisher')
            start_tf(OLD)
        raise
    finally:
        client.close()


if __name__ == '__main__':
    main()
