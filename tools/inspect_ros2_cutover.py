"""Read-only live cutover readiness; no motion publications."""
import argparse
import json
import shlex
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ros2_preflight import connect
from stage_ros2 import execute
from bench_ros2_forward import ROS1_SETUP, run_script

READINESS = '''
import json,time,math,rospy
from std_msgs.msg import String,Int32MultiArray
from nav_msgs.msg import Odometry
from ros_robot_controller.srv import GetBusServoState,GetBusServoStateRequest
from ros_robot_controller.msg import GetBusServoCmd
rospy.init_node('ros2_stationary_cutover_inventory',anonymous=True)
s=json.loads(rospy.wait_for_message('/auto_mapping/status',String,timeout=5).data)
assert s['state']=='IDLE' and not s['slam_running'], 'Autonomous mapping must be idle'
samples=[]
sub=rospy.Subscriber('/chassis_control/encoder_counts',Int32MultiArray,lambda m:samples.append(list(m.data)))
time.sleep(1.5)
assert len(samples)>10 and all(x==samples[0] for x in samples),'Wheels not stationary'
o=rospy.wait_for_message('/odom',Odometry,timeout=5)
v=o.twist.twist
print(json.dumps({'stationary_encoders':samples[-1],'sample_count':len(samples),'odom_velocity':[v.linear.x,v.linear.y,v.angular.z]}),flush=True)
assert abs(v.linear.x)+abs(v.linear.y)+abs(v.angular.z)<.01,'Odometry reports motion'
service=rospy.ServiceProxy('/ros_robot_controller/bus_servo/get_state',GetBusServoState)
def read_arm():
    q=GetBusServoStateRequest()
    for i in range(1,7):
        c=GetBusServoCmd();c.id=i;c.get_position=1;c.get_temperature=1;c.get_position_limit=1;c.get_max_temperature_limit=1
        q.cmd.append(c)
    r=service(q);assert r.success and len(r.state)==6
    return [{'id':i+1,'raw':a.position[0],'temperature':a.temperature[0],'limits':list(a.position_limit),'temperature_limit':list(a.max_temperature_limit)} for i,a in enumerate(r.state)]
a=read_arm();time.sleep(.5);b=read_arm()
assert all(abs(x['raw']-y['raw'])<=3 for x,y in zip(a,b)),'Arm is moving'
assert all(x['temperature']<65 for x in b),'Arm needs cooling'
p=o.pose.pose.position;q=o.pose.pose.orientation
print(json.dumps({'mapping':s,'encoder_counts':samples[-1],'old_odom_pose':[p.x,p.y,2*math.atan2(q.z,q.w)],'arm':b}))
'''

HOST = '''
import json,pathlib,subprocess,os,stat
def run(cmd):
    p=subprocess.run(cmd,capture_output=True,text=True,timeout=15)
    return {'exit':p.returncode,'stdout':p.stdout,'stderr':p.stderr}
devices=['/dev/rrc','/dev/ld06','/dev/i2c-1','/dev/i2c-22']
owners={}
for d in devices:
    target=os.stat(d).st_rdev;found=[]
    for p in pathlib.Path('/proc').glob('[0-9]*'):
        try:
            for f in (p/'fd').iterdir():
                try:
                    s=f.stat()
                    if stat.S_ISCHR(s.st_mode) and s.st_rdev==target:
                        found.append({'pid':int(p.name),'fd':f.name,'command':(p/'cmdline').read_bytes().replace(b'\\x00',b' ').decode(errors='replace')})
                except OSError:pass
        except OSError:pass
    owners[d]=found
files=['/home/pi/armpi_pro/start_node.sh','/etc/systemd/system/start_node.service','/etc/systemd/system/ld06.service','/etc/systemd/system/ch341-bmi270.service','/etc/systemd/system/odom.service']
container_files=['/home/ubuntu/armpi_pro/src/armpi_pro_bringup/launch/start_dependence.launch','/home/ubuntu/armpi_pro/src/armpi_pro_bringup/launch/start_sensor.launch','/home/ubuntu/armpi_pro/src/armpi_pro_bringup/launch/start_functions.launch','/home/ubuntu/armpi_pro/src/armpi_pro_bringup/scripts/start_node.sh']
print(json.dumps({'owners':owners,'files':{f:pathlib.Path(f).read_text() for f in files},'container_files':{f:run(['docker','exec','armpi_pro','sed','-n','1,220p',f]) for f in container_files},'nodes':run(['docker','exec','armpi_pro','bash','-lc','source /opt/ros/noetic/setup.bash && source /home/ubuntu/armpi_pro/devel/setup.bash && export ROS_MASTER_URI=http://raspberrypi:11311 && rosnode list'])},ensure_ascii=False))
'''

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--host',required=True);p.add_argument('--config',required=True)
    args=p.parse_args();sys.stdout.reconfigure(encoding='utf-8')
    c=connect(args.host,config_path=args.config)
    try:
        print(execute(c,'sudo -n python3 -c '+shlex.quote(HOST)),flush=True)
        run_script(c,'armpi_pro',ROS1_SETUP,READINESS)
    finally:c.close()

if __name__=='__main__':main()
