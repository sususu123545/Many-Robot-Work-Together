"""Latch the existing shared software estop; no reset or movement commands."""
import os,time,json
from pathlib import Path
for item in Path('/proc/1/environ').read_bytes().split(b'\0'):
 if item.startswith((b'ROS_DOMAIN_ID=',b'ROS_LOCALHOST_ONLY=',b'RMW_IMPLEMENTATION=')):
  k,v=item.decode().split('=',1);os.environ[k]=v
if os.environ.get('ROS_DOMAIN_ID')!='42':raise RuntimeError('Wrong physical endpoint domain')
import rclpy
from std_msgs.msg import String,Bool
rclpy.init();n=rclpy.create_node('t07_latch_base_stop',enable_rosout=False,start_parameter_services=False);states={};received={}
def state(kind,msg):states[kind]=json.loads(msg.data);received[kind]=time.monotonic()
n.create_subscription(String,'/chassis_control/status',lambda m:state('chassis',m),10);n.create_subscription(String,'/arm_controller/status',lambda m:state('arm',m),10);pub=n.create_publisher(Bool,'/safety/estop',10)
deadline=time.monotonic()+5;last=0;passed=False
while time.monotonic()<deadline:
 rclpy.spin_once(n,timeout_sec=.05)
 if time.monotonic()-last>.25:pub.publish(Bool(data=True));last=time.monotonic()
 c=states.get('chassis',{});a=states.get('arm',{})
 if c.get('estop') is True and c.get('wheel_command')==[0,0,0,0] and a.get('estop') is True and a.get('enable_arm_motion') is False and a.get('enable_gripper_motion') is False and all(time.monotonic()-received.get(k,0)<1.5 for k in ('arm','chassis')):passed=True;break
out={'passed':passed,'chassis':states.get('chassis'),'arm_stop':{k:states.get('arm',{}).get(k) for k in ('estop','enable_arm_motion','enable_gripper_motion')},'movement_commands_sent':0,'estop_true_published':True,'reset_called':False,'note':'Chassis enable_motion is a startup flag; runtime parameter change does not update driver attribute. Actual lock is verified latched chassis estop; do not reset chassis.'}
n.destroy_node();rclpy.shutdown();print(json.dumps(out));raise SystemExit(0 if passed else 1)
