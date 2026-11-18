"""Single explicitly authorized servo-1 restore request; no trajectory publisher."""
import os,time,json,base64
from pathlib import Path
for item in Path('/proc/1/environ').read_bytes().split(b'\0'):
 if item.startswith((b'ROS_DOMAIN_ID=',b'ROS_LOCALHOST_ONLY=',b'RMW_IMPLEMENTATION=')):
  k,v=item.decode().split('=',1);os.environ[k]=v
if os.environ.get('ROS_DOMAIN_ID')!='42':raise RuntimeError('Wrong endpoint')
import rclpy
from std_msgs.msg import Bool,String
from std_srvs.srv import Trigger
rclpy.init();n=rclpy.create_node('t07_gripper_restore_once_client',enable_rosout=False,start_parameter_services=False);states={}
for key,topic in [('arm','/arm_controller/status'),('base','/chassis_control/status')]:n.create_subscription(String,topic,lambda m,k=key:states.update({k:json.loads(m.data)}),1)
stop=n.create_publisher(Bool,'/safety/estop',10);c=n.create_client(Trigger,'/arm_controller/restore_gripper_torque_once');end=time.monotonic()+15;ready=False
while time.monotonic()<end:
 rclpy.spin_once(n,timeout_sec=.05);stop.publish(Bool(data=True));a=states.get('arm',{});b=states.get('base',{})
 ready=a.get('estop') is True and a.get('enable_arm_motion') is False and a.get('enable_gripper_motion') is False and b.get('enable_motion') is False and b.get('estop') is True and b.get('wheel_command')==[0,0,0,0]
 if ready and c.service_is_ready():break
out={'ready':ready,'service_called':False,'trajectory_publishers':0}
if ready and c.service_is_ready():
 f=c.call_async(Trigger.Request());out['service_called']=True;end=time.monotonic()+12
 while not f.done() and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
 if f.done() and f.exception() is None:
  v=f.result();out.update(success=v.success,message=v.message)
 else:out.update(success=False,error='Response timeout; do not repeat restore automatically')
else:out.update(success=False,error='Stopped gates or service unavailable')
n.destroy_node();rclpy.shutdown();print(json.dumps(out))
