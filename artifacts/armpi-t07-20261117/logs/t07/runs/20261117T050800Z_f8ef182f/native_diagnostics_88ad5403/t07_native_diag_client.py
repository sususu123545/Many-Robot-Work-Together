"""Latch stop, verify effective startup gates OFF, request read-only diagnostics."""
import os,time,json
from pathlib import Path
for item in Path('/proc/1/environ').read_bytes().split(b'\0'):
 if item.startswith((b'ROS_DOMAIN_ID=',b'ROS_LOCALHOST_ONLY=',b'RMW_IMPLEMENTATION=')):
  k,v=item.decode().split('=',1);os.environ[k]=v
if os.environ.get('ROS_DOMAIN_ID')!='42':raise RuntimeError('Wrong endpoint')
import rclpy
from std_msgs.msg import Bool,String
from std_srvs.srv import Trigger
rclpy.init();n=rclpy.create_node('t07_readonly_diagnostic_client',enable_rosout=False,start_parameter_services=False);states={}
for key,topic in [('arm','/arm_controller/status'),('base','/chassis_control/status')]:n.create_subscription(String,topic,lambda m,k=key:states.update({k:json.loads(m.data)}),1)
stop=n.create_publisher(Bool,'/safety/estop',10);client=n.create_client(Trigger,'/arm_controller/read_gripper_diagnostics');end=time.monotonic()+20;ready=False;last=0
while time.monotonic()<end:
 rclpy.spin_once(n,timeout_sec=.05)
 if time.monotonic()-last>.3:stop.publish(Bool(data=True));last=time.monotonic()
 a=states.get('arm',{});b=states.get('base',{})
 ready=(a.get('estop') is True and a.get('enable_arm_motion') is False and a.get('enable_gripper_motion') is False and b.get('estop') is True and b.get('enable_motion') is False and b.get('wheel_command')==[0,0,0,0])
 if ready and client.service_is_ready():break
out={'gates_verified_off':ready,'motion_commands_sent':0,'torque_writes':0,'diagnostic_response':None}
if ready and client.service_is_ready():
 f=client.call_async(Trigger.Request());end=time.monotonic()+12
 while not f.done() and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
 if f.done() and f.exception() is None:
  v=f.result();out['diagnostic_response']={'success':v.success,'message':v.message}
 else:out['error']='diagnostic response timeout or exception'
else:out['error']='startup gates or diagnostic service not ready; no query sent'
out['arm']={k:states.get('arm',{}).get(k) for k in ('estop','enable_arm_motion','enable_gripper_motion')};out['base']=states.get('base');n.destroy_node();rclpy.shutdown();print(json.dumps(out))
