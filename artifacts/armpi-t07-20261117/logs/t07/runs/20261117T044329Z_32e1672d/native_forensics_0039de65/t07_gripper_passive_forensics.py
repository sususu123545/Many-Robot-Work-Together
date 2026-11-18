"""Passive ROS/status/source/fd audit; no publishers or direct serial owner."""
import os,time,json,hashlib
from pathlib import Path
for item in Path('/proc/1/environ').read_bytes().split(b'\0'):
 if item.startswith((b'ROS_DOMAIN_ID=',b'ROS_LOCALHOST_ONLY=',b'RMW_IMPLEMENTATION=')):
  k,v=item.decode().split('=',1);os.environ[k]=v
if os.environ.get('ROS_DOMAIN_ID')!='42':raise RuntimeError('Wrong domain')
import rclpy
from std_msgs.msg import String
from std_srvs.srv import Trigger
rclpy.init();node=rclpy.create_node('t07_passive_gripper_forensics',enable_rosout=False,start_parameter_services=False)
samples=[];latest={}
def receive(key,message):
 data=json.loads(message.data);latest[key]=data;samples.append({'kind':key,'receive_monotonic':time.monotonic(),'data':data})
for key,topic in [('arm','/arm_controller/status'),('base','/chassis_control/status')]:node.create_subscription(String,topic,lambda m,k=key:receive(k,m),1)
end=time.monotonic()+15
while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.1)
a=latest.get('arm',{});b=latest.get('base',{})
ready=a.get('estop') is True and a.get('enable_arm_motion') is False and a.get('enable_gripper_motion') is False and b.get('estop') is True and b.get('enable_motion') is False and b.get('wheel_command')==[0,0,0,0]
diagnostic=None
client=node.create_client(Trigger,'/arm_controller/read_gripper_diagnostics')
if ready and client.wait_for_service(timeout_sec=1):
 future=client.call_async(Trigger.Request());end=time.monotonic()+12
 while not future.done() and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
 if future.done() and future.exception() is None:
  result=future.result();diagnostic={'success':result.success,'message':result.message}
files={}
base=Path('/opt/armpi_ros2/install/armpi_ros2/lib/python3.12/site-packages/armpi_ros2')
for name in ['servo_protocol.py','arm_node.py','arm_safety.py']:
 path=base/name
 if path.is_file():files[str(path)]={'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'source':path.read_text()}
owners=[]
device=Path('/dev/rrc').resolve()
for proc in Path('/proc').iterdir():
 if not proc.name.isdigit():continue
 try:
  matched=[]
  for fd in (proc/'fd').iterdir():
   try:
    if fd.resolve()==device:matched.append(fd.name)
   except OSError:pass
  if matched:
   raw=(proc/'cmdline').read_bytes().split(b'\0')
   owners.append({'pid':int(proc.name),'fds':matched,'executable':raw[0].decode(errors='replace'),'entrypoint':raw[1].decode(errors='replace') if len(raw)>1 else None})
 except (OSError,ProcessLookupError):pass
print(json.dumps({'samples':samples,'gates_verified_off':ready,'diagnostic_response':diagnostic,'files':files,'serial_device':str(device),'serial_owners':owners,'command_publishers':{topic:node.count_publishers(topic) for topic in ['/arm_controller/command','/gripper_controller/command','/safety/estop','/arm_controller/stop']},'motion_commands':0,'torque_writes':0,'direct_serial_opens':0}))
node.destroy_node();rclpy.shutdown()
