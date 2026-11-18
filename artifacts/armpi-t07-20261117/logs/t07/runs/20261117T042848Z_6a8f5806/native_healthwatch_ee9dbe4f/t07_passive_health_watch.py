"""Observe real feedback and inspect deployed read code; no control endpoints."""
import os,json,time,hashlib
from pathlib import Path
for entry in Path('/proc/1/environ').read_bytes().split(b'\0'):
 if entry.startswith((b'ROS_DOMAIN_ID=',b'ROS_LOCALHOST_ONLY=',b'RMW_IMPLEMENTATION=')):
  k,v=entry.decode().split('=',1);os.environ[k]=v
if os.environ.get('ROS_DOMAIN_ID')!='42':raise RuntimeError('Wrong domain')
import rclpy
from std_msgs.msg import String
rclpy.init();n=rclpy.create_node('t07_passive_health_watch',enable_rosout=False,start_parameter_services=False);rows=[]
def receive(k,msg):rows.append({'kind':k,'received_monotonic':time.monotonic(),'data':json.loads(msg.data)})
for k,t in [('arm','/arm_controller/status'),('base','/chassis_control/status')]:n.create_subscription(String,t,lambda m,k=k:receive(k,m),10)
until=time.monotonic()+30
while time.monotonic()<until:rclpy.spin_once(n,timeout_sec=.05)
sources={}
import armpi_ros2.servo_protocol as proto
for path in [Path(proto.__file__)]:
 b=path.read_bytes();sources[str(path)]={'sha256':hashlib.sha256(b).hexdigest(),'source':b.decode()}
summary={}
for name in ('joint1','joint2','joint3','joint4','joint5','r_joint'):
 values=[r['data']['joints'][name]['temperature'] for r in rows if r['kind']=='arm' and name in r['data'].get('joints',{})]
 summary[name]={'count':len(values),'first':values[0] if values else None,'last':values[-1] if values else None,'min':min(values) if values else None,'max':max(values) if values else None}
n.destroy_node();rclpy.shutdown();print(json.dumps({'motion_commands':0,'control_services_called':0,'temperature_summary':summary,'rows':rows,'deployed_sources':sources}))
