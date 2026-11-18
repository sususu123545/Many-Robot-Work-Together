"""Read actual ROS feedback and installed conversion source, without any writes."""
import os,time,json,hashlib,uuid,base64
from pathlib import Path
for item in Path('/proc/1/environ').read_bytes().split(b'\0'):
 if item.startswith((b'ROS_DOMAIN_ID=',b'ROS_LOCALHOST_ONLY=',b'RMW_IMPLEMENTATION=')):
  k,v=item.decode().split('=',1);os.environ[k]=v
if os.environ.get('ROS_DOMAIN_ID')!='42':raise RuntimeError('Wrong domain')
os.environ['FASTDDS_BUILTIN_TRANSPORTS']='UDPv4'
import rclpy
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
import armpi_ros2.servo_protocol as protocol
import armpi_ros2.arm_node as arm
rclpy.init();n=rclpy.create_node('t07_mapping_read_'+uuid.uuid4().hex[:8],enable_rosout=False,start_parameter_services=False)
samples=[];latest={};image={}
def receive(k,m):
 d=json.loads(m.data);latest[k]=d;samples.append({'kind':k,'receive_monotonic':time.monotonic(),'data':d})
for k,t in [('arm','/arm_controller/status'),('base','/chassis_control/status')]:n.create_subscription(String,t,lambda m,k=k:receive(k,m),10)
def capture(m):image.update(jpeg_base64=base64.b64encode(bytes(m.data)).decode(),stamp=[m.header.stamp.sec,m.header.stamp.nanosec])
n.create_subscription(CompressedImage,'/rear_camera/image_raw/compressed',capture,10)
end=time.monotonic()+8
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
sources={}
for module in [protocol,arm]:
 p=Path(module.__file__);sources[module.__name__]={'path':str(p),'sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'source':p.read_text()}
out={'motion_commands':0,'control_writes':0,'direct_serial_opens':0,'samples':samples,'latest':latest,'sources':sources,'image':image,'publishers':{t:n.count_publishers(t) for t in ['/arm_controller/command','/gripper_controller/command','/cmd_vel']}}
n.destroy_node();rclpy.shutdown();print(json.dumps(out))
