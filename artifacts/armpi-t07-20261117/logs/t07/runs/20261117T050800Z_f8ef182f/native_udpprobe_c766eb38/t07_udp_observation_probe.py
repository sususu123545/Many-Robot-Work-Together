"""Compare late-joiner observation using UDP transport; no publishers/services."""
import os,json,time
from pathlib import Path
for entry in Path('/proc/1/environ').read_bytes().split(b'\0'):
 if entry.startswith((b'ROS_DOMAIN_ID=',b'ROS_LOCALHOST_ONLY=',b'RMW_IMPLEMENTATION=')):
  k,v=entry.decode().split('=',1);os.environ[k]=v
if os.environ.get('ROS_DOMAIN_ID')!='42':raise RuntimeError('Wrong domain')
os.environ['FASTDDS_BUILTIN_TRANSPORTS']='UDPv4'
import rclpy
from rclpy.utilities import get_rmw_implementation_identifier
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from rclpy.qos import qos_profile_sensor_data
rclpy.init();node=rclpy.create_node('t07_udp_readonly_probe',enable_rosout=False,start_parameter_services=False);counts={};latest={};arrival={}
def receive(k,m):
 counts[k]=counts.get(k,0)+1;arrival[k]=time.monotonic()
 if k!='image':latest[k]=json.loads(m.data)
for k,t in [('arm','/arm_controller/status'),('base','/chassis_control/status')]:node.create_subscription(String,t,lambda m,k=k:receive(k,m),10)
node.create_subscription(CompressedImage,'/rear_camera/image_raw/compressed',lambda m:receive('image',m),qos_profile_sensor_data)
until=time.monotonic()+10
while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.05)
print(json.dumps({'motion_commands':0,'control_services_called':0,'transport_override':'UDPv4','rmw':get_rmw_implementation_identifier(),'counts':counts,'latest':latest,'last_receive_monotonic':arrival,'node_names':node.get_node_names_and_namespaces()}));node.destroy_node();rclpy.shutdown()
