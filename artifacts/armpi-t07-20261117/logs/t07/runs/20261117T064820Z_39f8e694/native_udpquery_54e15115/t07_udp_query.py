"""Inspect native control interfaces without publishing or calling services."""
import os,json,time,hashlib,base64
from pathlib import Path
for entry in Path('/proc/1/environ').read_bytes().split(b'\0'):
 if entry.startswith((b'ROS_DOMAIN_ID=',b'ROS_LOCALHOST_ONLY=',b'RMW_IMPLEMENTATION=')):
  k,v=entry.decode().split('=',1);os.environ[k]=v
os.environ['FASTDDS_BUILTIN_TRANSPORTS']='UDPv4'
import rclpy
from sensor_msgs.msg import CompressedImage
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
rclpy.init();n=rclpy.create_node('t07_native_interface_reader_'+__import__('uuid').uuid4().hex[:8],enable_rosout=False,start_parameter_services=False)
image={}
def receive_image(msg):image.update(image_base64=base64.b64encode(bytes(msg.data)).decode(),image_stamp={'sec':msg.header.stamp.sec,'nanosec':msg.header.stamp.nanosec},image_frame=msg.header.frame_id)
n.create_subscription(CompressedImage,'/rear_camera/image_raw/compressed',receive_image,qos_profile_sensor_data)
ages=[]
def arm_status(msg):
 d=json.loads(msg.data);now=time.monotonic();ages.append({'receive_monotonic':now,'status_stamp':d.get('stamp'),'estop':d.get('estop'),'feedback_complete':d.get('feedback_complete'),'fault':d.get('fault'),'published_ages':d.get('feedback_age_seconds'),'joint_ages':{k:{'position':now-v.get('position_time',0),'temperature':now-v.get('temperature_time',0),'raw':v.get('raw'),'temperature_C':v.get('temperature')} for k,v in d.get('joints',{}).items()}})
n.create_subscription(String,'/arm_controller/status',arm_status,10)
end=time.monotonic()+8
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
root=Path('/opt/armpi_ros2/src/armpi_ros2/armpi_ros2');files={}
for p in root.glob('*chassis*.py'):files[p.name]={'sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'source':p.read_text()}
launch_root=root.parent/'launch';launches={p.name:p.read_text() for p in launch_root.glob('*.py')}
processes=[]
for p in Path('/proc').iterdir():
 if not p.name.isdigit():continue
 try:
  args=(p/'cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace')
  if any(k in args for k in ('/arm_controller','ros2 launch','armpi_ros2_native.launch')) and not any(k in args.lower() for k in ('password','secret','token','proxy')):processes.append({'pid':int(p.name),'argv':args[:2000]})
 except (PermissionError,FileNotFoundError,ProcessLookupError):pass
arm_file=root/'arm_node.py'
out={'services':n.get_service_names_and_types(),'nodes':n.get_node_names_and_namespaces(),'chassis_sources':files,'source_filenames':[p.name for p in root.glob('*.py')],'motion_commands_sent':0,'control_services_called':0,'live_freshness_samples':ages,'launch_files':launches,'control_processes':processes,'arm_source':{'sha256':hashlib.sha256(arm_file.read_bytes()).hexdigest(),'source':arm_file.read_text()},**image}
out['topic_publishers']={topic:[{'name':v.node_name,'namespace':v.node_namespace,'type':v.topic_type} for v in n.get_publishers_info_by_topic(topic)] for topic in ['/arm_controller/status','/rear_camera/image_raw','/rear_camera/image_raw/compressed','/tf']}
n.destroy_node();rclpy.shutdown();print(json.dumps(out))
