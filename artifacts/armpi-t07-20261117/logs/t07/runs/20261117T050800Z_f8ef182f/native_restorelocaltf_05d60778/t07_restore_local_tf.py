"""Restore a passive camera TF from fresh measured feedback, also while stopped."""
import os,json,time,hashlib,subprocess,sys,uuid
from pathlib import Path
for entry in Path('/proc/1/environ').read_bytes().split(b'\0'):
 if entry.startswith((b'ROS_DOMAIN_ID=',b'ROS_LOCALHOST_ONLY=',b'RMW_IMPLEMENTATION=')):
  k,v=entry.decode().split('=',1);os.environ[k]=v
if os.environ.get('ROS_DOMAIN_ID')!='42':raise RuntimeError('Wrong physical domain')
import rclpy
from std_msgs.msg import String
rclpy.init();node=rclpy.create_node('t07_restore_passive_camera_tf',enable_rosout=False,start_parameter_services=False)
until=time.monotonic()+2
while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.05)
if any(name=='t07_local_measured_camera_transform' for name,_ in node.get_node_names_and_namespaces()):raise RuntimeError('Existing camera transform node: inspect before duplicate launch')
path=Path('/opt/armpi_ros2/install/armpi_ros2/lib/python3.12/site-packages/armpi_ros2/arm_camera_tf.py');raw=path.read_bytes()
if hashlib.sha256(raw).hexdigest()!='951b7796d24d9f9d7b4de9c7837ac2adf39e9768fb712be755f0558ff3622bd9':raise RuntimeError('Unreviewed camera TF source')
source=raw.decode();old="status.get('estop') or status.get('health_state') != 'READY'";new="status.get('health_state') not in ('READY','RECOVERED_AWAIT_RESET','RECOVERING')"
if source.count(old)!=1:raise RuntimeError('Unexpected passive validity condition')
source=source.replace(old,new)
source=source.replace("ages = status.get('feedback_age_seconds') or {}", "ages = {name:time.monotonic()-float(row['position_time']) for name,row in joints.items()}")
source=source.replace('node = ArmCameraTransform()', "node = ArmCameraTransform();node.camera_frame='t07_rear_camera_measured'")
compile(source,'measured_camera_tf','exec')
folder=Path('/tmp')/('t07_passive_tf_'+uuid.uuid4().hex[:8]);folder.mkdir();script=folder/'measured_camera_tf.py';script.write_text(source)
argv=[sys.executable,str(script),'--ros-args','-r','__node:=t07_local_measured_camera_transform','-p','calibration_file:=/opt/armpi-calibration/rear_camera_hand_eye.json','-p','max_feedback_age_sec:=0.5']
states=[];node.create_subscription(String,'/grasp/camera_tf/status',lambda m:states.append(json.loads(m.data)),10)
with (folder/'stdout.log').open('wb') as out,(folder/'stderr.log').open('wb') as err:p=subprocess.Popen(argv,stdout=out,stderr=err,start_new_session=True)
until=time.monotonic()+10
while time.monotonic()<until and not any(s.get('valid') for s in states):rclpy.spin_once(node,timeout_sec=.05)
print(json.dumps({'motion_commands':0,'control_services_called':0,'pid':p.pid,'alive':p.poll() is None,'argv':argv,'source':source,'source_sha256':hashlib.sha256(source.encode()).hexdigest(),'status_samples':states,'verified':p.poll() is None and any(s.get('valid') for s in states),'scope':'Isolated t07_rear_camera_measured is the same physical camera optical frame with the same accepted hand-eye extrinsic; only fresh real encoder positions on robot clock drive TF; native motion protections unchanged','log_directory':str(folder),'stderr':(folder/'stderr.log').read_text()}))
node.destroy_node();rclpy.shutdown()
