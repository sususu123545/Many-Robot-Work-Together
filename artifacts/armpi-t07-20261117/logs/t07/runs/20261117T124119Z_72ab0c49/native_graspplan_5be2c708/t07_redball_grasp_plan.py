"""Read-only grasp geometry preflight. Produces candidates, never commands."""
import hashlib,json,os,signal,subprocess,sys,time,uuid
from pathlib import Path
for entry in Path('/proc/1/environ').read_bytes().split(b'\0'):
 if entry.startswith((b'ROS_DOMAIN_ID=',b'ROS_LOCALHOST_ONLY=',b'RMW_IMPLEMENTATION=')):
  k,v=entry.decode().split('=',1);os.environ[k]=v
if os.environ.get('ROS_DOMAIN_ID')!='42':raise RuntimeError('Wrong domain')
import numpy as np
import rclpy
from rclpy.time import Time
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
import tf2_ros
from armpi_ros2.arm_kinematics import forward_tool_pose_raw,inverse_kinematics_all
def complete_sphere(d,width=640,height=480,margin=4):
 if not d:return False
 x,y,w,h=d['bbox'];cx=d['center_x'];cy=d['center_y'];r=d['radius_px']
 return x>=margin and y>=margin and x+w<=width-margin and y+h<=height-margin and cx-r>=margin and cy-r>=margin and cx+r<=width-margin and cy+r<=height-margin
rclpy.init();node=rclpy.create_node('t07_grasp_geometry_reader',enable_rosout=False,start_parameter_services=False)
buf=tf2_ros.Buffer();listener=tf2_ros.TransformListener(buf,node)
prefix='/t07_geometry_'+uuid.uuid4().hex[:8];latest={};points=[];states=[];detections=[]
def status(k,m):
 d=json.loads(m.data);latest[k]=d
 if k=='arm':states.append({'receive_monotonic':time.monotonic(),'data':d})
for k,topic in [('arm','/arm_controller/status'),('base','/chassis_control/status')]:node.create_subscription(String,topic,lambda m,k=k:status(k,m),10)
node.create_subscription(String,prefix+'/detection',lambda m:detections.append(json.loads(m.data)),10)
node.create_subscription(PointStamped,prefix+'/point',lambda m:points.append(m),10)
detector_boot="import cv2;cv2.setNumThreads(1);import armpi_ros2.red_ball_detector as m;original=m.find_red_sphere;m.find_red_sphere=lambda image,*args,**kwargs:original(cv2.GaussianBlur(image,(5,5),0),*args,**kwargs);m.main()"
argv=[sys.executable,'-c',detector_boot,'--ros-args','-r','__node:=t07_geometry_detector','-p','ball_diameter_m:=0.038','-p','stability_center_px:=3.5','-p','stability_radius_fraction:=0.08','-p','annotated_topic:='+prefix+'/annotated','-r','/grasp/target_camera:='+prefix+'/point','-r','/grasp/red_ball/detection:='+prefix+'/detection']
out={'motion_commands':0,'control_services_called':0,'candidate_only':True,'motion_path_accepted':False,'detector_argv':argv};proc=None
def rotate(q,p):
 x,y,z,w=q;x2,y2,z2=x*x,y*y,z*z
 return np.array([[1-2*(y2+z2),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x2+z2),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x2+y2)]])@np.asarray(p)
try:
 proc=subprocess.Popen(argv,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,start_new_session=True)
 until=time.monotonic()+12
 while time.monotonic()<until:
  rclpy.spin_once(node,timeout_sec=.05)
  if len(points)>=10 and len(detections)>=5 and all(d.get('valid') is True and complete_sphere(d.get('detection')) for d in detections[-5:]):break
 raw_points=len(points);eligible={(d['stamp_sec'],d['stamp_nanosec']) for d in detections if d.get('valid') is True and complete_sphere(d.get('detection'))}
 points=[p for p in points if (p.header.stamp.sec,p.header.stamp.nanosec) in eligible]
 out.update(arm=latest.get('arm'),base=latest.get('base'),frames=buf.all_frames_as_yaml(),detection_samples=detections,arm_samples=states,detection_count=len(detections),raw_camera_point_count=raw_points,camera_point_count=len(points),partial_sphere_rejections=sum(d.get('detection') is not None and not complete_sphere(d['detection']) for d in detections),competing_command_publishers={t:node.count_publishers(t) for t in ['/arm_controller/command','/gripper_controller/command','/cmd_vel']})
 if not points or 'arm' not in latest:raise RuntimeError('No stable camera target or arm feedback')
 raw={n:v['raw'] for n,v in latest['arm']['joints'].items()};pose=forward_tool_pose_raw(raw);out['current_raw']=raw;out['current_tool_pose']=pose
 calib_path=Path('/opt/armpi-calibration/rear_camera_hand_eye.json')
 if not calib_path.exists():raise RuntimeError('Mounted hand-eye calibration absent')
 raw_cal=calib_path.read_bytes();cal=json.loads(raw_cal);out['hand_eye']={'path':str(calib_path),'sha256':hashlib.sha256(raw_cal).hexdigest(),'data':cal}
 p=points[-1];camera=np.array([p.point.x,p.point.y,p.point.z]);out['latest_camera_point']={'xyz':camera.tolist(),'frame':p.header.frame_id,'stamp':{'sec':p.header.stamp.sec,'nanosec':p.header.stamp.nanosec}}
 out['target_complete_sphere_verified']=True
 out['source_robot_ros_ns']=node.get_clock().now().nanoseconds
 out['target_age_seconds']=(out['source_robot_ros_ns']-(p.header.stamp.sec*10**9+p.header.stamp.nanosec))*1e-9
 c=cal['tool_T_camera'];offline=np.array(pose['translation_m'])+np.array(pose['rotation_matrix'])@(rotate(c['quaternion_xyzw'],camera)+np.array(c['translation_m']))
 out['offline_handeye_candidate_base_xyz']=offline.tolist()
 try:
  tf=buf.lookup_transform('arm_base',p.header.frame_id,Time.from_msg(p.header.stamp));t=tf.transform.translation;q=tf.transform.rotation
  live=np.array([t.x,t.y,t.z])+rotate([q.x,q.y,q.z,q.w],camera)
  out['live_tf_candidate_base_xyz']=live.tolist();out['live_offline_distance_m']=float(np.linalg.norm(live-offline))
 except Exception as error:out['live_tf_error']=str(error)
 out['ik_candidates']=[]
 for pitch in (-180,-170,-160,-150,-140,-130,-120,-110,-100,-90):
  for dz,phase in ((.05,'pregrasp'),(0,'sphere_center'),(.02,'lift')):
   try:
    solutions=inverse_kinematics_all(*[float(offline[0]),float(offline[1]),float(offline[2]+dz)],pitch)
    out['ik_candidates'].append({'pitch':pitch,'phase':phase,'solutions':solutions})
   except ValueError:pass
 out['pending_before_motion']=['Validate live/hand-eye transform agreement','Verify IK tool point versus jaw grasp center','Check floor and full-link swept clearance','Select bounded same-branch approach and descent','Verify base lock, fresh feedback and no competing controllers']
except Exception as error:out['error']=repr(error)
finally:
 if proc is not None and proc.poll() is None:
  os.killpg(proc.pid,signal.SIGINT)
  try:proc.wait(timeout=4)
  except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=3)
 node.destroy_node();rclpy.shutdown()
print(json.dumps(out))
