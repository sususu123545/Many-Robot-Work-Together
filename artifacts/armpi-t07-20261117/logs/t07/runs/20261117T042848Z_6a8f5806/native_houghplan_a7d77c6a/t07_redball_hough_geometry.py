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

import cv2
cv2.setNumThreads(1)
from sensor_msgs.msg import Image,CameraInfo
from rclpy.qos import qos_profile_sensor_data
from armpi_ros2.red_ball_detector import image_msg_to_bgr,sphere_center_in_camera
"""Circle geometry with red-interior and edge support; pure perception only."""
import cv2
import numpy as np

def refine_red_boundary(red,cx,cy,radius):
    # The lower arc can merge with the floor reflection. Fit the other 270
    # degrees using observed red-to-background transitions, then reject outliers.
    points=[];height,width=red.shape
    for angle in np.deg2rad(np.linspace(135,405,91)):
        previous=True
        for distance in np.arange(radius*.60,radius*1.45,.5):
            x=round(cx+distance*np.cos(angle));y=round(cy+distance*np.sin(angle))
            if not 0<=x<width or not 0<=y<height:break
            inside=red[y,x]>0
            if previous and not inside:
                points.append([cx+(distance-.25)*np.cos(angle),cy+(distance-.25)*np.sin(angle)]);break
            previous=inside
    if len(points)<50:return None
    points=np.asarray(points)
    for _ in range(4):
        a=np.column_stack((2*points[:,0],2*points[:,1],np.ones(len(points))))
        x,y,c=np.linalg.lstsq(a,np.sum(points**2,axis=1),rcond=None)[0]
        rr=np.sqrt(max(0,c+x*x+y*y));residual=np.abs(np.linalg.norm(points-[x,y],axis=1)-rr)
        keep=residual<=1.5
        if np.sum(keep)<45:return None
        points=points[keep]
    if not .7*radius<=rr<=1.3*radius or np.hypot(x-cx,y-cy)>radius*.25:return None
    return float(x),float(y),float(rr)

def find_hough_ball(image_bgr):
    height,width=image_bgr.shape[:2]
    gray=cv2.GaussianBlur(cv2.cvtColor(image_bgr,cv2.COLOR_BGR2GRAY),(5,5),0)
    hsv=cv2.cvtColor(cv2.GaussianBlur(image_bgr,(3,3),0),cv2.COLOR_BGR2HSV)
    red=cv2.inRange(hsv,np.array([0,75,45],np.uint8),np.array([12,255,255],np.uint8)) | cv2.inRange(hsv,np.array([168,75,45],np.uint8),np.array([180,255,255],np.uint8))
    count,labels,stats,_=cv2.connectedComponentsWithStats(red)
    boxes=[s for s in stats[1:] if s[cv2.CC_STAT_AREA]>=80]
    if not boxes:return None,0
    x0=max(0,min(s[0] for s in boxes)-20);y0=max(0,min(s[1] for s in boxes)-20)
    x1=min(width,max(s[0]+s[2] for s in boxes)+20);y1=min(height,max(s[1]+s[3] for s in boxes)+20)
    circles=cv2.HoughCircles(gray[y0:y1,x0:x1],cv2.HOUGH_GRADIENT,dp=1,minDist=35,param1=100,param2=28,minRadius=12,maxRadius=180)
    if circles is None:return None,0
    circles[0,:,0]+=x0;circles[0,:,1]+=y0
    edges=cv2.Canny(gray,45,100);candidates=[]
    yy,xx=np.ogrid[:height,:width]
    for cx,cy,radius in circles[0]:
        cx,cy,radius=map(float,(cx,cy,radius))
        refined=refine_red_boundary(red,cx,cy,radius)
        if refined is None:continue
        cx,cy,radius=refined
        if cx-radius<4 or cy-radius<4 or cx+radius>width-4 or cy+radius>height-4:continue
        disk=(xx-cx)**2+(yy-cy)**2<=(radius*.72)**2
        fraction=float(np.mean(red[disk]>0))
        if fraction<.70:continue
        supported=0
        for angle in np.linspace(0,2*np.pi,72,endpoint=False):
            x=round(cx+radius*np.cos(angle));y=round(cy+radius*np.sin(angle))
            supported+=bool(np.any(edges[max(0,y-3):min(height,y+4),max(0,x-3):min(width,x+4)]))
        edge=supported/72
        if edge<.55:continue
        candidates.append({'center_x':cx,'center_y':cy,'radius_px':radius,'bbox':[round(cx-radius),round(cy-radius),round(2*radius),round(2*radius)],'red_fraction':fraction,'edge_support':edge,'confidence':fraction*edge,'method':'hough_gradient_red_core_edge_support'})
    candidates.sort(key=lambda c:c['confidence'],reverse=True)
    if not candidates:return None,0
    if len(candidates)>1 and candidates[1]['confidence']>.85*candidates[0]['confidence']:return None,len(candidates)
    return candidates[0],len(candidates)

from rclpy.qos import QoSProfile,ReliabilityPolicy,HistoryPolicy
camera_qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT,history=HistoryPolicy.KEEP_LAST)
latest={};points=[];states=[];detections=[];history=[];intrinsic={};vision_paused=[False]
def status(k,m):
 d=json.loads(m.data);latest[k]=d
 if k=='arm':states.append({'receive_monotonic':time.monotonic(),'data':d})
for k,topic in [('arm','/arm_controller/status'),('base','/chassis_control/status')]:node.create_subscription(String,topic,lambda m,k=k:status(k,m),10)
node.create_subscription(CameraInfo,'/rear_camera/camera_info',lambda m:intrinsic.update(message=m),qos_profile_sensor_data)
def frame(msg):
 if vision_paused[0]:return
 image=image_msg_to_bgr(msg);d,count=find_hough_ball(image)
 row={'stamp_sec':msg.header.stamp.sec,'stamp_nanosec':msg.header.stamp.nanosec,'frame_id':msg.header.frame_id,'candidate_count':count,'detection':d,'valid':False,'reason':'no_unique_supported_circle'}
 if d and complete_sphere(d):
  history.append(d);history[:]=history[-5:]
  if len(history)==5:
   xs=[p['center_x'] for p in history];ys=[p['center_y'] for p in history];rs=[p['radius_px'] for p in history]
   stable=max(xs)-min(xs)<=3.5 and max(ys)-min(ys)<=3.5 and (max(rs)-min(rs))/np.mean(rs)<=.08
   row['reason']='target_not_stable'
   if stable and 'message' in intrinsic:
    xyz,_=sphere_center_in_camera((float(np.median(xs)),float(np.median(ys))),float(np.median(rs)),intrinsic['message'],.038)
    p=PointStamped();p.header=msg.header;p.point.x,p.point.y,p.point.z=xyz;points.append(p);row.update(valid=True,reason='stable_and_localized',point_camera_m=list(xyz))
 else:history.clear()
 detections.append(row)
node.create_subscription(Image,'/rear_camera/image_raw',frame,camera_qos)
argv=['in-process passive hough detector; no child or motion publisher']
out={'motion_commands':0,'control_services_called':0,'candidate_only':True,'motion_path_accepted':False,'detector_argv':argv};proc=None
def rotate(q,p):
 x,y,z,w=q;x2,y2,z2=x*x,y*y,z*z
 return np.array([[1-2*(y2+z2),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x2+z2),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x2+y2)]])@np.asarray(p)
try:
 # Passive subscriptions only; no motion process
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
 vision_paused[0]=True
 available=[p for p in points if buf.can_transform('arm_base',p.header.frame_id,Time.from_msg(p.header.stamp)) and 0<=node.get_clock().now().nanoseconds-(p.header.stamp.sec*10**9+p.header.stamp.nanosec)<=500000000]
 p=available[-1] if available else points[-1]
 tf_deadline=time.monotonic()+.4
 while not buf.can_transform('arm_base',p.header.frame_id,Time.from_msg(p.header.stamp)) and time.monotonic()<tf_deadline:rclpy.spin_once(node,timeout_sec=.02)
 camera=np.array([p.point.x,p.point.y,p.point.z]);out['latest_camera_point']={'xyz':camera.tolist(),'frame':p.header.frame_id,'stamp':{'sec':p.header.stamp.sec,'nanosec':p.header.stamp.nanosec}}
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
