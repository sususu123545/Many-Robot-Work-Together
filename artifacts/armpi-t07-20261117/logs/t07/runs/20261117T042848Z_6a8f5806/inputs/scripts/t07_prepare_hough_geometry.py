from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
base=(ROOT/'scripts/t07_redball_grasp_plan.py').read_text(encoding='utf-8')
start=base.index("prefix='/t07_geometry_")
end=base.index("out={'motion_commands'",start)
prefix=base[:start]+"\nimport cv2\ncv2.setNumThreads(1)\nfrom sensor_msgs.msg import Image,CameraInfo\nfrom rclpy.qos import qos_profile_sensor_data\nfrom armpi_ros2.red_ball_detector import image_msg_to_bgr,sphere_center_in_camera\n"
prefix+=(ROOT/'scripts/t07_hough_ball.py').read_text(encoding='utf-8')
prefix+='''
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
node.create_subscription(Image,'/rear_camera/image_raw',frame,qos_profile_sensor_data)
argv=['in-process passive hough detector; no child or motion publisher']
'''
tail=base[end:]
tail=tail.replace(" proc=subprocess.Popen(argv,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,start_new_session=True)"," # Passive subscriptions only; no motion process")
tail=tail.replace(" p=points[-1];camera=", " vision_paused[0]=True\n available=[p for p in points if buf.can_transform('arm_base',p.header.frame_id,Time.from_msg(p.header.stamp)) and 0<=node.get_clock().now().nanoseconds-(p.header.stamp.sec*10**9+p.header.stamp.nanosec)<=500000000]\n p=available[-1] if available else points[-1]\n tf_deadline=time.monotonic()+.4\n while not buf.can_transform('arm_base',p.header.frame_id,Time.from_msg(p.header.stamp)) and time.monotonic()<tf_deadline:rclpy.spin_once(node,timeout_sec=.02)\n camera=")
text=prefix+tail
compile(text,'hough_geometry','exec')
(ROOT/'scripts/t07_redball_hough_geometry.py').write_text(text,encoding='utf-8')
