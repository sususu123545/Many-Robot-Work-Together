"""One bounded 5mm forward/up arm probe; no gripper or chassis movement."""
import json,math,os,signal,time
from pathlib import Path
for entry in Path('/proc/1/environ').read_bytes().split(b'\0'):
 if entry.startswith((b'ROS_DOMAIN_ID=',b'ROS_LOCALHOST_ONLY=',b'RMW_IMPLEMENTATION=')):
  k,v=entry.decode().split('=',1);os.environ[k]=v
if os.environ.get('ROS_DOMAIN_ID')!='42':raise RuntimeError('Wrong domain')
os.environ['FASTDDS_BUILTIN_TRANSPORTS']='UDPv4'
import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import String,Bool,Empty
from std_srvs.srv import Trigger
from rcl_interfaces.srv import SetParametersAtomically
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from armpi_ros2.arm_kinematics import forward_tool_pose_raw,inverse_kinematics_all
rclpy.init();n=rclpy.create_node('t07_upward_reset_preparation',enable_rosout=False,start_parameter_services=False)
latest={};received={};events=[];report={'movement_commands':0,'gripper_commands':0,'chassis_commands':0,'passed':False,'scope':'bounded pregrasp stage; not grasp'};changed=False
def cb(k,m):
 d=json.loads(m.data);latest[k]=d;received[k]=time.monotonic();events.append({'kind':k,'receive_monotonic':received[k],'receive_ros_ns':n.get_clock().now().nanoseconds,'data':d})
for k,t in [('arm','/arm_controller/status'),('base','/chassis_control/status'),('ack','/arm_controller/command_result')]:n.create_subscription(String,t,lambda m,k=k:cb(k,m),10)
estop=n.create_publisher(Bool,'/safety/estop',10);stop=n.create_publisher(Empty,'/arm_controller/stop',10);pub=n.create_publisher(JointTrajectory,'/arm_controller/command',10)
params=n.create_client(SetParametersAtomically,'/arm_controller/set_parameters_atomically');reset=n.create_client(Trigger,'/arm_controller/reset_estop')
def spin_until(pred,seconds):
 until=time.monotonic()+seconds
 while time.monotonic()<until:
  rclpy.spin_once(n,timeout_sec=.03)
  if pred():return
 raise TimeoutError('Precondition/response timeout')
def result(f):
 spin_until(f.done,4)
 if f.exception():raise f.exception()
 return f.result()
def gates(arm):
 req=SetParametersAtomically.Request();req.parameters=[Parameter('enable_arm_motion',value=arm).to_parameter_msg(),Parameter('enable_gripper_motion',value=False).to_parameter_msg()]
 ans=result(params.call_async(req)).result
 if not ans.successful:raise RuntimeError('Gates failed: '+ans.reason)
def guard(base_locked=False,require_gripper=False):
 now=time.monotonic();a=latest.get('arm',{});b=latest.get('base',{})
 if any(now-received.get(k,0)>1.5 for k in ('arm','base')):raise RuntimeError('Stale feedback')
 if b.get('wheel_command')!=[0,0,0,0] or b.get('hardware_fault') or (base_locked and b.get('estop') is not True):raise RuntimeError('Base not stationary/locked')
 if a.get('dry_run') or not a.get('feedback_real' if require_gripper else 'arm_feedback_real') or not a.get('feedback_complete' if require_gripper else 'arm_feedback_complete') or a.get('fault'):raise RuntimeError('Invalid measured feedback')
 for name in ('joint1','joint2','joint3','joint4','joint5')+(('r_joint',) if require_gripper else ()):
  j=a.get('joints',{}).get(name,{})
  if not 0<=now-j.get('position_time',0)<1.5 or not 0<=now-j.get('temperature_time',0)<1.5 or not 0<=j.get('temperature',999)<(40 if name=='r_joint' else 50):raise RuntimeError('Joint age/temperature guard')
 return a
def interrupted(sig,frame):raise RuntimeError('Signal '+str(sig))
for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP,signal.SIGALRM):signal.signal(sig,interrupted)
signal.alarm(60)

from sensor_msgs.msg import CompressedImage
import base64
images=[]
def capture(m):
 if len(images)<25 and (not images or time.monotonic()-images[-1]['received']>=1.5):images.append({'received':time.monotonic(),'stamp':[m.header.stamp.sec,m.header.stamp.nanosec],'jpeg_base64':base64.b64encode(bytes(m.data)).decode()})
n.create_subscription(CompressedImage,'/rear_camera/image_raw/compressed',capture,10)
def raw(a):return {k:v['raw'] for k,v in a['joints'].items()}
def heights(r):
 q1=math.radians(.24*(r['joint2']-500));q2=q1-math.radians(.24*(r['joint3']-500))
 return [.064605,.064605+.10048*math.cos(q1),.064605+.10048*math.cos(q1)+.094714*math.cos(q2),forward_tool_pose_raw(r)['translation_m'][2]]
def upward_path(before,target):
 previous=heights(before);z0=previous[-1]
 for i in range(1,101):
  rr=dict(before);rr['joint2']=before['joint2']+(target-before['joint2'])*i/100
  current=heights(rr)
  if any(b<a-1e-7 for a,b in zip(previous,current)):raise RuntimeError('Recovery predicted downward link movement')
  if math.hypot(*forward_tool_pose_raw(rr)['translation_m'][:2])>.35:raise RuntimeError('Radial envelope')
  previous=current
 if not .005<=previous[-1]-z0<=.025:raise RuntimeError('Recovery predicted height outside5to25mm')
 return previous[-1]-z0
try:
 def fresh():
  try:guard(True,True);return True
  except RuntimeError:return False
 spin_until(fresh,8);initial=raw(guard(True,True));report.update(initial_raw=initial,initial_tcp=forward_tool_pose_raw(initial)['translation_m'],step_results=[],scope='Upward-only reset preparation; not an established home pose, grasp or calibration')
 if n.count_publishers('/arm_controller/command')!=1 or n.count_publishers('/gripper_controller/command')!=0 or n.count_publishers('/cmd_vel')!=0:raise RuntimeError('Competing motion publisher')
 if not 125<=initial['joint2']+20<=875:raise RuntimeError('Native shoulder limit')
 report['first_predicted_rise_m']=upward_path(initial,initial['joint2']+20)
 if not params.wait_for_service(timeout_sec=2) or not reset.wait_for_service(timeout_sec=2):raise RuntimeError('Control services absent')
 changed=True;gates(False);estop.publish(Bool(data=True));stop_time=time.monotonic()
 report['reset_attempts']=[]
 def ready():
  a=guard(True,True)
  return time.monotonic()-stop_time>1.2 and a.get('reset_eligible') and not a.get('poll_pending') and a.get('healthy_seconds',0)>=10
 for _ in range(3):
  spin_until(ready,10);ans=result(reset.call_async(Trigger.Request()));report['reset_attempts'].append({'success':ans.success,'message':ans.message})
  if ans.success:break
 if not ans.success:raise RuntimeError('Native reset refused')
 gates(True);spin_until(lambda:not latest['arm'].get('estop') and latest['arm'].get('enable_arm_motion') and not latest['arm'].get('enable_gripper_motion'),3)
 if any(abs(raw(guard(True,True))[k]-initial[k])>3 for k in initial):raise RuntimeError('Pose changed at reset')
 for index in range(3):
  a=guard(True,True);before=raw(a);target=before['joint2']+20
  if not 125<=target<=875:raise RuntimeError('Native shoulder bounds')
  predicted=upward_path(before,target);temps={k:v['temperature'] for k,v in a['joints'].items()}
  msg=JointTrajectory();msg.header.stamp=n.get_clock().now().to_msg();msg.joint_names=['joint2'];pt=JointTrajectoryPoint();pt.positions=[(target-500)*math.pi/750];pt.time_from_start.sec=3;msg.points=[pt]
  stamp={'sec':msg.header.stamp.sec,'nanosec':msg.header.stamp.nanosec};sent=time.monotonic();events.append({'kind':'dispatch','index':index,'stamp':stamp,'target_raw':target,'duration':3,'predicted_rise_m':predicted});pub.publish(msg);report['movement_commands']+=1
  spin_until(lambda:latest.get('ack',{}).get('stamp')==stamp,2);ack=latest['ack']
  if not ack.get('accepted') or ack.get('targets')!=[[5,target]]:raise RuntimeError('Rejected or mismatched ACK')
  stable=None;arrived=False;until=sent+6
  while time.monotonic()<until:
   rclpy.spin_once(n,timeout_sec=.03);a=guard(True,True);actual=raw(a)
   if a.get('estop') or not a.get('enable_arm_motion') or a.get('enable_gripper_motion'):raise RuntimeError('Arm-only recovery permission lost')
   if any(abs(actual[k]-initial[k])>3 for k in initial if k!='joint2'):raise RuntimeError('Non-commanded joint moved')
   if actual['joint2']<before['joint2']-2 or actual['joint2']>target+6:raise RuntimeError('Shoulder excursion')
   if any(a['joints'][k]['temperature']-temps[k]>=6 for k in temps):raise RuntimeError('Point thermal rise6C')
   if forward_tool_pose_raw(actual)['translation_m'][2]<forward_tool_pose_raw(before)['translation_m'][2]-.002:raise RuntimeError('Measured downward excursion')
   good=time.monotonic()-sent>=3 and a['joints']['joint2']['position_time']>sent and abs(actual['joint2']-target)<=6
   if good:
    if stable is None:stable=time.monotonic()
    if time.monotonic()-stable>=.6:arrived=True;break
   else:stable=None
  rise=forward_tool_pose_raw(actual)['translation_m'][2]-forward_tool_pose_raw(before)['translation_m'][2]
  report['step_results'].append({'target_raw':target,'actual_raw':actual,'error_ticks':actual['joint2']-target,'model_rise_m':rise,'passed':arrived,'ack':ack})
  if not arrived or rise<.005:raise RuntimeError('Recovery measured arrival/rise failed; no next step')
  if forward_tool_pose_raw(actual)['translation_m'][2]-report['initial_tcp'][2]>=.04:break
 report.update(passed=True,final_before_stop_raw=actual,model_height_change_before_stop_m=forward_tool_pose_raw(actual)['translation_m'][2]-report['initial_tcp'][2])
except Exception as error:report['error']=repr(error)
finally:
 if changed:
  stop.publish(Empty())
  try:gates(False)
  except Exception as error:report['disable_error']=repr(error)
  for _ in range(12):estop.publish(Bool(data=True));rclpy.spin_once(n,timeout_sec=.1)
  report['cleanup_confirmed']=latest.get('arm',{}).get('estop') is True and latest.get('base',{}).get('estop') is True and not latest.get('arm',{}).get('enable_arm_motion') and not latest.get('arm',{}).get('enable_gripper_motion')
  if not report['cleanup_confirmed']:report['passed']=False
 report.update(final_arm=latest.get('arm'),final_base=latest.get('base'),events=events,images=images)
 n.destroy_node();rclpy.shutdown()
signal.alarm(0);print(json.dumps(report))
