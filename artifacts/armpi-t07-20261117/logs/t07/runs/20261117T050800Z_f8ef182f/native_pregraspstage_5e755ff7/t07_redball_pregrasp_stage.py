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
rclpy.init();n=rclpy.create_node('t07_guarded_pregrasp_stage',enable_rosout=False,start_parameter_services=False)
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
PLAN={'expected_raw': {'joint1': 468, 'joint2': 276, 'joint3': 768, 'joint4': 354, 'joint5': 495, 'r_joint': 156}, 'source_robot_ros_ns': 1794894882742810012, 'target_ball_base_xyz': [0.014155817624109637, 0.2730924134406871, -0.07212246596936996], 'pregrasp_target': [0.014155817624109637, 0.2730924134406871, -0.022122465969369953], 'floor_estimate_m': -0.09112246596936996, 'waypoints': [{'desired_raw': {'joint1': 472, 'joint2': 284, 'joint3': 775, 'joint4': 365}, 'command_raw': {'joint1': 469, 'joint2': 288, 'joint3': 766, 'joint4': 372}, 'desired_tcp': (0.02853112302668891, 0.24214439406953078, -0.059315693161486105)}, {'desired_raw': {'joint1': 475, 'joint2': 291, 'joint3': 780, 'joint4': 376}, 'command_raw': {'joint1': 472, 'joint2': 295, 'joint3': 771, 'joint4': 383}, 'desired_tcp': (0.026124706145345594, 0.24855997552628642, -0.051700822125463536)}, {'desired_raw': {'joint1': 478, 'joint2': 297, 'joint3': 784, 'joint4': 387}, 'command_raw': {'joint1': 475, 'joint2': 301, 'joint3': 775, 'joint4': 394}, 'desired_tcp': (0.023556237243240234, 0.254895876017642, -0.04404736608761223)}, {'desired_raw': {'joint1': 482, 'joint2': 302, 'joint3': 787, 'joint4': 398}, 'command_raw': {'joint1': 479, 'joint2': 306, 'joint3': 778, 'joint4': 405}, 'desired_tcp': (0.019734241613863664, 0.26123734475717897, -0.03635091454910269)}, {'desired_raw': {'joint1': 485, 'joint2': 305, 'joint3': 789, 'joint4': 408}, 'command_raw': {'joint1': 482, 'joint2': 309, 'joint3': 780, 'joint4': 415}, 'desired_tcp': (0.01676665529830501, 0.2664983545205408, -0.030188801134229837)}], 'scope': 'pregrasp only; no contact/closure', 'local_elbow_compensation': -8, 'compensation_validation': 'local single-joint result; every new waypoint measured independently', 'temperature_guards': 'project five arm joints50C as original response probe; gripper40C; rise6C per3second command; native65C and feedback-age guards unchanged; engineering test envelope, not manufacturer rating', 'source_report': 'logs\\t07\\runs\\20261117T050800Z_f8ef182f\\native_houghplan_6cec88ea\\stdout.log', 'kinematics_sha256': 'b13ea9e2bd7f3c0cb11584184e37f98d07f1bec8bec46803958020da5f42a178', 'pregrasp_pitch_deg': -135, 'command_compensation_ticks': {'joint1': -3, 'joint2': 4, 'joint3': -9, 'joint4': 7}, 'compensation_source_report': 'logs/t07/runs/20261117T050800Z_f8ef182f/native_pregraspstage_87f0e089/stdout.log', 'compensation_scope': 'Local plateau only; actual command and actual measured feedback remain distinct; all measured arrival checks unchanged', 'yaw_planning': 'Use continuous atan2 heading rounded to one encoder tick; remove vendor whole-degree heading quantization; measured arrival guards unchanged'}
try:
 def initial_fresh():
  try:guard(require_gripper=True);return True
  except RuntimeError as error:report['initial_feedback_wait_reason']=str(error);return False
 spin_until(initial_fresh,8);a=guard(require_gripper=True);raw={k:v['raw'] for k,v in a['joints'].items()}
 if not 0<=n.get_clock().now().nanoseconds-PLAN['source_robot_ros_ns']<=30*10**9:raise RuntimeError('Stage geometry report expired')
 if any(abs(raw[k]-v)>2 for k,v in PLAN['expected_raw'].items()):raise RuntimeError('Stage start changed')
 if n.count_publishers('/arm_controller/command')!=1 or n.count_publishers('/gripper_controller/command')!=0 or n.count_publishers('/cmd_vel')!=0:raise RuntimeError('Competing command source')
 initial_temperatures={k:v['temperature'] for k,v in a['joints'].items()};report.update(plan=PLAN,initial_raw=raw,step_results=[]);stage_deadline=time.monotonic()+40
 if not params.wait_for_service(timeout_sec=2) or not reset.wait_for_service(timeout_sec=2):raise RuntimeError('Control services unavailable')
 changed=True;gates(False);estop.publish(Bool(data=True));last_stop_sent=time.monotonic()
 spin_until(lambda:latest['arm'].get('estop') is True and latest['base'].get('estop') is True,3)
 def ready():
  try:return time.monotonic()-last_stop_sent>=1.2 and received.get('arm',0)>last_stop_sent+1.0 and guard(True).get('reset_eligible') is True and latest['arm'].get('poll_pending') is False and latest['arm'].get('healthy_seconds',0)>=10
  except RuntimeError:return False
 report['reset_attempts']=[]
 for reset_attempt in range(5):
  spin_until(ready,10);ans=result(reset.call_async(Trigger.Request()));report['reset_attempts'].append({'success':ans.success,'message':ans.message})
  if ans.success:break
  retry_wait=time.monotonic()+.2
  spin_until(lambda:time.monotonic()>=retry_wait,1)
 if not ans.success:raise RuntimeError('Native reset denied without movement: '+ans.message)
 spin_until(lambda:latest['arm'].get('estop') is False,3);gates(True);spin_until(lambda:latest['arm'].get('enable_arm_motion') is True and latest['arm'].get('enable_gripper_motion') is False,3)
 names=['joint1','joint2','joint3','joint4'];servo={'joint1':6,'joint2':5,'joint3':4,'joint4':3}
 for waypoint in PLAN['waypoints']:
  if time.monotonic()+7>stage_deadline:raise RuntimeError('Bounded stage time budget exhausted')
  a=guard(True);before={k:v['raw'] for k,v in a['joints'].items()};command=waypoint['command_raw'];desired=waypoint['desired_raw'];point_start_temperatures={k:v['temperature'] for k,v in a['joints'].items()}
  if max(abs(command[k]-before[k]) for k in names)>35 or abs(before['joint5']-PLAN['expected_raw']['joint5'])>2 or (before.get('r_joint') is not None and abs(before['r_joint']-PLAN['expected_raw']['r_joint'])>3):raise RuntimeError('Step/reference bound changed')
  for i in range(21):
   intermediate=dict(before);intermediate.update({k:before[k]+i/20*(command[k]-before[k]) for k in names})
   tcp=forward_tool_pose_raw(intermediate)['translation_m']
   if tcp[2]<PLAN['floor_estimate_m']+.03 or math.hypot(tcp[0],tcp[1])>.35:raise RuntimeError('Swept TCP clearance guard')
  msg=JointTrajectory();msg.header.stamp=n.get_clock().now().to_msg();msg.joint_names=names;pt=JointTrajectoryPoint();pt.positions=[(command[k]-500)*math.pi/750*(-1 if k=='joint3' else 1) for k in names];pt.time_from_start.sec=3;msg.points=[pt]
  stamp={'sec':msg.header.stamp.sec,'nanosec':msg.header.stamp.nanosec};events.append({'kind':'dispatch','stamp':stamp,'joint_names':names,'positions':list(pt.positions),'command_raw':command,'desired_measured_raw':desired,'duration':3})
  pub.publish(msg);report['movement_commands']+=1;sent=time.monotonic();spin_until(lambda:latest.get('ack',{}).get('stamp')==stamp,2)
  ack=latest['ack']
  if not ack.get('accepted') or ack.get('targets')!=[[servo[k],command[k]] for k in names]:raise RuntimeError('Rejected/mismatched ACK')
  stable=None;arrived=False;until=sent+6
  while time.monotonic()<until:
   rclpy.spin_once(n,timeout_sec=.03);a=guard(True);actual={k:v['raw'] for k,v in a['joints'].items()}
   if a.get('estop') or not a.get('enable_arm_motion'):raise RuntimeError('Arm permission lost')
   tcp=forward_tool_pose_raw(actual)['translation_m']
   if tcp[2]<PLAN['floor_estimate_m']+.03 or any(abs(actual[k]-before[k])>40 for k in names):raise RuntimeError('Measured path bound exceeded')
   if any(a['joints'][k]['temperature']-point_start_temperatures[k]>=6 for k in names):raise RuntimeError('Point temperature-rise guard')
   good=time.monotonic()-sent>=3 and all(a['joints'][k]['position_time']>sent and abs(actual[k]-desired[k])<=6 for k in names) and math.dist(tcp,waypoint['desired_tcp'])<=.008
   if good:
    if stable is None:stable=time.monotonic()
    if time.monotonic()-stable>=.6:arrived=True;break
   else:stable=None
  report['step_results'].append({'ack':ack,'command_raw':command,'desired_raw':desired,'measured_raw':actual,'measured_tcp':tcp,'passed':arrived,'point_start_temperatures':point_start_temperatures})
  if not arrived:raise RuntimeError('Stage waypoint failed measured arrival; no next waypoint')
 report.update(passed=True,final_raw=actual)
except Exception as error:report['error']=repr(error)
finally:
 if changed:
  stop.publish(Empty())
  try:gates(False)
  except Exception as error:report['disable_error']=repr(error)
  for _ in range(8):estop.publish(Bool(data=True));rclpy.spin_once(n,timeout_sec=.1)
  report['cleanup_confirmed']=latest.get('arm',{}).get('estop') is True and latest.get('base',{}).get('estop') is True and latest.get('arm',{}).get('enable_arm_motion') is False and latest.get('arm',{}).get('enable_gripper_motion') is False
  if not report['cleanup_confirmed']:report['passed']=False
 report['final_arm']=latest.get('arm');report['final_base']=latest.get('base');report['events']=events
 n.destroy_node();rclpy.shutdown()
signal.alarm(0);print(json.dumps(report))
