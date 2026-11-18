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
rclpy.init();n=rclpy.create_node('t07_guarded_pick_'+__import__('uuid').uuid4().hex[:8],enable_rosout=False,start_parameter_services=False)
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
def gates(arm,gripper=False):
 req=SetParametersAtomically.Request();req.parameters=[Parameter('enable_arm_motion',value=arm).to_parameter_msg(),Parameter('enable_gripper_motion',value=gripper).to_parameter_msg()]
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
signal.alarm(90)
"""Pure candidate-path checks. Callers provide the existing physical FK/IK."""
import math
ARM_NAMES=('joint1','joint2','joint3','joint4');SERVO_IDS=dict(zip(ARM_NAMES,(6,5,4,3)))
def current_pitch(raw):return .24*((raw['joint2']-500)-(raw['joint3']-500)+(raw['joint4']-500))
def tool(raw):return forward_tool_pose_raw(raw)['translation_m']
def clearance(raw,floor,minimum):
 q1=math.radians(.24*(raw['joint2']-500));q2=q1-math.radians(.24*(raw['joint3']-500));q3=q2+math.radians(.24*(raw['joint4']-500))
 heights=[.064605,.064605+.10048*math.cos(q1),.064605+.10048*math.cos(q1)+.094714*math.cos(q2)];xyz=tool(raw)
 if xyz[2]<floor+minimum or min(heights)<floor+.012 or math.hypot(xyz[0],xyz[1])>.35:raise ValueError('Model link/TCP clearance')
def make_path(raw,target,pitch,compensation,floor,minimum):
 start=tool(raw);start_pitch=current_pitch(raw);count=max(1,math.ceil(math.dist(start,target)/.010),math.ceil(abs(pitch-start_pitch)/5))
 if len(target)!=3 or not all(math.isfinite(v) for v in (*target,pitch,floor,minimum)) or not .012<=minimum<=.05:raise ValueError('Invalid path inputs')
 if any(abs(compensation[k])>10 for k in ARM_NAMES):raise ValueError('Compensation magnitude')
 inverse_kinematics_all(*target,pitch)
 while count<=20:
  previous=dict(raw);path=[];too_large=False
  for i in range(1,count+1):
   ratio=i/count;xyz=[a+(b-a)*ratio for a,b in zip(start,target)];angle=start_pitch+(pitch-start_pitch)*ratio
   solutions=inverse_kinematics_all(*xyz,angle)
   solutions=[s for s in solutions if 125<=s['servo_targets'][5]<=875]
   if not solutions:raise ValueError('No native-bounded IK branch')
   solutions.sort(key=lambda s:max(abs(s['servo_targets'][SERVO_IDS[k]]-previous[k]) for k in ARM_NAMES))
   desired={k:solutions[0]['servo_targets'][SERVO_IDS[k]] for k in ARM_NAMES};desired['joint1']=round(500-math.degrees(math.atan2(xyz[0],xyz[1]))/.24)
   command={k:desired[k]+compensation[k] for k in ARM_NAMES}
   if not 125<=command['joint2']<=875 or any(not 0<=v<=1000 for v in command.values()):raise ValueError('Command bounds')
   if max(abs(command[k]-previous[k]) for k in ARM_NAMES)>35:too_large=True;break
   rr=dict(raw);rr.update(desired);desired_tcp=tool(rr)
   if math.dist(desired_tcp,xyz)>.002:raise ValueError('IK/FK residual')
   for j in range(21):
    swept=dict(previous);swept.update({k:previous[k]+j/20*(command[k]-previous[k]) for k in ARM_NAMES});clearance(swept,floor,minimum)
   path.append({'desired_raw':desired,'command_raw':command,'desired_tcp':desired_tcp,'minimum_clearance':minimum});previous.update(desired)
  if not too_large:return path,previous
  count*=2
 raise ValueError('Path needs too many points for bounded attempt')
def prepare_pick_paths(raw,ball,compensation,pitch=-135):
 floor=ball[2]-.019;start=tool(raw);height=start[2]-floor
 if height<.014:raise ValueError('Initial floor clearance too small for this attempt')
 recovery=[start[0],start[1],start[2]+.025]
 paths={};paths['recover'],r=make_path(raw,recovery,current_pitch(raw),compensation,floor,max(.012,min(.03,height-.002)))
 paths['pregrasp'],r=make_path(r,[ball[0],ball[1],ball[2]+.05],pitch,compensation,floor,.03)
 paths['contact'],r=make_path(r,list(ball),pitch,compensation,floor,.012)
 contact_raw=dict(r);contact_tcp=tool(r)
 paths['lift'],r=make_path(r,[contact_tcp[0],contact_tcp[1],contact_tcp[2]+.020],pitch,compensation,floor,.012)
 paths['return'],r=make_path(r,contact_tcp,pitch,compensation,floor,.012)
 if sum(len(v) for v in paths.values())>24:raise ValueError('Attempt exceeds waypoint budget')
 return {'floor_m':floor,'paths':paths,'contact_raw':contact_raw,'contact_tcp':contact_tcp,'initial_tcp':start,'ball_base_xyz':ball,'scope':'Low-speed experimental pick path; TCP/link centerline model checks are not a full collision-mesh proof'}

GEOMETRY={'source_robot_ros_ns': 1794897071334481968, 'expected_raw': {'joint1': 468, 'joint2': 269, 'joint3': 770, 'joint4': 354, 'joint5': 495, 'r_joint': 155}, 'ball': [0.014174894811812312, 0.271397715854721, -0.07503735314169631], 'compensation': {'joint1': -3, 'joint2': 4, 'joint3': -9, 'joint4': 7}, 'source_report': 'logs\\t07\\runs\\20261117T050800Z_f8ef182f\\native_houghplan_630aad67\\stdout.log', 'source_camera_point': {'xyz': [-0.021756967692213084, 0.013800149276805014, 0.111951948810299], 'frame': 't07_rear_camera_measured', 'stamp': {'sec': 1794897071, 'nanosec': 175386595}}, 'static_scene_assumption': 'One finite attempt on the last complete observed sphere; no base motion or manual scene change; no vision refresh claimed during final approach'}
# Bounded experimental pick: no intermediate STOP on successful waypoints.
from std_msgs.msg import Int32
buzzer=n.create_publisher(Int32,'/arm_controller/buzzer',10)
grip_pub=n.create_publisher(JointTrajectory,'/gripper_controller/command',10)
reference_roll=None;task_deadline=None;current_phase='preflight'
report.update(grasp_verified=False,step_results=[],gripper_results=[],hold_seconds=0,scope='experimental physical pick attempt; success requires observed unsupported ball hold')
def stage(name):
 global current_phase
 current_phase=name;report['last_phase']=name;events.append({'kind':'phase','phase':name,'monotonic':time.monotonic()})
def fresh_all():
 try:guard(True,True);return True
 except RuntimeError as error:report['feedback_wait_reason']=str(error);return False
def check_live():
 a=guard(True,True)
 if reference_roll is not None and abs(a['joints']['joint5']['raw']-reference_roll)>2:raise RuntimeError('Fixed wrist roll changed')
 if task_deadline is not None and time.monotonic()>task_deadline:raise RuntimeError('Task time budget expired')
 return a
def measured(a):return {k:v['raw'] for k,v in a['joints'].items()}
def wait_live(seconds):
 until=time.monotonic()+seconds
 while time.monotonic()<until:rclpy.spin_once(n,timeout_sec=.03);check_live()
def send_arm(point,floor):
 a=check_live();before=measured(a);command=point['command_raw'];desired=point['desired_raw'];temps={k:v['temperature'] for k,v in a['joints'].items()}
 if max(abs(command[k]-before[k]) for k in ARM_NAMES)>35:raise RuntimeError('Measured command delta exceeds35ticks')
 for i in range(21):
  rr=dict(before);rr.update({k:before[k]+i/20*(command[k]-before[k]) for k in ARM_NAMES});clearance(rr,floor,point['minimum_clearance'])
 msg=JointTrajectory();msg.header.stamp=n.get_clock().now().to_msg();msg.joint_names=list(ARM_NAMES);p=JointTrajectoryPoint();p.positions=[(command[k]-500)*math.pi/750*(-1 if k=='joint3' else 1) for k in ARM_NAMES];p.time_from_start.sec=2;msg.points=[p]
 stamp={'sec':msg.header.stamp.sec,'nanosec':msg.header.stamp.nanosec};events.append({'kind':'dispatch','phase':current_phase,'stamp':stamp,'command_raw':command,'desired_measured_raw':desired,'duration':2})
 pub.publish(msg);report['movement_commands']+=1;sent=time.monotonic();spin_until(lambda:latest.get('ack',{}).get('stamp')==stamp,2);ack=latest['ack']
 if not ack.get('accepted') or ack.get('targets')!=[[SERVO_IDS[k],command[k]] for k in ARM_NAMES]:raise RuntimeError('Arm ACK rejected or mismatched')
 stable=None;arrived=False;until=sent+5
 while time.monotonic()<until:
  rclpy.spin_once(n,timeout_sec=.03);a=check_live();actual=measured(a)
  if a.get('estop') or not a.get('enable_arm_motion'):raise RuntimeError('Arm permission lost')
  clearance(actual,floor,point['minimum_clearance'])
  if any(abs(actual[k]-before[k])>40 for k in ARM_NAMES):raise RuntimeError('Measured joint excursion')
  if any(a['joints'][k]['temperature']-temps[k]>=6 for k in temps):raise RuntimeError('Point thermal rise6C')
  good=time.monotonic()-sent>=2 and all(a['joints'][k]['position_time']>sent and abs(actual[k]-desired[k])<=6 for k in ARM_NAMES) and math.dist(tool(actual),point['desired_tcp'])<=.008
  if good:
   if stable is None:stable=time.monotonic()
   if time.monotonic()-stable>=.6:arrived=True;break
  else:stable=None
 report['step_results'].append({'phase':current_phase,'ack':ack,'command_raw':command,'desired_raw':desired,'measured_raw':actual,'measured_tcp':tool(actual),'passed':arrived})
 if not arrived:raise RuntimeError('Measured arm arrival failed; no following action')
def run_path(name,points,floor):
 stage(name)
 for point in points:send_arm(point,floor)
def send_gripper(target,direction):
 a=check_live();initial=a['joints']['r_joint']['raw'];temperature=a['joints']['r_joint']['temperature']
 if not 0<=target<=320 or abs(target-initial)>100:raise RuntimeError('Bounded gripper target/delta')
 msg=JointTrajectory();msg.header.stamp=n.get_clock().now().to_msg();msg.joint_names=['r_joint'];p=JointTrajectoryPoint();p.positions=[(target-700)*math.pi/1000];p.time_from_start.sec=1;p.time_from_start.nanosec=500000000;msg.points=[p]
 stamp={'sec':msg.header.stamp.sec,'nanosec':msg.header.stamp.nanosec};events.append({'kind':'gripper_dispatch','phase':current_phase,'stamp':stamp,'target_raw':target,'duration':1.5});grip_pub.publish(msg);report['gripper_commands']+=1;sent=time.monotonic();spin_until(lambda:latest.get('ack',{}).get('stamp')==stamp,2);ack=latest['ack']
 if not ack.get('accepted') or ack.get('targets')!=[[1,target]]:raise RuntimeError('Gripper ACK rejected/mismatched')
 while time.monotonic()-sent<2.5:
  rclpy.spin_once(n,timeout_sec=.03);a=check_live()
  if a.get('estop') or not a.get('enable_gripper_motion'):raise RuntimeError('Gripper permission lost')
  if a['joints']['r_joint']['temperature']-temperature>=6:raise RuntimeError('Gripper thermal rise')
 actual=a['joints']['r_joint']['raw'];delta=actual-initial
 report['gripper_results'].append({'phase':current_phase,'ack':ack,'command_raw':target,'initial_raw':initial,'measured_raw':actual,'delta':delta,'contact_verified':False})
 if direction*delta<8:raise RuntimeError('No sufficient measured gripper response')
 return actual
try:
 spin_until(fresh_all,8);a=check_live();raw=measured(a);reference_roll=raw['joint5']
 if not 0<=n.get_clock().now().nanoseconds-GEOMETRY['source_robot_ros_ns']<=30*10**9:raise RuntimeError('Geometry report expired')
 if any(abs(raw[k]-GEOMETRY['expected_raw'][k])>3 for k in GEOMETRY['expected_raw']):raise RuntimeError('Start changed; get new scene/feedback')
 if n.count_publishers('/arm_controller/command')!=1 or n.count_publishers('/gripper_controller/command')!=1 or n.count_publishers('/cmd_vel')!=0:raise RuntimeError('Competing command source')
 # Compute every candidate before any reset, permission enable or movement.
 plan=prepare_pick_paths(raw,GEOMETRY['ball'],GEOMETRY['compensation']);report.update(plan=plan,geometry=GEOMETRY,initial_raw=raw)
 if not params.wait_for_service(timeout_sec=2) or not reset.wait_for_service(timeout_sec=2):raise RuntimeError('Services unavailable')
 changed=True;gates(False,False);estop.publish(Bool(data=True));last_stop=time.monotonic()
 def ready():
  try:return time.monotonic()-last_stop>1.2 and received.get('arm',0)>last_stop+1 and check_live().get('reset_eligible') and not latest['arm'].get('poll_pending') and latest['arm'].get('healthy_seconds',0)>=10
  except RuntimeError:return False
 report['reset_attempts']=[]
 for _ in range(5):
  spin_until(ready,10);answer=result(reset.call_async(Trigger.Request()));report['reset_attempts'].append({'success':answer.success,'message':answer.message})
  if answer.success:break
  wait_live(.2)
 if not answer.success:raise RuntimeError('Reset refused; no motion')
 gates(True,False);spin_until(lambda:latest['arm'].get('enable_arm_motion') and not latest['arm'].get('estop'),3)
 if any(abs(measured(check_live())[k]-raw[k])>3 for k in ARM_NAMES):raise RuntimeError('Pose shifted at reset')
 task_deadline=time.monotonic()+80
 for name in ('recover','pregrasp','contact'):run_path(name,plan['paths'][name],plan['floor_m'])
 stage('close');gates(True,True);spin_until(lambda:latest['arm'].get('enable_gripper_motion') is True,3)
 for target in (240,320):closed=send_gripper(target,1)
 if not 275<=closed<=330:raise RuntimeError('Closure outside bounded candidate range; no lift')
 report['contact_pose_measured']=tool(measured(check_live()));report['contact_candidate_only']=True
 buzzer.publish(Int32(data=2400));wait_live(.5)
 run_path('lift',plan['paths']['lift'],plan['floor_m']);report['lift_pose_measured']=tool(measured(check_live()))
 report['measured_lift_height_m']=report['lift_pose_measured'][2]-report['contact_pose_measured'][2]
 if not .012<=report['measured_lift_height_m']<=.035:raise RuntimeError('Measured lift height outside envelope')
 stage('hold');began=time.monotonic();lift_target=plan['paths']['lift'][-1]
 while time.monotonic()-began<5:
  rclpy.spin_once(n,timeout_sec=.03);a=check_live();rr=measured(a);report['hold_seconds']=time.monotonic()-began
  if tool(rr)[2]-report['contact_pose_measured'][2]<.012 or any(abs(rr[k]-lift_target['desired_raw'][k])>6 for k in ARM_NAMES) or math.dist(tool(rr),lift_target['desired_tcp'])>.008:raise RuntimeError('Measured lift pose not maintained during hold')
 run_path('return',plan['paths']['return'],plan['floor_m']);stage('controlled_release')
 for target in (240,155):opened=send_gripper(target,-1)
 if opened>185:raise RuntimeError('Release opening not confirmed')
 report.update(passed=True,sequence_completed=True,grasp_verified=False,verification_pending='Operator must confirm the red ball was fully unsupported during hold; encoders alone do not verify grasp')
except Exception as error:report['error']=repr(error)
finally:
 if changed:
  stop.publish(Empty())
  try:gates(False,False)
  except Exception as error:report['disable_error']=repr(error)
  for _ in range(8):estop.publish(Bool(data=True));rclpy.spin_once(n,timeout_sec=.1)
  report['cleanup_confirmed']=latest.get('arm',{}).get('estop') is True and latest.get('base',{}).get('estop') is True and not latest.get('arm',{}).get('enable_arm_motion') and not latest.get('arm',{}).get('enable_gripper_motion')
  if not report['cleanup_confirmed']:report['passed']=False
 report['final_arm']=latest.get('arm');report['final_base']=latest.get('base');report['events']=events
 n.destroy_node();rclpy.shutdown()
signal.alarm(0);print(json.dumps(report))
