"""Prepare an upward-only reset preparation, retaining native safety and arrival guards."""
import hashlib,json,math
from pathlib import Path
RUN=Path(__file__).resolve().parent
ROOT=RUN.parents[3]
base=(ROOT/'scripts/t07_redball_pregrasp_stage.py').read_text(encoding='utf-8')
prefix=base[:base.index('\nPLAN=')].replace("'t07_guarded_pregrasp_stage'","'t07_upward_reset_preparation'")
runtime=r'''
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
'''
source=prefix+'\n'+runtime
compile(source,'bounded_upward_reset_preparation','exec')
(RUN/'bounded_recovery.py').write_text(source,encoding='utf-8')
# Check all three possible local steps at the last measured starting pose.
scope={};pure=(ROOT/'scripts/t07_pick_paths.py').read_text(encoding='utf-8')
import importlib.util
file=ROOT/'logs/t07/runs/20261116T092847Z_8844851d/candidate_sources/52_arm_kinematics.py'
spec=importlib.util.spec_from_file_location('geo',file);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
rr={'joint1':558,'joint2':222,'joint3':718,'joint4':334,'joint5':495,'r_joint':156}
samples=[]
for _ in range(3):
 z0=m.forward_tool_pose_raw(rr)['translation_m'][2];zz=[]
 for i in range(101):
  r=dict(rr);r['joint2']=rr['joint2']+20*i/100;zz.append(m.forward_tool_pose_raw(r)['translation_m'][2])
 assert all(b>=a for a,b in zip(zz,zz[1:])) and .005<=zz[-1]-z0<=.025
 samples.append({'start_raw':dict(rr),'rise_m':zz[-1]-z0});rr['joint2']+=20
(RUN/'recovery_preparation.json').write_text(json.dumps({'source_sha256':hashlib.sha256(source.encode()).hexdigest(),'offline_monotonic_tcp_checks':samples,'max_commands':3,'max_step_ticks':20,'duration_seconds':3,'arrival_tolerance_ticks':6,'base_and_gripper_motion':False,'physical_home_verified':False},indent=2)+'\n')
print(json.dumps({'prepared':True,'sha256':hashlib.sha256(source.encode()).hexdigest(),'offline_steps_passed':len(samples)}))
