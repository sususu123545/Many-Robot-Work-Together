"""Make a reviewed short pregrasp stage from a fresh read-only geometry report."""
import argparse,hashlib,importlib.util,json,math
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
MAIN=r'''
try:
 spin_until(lambda:all(k in latest for k in ('arm','base')),6);a=guard(require_gripper=True);raw={k:v['raw'] for k,v in a['joints'].items()}
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
  a=guard(True);before={k:v['raw'] for k,v in a['joints'].items()};command=waypoint['command_raw'];desired=waypoint['desired_raw']
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
   if any(a['joints'][k]['temperature']-initial_temperatures[k]>=6 for k in names):raise RuntimeError('Stage temperature-rise guard')
   good=time.monotonic()-sent>=3 and all(a['joints'][k]['position_time']>sent and abs(actual[k]-desired[k])<=6 for k in names) and math.dist(tcp,waypoint['desired_tcp'])<=.008
   if good:
    if stable is None:stable=time.monotonic()
    if time.monotonic()-stable>=.6:arrived=True;break
   else:stable=None
  report['step_results'].append({'ack':ack,'command_raw':command,'desired_raw':desired,'measured_raw':actual,'measured_tcp':tcp,'passed':arrived})
  if not arrived:raise RuntimeError('Stage waypoint failed measured arrival; no next waypoint')
 report.update(passed=True,final_raw=actual)
'''
def main():
 p=argparse.ArgumentParser();p.add_argument('--report',required=True);p.add_argument('--output',required=True);p.add_argument('--plan-output',required=True);a=p.parse_args()
 d=json.loads((ROOT/a.report).read_text(encoding='utf-8'));raw=d['current_raw'];target=d['live_tf_candidate_base_xyz']
 if any(k not in raw for k in ('joint1','joint2','joint3','joint4','joint5','r_joint')):raise ValueError('Complete initial feedback required before stage starts')
 if d.get('live_offline_distance_m',1)>.005 or d.get('camera_point_count',0)<5 or d.get('target_complete_sphere_verified') is not True or not 0<=d.get('target_age_seconds',999)<=.5:raise ValueError('Fresh complete-sphere geometry required')
 source=ROOT/'logs/t07/runs/20261116T092847Z_8844851d/candidate_sources/52_arm_kinematics.py';spec=importlib.util.spec_from_file_location('geo',source);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
 pose=m.forward_tool_pose_raw(raw);start=pose['translation_m'];pitch=sum(pose['angles_deg'][k] for k in ('theta3','theta4','theta5'))
 final=[target[0],target[1],target[2]+.05];distance=math.dist(start,final);span=min(.05,distance);ratio=min(1,span/distance);end=[x+(y-x)*ratio for x,y in zip(start,final)];endpitch=pitch+(-140-pitch)*ratio;steps=max(1,math.ceil(span/.01))
 names=['joint1','joint2','joint3','joint4'];servo={'joint1':6,'joint2':5,'joint3':4,'joint4':3};waypoints=[];previous=dict(raw)
 for i in range(1,steps+1):
  fraction=i/steps;xyz=[x+(y-x)*fraction for x,y in zip(start,end)];angle=pitch+(endpitch-pitch)*fraction
  solutions=m.inverse_kinematics_all(*xyz,angle);solutions.sort(key=lambda s:max(abs(s['servo_targets'][servo[k]]-previous[k]) for k in names))
  desired={k:solutions[0]['servo_targets'][servo[k]] for k in names};command=dict(desired);command['joint3']-=8;command['joint4']+=3
  if not 125<=command['joint2']<=875 or any(not 0<=v<=1000 for v in command.values()) or max(abs(command[k]-previous[k]) for k in names)>35:raise ValueError('No bounded stage')
  rr=dict(raw);rr.update(desired);fk=m.forward_tool_pose_raw(rr)['translation_m']
  if math.dist(fk,xyz)>.002:raise ValueError('IK/FK residual')
  waypoints.append({'desired_raw':desired,'command_raw':command,'desired_tcp':fk});previous.update(desired)
 plan={'expected_raw':raw,'source_robot_ros_ns':d['source_robot_ros_ns'],'target_ball_base_xyz':target,'pregrasp_target':final,'floor_estimate_m':target[2]-.019,'waypoints':waypoints,'scope':'pregrasp only; no contact/closure','local_elbow_compensation':-8,'compensation_validation':'local single-joint result; every new waypoint measured independently','temperature_guards':'absolute40C and rise6C over max5-waypoint stage; native guards unchanged','source_report':a.report,'kinematics_sha256':hashlib.sha256(source.read_bytes()).hexdigest()}
 base=(ROOT/'scripts/t07_redball_approach_step.py').read_text(encoding='utf-8');begin=base.index('\ntry:\n spin_until');endbase=base.index('\nexcept Exception as error:report',begin)
 text=base[:begin]+'\nPLAN='+repr(plan)+'\n'+MAIN.strip()+base[endbase:]
 text=text.replace('t07_redball_approach_one_step','t07_guarded_pregrasp_stage').replace('5mm approach response check; not grasp','bounded pregrasp stage; not grasp').replace("<50:","<40:")
 text=text.replace('def guard(base_locked=False):','def guard(base_locked=False,require_gripper=False):')
 text=text.replace("not a.get('feedback_real') or not a.get('feedback_complete')","not a.get('feedback_real' if require_gripper else 'arm_feedback_real') or not a.get('feedback_complete' if require_gripper else 'arm_feedback_complete')")
 text=text.replace("for name in ('joint1','joint2','joint3','joint4','joint5','r_joint'):","for name in ('joint1','joint2','joint3','joint4','joint5')+(('r_joint',) if require_gripper else ()):")
 compile(text,'guarded_pregrasp','exec');(ROOT/a.output).write_text(text,encoding='utf-8');(ROOT/a.plan_output).write_text(json.dumps(plan,indent=2)+'\n',encoding='utf-8')
 print(json.dumps({'waypoints':len(waypoints),'distance_m':math.dist(start,end),'output':a.output}))
if __name__=='__main__':main()
