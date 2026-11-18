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
