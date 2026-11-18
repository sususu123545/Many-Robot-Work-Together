import os
from pathlib import Path
for item in Path('/proc/1/environ').read_bytes().split(b'\0'):
 if item.startswith((b'ROS_DOMAIN_ID=',b'ROS_LOCALHOST_ONLY=',b'RMW_IMPLEMENTATION=')):
  k,v=item.decode().split('=',1);os.environ[k]=v
if os.environ.get('ROS_DOMAIN_ID')!='42':raise RuntimeError('Wrong robot domain')
os.environ['FASTDDS_BUILTIN_TRANSPORTS']='UDPv4'
"""Closed-loop grasp: drive the IK tool point onto the measured ball centre.

Runs inside the armpi_ros2 core container. Every move is verified against the
servo feedback and retransmitted when the bus silently drops the packet
(observed on ~30-40 % of writes: "move written" but no joint motion).

Phases:
  1  coarse: swing to 50 mm above the ball
  2  servo:  measure -> delta = ball - tool -> move by delta (repeat)
  3  clamp:  close the gripper until the pads stall on the ball
  4  verify: lift 15 mm, re-measure; report whether the ball came along
"""
import json
import math
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# ---- geometry (identical constants to armpi_ros2/arm_kinematics.py) ----
BASE_H, L1, L2, L3 = 0.064605, 0.10048, 0.094714, 0.16331
TPD = 1000.0 / 240.0
ARM = ('joint1', 'joint2', 'joint3', 'joint4')
CENTRE = {'joint1': 500, 'joint2': 500, 'joint3': 500, 'joint4': 500,
          'joint5': 500, 'r_joint': 700}
SIGN = {'joint1': 1, 'joint2': 1, 'joint3': -1, 'joint4': 1, 'joint5': 1, 'r_joint': 1}
LIMITS = {'joint1': (0, 1000), 'joint2': (125, 875), 'joint3': (0, 1000),
          'joint4': (0, 1000), 'joint5': (0, 1000), 'r_joint': (0, 700)}
MAX_STEP = 40


def tick_rad(name):
    return math.radians(180.0 if name == 'r_joint' else 240.0) / 1000.0


def to_rad(name, raw):
    return SIGN[name] * (raw - CENTRE[name]) * tick_rad(name)


def fk(raw):
    th4 = -(raw['joint3'] - 500) * 0.24
    th5 = (raw['joint2'] - 500) * 0.24
    th3 = (raw['joint4'] - 500) * 0.24
    th6 = (raw['joint1'] - 500) * 0.24
    q1, q2, q3 = (math.radians(th5), math.radians(th5 + th4),
                  math.radians(th5 + th4 + th3))
    radial = -(L1 * math.sin(q1) + L2 * math.sin(q2) + L3 * math.sin(q3))
    z = BASE_H + L1 * math.cos(q1) + L2 * math.cos(q2) + L3 * math.cos(q3)
    yaw = math.radians(th6)
    return -radial * math.sin(yaw), radial * math.cos(yaw), z


def ik(x, y, z, pitch_deg):
    radius = math.hypot(x, y)
    a = math.radians(pitch_deg)
    wr = radius + L3 * math.sin(a)
    wz = z - BASE_H - L3 * math.cos(a)
    ce = ((wr * wr + wz * wz - L1 * L1 - L2 * L2) / (2.0 * L1 * L2))
    if ce < -1.0 - 1e-9 or ce > 1.0 + 1e-9:
        return []
    ce = min(1.0, max(-1.0, ce))
    elbow = math.acos(ce)
    yaw = -math.degrees(math.atan2(x, y)) if radius > 1e-12 else 0.0
    out = []
    for t4 in (-elbow, elbow):
        sh = (math.atan2(wz, wr)
              - math.atan2(L2 * math.sin(t4), L1 + L2 * math.cos(t4)))
        t5 = sh - math.pi / 2.0
        t3 = a - t5 - t4
        deg = tuple(math.degrees(v) for v in (t3, t4, t5))
        tgt = {'joint1': round(round(120.0 + yaw) * TPD),
               'joint2': round(deg[2] * TPD + 500.0),
               'joint3': round(-deg[1] * TPD + 500.0),
               'joint4': round(deg[0] * TPD + 500.0)}
        if all(LIMITS[n][0] <= v <= LIMITS[n][1] for n, v in tgt.items()):
            out.append(tgt)
    return out


class Grasp(Node):
    def __init__(self):
        super().__init__('t07_reference_'+uuid.uuid4().hex[:8])
        self.status = {}
        self.ball = None
        self.traj = self.create_publisher(JointTrajectory, '/arm_controller/command', 10)
        self.create_subscription(String, '/arm_controller/status',
                                 lambda m: self.status.update(json.loads(m.data)), 10)
        self.create_subscription(PointStamped, '/grasp/target_base',
                                 lambda m: setattr(self, 'ball', m), 10)

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def raw(self):
        joints = self.status.get('joints', {})
        return {n: joints.get(n, {}).get('raw') for n in CENTRE}

    def temp(self, name):
        return self.status.get('joints', {}).get(name, {}).get('temperature')

    def wait_status(self, seconds=10.0):
        end = time.monotonic() + seconds
        while time.monotonic() < end and not self.status:
            rclpy.spin_once(self, timeout_sec=0.1)

    # ----- one command with verify + retransmit -----
    # The servos stop inside their own positioning deadband (measured 8-12
    # ticks), so "arrived" must be looser than that or every successful move
    # looks like a dropped packet.
    def send(self, targets, duration=2.0, tries=4, tolerance=14):
        for attempt in range(1, tries + 1):
            msg = JointTrajectory()
            msg.joint_names = list(targets)
            point = JointTrajectoryPoint()
            point.positions = [to_rad(n, v) for n, v in targets.items()]
            point.time_from_start.sec = int(duration)
            point.time_from_start.nanosec = int(round((duration - int(duration)) * 1e9))
            msg.points = [point]
            msg.header.stamp = self.get_clock().now().to_msg()
            self.traj.publish(msg)
            self.spin_for(duration + 0.9)
            now = self.raw()
            err = max(abs((now.get(n) or 0) - v) for n, v in targets.items())
            if err <= tolerance:
                return True, attempt, err
        return False, tries, err

    def move_tool(self, target, pitch_pref=(-152, -148, -144, -140, -136)):
        """Move the IK tool point to `target` in <=MAX_STEP sub-steps."""
        sols = []
        for pitch in pitch_pref:
            sols = ik(target[0], target[1], target[2], pitch)
            if sols:
                break
        if not sols:
            return 'NO_IK'
        start = self.raw()
        best = min(sols, key=lambda s: max(abs(s[n] - start[n]) for n in ARM))
        cur = {n: start[n] for n in ARM}
        n_steps = max(1, int(math.ceil(max(abs(best[n] - cur[n]) for n in ARM) / MAX_STEP)))
        problems = 0
        for i in range(1, n_steps + 1):
            s = i / n_steps
            step = {n: int(round(cur[n] + (best[n] - cur[n]) * s)) for n in ARM}
            step['joint5'] = int(self.raw().get('joint5') or 495)
            ok, tries, err = self.send(step)
            if not ok:
                problems += 1
                # keep going: the servo loop re-measures and corrects anyway
        return 'OK' if problems == 0 else 'OK_WITH_%d_RETRIES' % problems

    def measure(self, timeout=8.0):
        """Fresh ball sample in arm_base (wait for a new message after a move)."""
        self.ball = None
        end = time.monotonic() + timeout
        while time.monotonic() < end and self.ball is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.ball is None:
            return None
        p = self.ball.point
        return (p.x, p.y, p.z)

    def clamp(self, stop_raw, step=70, tries=3):
        """Close the jaws; report contact when a step settles far short of its
        target (a normal move only falls short by the servo deadband)."""
        cur = self.raw()['r_joint']
        while cur < stop_raw:
            nxt = min(stop_raw, cur + step)
            self.send({'r_joint': nxt}, duration=1.5, tries=tries, tolerance=12)
            new = self.raw()['r_joint']
            if new < nxt - 22:
                return new, True
            if new - cur < 12:
                return new, True
            cur = new
        return cur, False


from pathlib import Path
import os,signal,uuid,statistics,base64
from std_msgs.msg import Bool,Empty
from sensor_msgs.msg import CompressedImage
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger
from armpi_ros2.arm_kinematics import forward_tool_pose_raw
RUN_EVENTS=[];REPORT={'movement_commands':0,'gripper_commands':0,'chassis_commands':0,'grasp_verified':False};PROCS=[]
class Trial(Grasp):
 def __init__(self):
  super().__init__();self.base={};self.acks={};self.points=[];self.detections={};self.images=[];self.after_ns=0;self.floor=None;self.mode='preflight'
  self.create_subscription(String,'/chassis_control/status',lambda m:setattr(self,'base',json.loads(m.data)),10)
  self.create_subscription(String,'/arm_controller/command_result',lambda m:self.acks.update({(json.loads(m.data)['stamp']['sec'],json.loads(m.data)['stamp']['nanosec']):json.loads(m.data)}),10)
  self.prefix='/t07_trial_'+uuid.uuid4().hex[:8]
  self.create_subscription(PointStamped,self.prefix+'/point',lambda m:self.points.append(m),10)
  self.create_subscription(String,self.prefix+'/detection',lambda m:self.detections.update({(json.loads(m.data)['stamp_sec'],json.loads(m.data)['stamp_nanosec']):json.loads(m.data)}),10)
  self.create_subscription(CompressedImage,'/rear_camera/image_raw/compressed',self.capture,10)
  self.estop=self.create_publisher(Bool,'/safety/estop',10);self.stop=self.create_publisher(Empty,'/arm_controller/stop',10)
  self.params=self.create_client(SetParametersAtomically,'/arm_controller/set_parameters_atomically');self.reset=self.create_client(Trigger,'/arm_controller/reset_estop')
  self.cal=json.loads(Path('/opt/armpi-calibration/rear_camera_hand_eye.json').read_text())['tool_T_camera']
 def capture(self,m):
  if len(self.images)<24 and (not self.images or time.monotonic()-self.images[-1]['receive_monotonic']>=3):self.images.append({'phase':self.mode,'receive_monotonic':time.monotonic(),'stamp':[m.header.stamp.sec,m.header.stamp.nanosec],'jpeg_base64':base64.b64encode(bytes(m.data)).decode()})
 def wait(self,pred,seconds):
  end=time.monotonic()+seconds
  while time.monotonic()<end:
   rclpy.spin_once(self,timeout_sec=.04)
   if pred():return
  raise RuntimeError('Wait timed out: '+self.mode)
 def future(self,f):
  self.wait(f.done,4);return f.result()
 def gates(self,arm,grip):
  if not self.params.wait_for_service(timeout_sec=3):raise RuntimeError('Parameter service discovery timeout')
  q=SetParametersAtomically.Request();q.parameters=[Parameter('enable_arm_motion',value=arm).to_parameter_msg(),Parameter('enable_gripper_motion',value=grip).to_parameter_msg()]
  a=self.future(self.params.call_async(q)).result
  if not a.successful:raise RuntimeError('Permissions rejected: '+a.reason)
 def guard(self,active=False):
  a=self.status;now=time.monotonic();stamp=a.get('stamp',{});age=self.get_clock().now().nanoseconds-(stamp.get('sec',0)*10**9+stamp.get('nanosec',0))
  if not 0<=age<1.5e9 or a.get('dry_run') or not a.get('feedback_complete') or not a.get('feedback_real') or a.get('fault'):raise RuntimeError('Invalid actual arm feedback')
  if self.base.get('estop') is not True or self.base.get('wheel_command')!=[0,0,0,0] or self.base.get('hardware_fault'):raise RuntimeError('Base lock missing')
  for name in CENTRE:
   j=a['joints'][name]
   if not 0<=now-j.get('position_time',0)<1.5 or not 0<=now-j.get('temperature_time',0)<1.5 or j['temperature']>=(40 if name=='r_joint' else 50):raise RuntimeError('Joint freshness/temperature: '+name)
  if active and (a.get('estop') or not a.get('enable_arm_motion') or not a.get('enable_gripper_motion')):raise RuntimeError('Motion permission lost')
 def clearance(self,raw):
  if self.floor is None:return
  if hasattr(self,'recovery_minimum_z'):
   if fk(raw)[2]<self.recovery_minimum_z or math.hypot(*fk(raw)[:2])>.35:raise RuntimeError('Upward recovery starting envelope')
   return
  q1=math.radians(.24*(raw['joint2']-500));q2=q1-math.radians(.24*(raw['joint3']-500))
  zs=[BASE_H,BASE_H+L1*math.cos(q1),BASE_H+L1*math.cos(q1)+L2*math.cos(q2)]
  if fk(raw)[2]<self.floor+.010 or min(zs)<self.floor+.012 or math.hypot(*fk(raw)[:2])>.35:raise RuntimeError('Model clearance bound')
 def send(self,targets,duration=2.0,tries=2,tolerance=14,contact=False,desired=None):
  for attempt in range(tries):
   self.guard(True);before=self.raw();grip=list(targets)==['r_joint'];delta=max(abs(before[k]-v) for k,v in targets.items())
   if delta>(70 if grip else 40):raise RuntimeError('Measured step bound')
   for i in range(21):
    rr=dict(before);rr.update({k:before[k]+(v-before[k])*i/20 for k,v in targets.items()});self.clearance(rr)
   msg=JointTrajectory();msg.joint_names=list(targets);p=JointTrajectoryPoint();p.positions=[to_rad(k,v) for k,v in targets.items()];p.time_from_start.sec=int(duration);p.time_from_start.nanosec=round((duration-int(duration))*1e9);msg.points=[p];msg.header.stamp=self.get_clock().now().to_msg();key=(msg.header.stamp.sec,msg.header.stamp.nanosec)
   self.traj.publish(msg);REPORT['gripper_commands' if grip else 'movement_commands']+=1;sent=time.monotonic();temps={k:self.temp(k) for k in CENTRE};self.wait(lambda:key in self.acks,2);ack=self.acks[key]
   if not ack.get('accepted'):raise RuntimeError('Motion rejected: '+ack.get('reason',''))
   until=sent+duration+1.0
   while time.monotonic()<until:
    rclpy.spin_once(self,timeout_sec=.04);self.guard(True);self.clearance(self.raw())
    if any(self.temp(k)-temps[k]>=6 for k in CENTRE):raise RuntimeError('Single-command temperature rise')
   now=self.raw()
   if any(self.status['joints'][k]['position_time']<=sent for k in targets):raise RuntimeError('No post-command encoder sample')
   expected=desired if desired is not None else targets
   err=max(abs(now[k]-v) for k,v in expected.items());RUN_EVENTS.append({'phase':self.mode,'targets':targets,'desired_raw':expected,'actual':now,'ack':ack,'error_ticks':err,'attempt':attempt+1})
   self.after_ns=self.get_clock().now().nanoseconds
   if err<=tolerance:return True,attempt+1,err
   if grip and targets['r_joint']<before['r_joint'] and 8<=before['r_joint']-now['r_joint']<=70 and err<=25:
    RUN_EVENTS.append({'phase':self.mode,'decision':'opening_progress_only','residual_ticks':err,'precision_validated':False});return True,attempt+1,err
   if grip and contact and now['r_joint']<targets['r_joint']-22:return False,attempt+1,err
   if grip and contact and 8<=now['r_joint']-before['r_joint']<=70 and err<=22:
    RUN_EVENTS.append({'phase':self.mode,'decision':'closing_progress_only','residual_ticks':err,'contact_verified':False});return True,attempt+1,err
  raise RuntimeError('Actual arrival failed; no subsequent motion')
 def move_tool(self,target,pitch_pref=(-152,-148,-144,-140,-136)):
  solutions=[]
  for pitch in pitch_pref:
   solutions=ik(*target,pitch)
   if solutions:break
  if not solutions:raise RuntimeError('No bounded IK')
  start=self.raw();best=min(solutions,key=lambda s:max(abs(s[k]-start[k]) for k in ARM));steps=max(1,math.ceil(max(abs(best[k]-start[k]) for k in ARM)/25))
  for i in range(1,steps+1):
   desired={k:round(start[k]+(best[k]-start[k])*i/steps) for k in ARM};desired['joint5']=start['joint5'];offsets={'joint1':1,'joint2':1,'joint3':-4,'joint4':6};t={k:v+offsets.get(k,0) for k,v in desired.items()}
   if not 125<=t['joint2']<=875 or any(not 0<=v<=1000 for k,v in t.items()):raise RuntimeError('Compensated native limits')
   self.send(t,desired=desired)
  return 'OK'
 def measure(self,timeout=5):
  self.points=[];eligible=[];end=time.monotonic()+timeout
  while time.monotonic()<end:
   rclpy.spin_once(self,timeout_sec=.04);self.guard(True)
   for p in self.points:
    ns=p.header.stamp.sec*10**9+p.header.stamp.nanosec;key=(p.header.stamp.sec,p.header.stamp.nanosec);d=self.detections.get(key,{});v=d.get('detection') or {};cx=v.get('center_x',-1000);cy=v.get('center_y',-1000);r=v.get('radius_px',1000)
    if not d.get('valid') or not 0<=self.get_clock().now().nanoseconds-ns<.5e9 or ns<=self.after_ns or not (cx-r>=4 and cx+r<=636 and cy-r>=4 and cy+r<=476):continue
    pose=forward_tool_pose_raw(self.raw());x,y,z,w=self.cal['quaternion_xyzw'];c=(p.point.x,p.point.y,p.point.z)
    rot=((1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)),(2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)),(2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)))
    local=[sum(rot[i][j]*c[j] for j in range(3))+self.cal['translation_m'][i] for i in range(3)]
    point=[pose['translation_m'][i]+sum(pose['rotation_matrix'][i][j]*local[j] for j in range(3)) for i in range(3)]
    eligible.append((ns,point));self.after_ns=ns
   self.points=[p for p in self.points[-30:] if p.header.stamp.sec*10**9+p.header.stamp.nanosec>self.after_ns]
   if len(eligible)>=3:
    samples=[p for _,p in eligible[-3:]]
    if max(math.dist(a,b) for a in samples for b in samples)<=.004:
     result=tuple(statistics.median(p[i] for p in samples) for i in range(3));RUN_EVENTS.append({'phase':self.mode,'measured_ball':result,'raw':self.raw()});return result
  raise RuntimeError('No fresh complete stable sphere')
class AlignedTrial(Trial):
 def clearance(self,raw):
  if hasattr(self,'contact_height') and fk(raw)[2]<self.contact_height-.002:raise RuntimeError('Do not descend below operator-confirmed contact pose')
  if math.hypot(*fk(raw)[:2])>.35:raise RuntimeError('Reach envelope')
def aligned_main():
 rclpy.init();n=AlignedTrial();changed=False;signal.signal(signal.SIGALRM,lambda *_:(_ for _ in ()).throw(RuntimeError('Aligned trial105s deadline')));signal.alarm(105);REPORT['scope']='Operator-confirmed physical alignment, no stale sphere-depth target; not autonomous or VLA'
 try:
  n.wait(lambda:bool(n.status) and bool(n.base),6);n.guard();initial=n.raw();REPORT['pre_reset_raw']=initial;n.mode='aligned_reset';changed=True;n.gates(False,False)
  if n.count_publishers('/arm_controller/command')!=1 or n.count_publishers('/gripper_controller/command')!=0 or n.count_publishers('/cmd_vel')!=0:raise RuntimeError('Other motion source')
  if not n.reset.wait_for_service(timeout_sec=3):raise RuntimeError('Reset unavailable')
  REPORT['reset_attempts']=[]
  for _ in range(3):
   n.wait(lambda:n.status.get('reset_eligible') and not n.status.get('poll_pending'),15);a=n.future(n.reset.call_async(Trigger.Request()));REPORT['reset_attempts'].append({'success':a.success,'message':a.message})
   if a.success:break
   n.spin_for(.35)
  if not a.success:raise RuntimeError('Reset refused')
  n.gates(True,True);n.wait(lambda:not n.status.get('estop') and n.status.get('enable_arm_motion') and n.status.get('enable_gripper_motion'),3)
  if any(abs(n.raw()[k]-initial[k])>5 for k in ARM):raise RuntimeError('Alignment shifted during reset; operator observation no longer current')
  n.mode='close';contact=False
  for _ in range(7):
   if n.raw()['r_joint']>=416:break
   target=min(430,n.raw()['r_joint']+65);ok,_,err=n.send({'r_joint':target},duration=1.5,tries=1,contact=True)
   if any(abs(n.raw()[k]-initial[k])>8 for k in ARM):raise RuntimeError('Arm shifted during stationary closure')
   if not ok:contact=True;break
  REPORT['contact_candidate']=contact
  if not contact:raise RuntimeError('No contact candidate; no lift')
  before=n.raw();tcp=fk(before);pitch=.24*((before['joint2']-500)-(before['joint3']-500)+(before['joint4']-500));n.contact_height=tcp[2];REPORT['contact_raw']=before;REPORT['contact_tcp']=tcp
  n.mode='lift';n.move_tool((tcp[0],tcp[1],tcp[2]+.025),pitch_pref=(pitch,));actual=fk(n.raw());height=actual[2]-tcp[2];REPORT['measured_lift_height_m']=height
  if not .015<=height<=.040:raise RuntimeError('Measured lift height gate')
  n.mode='hold';began=time.monotonic()
  while time.monotonic()-began<5:
   rclpy.spin_once(n,timeout_sec=.04);n.guard(True)
   if fk(n.raw())[2]-tcp[2]<.015:raise RuntimeError('Lift not maintained')
  REPORT['hold_seconds']=time.monotonic()-began;REPORT['physical_grasp_verdict']='pending operator observation'
  n.mode='return';n.move_tool(tcp,pitch_pref=(pitch,))
  if math.dist(fk(n.raw()),tcp)>.010:raise RuntimeError('Return pose not close enough for controlled release')
  n.mode='release'
  while n.raw()['r_joint']>185:n.send({'r_joint':max(145,n.raw()['r_joint']-65)},duration=1.5)
  REPORT['sequence_completed']=True
 except Exception as e:REPORT.update(error=repr(e),failure_status=json.loads(json.dumps(n.status)),failure_monotonic=time.monotonic())
 finally:
  signal.alarm(0);REPORT['last_phase']=n.mode
  if changed:
   n.stop.publish(Empty())
   try:n.gates(False,False)
   except Exception as e:REPORT['cleanup_error']=repr(e)
   n.estop.publish(Bool(data=True));n.spin_for(1);REPORT['cleanup_confirmed']=n.status.get('estop') is True and not n.status.get('enable_arm_motion') and not n.status.get('enable_gripper_motion') and n.base.get('estop') is True
  REPORT.update(events=RUN_EVENTS,images=n.images,final_status=n.status,final_base=n.base);n.destroy_node();rclpy.shutdown();print(json.dumps(REPORT))
aligned_main()
