"""One explicitly confirmed, bounded ROS1 ground pulse; no configuration edits.

An independent robot-side process stops on a 300 ms heartbeat expiry, pipe EOF,
or a five-second hard deadline. It cannot replace a physical emergency stop.
Requires explicit confirmation of the observed self echoes. Temporary narrow
echo exemptions apply only to this pulse, not to any production safety node.
"""
import argparse
import ast
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ros2_preflight import connect
from bench_ros2_forward import ROS1_SETUP, run_script


GUARD = r'''
import select,sys,time,rospy,rosgraph
from std_msgs.msg import Empty
rospy.init_node('imu_ground_stop_guard',anonymous=True,disable_signals=True)
p=rospy.Publisher('/chassis_control/stop',Empty,queue_size=10)
deadline=time.monotonic()+3
while p.get_num_connections()!=2 and time.monotonic()<deadline:time.sleep(.02)
assert p.get_num_connections()==2,'Missing chassis or observer stop subscriber'
_,subs,_=rosgraph.Master(rospy.get_name()).getSystemState()
nodes=[ns for t,ns in subs if t=='/chassis_control/stop']
assert len(nodes)==1 and len(nodes[0])==2 and '/chassis_control' in nodes[0]
assert any(n.startswith('/imu_ground_direction_probe_') for n in nodes[0])
print('GUARD_READY',flush=True)
hard=time.monotonic()+5
lease=time.monotonic()+.3
try:
 while time.monotonic()<min(hard,lease):
  ready,_,_=select.select([sys.stdin],[],[],.02)
  if ready:
   line=sys.stdin.buffer.readline()
   if line!=b'TICK\n':break
   lease=time.monotonic()+.3
finally:
 for _ in range(15):p.publish(Empty());time.sleep(.03)
 print('GUARD_STOP_SENT',flush=True)
'''

PROBE = r'''
import json,math,statistics,threading,time,subprocess,select,sys,rospy,rosgraph
from sensor_msgs.msg import Imu,LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty,String,Int32MultiArray
from chassis_control.msg import SetVelocity,SetTranslation

rospy.init_node('imu_ground_direction_probe',anonymous=True,disable_signals=True)
lock=threading.RLock()
rows={'imu':[],'enc':[],'scan':[],'odom':[],'mapping':[],'wall':[]}
foreign=[]
def store(m,key):
 with lock:rows[key].append((time.monotonic(),m))
def observe(m):
 caller=m._connection_header.get('callerid','')
 # A guard stop is an abort too: never resume motion after its lease expires.
 if caller != rospy.get_name():
  with lock:foreign.append({'caller':caller,'time':time.monotonic()})
subs=[rospy.Subscriber(topic,typ,store,callback_args=key,queue_size=500)
 for topic,typ,key in [('/bmi270/imu_raw',Imu,'imu'),('/scan',LaserScan,'scan'),
 ('/chassis_control/encoder_counts',Int32MultiArray,'enc'),('/odom',Odometry,'odom'),
 ('/auto_mapping/status',String,'mapping'),('/wall_follow/status',String,'wall')]]
for topic,typ in [('/chassis_control/set_velocity',SetVelocity),
                  ('/chassis_control/set_translation',SetTranslation),('/chassis_control/stop',Empty)]:
 subs.append(rospy.Subscriber(topic,typ,observe,queue_size=100))
p=rospy.Publisher('/chassis_control/set_velocity',SetVelocity,queue_size=1)
stop=rospy.Publisher('/chassis_control/stop',Empty,queue_size=10)
guard=None;started=False;events={};result={}
METRES_PER_PULSE=math.pi*.0965/7832
ROTATING=DIRECTION in ('ccw','cw')
def latest(key):
 with lock:
  assert rows[key],'Missing '+key
  t,m=rows[key][-1]
 assert time.monotonic()-t<{'imu':.15,'scan':.3,'enc':.2,'odom':.2,'mapping':.7,'wall':1.1}[key],key+' stale'
 return m
def halt():
 for _ in range(5):stop.publish(Empty());time.sleep(.03)
def counts():return list(latest('enc').data)
def delta(a,b):return [((x-y+2**31)%2**32)-2**31 for x,y in zip(a,b)]
def travel(d):
 m=[x*METRES_PER_PULSE for x in d]
 return ((m[0]-m[1]-m[2]+m[3])/4,sum(m)/4,(m[0]-m[1]+m[2]-m[3])/(4*.2075))
def readiness(moving=False):
 for key in rows:latest(key)
 state=json.loads(latest('mapping').data);wall=json.loads(latest('wall').data)
 assert state['state']=='IDLE' and not state['slam_running'],'Autonomy active'
 assert wall['state']=='IDLE','Wall follower active'
 assert not foreign,'Other controller issued a command'
 imu=latest('imu')
 vals=[getattr(imu.linear_acceleration,k) for k in 'xyz']+[getattr(imu.angular_velocity,k) for k in 'xyz']
 assert all(math.isfinite(x) for x in vals),'Nonfinite IMU'
 assert max(abs(x) for x in vals[3:])<.12,'Unexpected rotation: gyro_rad_s='+str(vals[3:])
 scan=latest('scan')
 if ROTATING:
  sectors=[0]*8;valid=[];blocked=[]
  for i,r in enumerate(scan.ranges):
   if not (math.isfinite(r) and scan.range_min<=r<=scan.range_max):continue
   a=scan.angle_min+i*scan.angle_increment
   deg=math.degrees(math.atan2(math.sin(a),math.cos(a)))
   self_echo=(((-135<=deg<-75) and .06<=r<=.20)
              or ((75<=deg<90) and .06<=r<=.10))
   if self_echo:continue
   sectors[min(7,int((deg+180)/45))]+=1
   valid.append(r)
   if r<.35:blocked.append((round(deg,1),round(r,3)))
  assert min(sectors)>=6,'Insufficient surrounding scan coverage'
  assert not blocked,'Rotation clearance blocked '+str(blocked[:8])
  return {'rotation_min_m':round(min(valid),3),'valid_by_sector':sectors}
 # 20 cm corridor half-width includes margin over the 12.5 cm body half-width.
 # Only the user's specifically confirmed current self echoes are exempted.
 # User confirmed all current sub-20-cm returns are self echoes. Limit the
 # exemption to the observed rear-side and side angles; never persist it.
 clear={}
 for direction in (1,-1):
  rays=[];blocked=[]
  for i,r in enumerate(scan.ranges):
   a=scan.angle_min+i*scan.angle_increment
   x=math.cos(a)*direction
   if x>math.cos(math.radians(30)) and math.isfinite(r) and scan.range_min<=r<=scan.range_max:rays.append(r)
   if math.isfinite(r) and scan.range_min<=r<=scan.range_max:
    deg=math.degrees(math.atan2(math.sin(a),math.cos(a)))
    self_echo=(((-135<=deg<-75) and .06<=r<=.20)
               or ((75<=deg<90) and .06<=r<=.10))
    if self_echo:continue
    longitudinal=r*x;lateral=r*math.sin(a)
    if 0<longitudinal<.36 and abs(lateral)<.20:blocked.append((round(math.degrees(a),1),round(r,3)))
  assert len(rays)>=12,'Insufficient longitudinal scan coverage'
  assert not blocked,'Swept corridor blocked '+str(direction)+' '+str(blocked[:8])
  assert min(rays)>.40,'Insufficient straight-ahead clearance'
  clear[str(direction)]=round(min(rays),3)
 return clear
try:
 deadline=time.monotonic()+5
 while (any(not rows[k] for k in rows) or p.get_num_connections()!=2 or stop.get_num_connections()!=2) and time.monotonic()<deadline:time.sleep(.02)
 assert p.get_num_connections()==stop.get_num_connections()==2,'Missing chassis or observer drive/stop subscriber'
 pub,ss,_=rosgraph.Master(rospy.get_name()).getSystemState()
 for topic in ('/chassis_control/set_velocity','/chassis_control/stop'):
  nodes=[n for t,n in ss if t==topic]
  assert len(nodes)==1 and set(nodes[0])=={'/chassis_control',rospy.get_name()},'Unexpected control subscriber '+str(nodes)
 time.sleep(3)
 clear=readiness()
 c=counts();time.sleep(.5)
 assert max(abs(x) for x in delta(counts(),c))<=2,'Wheels not stationary'
 with lock:base=[(t,m) for t,m in rows['imu']]
 base_acc=[statistics.mean(getattr(m.linear_acceleration,k) for t,m in base) for k in 'xyz']
 base_gyro=[statistics.mean(getattr(m.angular_velocity,k) for t,m in base) for k in 'xyz']
 print(json.dumps({'preflight':'PASS','longitudinal_clear_m':clear,'baseline_accel':base_acc,'baseline_gyro':base_gyro,'motion_sent':False}),flush=True)
 guard=subprocess.Popen([sys.executable,'-u','-c',GUARD],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
 wait=time.monotonic()+4
 while time.monotonic()<wait:
  ready,_,_=select.select([guard.stdout],[],[],.05)
  if ready:
   line=guard.stdout.readline().decode().strip()
   if line=='GUARD_READY':break
  assert guard.poll() is None,'Stop guard failed'
 else:raise RuntimeError('Stop guard not ready')
 # Any external stop (including our guard) aborts; no reset or auto-resume.
 guard.stdin.write(b'TICK\n');guard.stdin.flush()
 c0=counts();events['start']=time.monotonic()
 started=True
 deadline=events['start']+PULSE_DURATION
 while time.monotonic()<deadline:
  assert guard.poll() is None,'Stop guard exited'
  readiness(True)
  dx,dy,da=travel(delta(counts(),c0))
  if ROTATING:
   assert math.hypot(dx,dy)<.006 and abs(da)<.085,'Rotation motion bounds exceeded'
  else:
   assert abs(dx)<.03 and abs(dy)<.006 and abs(da)<.025,'Encoder motion bounds exceeded'
  guard.stdin.write(b'TICK\n');guard.stdin.flush()
  if ROTATING:
   p.publish(SetVelocity(velocity=0,direction=90,angular=ROTATION_SPEED if DIRECTION=='ccw' else -ROTATION_SPEED))
  else:
   p.publish(SetVelocity(velocity=25,direction=90 if DIRECTION=='forward' else 270,angular=0))
  time.sleep(.04)
 events['stop']=time.monotonic()
 halt()
 guard.stdin.close();guard.wait(timeout=2)
 time.sleep(1)
 c1=counts();time.sleep(.5);c2=counts()
 assert max(abs(x) for x in delta(c2,c1))<=2,'Encoders still moving after stop'
 with lock:saved={key:list(value) for key,value in rows.items()}
 def vec(m,field):return [getattr(getattr(m,field),k) for k in 'xyz']
 result={'direction':DIRECTION,'pulse_secs':events['stop']-events['start'],
 'speed_command_mm_s':0 if ROTATING else 25,
 'angular_command_rad_s':(ROTATION_SPEED if DIRECTION=='ccw' else -ROTATION_SPEED) if ROTATING else 0,
 'encoder_delta':delta(c2,c0),'encoder_body_delta_m_m_rad':travel(delta(c2,c0)),
 'encoders_stopped':True,'baseline_accel':base_acc,'baseline_gyro':base_gyro,
 'imu':[{'t':t-events['start'],'a':vec(m,'linear_acceleration'),'g':vec(m,'angular_velocity'),'stamp':m.header.stamp.to_sec()} for t,m in saved['imu']],
 'encoders':[{'t':t-events['start'],'counts':list(m.data)} for t,m in saved['enc']],
 'odom':[{'t':t-events['start'],'xy':[m.pose.pose.position.x,m.pose.pose.position.y],
 'linear':[m.twist.twist.linear.x,m.twist.twist.linear.y],'wz':m.twist.twist.angular.z} for t,m in saved['odom']],
 'physical_direction_requires_user_observation':True,'config_changed':False}
 print('GROUND_IMU_RESULT='+json.dumps(result),flush=True)
except Exception as error:
 # Keep the samples that triggered an abort. Previously an early exit lost
 # the transient readings, so the offending gyro axis could not be diagnosed.
 if started:
  events.setdefault('stop',time.monotonic())
  halt()
  with lock:partial={key:list(value) for key,value in rows.items()}
  moving_imu=[(t,m) for t,m in partial['imu'] if events['start']<=t<=events['stop']]
  partial_result={'reason':str(error),'direction':DIRECTION,
                  'pulse_secs_before_abort':events['stop']-events['start'],
                  'stop_sent':True,'config_changed':False}
  if moving_imu:
   partial_result['gyro_peak_abs_by_axis_rad_s']=[max(abs(getattr(m.angular_velocity,k)) for t,m in moving_imu) for k in 'xyz']
   partial_result['gyro_mean_by_axis_rad_s']=[statistics.mean(getattr(m.angular_velocity,k) for t,m in moving_imu) for k in 'xyz']
   partial_result['last_gyro_samples']=[{'t':t-events['start'],'xyz':[getattr(m.angular_velocity,k) for k in 'xyz']} for t,m in moving_imu[-8:]]
  if partial['enc']:
   d=delta(list(partial['enc'][-1][1].data),c0)
   partial_result['encoder_delta_at_abort']=d
   partial_result['encoder_body_delta_at_abort_m_m_rad']=travel(d)
  print('GROUND_IMU_ABORT='+json.dumps(partial_result),flush=True)
 raise
finally:
 if started:halt()
 if guard:
  if guard.stdin and not guard.stdin.closed:guard.stdin.close()
  try:guard.wait(timeout=2)
  except subprocess.TimeoutExpired:halt()
 for s in subs:s.unregister()
 print('PROBE_FINISHED motion_sent='+str(started),flush=True)
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', required=True)
    parser.add_argument('--config', required=True)
    parser.add_argument('--confirm-clear-ground', action='store_true', required=True)
    parser.add_argument('--confirm-observed-self-echoes', action='store_true', required=True)
    parser.add_argument('--confirm-clear-rotation', action='store_true')
    parser.add_argument('--direction', choices=('forward', 'backward', 'ccw', 'cw'), required=True)
    parser.add_argument('--pulse-duration', type=float, choices=(.4, .6, .8), default=.8)
    parser.add_argument('--rotation-speed', type=float, choices=(.04, .08), default=.08)
    args = parser.parse_args()
    if args.direction in ('ccw', 'cw') and not args.confirm_clear_rotation:
        parser.error('Rotation requires explicit physical swept-envelope clearance confirmation')
    source = ('GUARD=' + repr(GUARD) + '\nDIRECTION=' + repr(args.direction)
              + '\nPULSE_DURATION=' + repr(args.pulse_duration)
              + '\nROTATION_SPEED=' + repr(args.rotation_speed) + '\n' + PROBE)
    ast.parse(source)
    client = connect(args.host, config_path=args.config)
    try:
        import contextlib, io, json, statistics
        output = io.StringIO()
        try:
            with contextlib.redirect_stdout(output):
                run_script(client, 'armpi_pro', ROS1_SETUP, source)
        finally:
            for line in output.getvalue().splitlines():
                if not line.startswith('GROUND_IMU_RESULT='):
                    print(line, flush=True)
        lines = [line for line in output.getvalue().splitlines() if line.startswith('GROUND_IMU_RESULT=')]
        assert len(lines) == 1, 'Missing pulse result'
        result = json.loads(lines[0].split('=', 1)[1])
        imu = result.pop('imu')
        enc = result.pop('encoders')
        odom = result.pop('odom')
        baseline = result['baseline_accel']
        phases = {}
        for label, start, end in [('start', 0, .20), ('moving', .20, .65),
                                  ('stopping', result['pulse_secs'], result['pulse_secs']+.20)]:
            samples = [r for r in imu if start <= r['t'] <= end]
            if samples:
                phases[label] = {
                    'samples': len(samples),
                    'accel_delta_mean': [statistics.mean(r['a'][i]-baseline[i] for r in samples) for i in range(3)],
                    'peak_accel_delta_vector': max(([r['a'][i]-baseline[i] for i in range(3)] for r in samples), key=lambda v: sum(x*x for x in v)),
                }
        result['transient_acceleration'] = phases
        result['sample_counts'] = {'imu': len(imu), 'encoders': len(enc), 'odom': len(odom)}
        result['gyro_max_abs_during_pulse'] = max(abs(x) for r in imu if 0<=r['t']<=result['pulse_secs'] for x in r['g'])
        moving = [r for r in imu if .20 <= r['t'] <= result['pulse_secs']]
        result['gyro_mean_during_motion_rad_s'] = [statistics.mean(r['g'][i] for r in moving) for i in range(3)]
        # Include deceleration, subtract only this run's stationary residual,
        # and integrate with sensor stamps rather than receive-time jitter.
        turning = [r for r in imu if 0 <= r['t'] <= result['pulse_secs']+.50]
        angle = [0.0, 0.0, 0.0]
        for a, b in zip(turning, turning[1:]):
            dt = b['stamp'] - a['stamp']
            assert 0 < dt < .1, 'IMU stamp discontinuity; angle unverified'
            for i in range(3):
                angle[i] += (.5*(a['g'][i]+b['g'][i])-result['baseline_gyro'][i])*dt
        result['gyro_integrated_delta_deg'] = [v*180/3.141592653589793 for v in angle]
        result['encoder_delta_deg'] = result['encoder_body_delta_m_m_rad'][2]*180/3.141592653589793
        result['odom_reported_linear_during_pulse'] = [r['linear'] for r in odom if .25<=r['t']<=.60]
        result['temporary_self_echo_exemptions'] = 'Only confirmed angle/range bands; no production configuration changed'
        print(json.dumps(result, indent=2), flush=True)
    finally:
        client.close()


if __name__ == '__main__':
    main()
