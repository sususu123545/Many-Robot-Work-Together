"""Bounded read-only native arm diagnosis; never reset, authorize or move."""
import argparse
import json
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from ros2_preflight import connect
from stage_ros2 import execute
from bench_ros2_forward import ROS2_SETUP,run_script

WATCH='''
import json,time,rclpy
from rclpy.node import Node
from std_msgs.msg import String,Int32MultiArray
rclpy.init();n=Node('readonly_arm_diagnostics');rows=[];base=[];enc=[]
n.create_subscription(String,'/arm_controller/status',lambda m:rows.append(json.loads(m.data)),10)
n.create_subscription(String,'/chassis_control/status',lambda m:base.append(json.loads(m.data)),10)
n.create_subscription(Int32MultiArray,'/chassis_control/encoder_counts',lambda m:enc.append(list(m.data)),10)
end=time.monotonic()+SECONDS
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.04)
assert rows and base and enc,'Missing native status'
assert all(not s['enable_arm_motion'] and not s['enable_gripper_motion'] and s['estop'] for s in rows),'Motion gate changed during diagnosis'
assert all(not s['enable_motion'] and s['wheel_command']==[0,0,0,0] for s in base)
assert all(e==enc[0] for e in enc),'Physical wheels changed'
changes=[];last=None
for s in rows:
 key=(s['health_state'],s['fault_count'])
 if key!=last:changes.append({'state':s['health_state'],'fault_count':s['fault_count'],'last_fault':s['last_fault']});last=key
s=rows[-1];t=s['transport']
print(json.dumps({'observed_seconds':SECONDS,'status_samples':len(rows),'motion_disabled':True,
 'state_changes':changes,'latest_state':s['health_state'],'current_read_error':s['current_read_error'],
 'healthy_seconds':s['healthy_seconds'],'reset_eligible':s['reset_eligible'],'stop_reasons':s['stop_reasons'],
 'feedback_complete':s['feedback_complete'],'fault_count':s['fault_count'],'last_fault':s['last_fault'],
 'positions':{k:v['raw'] for k,v in s['joints'].items()},'temperatures':{k:v['temperature'] for k,v in s['joints'].items()},
 'counters':t['counters'],'last_failed_query':t['last_failed_query'],'last_failure_events':t['last_failure_events'],
 'encoder_counts':enc[-1],'no_motion_or_reset_command_sent':True}))
n.destroy_node();rclpy.shutdown()
'''

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--host',required=True);p.add_argument('--config',required=True)
 p.add_argument('--seconds',type=int,default=20);a=p.parse_args()
 if not 5<=a.seconds<=25:p.error('Use a bounded 5..25 second read-only window')
 sys.stdout.reconfigure(encoding='utf-8');c=connect(a.host,config_path=a.config)
 try:
  r=json.loads(execute(c,'sudo -n cat /run/armpi-ros2/container.json'))
  run_script(c,r['name'],ROS2_SETUP,'SECONDS='+str(a.seconds)+'\n'+WATCH)
 finally:c.close()

if __name__=='__main__':main()
