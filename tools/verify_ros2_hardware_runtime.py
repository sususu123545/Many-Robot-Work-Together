"""ROS2 core+arm isolated runtime validation: NO devices and NO host network."""
import argparse
import re
import shlex
import sys
import uuid
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ros2_preflight import connect
from stage_ros2 import execute
from bench_ros2_forward import ROS2_SETUP, run_script

PROBE = '''
import json,time,serial,rclpy,tf2_ros
from rclpy.node import Node
from std_msgs.msg import String,Int32MultiArray,Bool
from sensor_msgs.msg import JointState
rclpy.init();n=Node('isolated_core_arm_readiness')
arm=[];base=[];enc=[];joints=[]
b=tf2_ros.Buffer();listener=tf2_ros.TransformListener(b,n)
n.create_subscription(String,'/arm_controller/status',lambda m:arm.append(json.loads(m.data)),10)
n.create_subscription(String,'/chassis_control/status',lambda m:base.append(json.loads(m.data)),10)
n.create_subscription(Int32MultiArray,'/chassis_control/encoder_counts',lambda m:enc.append(list(m.data)),10)
n.create_subscription(JointState,'/joint_states',lambda m:joints.append(m),10)
deadline=time.monotonic()+15
while time.monotonic()<deadline and not(len(arm)>2 and len(base)>1 and len(enc)>20):rclpy.spin_once(n,timeout_sec=.05)
assert arm and base and len(enc)>20
assert all(s['dry_run'] and not s['enable_arm_motion'] and not s['feedback_real'] and not s['joints'] for s in arm)
assert all(not s['enable_gripper_motion'] and not s['feedback_complete'] and not s['reset_eligible'] for s in arm)
assert not joints,'Dry arm must not publish fabricated joint positions'
assert all(s['dry_run'] and not s['enable_motion'] and s['wheel_command']==[0,0,0,0] for s in base)
assert all(e==[0,0,0,0] for e in enc)
t=b.lookup_transform('base_link','base_laser',rclpy.time.Time()).transform.translation
assert abs(t.x+.01)<1e-9 and abs(t.y-.01)<1e-9
nodes=[name for name,ns in n.get_node_names_and_namespaces()]
assert all(name in nodes for name in ['chassis_control','mecanum_odom','arm_controller','rosbridge_websocket'])
pub=n.create_publisher(Bool,'/safety/estop',10)
deadline=time.monotonic()+3
while pub.get_subscription_count()<2 and time.monotonic()<deadline:rclpy.spin_once(n,timeout_sec=.05)
assert pub.get_subscription_count()>=2
pub.publish(Bool(data=True))
deadline=time.monotonic()+2
while time.monotonic()<deadline:rclpy.spin_once(n,timeout_sec=.05)
assert arm[-1]['estop'] and base[-1]['estop']
pub.publish(Bool(data=False))
deadline=time.monotonic()+2
while time.monotonic()<deadline:rclpy.spin_once(n,timeout_sec=.05)
assert arm[-1]['estop'] and base[-1]['estop'],'False must not clear emergency'
from rcl_interfaces.srv import SetParametersAtomically
from rcl_interfaces.msg import Parameter,ParameterValue
from std_srvs.srv import Trigger
client=n.create_client(SetParametersAtomically,'/arm_controller/set_parameters_atomically')
assert client.wait_for_service(timeout_sec=3)
request=SetParametersAtomically.Request(parameters=[Parameter(name='enable_gripper_motion',value=ParameterValue(type=1,bool_value=True))])
future=client.call_async(request);rclpy.spin_until_future_complete(n,future,timeout_sec=3)
assert future.done() and future.result() and not future.result().result.successful,'Dry-run gripper grant must be rejected'
reset=n.create_client(Trigger,'/arm_controller/reset_estop');assert reset.wait_for_service(timeout_sec=3)
future=reset.call_async(Trigger.Request());rclpy.spin_until_future_complete(n,future,timeout_sec=3)
assert future.done() and future.result() and not future.result().success,'No physical feedback must not reset latch'
print(json.dumps({'ROS2_runtime_ready':True,'dry_run':True,'no_devices':True,'no_fake_joint_feedback':True,'arm_gripper_native_interfaces':True,'estop_shared_and_latched':True,'measured_lidar_tf':[-.01,.01,.18],'hardware_control_not_yet_validated':True}))
n.destroy_node();rclpy.shutdown()
'''


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--host',required=True)
    p.add_argument('--config',required=True)
    p.add_argument('--image',required=True)
    p.add_argument('--expected-image-id',required=True)
    args=p.parse_args()
    if not re.fullmatch(r'armpi-ros2-migration:stage-[a-z0-9]+',args.image):p.error('Unexpected image')
    if not re.fullmatch(r'sha256:[0-9a-f]{64}',args.expected_image_id):p.error('Unexpected image ID')
    sys.stdout.reconfigure(encoding='utf-8')
    c=connect(args.host,config_path=args.config)
    name='armpi_ros2_core_verify_'+uuid.uuid4().hex[:12]
    created=False
    try:
        assert execute(c,'docker image inspect '+shlex.quote(args.image)+" --format '{{.Id}}'")==args.expected_image_id
        command=ROS2_SETUP+'exec ros2 launch armpi_ros2 base.launch.py arm:=true dry_run:=true enable_motion:=false enable_arm_motion:=false'
        created=True
        print(execute(c,'docker run -d --name '+name+' --network none '+shlex.quote(args.image)+' bash -lc '+shlex.quote(command)),flush=True)
        run_script(c,name,ROS2_SETUP,PROBE)
    finally:
        if created:
            print(execute(c,'docker logs --tail 35 '+name),flush=True)
            print(execute(c,'docker stop -t 10 '+name),flush=True)
        c.close()


if __name__=='__main__':main()
