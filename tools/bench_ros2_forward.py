"""One confirmed wheel-off-ground ROS 2 test, with ROS 1 restoration in finally.

No arm device, no sensors, no network exposure, no startup configuration changes.
Back up configurations before using; one explicitly selected, bounded axis pulse.
"""
import argparse
import json
import re
import shlex
import sys
import uuid
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ros2_preflight import connect
from stage_ros2 import execute

ROS1_SETUP = ('source /opt/ros/noetic/setup.bash && '
              'source /home/ubuntu/armpi_pro/devel/setup.bash && '
              'export ROS_MASTER_URI=http://raspberrypi:11311 && ')
ROS2_SETUP = 'source /opt/armpi_ros2/install/setup.bash && '
IMAGE = 'armpi-ros2-migration:stage-encoders1'
IMAGE_ID = 'sha256:c06d04049a9650aef1044e5811887a4fb6e459871ae7bbedc8cba9b65addca08'

READINESS = '''
import json, math, rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
rospy.init_node('migration_bench_readiness', anonymous=True)
state=json.loads(rospy.wait_for_message('/auto_mapping/status',String,timeout=5).data)
o=rospy.wait_for_message('/odom',Odometry,timeout=5)
p=o.pose.pose.position; q=o.pose.pose.orientation; v=o.twist.twist
assert state['state']=='IDLE' and state['slam_running'] is False, 'An autonomous task is active'
# A restored controller can settle by 1-2 counts. Only a confirmed bench,
# IDLE + no SLAM, sub-0.5-mm / sub-0.002-rad startup pose may be discarded.
# A nontrivial pose still requires preserving state, not relaxing this guard.
yaw=2*math.atan2(q.z,q.w)
assert math.hypot(p.x,p.y)<.0005 and abs(yaw)<.002 and abs(q.w-1)<1e-5, 'Refuse to discard a meaningful ROS1 odometry pose'
assert abs(v.linear.x)+abs(v.linear.y)+abs(v.angular.z)<1e-6
print(json.dumps({'mapping_state':state['state'],'odom_pose_near_zero':True,
                  'old_pose':[p.x,p.y,yaw]}))
'''

STOP_OLD = '''
import json,time,rospy,rosnode
from std_msgs.msg import Empty,Int32MultiArray
rospy.init_node('migration_bench_stop_old',anonymous=True)
p=rospy.Publisher('/chassis_control/stop',Empty,queue_size=1)
deadline=time.monotonic()+5
while p.get_num_connections()<1 and time.monotonic()<deadline:
    time.sleep(.05)
assert p.get_num_connections()>=1, 'Old stop topic is not connected'
for _ in range(3):
    p.publish(Empty()); time.sleep(.1)
a=rospy.wait_for_message('/chassis_control/encoder_counts',Int32MultiArray,timeout=3)
time.sleep(.4)
b=rospy.wait_for_message('/chassis_control/encoder_counts',Int32MultiArray,timeout=3)
assert list(a.data)==list(b.data), 'Old wheels did not stop'
succeeded,failed=rosnode.kill_nodes(['/chassis_control'])
assert succeeded==['/chassis_control'] and not failed
print(json.dumps({'old_chassis_stopped':True,'counts':list(b.data)}))
'''

OLD_PROCESSES = '''
import pathlib,json
found=[]
for p in pathlib.Path('/proc').glob('[0-9]*/cmdline'):
    try:
        data=p.read_bytes().split(b'\\x00')
        if any(x.endswith(b'/chassis_control_node.py') for x in data):
            found.append(int(p.parent.name))
    except (OSError,ProcessLookupError):
        pass
print(json.dumps(found))
'''


def run_script(client, container, setup, source, args=''):
    command = ('docker exec -i ' + shlex.quote(container) + ' bash -lc ' +
               shlex.quote(setup + 'python3 - ' + args))
    stdin, out, err = client.exec_command(command, timeout=35)
    stdin.write(source)
    stdin.flush()
    stdin.channel.shutdown_write()
    output, errors = out.read().decode(), err.read().decode()
    if output:
        print(output, flush=True)
    if out.channel.recv_exit_status() != 0:
        raise RuntimeError(errors or output)


def old_processes(client):
    return json.loads(execute(client, 'docker exec armpi_pro python3 -c ' + shlex.quote(OLD_PROCESSES)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', required=True)
    parser.add_argument('--config', required=True)
    parser.add_argument('--confirm-wheels-off-ground', action='store_true', required=True)
    parser.add_argument('--direction', choices=('forward', 'left', 'ccw'), default='forward')
    parser.add_argument('--calibration-only', action='store_true')
    parser.add_argument('--image', default=IMAGE)
    parser.add_argument('--expected-image-id', default=IMAGE_ID)
    args = parser.parse_args()
    if (not re.fullmatch(r'armpi-ros2-migration:stage-[a-zA-Z0-9]+', args.image)
            or not re.fullmatch(r'sha256:[0-9a-f]{64}', args.expected_image_id)):
        parser.error('Use the exact migration image tag and its verified image ID')
    client = connect(args.host, config_path=args.config)
    container = 'armpi_ros2_bench_' + uuid.uuid4().hex[:12]
    created, old_may_be_stopped, odom_may_be_stopped = False, False, False
    restore_allowed = True
    try:
        assert execute(client, 'docker image inspect ' + shlex.quote(args.image) +
                       " --format '{{.Id}}'") == args.expected_image_id
        assert len(old_processes(client)) == 1, 'Expected exactly one old chassis process'
        assert execute(client, 'systemctl is-active odom') == 'active'
        run_script(client, 'armpi_pro', ROS1_SETUP, READINESS)
        odom_may_be_stopped = True
        execute(client, 'sudo -n systemctl stop odom')
        old_may_be_stopped = True
        run_script(client, 'armpi_pro', ROS1_SETUP, STOP_OLD)
        assert old_processes(client) == [], 'Old I2C owner still running; refuse hardware access'
        launch = (ROS2_SETUP + 'exec timeout --signal=INT --kill-after=5 120 '
                  'ros2 launch armpi_ros2 base.launch.py dry_run:=false enable_motion:=true rosbridge:=false')
        created = True  # A command can succeed remotely even if the SSH reply is lost.
        print(execute(client, 'docker run -d --network none --stop-signal SIGINT '
                      '--device /dev/i2c-1:/dev/i2c-1 --name ' + container + ' ' + shlex.quote(args.image) +
                      ' bash -lc ' + shlex.quote(launch)), flush=True)
        print(execute(client, 'docker inspect ' + container +
                      " --format 'Privileged={{.HostConfig.Privileged}} Devices={{json .HostConfig.Devices}} Mounts={{json .Mounts}} Network={{.HostConfig.NetworkMode}}'"), flush=True)
        assert old_processes(client) == [], 'Old driver unexpectedly restarted'
        probe = Path(__file__).resolve().parents[1] / 'ros2_ws' / 'bench_forward_probe.py'
        flags = '--confirm-wheels-off-ground --direction ' + args.direction
        if args.calibration_only:
            flags += ' --calibration-only'
        run_script(client, container, ROS2_SETUP, probe.read_text(encoding='utf-8'), flags)
    finally:
        try:
            if created:
                names = execute(client, "docker ps -a --format '{{.Names}}'").splitlines()
                if container in names:
                    execute(client, 'docker stop -t 15 ' + container)
                    restore_allowed = execute(client, 'docker inspect ' + container + " --format '{{.State.Running}}'") == 'false'
                    print('STOPPED_BENCH_CONTAINER=' + container, flush=True)
            if not restore_allowed:
                raise RuntimeError('New I2C driver still running; DO NOT restore the old owner')
            if old_may_be_stopped and not old_processes(client):
                execute(client, 'docker exec -d armpi_pro bash -lc ' + shlex.quote(
                    ROS1_SETUP + 'exec rosrun chassis_control chassis_control_node.py __name:=chassis_control'))
            if odom_may_be_stopped:
                execute(client, 'sudo -n systemctl start odom')
            if old_may_be_stopped:
                verify = '''
import time,json,rospy
from std_msgs.msg import Empty,Int32MultiArray
from nav_msgs.msg import Odometry
rospy.init_node('migration_bench_restore_check',anonymous=True)
p=rospy.Publisher('/chassis_control/stop',Empty,queue_size=1)
deadline=time.monotonic()+8
while p.get_num_connections()<1 and time.monotonic()<deadline: time.sleep(.05)
assert p.get_num_connections()>=1
for _ in range(3): p.publish(Empty()); time.sleep(.1)
e=rospy.wait_for_message('/chassis_control/encoder_counts',Int32MultiArray,timeout=5)
o=rospy.wait_for_message('/odom',Odometry,timeout=5)
assert o.twist.twist.linear.x==0 and o.twist.twist.linear.y==0 and o.twist.twist.angular.z==0
print(json.dumps({'ROS1_restored':True,'encoder_counts':list(e.data),'odom_velocity_zero':True}))
'''
                run_script(client, 'armpi_pro', ROS1_SETUP, verify)
        finally:
            client.close()


if __name__ == '__main__':
    main()
