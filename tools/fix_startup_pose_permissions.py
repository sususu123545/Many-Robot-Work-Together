"""Back up/deploy pose cleanup, then revoke gates only. Never start a pose."""
import argparse
import hashlib
import json
from pathlib import Path
import shlex
import subprocess
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ros2_preflight import connect
from install_ros2_only_boot import execute

ROOT=Path(__file__).resolve().parents[1]
FILES=('arm_startup_pose_ros.py','arm_startup_pose.py')
PROBE=r'''
import rclpy,json,time
from std_msgs.msg import String
rclpy.init();n=rclpy.create_node('pose_permissions_readonly_probe');seen=[]
n.create_subscription(String,'/arm_controller/status',lambda m:seen.append(json.loads(m.data)),10)
end=time.monotonic()+8
while time.monotonic()<end and len(seen)<3:rclpy.spin_once(n,timeout_sec=.05)
assert len(seen)>=3,'No fresh arm status'
s=seen[-1]
print('ARM_PROBE='+json.dumps({k:s.get(k) for k in ['enable_arm_motion','enable_gripper_motion','estop','health_state','fault_count','current_read_error']} | {'raw':{k:v['raw'] for k,v in s['joints'].items()}}))
n.destroy_node();rclpy.shutdown()
'''


def run_source(client,name,source,revoke=False):
    env=' ARMPI_POSE_REVOKE_ONLY=1' if revoke else ''
    command=('source /opt/armpi_ros2/install/setup.bash && '
             'export ROS_DOMAIN_ID=42 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST'+env+
             ' && python3 -u -c '+shlex.quote(source))
    return execute(client,'docker exec '+shlex.quote(name)+' bash -lc '+shlex.quote(command),seconds=30)


def probe(client,name):
    output=run_source(client,name,PROBE)
    return json.loads(next(line.split('=',1)[1] for line in output.splitlines() if line.startswith('ARM_PROBE=')))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--host',required=True);p.add_argument('--config',required=True)
    args=p.parse_args();sys.stdout.reconfigure(encoding='utf-8')
    c=connect(args.host,config_path=args.config)
    try:
        assert execute(c,'systemctl show armpi-startup-pose -p ActiveState --value') in ('inactive','failed'), 'Pose service not idle'
        assert execute(c,'systemctl show armpi-ros2 -p ActiveState --value')=='active'
        runtime=json.loads(execute(c,'sudo -n cat /run/armpi-ros2/container.json'));name=runtime['name']
        assert name.startswith('armpi_ros2_native_') and name.replace('_','').isalnum()
        assert execute(c,'docker inspect '+name+" --format '{{.State.Running}}'")=='true'
        before=probe(c,name);print('BEFORE='+json.dumps(before),flush=True)
        assert before['estop'] or (before['enable_arm_motion'] is False and before['enable_gripper_motion'] is False), 'Unexpected active authorization; do not interrupt an operator'
        # Refuse remote changes not present in the checked-out baseline or this fix.
        sources={};hashes={}
        for filename in FILES:
            relative='car_setup/ros2/'+filename
            source=(ROOT/relative).read_bytes();sources[filename]=source
            baseline=subprocess.check_output(['git','show','HEAD:'+relative],cwd=ROOT)
            current=execute(c,'sudo -n sha256sum /opt/armpi-ros2/'+filename).split()[0]
            expected=hashlib.sha256(source).hexdigest();hashes[filename]=expected
            assert current in (expected,hashlib.sha256(baseline).hexdigest()), 'Unrecognized remote changes: '+filename
        stage=execute(c,'mktemp -d /home/pi/armpi-pose-permissions.XXXXXX')
        assert stage.startswith('/home/pi/armpi-pose-permissions.') and stage.replace('/','').replace('-','').replace('.','').isalnum()
        with c.open_sftp() as sftp:
            sftp.get_channel().settimeout(15)
            for filename in FILES:
                staged=stage+'/'+filename+'.new'
                sftp.put(str(ROOT/'car_setup/ros2'/filename),staged)
                assert execute(c,'sha256sum '+staged).split()[0]==hashes[filename]
                execute(c,'sudo -n cp -p /opt/armpi-ros2/'+filename+' '+stage+'/'+filename+'.previous')
        print('BACKUP='+stage,flush=True)
        # No systemd start/restart, daemon reload or image change is needed.
        for filename in FILES:
            mode='755' if filename=='arm_startup_pose.py' else '644'
            execute(c,'sudo -n install -m '+mode+' '+stage+'/'+filename+'.new /opt/armpi-ros2/'+filename)
            assert execute(c,'sudo -n sha256sum /opt/armpi-ros2/'+filename).split()[0]==hashes[filename]
        print('DEPLOYED='+json.dumps(hashes),flush=True)
        assert execute(c,'systemctl show armpi-startup-pose -p ActiveState --value') in ('inactive','failed')
        # Execute ONLY the tested false-only maintenance/finally branch. This
        # is not the startup service and never enters reset/authorize/travel.
        output=run_source(c,name,sources['arm_startup_pose_ros.py'].decode('utf-8'),revoke=True)
        print(output,flush=True)
        cleanup=json.loads(next(line.split('=',1)[1] for line in output.splitlines() if line.startswith('CLEANUP_JSON=')))
        assert cleanup['confirmed']
        after=probe(c,name);print('AFTER='+json.dumps(after),flush=True)
        assert after['enable_arm_motion'] is False and after['enable_gripper_motion'] is False
        if before['estop']:assert after['estop'],'Emergency unexpectedly cleared'
        assert json.loads(execute(c,'sudo -n cat /run/armpi-ros2/container.json'))['name']==name,'Core changed during deployment'
        print('POSE_PERMISSION_FIX=DEPLOYED_AND_OFF_CONFIRMED; NO_POSE_RUN',flush=True)
    finally:c.close()


if __name__=='__main__':main()
