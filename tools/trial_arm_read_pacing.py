"""Supported, motion-disabled trial of a pinned read-pacing image; rollback on failure."""
import argparse
import json
from pathlib import Path
import re
import shlex
import sys
import time
import uuid

sys.path.insert(0,str(Path(__file__).resolve().parent))
from ros2_preflight import connect
from install_ros2_only_boot import execute
from fix_startup_pose_permissions import probe,run_source

WATCH=r'''
import rclpy,time,json
from std_msgs.msg import String,Int32MultiArray
rclpy.init();n=rclpy.create_node('pacing_trial_readonly');arm=[];base=[];enc=[]
n.create_subscription(String,'/arm_controller/status',lambda m:arm.append(json.loads(m.data)),10)
n.create_subscription(String,'/chassis_control/status',lambda m:base.append(json.loads(m.data)),10)
n.create_subscription(Int32MultiArray,'/chassis_control/encoder_counts',lambda m:enc.append(list(m.data)),10)
end=time.monotonic()+25
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.03)
assert arm and base and enc,'Telemetry missing'
issues=[]
if any(s['enable_arm_motion'] or s['enable_gripper_motion'] for s in arm):issues.append('Arm gates opened')
if any(s['fault_count'] or s['current_read_error'] or not s['feedback_complete'] for s in arm):issues.append('Arm read/feedback fault')
if any(s['enable_motion'] or s['wheel_command']!=[0,0,0,0] for s in base):issues.append('Base gate/command changed')
if any(v!=enc[0] for v in enc):issues.append('Wheel encoders changed')
if any(s['transport'].get('minimum_read_interval_seconds')!=.05 for s in arm):issues.append('Wrong runtime pacing')
s=arm[-1]
print(json.dumps({'samples':len(arm),'issues':issues,'fault_count':s['fault_count'],
 'state':s['health_state'],'healthy_seconds':s['healthy_seconds'],'last_fault':s['last_fault'],
 'counters':s['transport']['counters'],'max_reported_age':max(max(v['feedback_age_seconds'].values()) for v in arm),
 'raw':{k:v['raw'] for k,v in s['joints'].items()},'temperatures':{k:v['temperature'] for k,v in s['joints'].items()}}),flush=True)
n.destroy_node();rclpy.shutdown()
'''


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('host','config','image','image-id'):p.add_argument('--'+key,required=True)
    p.add_argument('--confirm-supported',action='store_true',required=True)
    p.add_argument('--windows',type=int,default=8)
    a=p.parse_args();sys.stdout.reconfigure(encoding='utf-8')
    assert re.fullmatch(r'armpi-ros2-migration:stage-[a-z0-9]+',a.image)
    assert re.fullmatch(r'sha256:[0-9a-f]{64}',a.image_id) and 2<=a.windows<=12
    c=connect(a.host,config_path=a.config);changed=False;inhibited=False;success=False;stage=None
    token=uuid.uuid4().hex[:10]
    block='/run/systemd/system/armpi-startup-pose.service.d/95-pacing-trial-'+token+'.conf'
    def await_core(old_name):
        end=time.monotonic()+65
        while time.monotonic()<end:
            runtime=json.loads(execute(c,'sudo -n cat /run/armpi-ros2/container.json'))
            if runtime['name']!=old_name:
                try:
                    state=probe(c,runtime['name'])
                    assert state['enable_arm_motion'] is False and state['enable_gripper_motion'] is False
                    return runtime['name']
                except RuntimeError:pass
            time.sleep(2)
        raise RuntimeError('Core recovery not confirmed')
    try:
        assert execute(c,'systemctl show armpi-startup-pose -p ActiveState --value') in ('inactive','failed')
        assert json.loads(execute(c,'sudo -n cat /run/armpi-ros2/arm-bootstrap.json')).get('ok') is True
        runtime=json.loads(execute(c,'sudo -n cat /run/armpi-ros2/container.json'))
        before=probe(c,runtime['name']);assert not before['enable_arm_motion'] and not before['enable_gripper_motion']
        assert execute(c,'docker image inspect '+a.image+" --format '{{.Id}}'")==a.image_id
        previous=json.loads(execute(c,'sudo -n cat /etc/armpi-ros2/core.json'))
        stage=execute(c,'mktemp -d /home/pi/armpi-pacing-trial.XXXXXX')
        assert re.fullmatch(r'/home/pi/armpi-pacing-trial\.[A-Za-z0-9]+',stage)
        execute(c,'sudo -n cp -p /etc/armpi-ros2/core.json '+stage+'/previous-core.json')
        updated=dict(previous,image=a.image,image_id=a.image_id)
        with c.open_sftp() as s:
            s.get_channel().settimeout(15)
            with s.open(stage+'/core.json','w') as f:f.write(json.dumps(updated))
            with s.open(stage+'/inhibit.conf','w') as f:f.write('[Unit]\nConditionPathExists='+stage+'/POSE_NOT_AUTHORIZED\n')
        execute(c,'test ! -e '+stage+'/POSE_NOT_AUTHORIZED')
        execute(c,'sudo -n mkdir -p /run/systemd/system/armpi-startup-pose.service.d')
        inhibited=True
        execute(c,'sudo -n install -m 644 '+stage+'/inhibit.conf '+block)
        execute(c,'sudo -n systemctl daemon-reload')
        assert stage+'/POSE_NOT_AUTHORIZED' in execute(c,'systemctl cat armpi-startup-pose')
        print('TRIAL_BACKUP='+stage,flush=True)
        changed=True
        execute(c,'sudo -n systemctl stop armpi-ros2',seconds=40)
        execute(c,'sudo -n install -m 644 '+stage+'/core.json /etc/armpi-ros2/core.json')
        execute(c,'sudo -n systemctl start armpi-ros2')
        name=await_core(runtime['name']);print('TRIAL_CONTAINER='+name,flush=True)
        for window in range(a.windows):
            row=json.loads(run_source(c,name,WATCH));print('TRIAL_WINDOW='+json.dumps(dict(row,window=window+1)),flush=True)
            assert not row['issues'],row['issues']
        success=True
        print('PACING_TRIAL_PASSED; gates remain OFF; no reset or motion',flush=True)
    finally:
        restored=success or not changed
        try:
            if changed and not success:
                execute(c,'sudo -n systemctl stop armpi-ros2',seconds=40)
                execute(c,'sudo -n install -m 644 '+stage+'/previous-core.json /etc/armpi-ros2/core.json')
                current=json.loads(execute(c,'sudo -n cat /run/armpi-ros2/container.json'))
                execute(c,'sudo -n systemctl start armpi-ros2')
                restored_name=await_core(current['name']);restored=True
                print('TRIAL_REJECTED_PREVIOUS_PIN_RESTORED='+restored_name,flush=True)
            if inhibited and restored:
                execute(c,'sudo -n rm -f '+block)
                execute(c,'sudo -n systemctl daemon-reload')
                print('HOMING_INHIBIT_REMOVED; startup pose not run',flush=True)
        finally:c.close()


if __name__=='__main__':main()
