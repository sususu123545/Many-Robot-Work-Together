"""Supported-arm read-only A/B test; temporary homing inhibit, no driver edits."""
import argparse
import hashlib
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
from fix_startup_pose_permissions import probe

ROOT=Path(__file__).resolve().parents[1]
OWNER_CHECK=r'''
import importlib.util,json
s=importlib.util.spec_from_file_location('core','/opt/armpi-ros2/run_native_core.py');m=importlib.util.module_from_spec(s);s.loader.exec_module(m)
assert not m.legacy_processes()
assert not m.owners(['/dev/rrc'])['/dev/rrc'],'Serial still occupied'
print('SERIAL_EXCLUSIVITY_GUARD=OK')
'''


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--host',required=True);p.add_argument('--config',required=True)
    p.add_argument('--confirm-supported',action='store_true',required=True)
    p.add_argument('--cycles',type=int,default=180)
    p.add_argument('--profiles',choices=('timing','interleaved','sdk','raw','voltage','matched'),default='timing')
    a=p.parse_args()
    if not 20<=a.cycles<=240:p.error('20..240 cycles per profile')
    if a.profiles=='sdk' and a.cycles>180:p.error('SDK comparison bounded to 180 cycles')
    if a.profiles=='voltage' and a.cycles>120:p.error('Voltage comparison bounded to 120 cycles')
    if a.profiles=='matched' and a.cycles>90:p.error('Matched comparison bounded to 90 cycles per segment')
    sys.stdout.reconfigure(encoding='utf-8')
    c=connect(a.host,config_path=a.config)
    token=uuid.uuid4().hex[:10]
    name='armpi_arm_readonly_ab_'+token
    inhibit='/run/systemd/system/armpi-startup-pose.service.d/95-readonly-ab-'+token+'.conf'
    inhibited=False;stopped=False;created=False;stage=None
    try:
        assert execute(c,'systemctl show armpi-ros2 -p ActiveState --value')=='active'
        assert execute(c,'systemctl show armpi-startup-pose -p ActiveState --value') in ('inactive','failed')
        marker=json.loads(execute(c,'sudo -n cat /run/armpi-ros2/arm-bootstrap.json'))
        assert marker.get('ok') is True,'Warm restart would risk boot recovery'
        runtime=json.loads(execute(c,'sudo -n cat /run/armpi-ros2/container.json'))
        before=probe(c,runtime['name']);print('PRECONDITION='+json.dumps(before),flush=True)
        assert before['enable_arm_motion'] is False and before['enable_gripper_motion'] is False and before['estop']
        cfg=json.loads(execute(c,'sudo -n cat /etc/armpi-ros2/core.json'))
        image=cfg['image']
        assert re.fullmatch(r'armpi-ros2-migration:stage-[a-z0-9]+',image)
        assert execute(c,'docker image inspect '+image+" --format '{{.Id}}'")==cfg['image_id']==runtime['image_id']
        stage=execute(c,'mktemp -d /home/pi/armpi-readonly-ab.XXXXXX')
        assert re.fullmatch(r'/home/pi/armpi-readonly-ab\.[A-Za-z0-9]+',stage)
        execute(c,'mkdir '+stage+'/results')
        with c.open_sftp() as s:
            s.get_channel().settimeout(15)
            probe_source={'sdk':'tools/arm_sdk_readonly_compare.py','raw':'tools/arm_raw_capture.py',
                          'voltage':'tools/arm_voltage_readonly.py','matched':'tools/arm_matched_readonly.py'}.get(a.profiles,'tools/arm_transport_readonly_ab.py')
            files=[('tools/arm_transport_readonly_ab.py','arm_transport_readonly_ab.py'),
                   (probe_source,'probe.py'),
                   ('ros2_ws/src/armpi_ros2/armpi_ros2/servo_protocol.py','servo_protocol.py')]
            if a.profiles=='sdk':
                files.append(('armpi_pro/src/ros_robot_controller/src/ros_robot_controller/ros_robot_controller_sdk.py','stock_sdk.py'))
            for source,target in files:
                s.put(str(ROOT/source),stage+'/'+target)
                assert execute(c,'sha256sum '+stage+'/'+target).split()[0]==hashlib.sha256((ROOT/source).read_bytes()).hexdigest()
            with s.open(stage+'/inhibit.conf','w') as f:
                f.write('[Unit]\n# Temporary read-only diagnosis: never home during core restore.\nConditionPathExists='+stage+'/POSE_NOT_AUTHORIZED\n')
        assert execute(c,'test ! -e '+stage+'/POSE_NOT_AUTHORIZED')==''
        execute(c,'sudo -n mkdir -p /run/systemd/system/armpi-startup-pose.service.d')
        inhibited=True
        execute(c,'sudo -n install -m 644 '+stage+'/inhibit.conf '+inhibit)
        execute(c,'sudo -n systemctl daemon-reload')
        assert stage+'/POSE_NOT_AUTHORIZED' in execute(c,'systemctl cat armpi-startup-pose')
        print('STAGE='+stage+' HOMING_TEMPORARILY_INHIBITED',flush=True)
        stopped=True
        execute(c,'sudo -n systemctl stop armpi-ros2',seconds=40)
        execute(c,'sudo -n python3 -c '+shlex.quote(OWNER_CHECK))
        assert execute(c,'systemctl show armpi-ros2 -p ActiveState --value') in ('inactive','failed')
        created=True
        command=('docker run -d --name '+name+' --network none --device /dev/rrc:/dev/rrc '
                 '-v '+stage+':/opt/probe:ro -v '+stage+'/results:/results '
                 '-e AB_CYCLES='+str(a.cycles)+' -e AB_PROFILES='+a.profiles+' '+image+' python3 -u /opt/probe/probe.py')
        execute(c,command)
        print('READONLY_PROBE_CONTAINER='+name,flush=True)
        deadline=time.monotonic()+a.cycles*({'sdk':3.0,'matched':6.5}.get(a.profiles,1.8))+100
        printed=0
        while time.monotonic()<deadline:
            logs=execute(c,'docker logs '+name,seconds=15)
            lines=logs.splitlines()
            for line in lines[printed:]:print(line,flush=True)
            printed=len(lines)
            status=execute(c,'docker inspect '+name+" --format '{{.State.Running}} {{.State.ExitCode}}'")
            if status.startswith('false '):
                assert status=='false 0',logs[-4000:]
                break
            time.sleep(5)
        else:raise TimeoutError('A/B diagnosis exceeded deadline')
        local=ROOT/'tools/runtime'/('arm_readonly_ab_'+token+'.json')
        with c.open_sftp() as s:
            s.get_channel().settimeout(15);s.get(stage+'/results/report.json',str(local))
            if a.profiles=='raw':
                capture=ROOT/'tools/runtime'/('arm_raw_capture_'+token+'.json')
                s.get(stage+'/results/capture.json',str(capture))
                print('LOCAL_CAPTURE='+str(capture),flush=True)
        print('LOCAL_REPORT='+str(local),flush=True)
    finally:
        restored=False
        try:
            if created:
                execute(c,'docker stop -t 3 '+name,seconds=15)
                assert execute(c,'docker inspect '+name+" --format '{{.State.Running}}'")=='false'
            if stopped:
                execute(c,'sudo -n python3 -c '+shlex.quote(OWNER_CHECK))
                execute(c,'sudo -n systemctl start armpi-ros2')
                end=time.monotonic()+65
                while time.monotonic()<end:
                    current=json.loads(execute(c,'sudo -n cat /run/armpi-ros2/container.json'))
                    if current['name']!=runtime['name']:
                        try:
                            after=probe(c,current['name'])
                            assert after['enable_arm_motion'] is False and after['enable_gripper_motion'] is False
                            print('CORE_RESTORED_MOTION_OFF='+json.dumps(after),flush=True)
                            restored=True;break
                        except RuntimeError:pass
                    time.sleep(2)
                if not restored:raise RuntimeError('Core restoration not verified; homing inhibit retained')
            else:restored=True
            if inhibited and restored:
                # Exact temporary file only; source copy remains in stage.
                execute(c,'sudo -n rm -f '+inhibit)
                execute(c,'sudo -n systemctl daemon-reload')
                print('TEMPORARY_HOMING_INHIBIT_REMOVED; POSE_NOT_STARTED',flush=True)
        finally:c.close()


if __name__=='__main__':main()
