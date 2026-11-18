"""Run explicitly reviewed native preflight/calibration scripts in one SSH session."""
import argparse,getpass,hashlib,json,shlex,sys,uuid
from pathlib import Path
import paramiko
ROOT=Path(__file__).resolve().parents[1]
def main():
 p=argparse.ArgumentParser();p.add_argument('--run-id',required=True);p.add_argument('--gateway',default='192.0.2.1');p.add_argument('--robot',default='192.0.2.1');p.add_argument('--direct-robot',action='store_true');a=p.parse_args();dest=ROOT/'logs/t07/runs'/a.run_id
 if not a.run_id.replace('_','').isalnum() or not (dest/'run.json').exists():raise ValueError('Unknown work-order run')
 gateway=paramiko.SSHClient();gateway.load_system_host_keys();gateway.set_missing_host_key_policy(paramiko.RejectPolicy());robot=paramiko.SSHClient();robot.load_system_host_keys();robot.set_missing_host_key_policy(paramiko.RejectPolicy());password=None
 known=paramiko.HostKeys(str(Path.home()/'.ssh/known_hosts'))
 for client,address,alias in [(gateway,a.gateway,'192.0.2.1'),(robot,a.robot,'192.0.2.1')]:
  trusted=known.lookup(alias)
  if not trusted:raise ValueError('Previous host identity is not trusted')
  for kind,key in trusted.items():client.get_host_keys().add(address,kind,key)
 try:
  sock=None
  if not a.direct_robot:
   gateway.connect(a.gateway,username='ycw',key_filename=str(Path.home()/'.ssh/id_ed25519'),timeout=8);gateway.get_transport().set_keepalive(15);sock=gateway.get_transport().open_channel('direct-tcpip',(a.robot,22),('127.0.0.1',0),timeout=8)
  password=getpass.getpass('Robot SSH password (hidden, not archived): ');robot.connect(a.robot,username='pi',password=password,sock=sock,look_for_keys=False,allow_agent=False,timeout=8,banner_timeout=8);password=None;robot.get_transport().set_keepalive(15);print('REAL_CONSOLE_READY',flush=True)
  mapping={'query':'t07_native_query.py','lock':'t07_native_chassis_lock.py','gripper':'t07_native_gripper_step.py','diagnostics':'t07_native_diag_client.py','calibrate':'t07_single_calibration.py','audit':'t07_runtime_patch_audit.py','forensics':'t07_gripper_passive_forensics.py','hostaudit':'t07_gripper_host_audit.py','enabledprobe':'t07_gripper_enabled_probe.py','reverseprobe':'t07_gripper_reverse_probe.py'}
  for line in sys.stdin:
   action=line.strip()
   if action=='quit':break
   mapping.update({'originalinventory':'t07_original_inventory.py','originalread':'t07_original_read.py','torquesemantics':'t07_torque_semantics_read.py','close30':'t07_gripper_close30.py','cameratiming':'t07_camera_timing_read.py','redballinventory':'t07_redball_inventory.py','redballprobe':'t07_redball_detector_probe.py','graspplan':'t07_redball_grasp_plan.py','approachstep':'t07_redball_approach_step.py'})
   mapping['elbowobserve']='t07_redball_elbow_observe.py'
   registry=ROOT/'configs/t07_console_actions.json'
   if registry.exists():
    extra=json.loads(registry.read_text(encoding='utf-8'))
    if any(not isinstance(v,str) or Path(v).name!=v or not v.startswith('t07_') or not v.endswith('.py') for v in extra.values()):raise ValueError('Invalid reviewed script registry')
    mapping.update(extra)
   if action not in mapping:print('Unknown action',flush=True);continue
   src=ROOT/'scripts'/mapping[action]
   if not src.is_file():print('Reviewed script not available',flush=True);continue
   folder=dest/('native_'+action+'_'+uuid.uuid4().hex[:8]);folder.mkdir();code=src.read_text(encoding='utf-8');(folder/src.name).write_text(code,encoding='utf-8')
   argv=['docker','exec','armpi_ros2_native','bash','-c','source /opt/ros/jazzy/setup.bash\nsource /opt/armpi_ros2/install/setup.bash\nexec '+shlex.join(['python3','-u','-c',code])]
   if action in ('hostaudit','originalinventory','originalread','torquesemantics','redballinventory'):argv=['python3','-u','-c',code]
   try:
    _,out,err=robot.exec_command(shlex.join(argv),timeout=120 if action=='pickattempt' else 55);text=out.read().decode(errors='replace');error=err.read().decode(errors='replace');status=out.channel.recv_exit_status();(folder/'stdout.log').write_text(text,encoding='utf-8');(folder/'stderr.log').write_text(error,encoding='utf-8');(folder/'command.json').write_text(json.dumps({'action':action,'argv':argv,'source_sha256':hashlib.sha256(src.read_bytes()).hexdigest(),'returncode':status},indent=2)+'\n');print('RESULT',folder.name,status,text[:800],error[:300],flush=True)
   except Exception as e:(folder/'transport_error.json').write_text(json.dumps({'error':repr(e)}));print('TRANSPORT_ERROR',repr(e),flush=True)
 finally:password=None;robot.close();gateway.close()
if __name__=='__main__':main()
