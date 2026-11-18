"""Approved, backed-up stack restart with forced OFF gates and read-only service."""
import argparse,getpass,hashlib,json,shlex,time,uuid
from pathlib import Path,PurePosixPath
import paramiko
ROOT=Path(__file__).resolve().parents[1]
def main():
 p=argparse.ArgumentParser();p.add_argument('--run-id',required=True);p.add_argument('--ticket',choices=['T05','T07'],default='T07');p.add_argument('--direct-robot',action='store_true');p.add_argument('--gateway',default='192.0.2.1');p.add_argument('--robot',default='192.0.2.1');a=p.parse_args();workrun=ROOT/'logs'/a.ticket.lower()/'runs'/a.run_id;patch=workrun/'driver_patch';proposal=json.loads((patch/'proposal.json').read_text());assert proposal['operator_approval_for_restart'].startswith('approved')
 module_name=proposal.get('module_filename','gripper_diag.py')
 if module_name not in ('gripper_diag.py','gripper_restore.py'):raise ValueError('Unexpected staged module')
 run=workrun/'deployment_attempts'/(time.strftime('%Y%m%dT%H%M%SZ',time.gmtime())+'_'+uuid.uuid4().hex[:6]);run.mkdir(parents=True,exist_ok=False)
 gateway=paramiko.SSHClient();gateway.load_system_host_keys();gateway.set_missing_host_key_policy(paramiko.RejectPolicy());robot=paramiko.SSHClient();robot.load_system_host_keys();robot.set_missing_host_key_policy(paramiko.RejectPolicy());password=None;log=(run/'deployment_commands.jsonl').open('w');report={'passed':False,'motion_commands_sent':0,'torque_writes':0,'stack_stopped':False,'stack_restarted':False}
 known=paramiko.HostKeys(str(Path.home()/'.ssh/known_hosts'))
 for client,address,alias in [(gateway,a.gateway,'192.0.2.1'),(robot,a.robot,'192.0.2.1')]:
  trusted=known.lookup(alias)
  if not trusted:raise ValueError('Previous host identity is not trusted')
  for kind,key in trusted.items():client.get_host_keys().add(address,kind,key)
 def command(argv,timeout=50):
  _,stdout,stderr=robot.exec_command(shlex.join(argv),timeout=timeout);out=stdout.read().decode(errors='replace');err=stderr.read().decode(errors='replace');code=stdout.channel.recv_exit_status();log.write(json.dumps({'argv':argv,'returncode':code,'stdout':out,'stderr':err})+'\n');log.flush()
  if code:raise RuntimeError(err or out)
  return out
 def native(code):return command(['docker','exec','armpi_ros2_native','bash','-c','source /opt/ros/jazzy/setup.bash\nsource /opt/armpi_ros2/install/setup.bash\nexec '+shlex.join(['python3','-c',code])])
 try:
  sock=None
  if not a.direct_robot:
   gateway.connect(a.gateway,username='ycw',key_filename=str(Path.home()/'.ssh/id_ed25519'),timeout=8);gateway.get_transport().set_keepalive(15);sock=gateway.get_transport().open_channel('direct-tcpip',(a.robot,22),('127.0.0.1',0),timeout=8)
  password=getpass.getpass('Robot pi@'+a.robot+' SSH password (hidden, not archived): ');robot.connect(a.robot,username='pi',password=password,sock=sock,look_for_keys=False,allow_agent=False,timeout=8);password=None;robot.get_transport().set_keepalive(15)
  code="import json,importlib.util,hashlib; from pathlib import Path; from ament_index_python.packages import get_package_share_directory; s=importlib.util.find_spec('armpi_ros2.arm_node'); arm=Path(s.origin).resolve(); pkg=Path(s.origin).parent.resolve(); share=Path(get_package_share_directory('armpi_ros2')); launch=(share/'launch/base.launch.py').resolve(); print(json.dumps({'arm':str(arm),'helper':str(pkg/'gripper_diag.py'),'launch':str(launch),'marker':str((share/'config').resolve()/'t07_readonly_lock.flag'),'arm_sha256':hashlib.sha256(arm.read_bytes()).hexdigest(),'launch_sha256':hashlib.sha256(launch.read_bytes()).hexdigest(),'helper_exists':(pkg/'gripper_diag.py').exists()}))"
  code=code.replace("'gripper_diag.py'",repr(module_name));paths=json.loads(native(code));report['paths']=paths
  if proposal.get('latch_stop_before_restart'):
   locked=json.loads(native((ROOT/'scripts/t07_native_chassis_lock.py').read_text()));report['pre_restart_stop']=locked
   if locked.get('passed') is not True:raise RuntimeError('Pre-restart stop not confirmed; no deployment')
  expected_helper=proposal.get('baseline_helper_source_sha256')
  if paths['helper_exists'] and expected_helper:
   helper_hash=native('import hashlib; from pathlib import Path; print(hashlib.sha256(Path('+repr(paths['helper'])+').read_bytes()).hexdigest())').strip()
   if helper_hash!=expected_helper:raise RuntimeError('Existing helper baseline differs; no deployment')
  elif paths['helper_exists'] or expected_helper:raise RuntimeError('Unexpected helper existence; no deployment')
  if paths['arm_sha256']!=proposal['baseline_arm_source_sha256'] or paths['launch_sha256']!=proposal['baseline_launch_sha256']:raise RuntimeError('Baseline differs; no deployment')
  for key in ('arm','helper','launch','marker'):
   if not Path(paths[key]).is_relative_to('/opt/armpi_ros2'):raise RuntimeError('Unexpected package path')
  if proposal.get('protocol_patch'):
   paths['protocol']=str(PurePosixPath(paths['arm']).parent/'servo_protocol.py')
   current_hash=native('import hashlib; from pathlib import Path; print(hashlib.sha256(Path('+repr(paths['protocol'])+').read_bytes()).hexdigest())').strip()
   if current_hash!=proposal['baseline_protocol_sha256']:raise RuntimeError('Protocol baseline differs')
  extra=[('servo_protocol.py','protocol')] if proposal.get('protocol_patch') else []
  host='/home/pi/vla_t07/'+a.run_id+'_diagnostic_deploy';command(['mkdir','-p',host]);backup=run/'deployment_backup';backup.mkdir(exist_ok=False)
  with robot.open_sftp() as sftp:
   for key in (('arm','launch','helper') if paths['helper_exists'] else ('arm','launch'))+tuple(k for _,k in extra):
    command(['docker','cp','armpi_ros2_native:'+paths[key],host+'/'+key+'.before']);sftp.get(host+'/'+key+'.before',str(backup/(key+'.before')))
   for name in ('arm_node.py',module_name,'base.launch.py')+tuple(n for n,_ in extra):sftp.put(str(patch/name),host+'/'+name)
   marker=patch/'t07_readonly_lock.flag';marker.write_text('T07 approved read-only diagnostics: force all startup motion gates OFF\n');sftp.put(str(marker),host+'/t07_readonly_lock.flag')
  print('BACKUPS_AND_BASELINES_VERIFIED',flush=True)
  command(['docker','stop','--time','20','armpi_ros2_native']);report['stack_stopped']=True
  for name,key in [('t07_readonly_lock.flag','marker'),('base.launch.py','launch'),(module_name,'helper'),('arm_node.py','arm')]+extra:command(['docker','cp',host+'/'+name,'armpi_ros2_native:'+paths[key]])
  command(['docker','start','armpi_ros2_native']);report['stack_restarted']=True;print('RESTARTED_WITH_READONLY_STARTUP_LOCK',flush=True)
  expected={key:hashlib.sha256((patch/name).read_bytes()).hexdigest() for name,key in [('arm_node.py','arm'),(module_name,'helper'),('base.launch.py','launch'),('t07_readonly_lock.flag','marker')]+extra}
  actual=json.loads(native('import json,hashlib; from pathlib import Path; paths='+repr(paths)+'; print(json.dumps({k:hashlib.sha256(Path(paths[k]).read_bytes()).hexdigest() for k in '+repr(list(expected))+'}))'))
  if actual!=expected:raise RuntimeError('Installed hashes differ; diagnostic client not run')
  report['installed_hashes']=actual;result=json.loads(native((ROOT/'scripts/t07_native_diag_client.py').read_text()));report['diagnostics']=result;report['passed']=result.get('gates_verified_off') is True and result.get('diagnostic_response',{}).get('success') is True
  if proposal.get('restore_stage'):
   if not report['passed']:raise RuntimeError('Initial diagnostic/gates did not pass')
   measurements=json.loads(result['diagnostic_response']['message'])['measurements']
   if measurements['torque_enabled'] is not False:raise RuntimeError('Initial torque state changed; no restore')
   permit=paths['marker'].replace('t07_readonly_lock.flag','t07_gripper_restore_once.flag')
   native('from pathlib import Path; p=Path('+repr(permit)+'); assert not p.exists(), "Existing permit retained; no repeat"; p.write_text('+repr('User authorized only servo1 torque restore; one <=10-tick step; '+a.run_id)+')')
   restored=json.loads(native((ROOT/'scripts/t07_native_restore_client.py').read_text()));report['torque_restore']=restored;report['passed']=restored.get('success') is True
   if restored.get('message'):
    detail=json.loads(restored['message']);report['torque_writes']=detail.get('torque_writes',0);report['holding_target_commands']=detail.get('holding_target_commands',0)
 except Exception as e:report['error']=repr(e)
 finally:
  password=None;robot.close();gateway.close();log.close();(run/'deployment_report.json').write_text(json.dumps(report,indent=2)+'\n')
 print(json.dumps(report,indent=2),flush=True)
if __name__=='__main__':main()
