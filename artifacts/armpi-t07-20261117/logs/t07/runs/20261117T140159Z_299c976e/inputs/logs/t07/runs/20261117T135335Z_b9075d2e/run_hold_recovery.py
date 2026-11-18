"""Execute reviewed upward-only recovery and fresh read-only checks; no stored credentials."""
import base64,getpass,hashlib,json,shlex
from pathlib import Path
import paramiko
RUN=Path(__file__).resolve().parent
ROOT=RUN.parents[3]
ssh=paramiko.SSHClient();ssh.load_system_host_keys();ssh.set_missing_host_key_policy(paramiko.RejectPolicy())
password=getpass.getpass('Robot SSH password (not archived): ')
try:
 ssh.connect('192.0.2.1',username='pi',password=password,look_for_keys=False,allow_agent=False,timeout=10);password=None;ssh.get_transport().set_keepalive(10)
 for source in (RUN/'bounded_hold.py',ROOT/'scripts/t07_native_query.py',ROOT/'scripts/t07_redball_grasp_plan.py'):
  code=source.read_text(encoding='utf-8');name='hold_'+source.stem
  if source.parent!=RUN:(RUN/source.name).write_bytes(source.read_bytes())
  argv=['docker','exec','armpi_ros2_native','bash','-c','source /opt/ros/jazzy/setup.bash\nsource /opt/armpi_ros2/install/setup.bash\nexec '+shlex.join(['python3','-u','-c',code])]
  (RUN/(name+'.command.json')).write_text(json.dumps({'argv':argv,'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),'status':'started'},indent=2))
  _,out,err=ssh.exec_command(shlex.join(argv),timeout=75);output,error=out.read(),err.read();rc=out.channel.recv_exit_status()
  (RUN/(name+'.stdout.json')).write_bytes(output);(RUN/(name+'.stderr.log')).write_bytes(error)
  (RUN/(name+'.command.json')).write_text(json.dumps({'argv':argv,'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),'returncode':rc},indent=2))
  data=json.loads(output)
  for i,im in enumerate(data.get('images',[])):(RUN/(name+f'_frame_{i:02}.jpg')).write_bytes(base64.b64decode(im['jpeg_base64']))
  if data.get('image_base64'):(RUN/'hold_post_recovery_camera.jpg').write_bytes(base64.b64decode(data['image_base64']))
  if name=='hold_bounded_hold':
   summary={k:data.get(k) for k in ('passed','error','movement_commands','gripper_commands','chassis_commands','reset_attempts','initial_raw','initial_tcp','step_results','model_height_change_before_stop_m','cleanup_confirmed')}
   summary['final_arm']={k:data.get('final_arm',{}).get(k) for k in ('estop','enable_arm_motion','enable_gripper_motion','fault','joints')}
   print(json.dumps({'script':name,'returncode':rc,**summary}),flush=True)
   if data.get('cleanup_confirmed') is False:raise RuntimeError('Recovery cleanup unconfirmed; no further steps')
  elif name=='hold_t07_native_query':
   print(json.dumps({'script':name,'returncode':rc,'fresh_samples':len(data.get('live_freshness_samples',[])),'image_stamp':data.get('image_stamp')}),flush=True)
  else:print(json.dumps({'script':name,'returncode':rc,'error':data.get('error'),'current_raw':data.get('current_raw'),'target_count':data.get('camera_point_count'),'target_age':data.get('target_age_seconds'),'pose':data.get('current_tool_pose'),'ball':data.get('live_tf_candidate_base_xyz'),'competing_publishers':data.get('competing_command_publishers')}),flush=True)
finally:password=None;ssh.close()
