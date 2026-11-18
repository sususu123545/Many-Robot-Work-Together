"""Select Docker lifecycle fields and filtered controller log lines only."""
import json,subprocess,re
def command(argv):
 result=subprocess.run(argv,capture_output=True,text=True,timeout=20)
 if result.returncode:raise RuntimeError(result.stderr[:500])
 return result.stdout
format_string='{{json .Id}}\n{{json .Created}}\n{{json .State.StartedAt}}\n{{json .RestartCount}}\n{{json .Config.Image}}\n{{json .Mounts}}'
values=command(['docker','inspect','--format',format_string,'armpi_ros2_native']).splitlines()
selected=dict(zip(['id','created','started','restart_count','image','mounts'],[json.loads(v) for v in values]))
result=subprocess.run(['docker','logs','--timestamps','--since','2h','--tail','1500','armpi_ros2_native'],capture_output=True,text=True,timeout=20)
lines=[]
for line in (result.stdout+'\n'+result.stderr).splitlines():
 if any(key in line for key in ['Transient arm feedback','Arm feedback fault','Gripper','gripper','Arm health','True feedback']):
  if not any(key in line.lower() for key in ['password','token','secret']):lines.append(line)
ownership={}
for key in ['com.docker.compose.project','com.docker.compose.project.working_dir','com.docker.compose.project.config_files','com.docker.compose.service']:
 ownership[key]=command(['docker','inspect','--format','{{index .Config.Labels "'+key+'"}}','armpi_ros2_native']).strip()
units=[]
listing=command(['systemctl','list-unit-files','--no-legend','--no-pager'])
for line in listing.splitlines():
 name=line.split()[0]
 if any(k in name.lower() for k in ['armpi','robot','vla']) and name.endswith(('.service','.timer')):
  info=command(['systemctl','show',name,'--property=Id,ActiveState,FragmentPath,MainPID,ExecStart','--no-pager'])
  fields={}
  for value in info.splitlines():
   k,_,v=value.partition('=')
   if k=='ExecStart':fields['script_paths']=re.findall(r'/[A-Za-z0-9_./-]+\.(?:py|sh|yaml|yml)',v)
   else:fields[k]=v
  units.append(fields)
print(json.dumps({'container':selected,'controller_log_lines':lines,'log_returncode':result.returncode,'ownership':ownership,'candidate_units':units,'motion_commands':0,'state_changes':0}))
