"""Read-only RRC transport endurance; requires both hardware stacks stopped."""
import argparse
import shlex
import sys
import uuid
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from ros2_preflight import connect
from stage_ros2 import execute
from bench_ros2_forward import run_script
from switch_ros2_core import IMAGE,IMAGE_ID

GUARD='''
import importlib.util,subprocess
s=importlib.util.spec_from_file_location('core','/opt/armpi-ros2/run_native_core.py');m=importlib.util.module_from_spec(s);s.loader.exec_module(m)
state=subprocess.run(['systemctl','is-active','armpi-ros2'],capture_output=True,text=True).stdout.strip()
assert state not in ('active','activating')
assert not m.legacy_processes() and not m.owners(['/dev/rrc'])['/dev/rrc']
print('No arm owner; readonly serial probe allowed')
'''

PROBE='''
import sys,json,time
sys.path.insert(0,'/opt/arm-probe')
from servo_protocol import ServoBoard
b=ServoBoard();errors=[];rows=[];start=time.monotonic()
try:
 for cycle in range(32):
  values={}
  for ident in range(1,7):
   try:values[str(ident)]={'raw':b.position(ident),'temperature':b.temperature(ident)}
   except (OSError,TimeoutError) as e:errors.append({'cycle':cycle,'servo':ident,'error':str(e)})
  rows.append(values);time.sleep(.4)
 assert not errors,errors
 assert all(abs(row[i]['raw']-rows[0][i]['raw'])<=3 for row in rows for i in rows[0]),'Unexpected physical arm movement'
 print(json.dumps({'readonly_query_count':32*12,'errors':errors,'seconds':round(time.monotonic()-start,2),'last_real_positions':rows[-1],'continuous_receiver':True}))
finally:b.close()
'''

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--host',required=True);p.add_argument('--config',required=True)
 a=p.parse_args();sys.stdout.reconfigure(encoding='utf-8');c=connect(a.host,config_path=a.config)
 name='armpi_ros2_arm_read_'+uuid.uuid4().hex[:12];created=False
 try:
  execute(c,'sudo -n python3 -c '+shlex.quote(GUARD))
  assert execute(c,'docker image inspect '+IMAGE+" --format '{{.Id}}'")==IMAGE_ID
  stage=execute(c,'mktemp -d /home/pi/armpi-arm-read.XXXXXX')
  with c.open_sftp() as f:f.put(str(Path(__file__).resolve().parents[1]/'ros2_ws/src/armpi_ros2/armpi_ros2/servo_protocol.py'),stage+'/servo_protocol.py')
  created=True
  execute(c,'docker run -d --name '+name+' --network none --device /dev/rrc:/dev/rrc -v '+stage+':/opt/arm-probe:ro '+IMAGE+' sleep 60')
  run_script(c,name,'',PROBE)
 finally:
  if created:print(execute(c,'docker stop -t 3 '+name),flush=True)
  c.close()

if __name__=='__main__':main()
