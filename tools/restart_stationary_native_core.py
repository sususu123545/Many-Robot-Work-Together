"""Repair the confirmed waiting-overflow fault only, while motion is disabled."""
import argparse
import shlex
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from ros2_preflight import connect
from stage_ros2 import execute
from bench_ros2_forward import ROS2_SETUP,run_script
from switch_ros2_core import PROBE

GUARD='''
import importlib.util,json,pathlib,subprocess
s=importlib.util.spec_from_file_location('core','/opt/armpi-ros2/run_native_core.py');m=importlib.util.module_from_spec(s);s.loader.exec_module(m)
r=json.loads(pathlib.Path('/run/armpi-ros2/container.json').read_text())
assert r['motion_enabled'] is False and r['arm_motion_enabled'] is False
assert subprocess.check_output(['docker','inspect',r['name'],'--format','{{.State.Running}}']).decode().strip()=='false'
assert not m.legacy_processes() and not any(m.owners(r['devices']).values()),'Some hardware driver remains active'
old=pathlib.Path('/opt/armpi-ros2/run_native_core.py')
assert 'timeout=86400 * 30' in old.read_text(),'Unexpected runner; inspect rather than overwrite'
backup=pathlib.Path('/home/pi/armpi-ros2-cutover.790bU7/backup')
assert not (backup/'run_native_core.wait-overflow.py').exists()
import shutil
shutil.copy2(old,backup/'run_native_core.wait-overflow.py')
shutil.copy2('/home/pi/armpi-ros2-runner-fixed.py',old)
shutil.copy2(backup/'cutover-seed.applied.json','/run/armpi-ros2/cutover-seed.json')
print('Verified no owners; installed only the runner wait fix')
'''

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--host',required=True);p.add_argument('--config',required=True)
 a=p.parse_args();sys.stdout.reconfigure(encoding='utf-8');c=connect(a.host,config_path=a.config)
 try:
  src=Path(__file__).resolve().parents[1]/'car_setup'/'ros2'/'run_native_core.py'
  with c.open_sftp() as f:f.put(str(src),'/home/pi/armpi-ros2-runner-fixed.py')
  print(execute(c,'sudo -n python3 -c '+shlex.quote(GUARD)),flush=True)
  execute(c,'sudo -n systemctl start armpi-ros2')
  import time,json
  time.sleep(3)
  r=json.loads(execute(c,'sudo -n cat /run/armpi-ros2/container.json'))
  baseline=json.loads(execute(c,'sudo -n cat /run/armpi-ros2/cutover-seed.json'))
  run_script(c,r['name'],ROS2_SETUP,'BASELINE='+repr(baseline)+'\n'+PROBE)
  print('Native core repaired and stationary feedback verified: '+r['name'],flush=True)
 finally:c.close()

if __name__=='__main__':main()
