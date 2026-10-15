"""Activate a pinned native-core image from an inspected STOPPED state only."""
import argparse
import json
import re
import shlex
import sys
import time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from ros2_preflight import connect
from stage_ros2 import execute
from bench_ros2_forward import ROS2_SETUP,run_script
from switch_ros2_core import PROBE

ACTIVATE='''
import importlib.util,pathlib,json,subprocess,shutil,sys
s=importlib.util.spec_from_file_location('core','/opt/armpi-ros2/run_native_core.py');m=importlib.util.module_from_spec(s);s.loader.exec_module(m)
record=json.load(open('/run/armpi-ros2/container.json'))
assert not m.legacy_processes() and not any(m.owners(record['devices']).values())
assert subprocess.check_output(['docker','inspect',record['name'],'--format','{{.State.Running}}']).decode().strip()=='false'
cfg_path=pathlib.Path('/etc/armpi-ros2/core.json');cfg=json.loads(cfg_path.read_text())
assert cfg['image_id']==sys.argv[4],'Prior image changed'
assert subprocess.check_output(['docker','image','inspect',sys.argv[2],'--format','{{.Id}}']).decode().strip()==sys.argv[3]
assert pathlib.Path('/run/armpi-ros2/cutover-seed.json').is_file(),'No stationary pose seed'
assert pathlib.Path('/etc/armpi-ros2/active').is_file()
assert 'timeout=None' in pathlib.Path('/opt/armpi-ros2/run_native_core.py').read_text()
backup=pathlib.Path(sys.argv[1]);shutil.copy2(cfg_path,backup/'previous-core.json')
shutil.copy2('/run/armpi-ros2/cutover-seed.json',backup/'seed.json')
cfg['image']=sys.argv[2];cfg['image_id']=sys.argv[3];cfg_path.write_text(json.dumps(cfg))
print('No hardware owners; pinned native image installed')
'''

def main():
 p=argparse.ArgumentParser(description=__doc__)
 for k in ('host','config','image','expected-image-id','expected-previous-image-id'):p.add_argument('--'+k,required=True)
 a=p.parse_args();sys.stdout.reconfigure(encoding='utf-8')
 assert re.fullmatch('armpi-ros2-migration:stage-[a-z0-9]+',a.image)
 assert all(re.fullmatch('sha256:[0-9a-f]{64}',v) for v in (a.expected_image_id,a.expected_previous_image_id))
 c=connect(a.host,config_path=a.config);started=False
 try:
  stage=execute(c,'mktemp -d /home/pi/armpi-ros2-activation.XXXXXX')
  assert re.fullmatch('/home/pi/armpi-ros2-activation.[A-Za-z0-9]+',stage)
  print(execute(c,'sudo -n python3 -c '+shlex.quote(ACTIVATE)+' '+shlex.quote(stage)+' '+a.image+' '+a.expected_image_id+' '+a.expected_previous_image_id),flush=True)
  started=True;execute(c,'sudo -n systemctl start armpi-ros2');time.sleep(3)
  r=json.loads(execute(c,'sudo -n cat /run/armpi-ros2/container.json'))
  baseline=json.loads(execute(c,'sudo -n cat /run/armpi-ros2/cutover-seed.json'))
  run_script(c,r['name'],ROS2_SETUP,'BASELINE='+repr(baseline)+'\n'+PROBE)
  print('NATIVE_IMAGE_ACCEPTED='+r['name']+' BACKUP='+stage,flush=True)
 except Exception:
  if started:
   try:execute(c,'sudo -n systemctl stop armpi-ros2')
   except Exception:print('Native stop could not be confirmed; physical emergency stop required')
  raise
 finally:c.close()

if __name__=='__main__':main()
