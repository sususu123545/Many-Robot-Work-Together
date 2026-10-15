"""Read native core ownership, startup profile and health without commands."""
import argparse
import shlex
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from ros2_preflight import connect
from stage_ros2 import execute
from diagnose_lidar_ssh import check

INSPECT = '''
import importlib.util,json,subprocess,pathlib
s=importlib.util.spec_from_file_location('core','/opt/armpi-ros2/run_native_core.py')
m=importlib.util.module_from_spec(s);s.loader.exec_module(m)
r=json.load(open('/run/armpi-ros2/container.json'))
def run(args):
 p=subprocess.run(args,capture_output=True,text=True,timeout=10)
 return {'code':p.returncode,'stdout':p.stdout,'stderr':p.stderr}
owned=m.owners(r['devices']);assert not m.legacy_processes()
result={'legacy_processes':m.legacy_processes(),'device_owners':owned,'native_container':r,
 'services':{u:run(['systemctl','show',u,'-p','ActiveState','-p','UnitFileState','-p','Conditions']) for u in ['armpi-ros2','start_node',*m.LEGACY_SERVICES]},
 'container':run(['docker','inspect',r['name'],'--format','Running={{.State.Running}} Privileged={{.HostConfig.Privileged}} Devices={{json .HostConfig.Devices}}']),
 'legacy_sidecar_launch_check':run(['docker','exec','armpi_pro','bash','-lc','source /opt/ros/noetic/setup.bash && source /home/ubuntu/armpi_pro/devel/setup.bash && roslaunch --nodes /home/ubuntu/armpi_ros2_legacy_sidecar.launch camera:=true']),
 'legacy_boot_wrapper':pathlib.Path('/home/pi/armpi_pro/start_node.sh').read_text()}
print(json.dumps(result,ensure_ascii=False))
print(run(['journalctl','-u','armpi-ros2','-n','45','--no-pager']))
print(run(['docker','logs','--tail','65',r['name']]))
assert len(owned['/dev/rrc'])==1 and len(owned['/dev/ld06'])==1,owned
assert len(owned[r['devices'][-1]])==1,owned
assert len(owned['/dev/i2c-1'])<=1,owned
'''

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--host',required=True);p.add_argument('--config',required=True)
 a=p.parse_args();sys.stdout.reconfigure(encoding='utf-8');c=connect(a.host,config_path=a.config)
 try:check(c,'native_core_health','sudo -n python3 -c '+shlex.quote(INSPECT),seconds=20)
 finally:c.close()

if __name__=='__main__':main()
