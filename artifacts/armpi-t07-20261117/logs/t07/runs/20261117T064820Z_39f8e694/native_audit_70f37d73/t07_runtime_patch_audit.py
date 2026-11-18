"""Read deployed files and process arguments; no ROS or serial writes."""
from pathlib import Path
import hashlib
import json
import time

base=Path('/opt/armpi_ros2/install/armpi_ros2')
files={}
for relative in ['lib/python3.12/site-packages/armpi_ros2/arm_node.py',
                 'lib/python3.12/site-packages/armpi_ros2/gripper_diag.py',
                 'lib/python3.12/site-packages/armpi_ros2/gripper_restore.py',
                 'share/armpi_ros2/launch/base.launch.py',
                 'share/armpi_ros2/config/t07_readonly_lock.flag',
                 'share/armpi_ros2/config/t07_gripper_restore_once.flag']:
    path=base/relative
    files[relative]={'exists':path.is_file()}
    if path.is_file():
        raw=path.read_bytes()
        files[relative].update(sha256=hashlib.sha256(raw).hexdigest(),text=raw.decode(errors='replace'))
processes=[]
for path in Path('/proc').iterdir():
    if not path.name.isdigit():continue
    try:
        args=(path/'cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace')
        if any(value in args for value in ['ros2 launch','/armpi_ros2/arm','/armpi_ros2/chassis']):
            if not any(value in args.lower() for value in ['password','secret','token']):
                processes.append({'pid':int(path.name),'argv':args[:2000]})
    except (OSError,ProcessLookupError):pass
print(json.dumps({'files':files,'processes':processes,'uptime_seconds':float(Path('/proc/uptime').read_text().split()[0]),'wall_time':time.time(),'motion_commands':0,'serial_calls':0}))
