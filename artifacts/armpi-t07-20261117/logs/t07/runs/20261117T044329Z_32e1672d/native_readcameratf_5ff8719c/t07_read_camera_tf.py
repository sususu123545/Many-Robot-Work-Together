"""Read the existing camera TF implementation/config; no launch or control."""
import json,hashlib
from pathlib import Path
root=Path('/opt/armpi_ros2/install/armpi_ros2')
paths=[root/'lib/python3.12/site-packages/armpi_ros2/arm_camera_tf.py',root/'share/armpi_ros2/config/arm_camera_tf.yaml']
files={}
for p in paths:
 b=p.read_bytes();files[str(p)]={'sha256':hashlib.sha256(b).hexdigest(),'source':b.decode()}
print(json.dumps({'files':files,'motion_commands':0,'processes_launched':0}))
