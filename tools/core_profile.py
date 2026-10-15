"""Public, credential-free ROS core selection shared by console and supervisor."""
import json
from pathlib import Path

PROFILE_FILE = Path(__file__).resolve().with_name('console_core_profile.json')


def load_core_profile(path=PROFILE_FILE):
    if not Path(path).exists():
        # ROS1 now requires an explicit profile; never silently fall back to it.
        return {'ros_version': 2, 'remote_ros_port': 9091}
    data = json.loads(Path(path).read_text(encoding='utf-8-sig'))
    version, port = data.get('ros_version'), data.get('remote_ros_port')
    if type(version) is not int or type(port) is not int or (version, port) not in ((1, 9090), (2, 9091)):
        raise ValueError('Unrecognized ROS version/bridge port pairing; refuse tunnel')
    return {'ros_version': version, 'remote_ros_port': port}
