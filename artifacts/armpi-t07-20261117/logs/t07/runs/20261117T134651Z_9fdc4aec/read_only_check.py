"""One fresh read-only physical robot inspection; passwords are never written."""
import base64, getpass, hashlib, json, shlex, sys
from pathlib import Path
import paramiko

DEST = Path(__file__).resolve().parent
ROOT = DEST.parents[3]
ssh = paramiko.SSHClient()
ssh.load_system_host_keys()
ssh.set_missing_host_key_policy(paramiko.RejectPolicy())
password = getpass.getpass('Robot SSH password (not archived): ')
try:
    ssh.connect('192.0.2.1', username='pi', password=password,
                look_for_keys=False, allow_agent=False, timeout=10)
    password = None
    ssh.get_transport().set_keepalive(15)
    for name in ('t07_native_query.py', 't07_redball_grasp_plan.py'):
        source = ROOT / 'scripts' / name
        code = source.read_text(encoding='utf-8')
        (DEST / name).write_text(code, encoding='utf-8')
        argv = ['docker', 'exec', 'armpi_ros2_native', 'bash', '-c',
                'source /opt/ros/jazzy/setup.bash\nsource /opt/armpi_ros2/install/setup.bash\nexec ' + shlex.join(['python3', '-u', '-c', code])]
        _, stdout, stderr = ssh.exec_command(shlex.join(argv), timeout=45)
        output, error = stdout.read(), stderr.read()
        status = stdout.channel.recv_exit_status()
        (DEST / (name + '.stdout.json')).write_bytes(output)
        (DEST / (name + '.stderr.log')).write_bytes(error)
        (DEST / (name + '.command.json')).write_text(json.dumps({'argv': argv, 'sha256': hashlib.sha256(source.read_bytes()).hexdigest(), 'returncode': status}, indent=2), encoding='utf-8')
        data = json.loads(output)
        if data.get('image_base64'):
            (DEST / 'camera.jpg').write_bytes(base64.b64decode(data['image_base64']))
        if name == 't07_native_query.py':
            print(json.dumps({'script': name, 'returncode': status, 'latest_feedback': data.get('live_freshness_samples', [])[-3:], 'image_stamp': data.get('image_stamp'), 'nodes': data.get('nodes')}, ensure_ascii=False), flush=True)
        else:
            print(json.dumps({'script': name, 'returncode': status, 'error': data.get('error'), 'arm': data.get('arm'), 'base': data.get('base'), 'target_count': data.get('camera_point_count'), 'complete': data.get('target_complete_sphere_verified'), 'target_age': data.get('target_age_seconds'), 'current_pose': data.get('current_tool_pose'), 'ball': data.get('live_tf_candidate_base_xyz'), 'offline_ball': data.get('offline_handeye_candidate_base_xyz'), 'competing_publishers': data.get('competing_command_publishers')}, ensure_ascii=False), flush=True)
finally:
    password = None
    ssh.close()
