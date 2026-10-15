"""Backed-up ROS2-only boot cutover. No nonzero velocity or arm pose call."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import shlex
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ros2_preflight import connect

ROOT = Path(__file__).resolve().parents[1]
LEGACY = ['start_node', 'ld06', 'odom', 'ch341-bmi270', 'auto-mapping', 'slam-control']


def execute(client, command, seconds=30):
    # This robot can stall non-PTY SSH exec/subsystem requests after a boot.
    wrapped = 'env SYSTEMD_PAGER=cat SYSTEMD_COLORS=0 TERM=dumb bash --noprofile --norc -c ' + shlex.quote(command)
    _, out, _ = client.exec_command(wrapped, get_pty=True, timeout=10)
    result = bytearray()
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        while out.channel.recv_ready():
            result.extend(out.channel.recv(65536))
        if out.channel.exit_status_ready():
            code = out.channel.recv_exit_status()
            text = result.decode(errors='replace').replace('\r', '').strip()
            if code:
                raise RuntimeError(text)
            return text
        time.sleep(.03)
    out.channel.close()
    raise TimeoutError('SSH command exceeded deadline; inspect before retry')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', required=True)
    parser.add_argument('--config', required=True)
    parser.add_argument('--image', required=True)
    parser.add_argument('--image-id', required=True)
    parser.add_argument('--confirm-stationary', action='store_true')
    args = parser.parse_args()
    if not args.confirm_stationary:
        parser.error('Confirm stationary and unattended motion range before cutover')
    if not re.fullmatch(r'armpi-ros2-migration:stage-[a-z0-9]+', args.image):
        parser.error('Use a staged image')
    if not re.fullmatch(r'sha256:[0-9a-f]{64}', args.image_id):
        parser.error('Use the exact verified image ID')
    client = connect(args.host, config_path=args.config)
    try:
        assert execute(client, 'systemctl show armpi-ros2 --property=ActiveState --value') in ('inactive','failed')
        assert execute(client, 'systemctl show armpi-startup-pose --property=ActiveState --value') in ('inactive','failed')
        assert execute(client, 'docker image inspect ' + args.image + " --format '{{.Id}}'") == args.image_id
        guard = """import importlib.util
s=importlib.util.spec_from_file_location('core','/opt/armpi-ros2/run_native_core.py');m=importlib.util.module_from_spec(s);s.loader.exec_module(m)
assert not m.legacy_processes(), 'Legacy motion hardware still running'
assert not any(m.owners(['/dev/rrc','/dev/ld06','/dev/i2c-1']).values()), 'Core hardware occupied'
"""
        execute(client, 'sudo -n python3 -c ' + shlex.quote(guard))
        stage = execute(client, 'mktemp -d /home/pi/armpi-ros2-only-boot.XXXXXX')
        assert re.fullmatch(r'/home/pi/armpi-ros2-only-boot\.[A-Za-z0-9]+', stage)
        old_config = json.loads(execute(client, 'cat /etc/armpi-ros2/core.json'))
        new_config = dict(old_config, image=args.image, image_id=args.image_id)
        states = {unit: execute(client, 'systemctl is-enabled ' + unit + ' 2>/dev/null || true') for unit in LEGACY}
        policy = execute(client, "docker inspect armpi_pro --format '{{json .HostConfig.RestartPolicy}}'")
        with client.open_sftp() as sftp:
            sftp.get_channel().settimeout(12)
            for name in ('run_native_core.py','lidar_preflight.py','armpi-ros2.service'):
                source = ROOT/'car_setup/ros2'/name
                sftp.put(str(source), stage+'/'+name)
                assert execute(client, 'sha256sum ' + stage+'/'+name).split()[0] == hashlib.sha256(source.read_bytes()).hexdigest()
            with sftp.open(stage+'/core.json','w') as out:
                out.write(json.dumps(new_config))
            with sftp.open(stage+'/legacy-state.json','w') as out:
                out.write(json.dumps({'units':states,'container_policy':json.loads(policy)}))
        for original, name in [('/etc/armpi-ros2/core.json','previous-core.json'),
                               ('/opt/armpi-ros2/run_native_core.py','previous-runner.py'),
                               ('/etc/systemd/system/armpi-ros2.service','previous-unit.service'),
                               ('/home/pi/armpi_pro/start_node.sh','previous-start_node.sh')]:
            execute(client, 'sudo -n cp ' + original + ' ' + stage+'/'+name)
        print('ROS2_ONLY_BACKUP='+stage, flush=True)
        # Gate the already existing ROS2 sidecar branch; retain the legacy source
        # and image for a deliberate, separately authorized rollback.
        wrapper = """from pathlib import Path
p=Path('/home/pi/armpi_pro/start_node.sh');s=p.read_text()
old='  exec docker exec -u ubuntu -w /home/ubuntu armpi_pro bash /home/ubuntu/armpi_ros2_legacy_sidecar.sh'
assert s.count(old)==1, 'Unexpected wrapper; do not overwrite it'
p.write_text(s.replace(old, '  echo "ROS2-only profile active; ROS1 launch suppressed"\\n  exit 0'))
"""
        execute(client, 'sudo -n python3 -c ' + shlex.quote(wrapper))
        execute(client, 'docker update --restart=no armpi_pro')
        execute(client, 'sudo -n systemctl disable --now ' + ' '.join(LEGACY), seconds=40)
        execute(client, 'docker stop -t 15 armpi_pro', seconds=25)
        assert execute(client, "docker inspect armpi_pro --format '{{.State.Running}}'") == 'false'
        for source, target in [('run_native_core.py','/opt/armpi-ros2/run_native_core.py'),
                               ('lidar_preflight.py','/opt/armpi-ros2/lidar_preflight.py'),
                               ('armpi-ros2.service','/etc/systemd/system/armpi-ros2.service'),
                               ('core.json','/etc/armpi-ros2/core.json')]:
            execute(client, 'sudo -n install -m 644 ' + stage+'/'+source+' '+target)
        execute(client, 'sudo -n systemd-analyze verify /etc/systemd/system/armpi-ros2.service')
        execute(client, 'sudo -n systemctl daemon-reload')
        execute(client, 'sudo -n systemctl enable armpi-ros2 armpi-startup-pose')
        execute(client, 'sudo -n systemctl start armpi-ros2')
        print('ROS2_ONLY_BOOT_APPLIED; warm restart does not run the pose service', flush=True)
    finally:
        client.close()


if __name__=='__main__':
    sys.stdout.reconfigure(encoding='utf-8')
    main()
