"""Read-only migration inventory; reject unknown SSH keys and never move robot.

Use the existing trusted hostname key as a pin when phone-hotspot IPs change.
This adds the key only to memory; it never rewrites known_hosts.
Requires paramiko, available in the existing console Python runtime.
"""
import argparse
import getpass
import json
import os
from pathlib import Path

import paramiko


INVENTORY_COMMAND = r'''python3 - <<'PY'
import json, os, pathlib, shutil, subprocess
def run(args):
    p = subprocess.run(args, capture_output=True, text=True, timeout=20)
    return {'code': p.returncode, 'stdout': p.stdout.strip(), 'stderr': p.stderr.strip()}
usage = shutil.disk_usage('/')
result = {
  'architecture': run(['uname', '-m']),
  'os_release': pathlib.Path('/etc/os-release').read_text(),
  'free_disk_bytes': usage.free,
  'containers': run(['docker', 'ps', '-a', '--format', '{{.Names}} {{.Image}} {{.Status}}']),
  'devices': {p:os.path.realpath(p) if os.path.exists(p) else None for p in ['/dev/ld06','/dev/rrc','/dev/i2c-1']},
  'services': {s:run(['systemctl','is-active',s]) for s in ['start_node','ld06','odom','ch341-bmi270','auto-mapping','slam-control']},
  'bmi_library': run(['docker','exec','armpi_pro','test','-f','/home/ubuntu/share/tmp/bmi270_py/bmi270/config_file.py']),
  'i2c_adapters': {str(p):p.read_text().strip() for p in pathlib.Path('/sys/class/i2c-adapter').glob('i2c-*/name')},
}
print(json.dumps(result, ensure_ascii=False, indent=2))
PY'''


def connect(host, user='pi', config_path=None, key_alias='raspberrypi', password=None):
    config = {}
    if config_path is not None:
        config = json.loads(Path(config_path).read_text(encoding='utf-8-sig'))
    keys = paramiko.HostKeys(str(Path.home() / '.ssh' / 'known_hosts'))
    pin = keys.lookup(host) or keys.lookup(key_alias)
    if not pin:
        raise RuntimeError('No trusted SSH host key. Verify the robot key before migrating.')
    client = paramiko.SSHClient()
    for algorithm, key in pin.items():
        client.get_host_keys().add(host, algorithm, key)
    client.set_missing_host_key_policy(paramiko.RejectPolicy())
    password = password or config.get('password') or getpass.getpass('Robot SSH password: ')
    try:
        client.connect(host, username=user, password=password, look_for_keys=False,
                       allow_agent=False, timeout=8, auth_timeout=8)
    except Exception:
        client.close()
        raise
    return client


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', required=True)
    parser.add_argument('--user', default='pi')
    parser.add_argument('--host-key-alias', default='raspberrypi')
    parser.add_argument('--config', help='Existing supervisor credential file (never printed)')
    args = parser.parse_args()
    client = connect(args.host, args.user, args.config, args.host_key_alias)
    try:
        _, out, err = client.exec_command(INVENTORY_COMMAND, timeout=45)
        text, errors = out.read().decode('utf-8'), err.read().decode('utf-8')
        if out.channel.recv_exit_status() != 0:
            raise RuntimeError(errors or text)
        print(text)
    finally:
        client.close()


if __name__ == '__main__':
    main()
