"""Install / control the opt-in boot-time arm pose service on the ArmPi Pro car.

Actions, deliberately separated so nothing is enabled by accident:
  install     upload the 3 runtime files and the unit, verify digests, daemon-reload
  enable      systemctl enable  (arm moves on every future boot)
  disable     systemctl disable --now (back to the audited "no automatic arm pose")
  run         start the oneshot now, i.e. a real pose travel, and print the result
  status      current enable/active state, last result file, journal tail
  uninstall   disable, remove unit and runtime files, daemon-reload

Nothing here touches arm_node.py, run_native_core.py or armpi-ros2.service, and
nothing takes the serial port: the arm node stays the only /dev/rrc owner.
"""
import argparse
import hashlib
import json
import posixpath
import shlex
import sys
from pathlib import Path

import paramiko

REPO = Path(__file__).resolve().parents[1]
UNIT_NAME = 'armpi-startup-pose.service'
UNIT_SRC = REPO / 'car_setup' / 'ros2' / UNIT_NAME
RUNTIME_DIR = '/opt/armpi-ros2'
STAGE_DIR = '/tmp/armpi-startup-pose-upload'
UPLOADS = [
    (REPO / 'car_setup' / 'ros2' / 'arm_startup_pose_ros.py',
     RUNTIME_DIR + '/arm_startup_pose_ros.py', '0644'),
    (REPO / 'car_setup' / 'ros2' / 'arm_startup_pose.py',
     RUNTIME_DIR + '/arm_startup_pose.py', '0755'),
    (REPO / 'car_setup' / 'ros2' / 'arm_startup_pose.json',
     RUNTIME_DIR + '/arm_startup_pose.json', '0644'),
    (UNIT_SRC, '/etc/systemd/system/' + UNIT_NAME, '0644'),
]


def sh(client, command, timeout=90):
    _, out, err = client.exec_command(command, timeout=timeout)
    text, errors = out.read().decode('utf-8', 'replace'), err.read().decode('utf-8', 'replace')
    code = out.channel.recv_exit_status()
    if code:
        raise RuntimeError('remote failed (%d): %s\n%s' % (code, command, errors or text))
    return text.strip()


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def install(client, sftp):
    sh(client, 'mkdir -p ' + STAGE_DIR)
    for source, remote, mode in UPLOADS:
        if not source.exists():
            raise SystemExit('Missing local file: ' + str(source))
        # posixpath, not Path: this runs on Windows where Path('/opt/x').parent
        # renders as '\opt' and would make the remote mkdir create a junk directory.
        sh(client, 'sudo -n mkdir -p ' + shlex.quote(posixpath.dirname(remote)))
        staged = STAGE_DIR + '/' + source.name
        sftp.put(str(source), staged)
        sh(client, 'sudo -n install -m %s %s %s' % (mode, shlex.quote(staged),
                                                    shlex.quote(remote)))
        remote_digest = sh(client, 'sudo -n sha256sum ' + shlex.quote(remote)).split()[0]
        local_digest = digest(source)
        state = 'OK' if remote_digest == local_digest else 'MISMATCH'
        print('  %-22s -> %-52s %s' % (source.name, remote, state))
        if state != 'OK':
            raise SystemExit('Digest mismatch after upload: ' + remote)
    sh(client, 'sudo -n rm -rf ' + STAGE_DIR)
    sh(client, 'sudo -n systemctl daemon-reload')
    print('  daemon-reload done. NOT enabled and NOT started.')


def status(client):
    print('enabled : ' + sh(client, 'systemctl is-enabled ' + UNIT_NAME + ' 2>&1 || true'))
    print('active  : ' + sh(client, 'systemctl is-active ' + UNIT_NAME + ' 2>&1 || true'))
    print('core    : ' + sh(client, 'systemctl is-active armpi-ros2 2>&1 || true'))
    for remote in (RUNTIME_DIR + '/arm_startup_pose_ros.py',
                   RUNTIME_DIR + '/arm_startup_pose.py',
                   RUNTIME_DIR + '/arm_startup_pose.json',
                   '/etc/systemd/system/' + UNIT_NAME):
        present = sh(client, 'test -f %s && echo present || echo MISSING' % shlex.quote(remote))
        print('%-52s %s' % (remote, present))
    for name in ('/run/armpi-ros2/arm-startup-pose.json', '/run/armpi-ros2/container.json'):
        print('--- %s ---' % name)
        print(sh(client, 'sudo -n cat %s 2>/dev/null | head -40 || echo "(none)"'
                 % shlex.quote(name)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', required=True)
    parser.add_argument('--user', default='pi')
    parser.add_argument('--password', default='raspberrypi')
    parser.add_argument('--action', required=True,
                        choices=('install', 'enable', 'disable', 'run', 'status', 'uninstall'))
    parser.add_argument('--confirm-arm-area-clear', action='store_true',
                        help='Required for action=run, which really moves the arm')
    args = parser.parse_args()
    if args.action == 'run' and not args.confirm_arm_area_clear:
        parser.error('action=run moves the arm; pass --confirm-arm-area-clear')

    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(args.host, username=args.user, password=args.password,
                   timeout=10, auth_timeout=10, allow_agent=False, look_for_keys=False)
    try:
        if args.action == 'install':
            install(client, client.open_sftp())
        elif args.action == 'enable':
            print(sh(client, 'sudo -n systemctl enable ' + UNIT_NAME + ' 2>&1'))
            print('The arm will now reset its own latch and move on every boot.')
            print('Undo with: --action disable')
        elif args.action == 'disable':
            sh(client, 'sudo -n systemctl disable --now ' + UNIT_NAME + ' 2>&1 || true')
            print('Disabled. Boot behaviour returns to the audited "no automatic arm pose".')
        elif args.action == 'run':
            # Type=oneshot, so `systemctl start` blocks until the travel is done.
            # A failing unit must still show its journal, so never let the
            # non-zero exit skip the evidence.
            failure = None
            try:
                sh(client, 'sudo -n systemctl start ' + UNIT_NAME, timeout=900)
            except RuntimeError as error:
                failure = error
            print('systemd Result: '
                  + sh(client, 'sudo -n systemctl show -p Result --value ' + UNIT_NAME))
            print(sh(client, 'sudo -n cat /run/armpi-ros2/arm-startup-pose.json 2>/dev/null '
                             '|| echo "(no result file)"'))
            print('--- journal ---')
            print(sh(client, 'sudo -n journalctl -u %s -n 60 --no-pager' % UNIT_NAME))
            if failure is not None:
                raise SystemExit('pose run failed: %s' % failure)
        elif args.action == 'uninstall':
            sh(client, 'sudo -n systemctl disable --now ' + UNIT_NAME + ' 2>&1 || true')
            sh(client, 'sudo -n rm -f /etc/systemd/system/' + UNIT_NAME)
            sh(client, 'sudo -n rm -f %s/arm_startup_pose.py %s/arm_startup_pose_ros.py '
                       '%s/arm_startup_pose.json' % (RUNTIME_DIR, RUNTIME_DIR, RUNTIME_DIR))
            sh(client, 'sudo -n systemctl daemon-reload')
            print('Removed. Nothing of this feature remains on the car.')
        else:
            status(client)
    finally:
        client.close()
    return 0


if __name__ == '__main__':
    sys.stdout.reconfigure(encoding='utf-8')
    sys.exit(main())
