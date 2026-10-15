#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Promote a newly built ROS 2 image to the pinned core, with a rollback copy.

WHY THIS EXISTS
---------------
tools/switch_ros2_core.py was the ORIGINAL one-shot cutover: it asserts that
/etc/armpi-ros2 and /opt/armpi-ros2 do not exist yet, so it cannot be re-run on
a car that is already migrated. Updating the pinned image afterwards needs a
narrower, repeatable tool.

WHAT IT CHANGES (exactly four things, nothing else)
---------------------------------------------------
  1. /home/pi/armpi-ros2-promote.<rand>/   backup of the current core.json and
                                           run_native_core.py, plus a manifest
  2. /opt/armpi-ros2/run_native_core.py    replaced from car_setup/ros2/
  3. /etc/armpi-ros2/core.json             image + image_id updated; the
                                           bmi_library fields are preserved
  4. (only with --restart) systemctl restart armpi-ros2

It never starts, stops or reconfigures ROS 1, never touches the lidar service,
never enables any unit, and never sends a motion command.

WHY image_id MATTERS
--------------------
Both runners (run_native_core.py and run_lidar_only.py) start by asserting
    docker image inspect <image> --format {{.Id}}  ==  core.json.image_id
so a rebuilt image whose ID is not written back here makes the core refuse to
start. That failure is deliberate: it stops a half-updated car from running.

Usage:
    python tools/promote_ros2_image.py --host 10.18.115.178 \
        --tag armpi-ros2-migration:cam1 [--expect-pkg v4l2_camera] [--restart]
        [--dry-run]
"""
import argparse
import hashlib
import json
import posixpath
import re
import shlex
import sys
import time
from pathlib import Path

import paramiko

REPO = Path(__file__).resolve().parents[1]
LOCAL_CORE_SCRIPT = REPO / 'car_setup' / 'ros2' / 'run_native_core.py'
REMOTE_CORE_SCRIPT = '/opt/armpi-ros2/run_native_core.py'
REMOTE_CORE_JSON = '/etc/armpi-ros2/core.json'
STAGE_PREFIX = '/home/pi/armpi-ros2-promote.'


def sh(client, command, timeout=90):
    _, out, err = client.exec_command(command, timeout=timeout)
    text = out.read().decode('utf-8', 'replace')
    errors = err.read().decode('utf-8', 'replace')
    code = out.channel.recv_exit_status()
    if code:
        raise RuntimeError('remote failed (%d): %s\n%s' % (code, command, errors or text))
    return text.strip()


def try_sh(client, command, timeout=90):
    """Same as sh() but returns (code, stdout, stderr) instead of raising."""
    _, out, err = client.exec_command(command, timeout=timeout)
    text = out.read().decode('utf-8', 'replace')
    errors = err.read().decode('utf-8', 'replace')
    return out.channel.recv_exit_status(), text, errors


def image_id(client, tag):
    return sh(client, "docker image inspect %s --format '{{.Id}}'" % shlex.quote(tag))


def package_present(client, tag, package):
    command = ("docker run --rm %s bash -lc "
               "'source /opt/ros/jazzy/setup.bash >/dev/null 2>&1 && ros2 pkg list 2>/dev/null'"
               % shlex.quote(tag))
    code, out, err = try_sh(client, command, timeout=180)
    if code:
        raise RuntimeError('could not list packages in %s: %s' % (tag, err or out))
    return package in out.split()


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--host', required=True)
    parser.add_argument('--user', default='pi')
    parser.add_argument('--password', default='raspberrypi')
    parser.add_argument('--port', type=int, default=22)
    parser.add_argument('--tag', required=True, help='new image tag to pin')
    parser.add_argument('--expect-pkg', action='append', default=[],
                        help='ROS 2 package that must exist in the image (repeatable)')
    parser.add_argument('--restart', action='store_true',
                        help='restart armpi-ros2 after promoting (arm stays unpowered)')
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args()
    sys.stdout.reconfigure(encoding='utf-8')

    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(args.host, port=args.port, username=args.user, password=args.password,
                   timeout=15, auth_timeout=15, allow_agent=False, look_for_keys=False)
    try:
        new_id = image_id(client, args.tag)
        print('NEW_IMAGE_TAG   = %s' % args.tag)
        print('NEW_IMAGE_ID    = %s' % new_id)

        current = json.loads(sh(client, 'sudo -n cat ' + REMOTE_CORE_JSON))
        print('OLD_IMAGE_TAG   = %s' % current.get('image'))
        print('OLD_IMAGE_ID    = %s' % current.get('image_id'))
        if current.get('image') == args.tag and current.get('image_id') == new_id:
            print('ALREADY_PROMOTED: core.json already pins this exact image. Nothing to do.')
            return 0

        for package in args.expect_pkg:
            present = package_present(client, args.tag, package)
            print('EXPECT_PKG      = %-24s %s' % (package, 'PRESENT' if present else 'MISSING'))
            if not present:
                raise SystemExit('REFUSED: %s is not in %s; promoting it would take the car '
                                 'backwards' % (package, args.tag))

        # A wrong tag is the one mistake that silently bricks the boot, so prove
        # the new image can at least import the core package before switching to it.
        probe = ("docker run --rm %s bash -lc 'source /opt/armpi_ros2/install/setup.bash "
                 ">/dev/null 2>&1 && python3 -c \"import armpi_ros2.servo_protocol as m; "
                 "print(sorted(k for k in dir(m) if not k.startswith(chr(95)))[:8])\"'"
                 % shlex.quote(args.tag))
        code, out, err = try_sh(client, probe, timeout=180)
        print('IMAGE_SELFCHECK = %s' % ('OK ' + out.strip() if code == 0 else 'FAILED'))
        if code:
            raise SystemExit('REFUSED: new image cannot import armpi_ros2: %s' % (err or out))

        if not LOCAL_CORE_SCRIPT.exists():
            raise SystemExit('Missing local file: %s' % LOCAL_CORE_SCRIPT)

        if args.dry_run:
            print('DRY_RUN: would back up, upload run_native_core.py and rewrite core.json')
            return 0

        stage = sh(client, 'mktemp -d %sXXXXXX' % STAGE_PREFIX)
        if not re.fullmatch(re.escape(STAGE_PREFIX) + r'[A-Za-z0-9]+', stage):
            raise RuntimeError('Unexpected staging path: %s' % stage)
        sh(client, 'mkdir -p %s' % shlex.quote(stage))
        sh(client, 'sudo -n cp -a %s %s/core.json.bak' % (REMOTE_CORE_JSON, shlex.quote(stage)))
        sh(client, 'sudo -n cp -a %s %s/run_native_core.py.bak'
           % (REMOTE_CORE_SCRIPT, shlex.quote(stage)))
        sh(client, 'sudo -n chown %s:%s %s/*.bak 2>/dev/null || true'
           % (args.user, args.user, shlex.quote(stage)))

        with client.open_sftp() as sftp:
            sftp.put(str(LOCAL_CORE_SCRIPT), posixpath.join(stage, 'run_native_core.py'))
        sh(client, 'sudo -n install -m 0755 %s/run_native_core.py %s'
           % (shlex.quote(stage), shlex.quote(REMOTE_CORE_SCRIPT)))

        updated = dict(current)
        updated['image'] = args.tag
        updated['image_id'] = new_id
        payload = json.dumps(updated, sort_keys=True)
        # Heredoc, not printf: core.json is JSON with quotes and braces, and it
        # must land byte-exact for the runners' image-ID assertion to pass.
        sh(client, 'sudo -n bash -c ' + shlex.quote(
            "cat > %s <<'ARMPI_CORE_EOF'\n%s\nARMPI_CORE_EOF"
            % (REMOTE_CORE_JSON, payload)))

        verify = json.loads(sh(client, 'sudo -n cat ' + REMOTE_CORE_JSON))
        assert verify['image'] == args.tag and verify['image_id'] == new_id, verify
        for key in ('bmi_library', 'bmi_library_sha256'):
            assert verify.get(key) == current.get(key), 'lost %s during promotion' % key
        digest = sh(client, 'sudo -n sha256sum ' + shlex.quote(REMOTE_CORE_SCRIPT)).split()[0]
        local_digest = hashlib.sha256(LOCAL_CORE_SCRIPT.read_bytes()).hexdigest()
        print('CORE_SCRIPT     = %s' % ('digest OK' if digest == local_digest
                                        else 'DIGEST MISMATCH %s != %s' % (digest, local_digest)))
        print('BACKUP          = %s' % stage)
        print('PROMOTED        = %s -> %s' % (current.get('image'), args.tag))

        if args.restart:
            sh(client, 'sudo -n systemctl restart armpi-ros2', timeout=180)
            time.sleep(20)
            print('SERVICE         = %s' % sh(client, 'systemctl is-active armpi-ros2'))
            code, out, _ = try_sh(client, 'sudo -n cat /run/armpi-ros2/container.json')
            print(out.strip() if code == 0 else '(no container.json yet)')
        else:
            print('NOT_RESTARTED: run `systemctl restart armpi-ros2` when the arm area is clear.')
    finally:
        client.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
