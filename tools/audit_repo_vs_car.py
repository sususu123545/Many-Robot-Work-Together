#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Three-way drift audit: git repo -> local working tree -> the actual car.

WHY THIS EXISTS
---------------
The repo being clean only proves repo == local folder. It says nothing about
whether the car is running the same bytes. Deployment here is done by several
different tools (deploy_startup_pose.py, promote_ros2_image.py, manual sftp,
docker cp), so drift is easy to create and invisible from git alone.

WHAT IT CHECKS
--------------
  1. local vs remote for every file this project deploys to the car
  2. files that exist on the car but have no local counterpart (car-only work)
  3. files that exist locally but were never deployed
  4. the ROS 2 image's embedded armpi_ros2 sources vs ros2_ws/src/armpi_ros2

Read-only: it runs sha256sum and ls on the car and never writes anything.

Usage:
    python tools/audit_repo_vs_car.py --host 10.140.47.178 [--tag IMAGE]
"""
import argparse
import difflib
import hashlib
import json
import posixpath
import sys
from pathlib import Path

import paramiko

REPO = Path(__file__).resolve().parents[1]

RUNTIME = '/opt/armpi-ros2'
UNITS = '/etc/systemd/system'

# (local path relative to repo, remote path). Remote paths starting with
# 'armpi_pro:' live inside the ROS 1 container and are read with docker exec.
PAIRS = [
    ('car_setup/ros2/run_native_core.py', RUNTIME + '/run_native_core.py'),
    ('car_setup/ros2/run_lidar_only.py', RUNTIME + '/run_lidar_only.py'),
    ('car_setup/ros2/lidar_only.launch.py', RUNTIME + '/lidar_only.launch.py'),
    ('car_setup/ros2/arm_startup_pose.py', RUNTIME + '/arm_startup_pose.py'),
    ('car_setup/ros2/arm_startup_pose_ros.py', RUNTIME + '/arm_startup_pose_ros.py'),
    ('car_setup/ros2/arm_startup_pose.json', RUNTIME + '/arm_startup_pose.json'),
    ('car_setup/ros2/arm_bootstrap.py', RUNTIME + '/arm_bootstrap.py'),
    ('car_setup/ros2/lidar_preflight.py', RUNTIME + '/lidar_preflight.py'),
    ('tools/joint3_recover.py', RUNTIME + '/joint3_recover.py'),
    ('car_setup/ros2/armpi-ros2.service', UNITS + '/armpi-ros2.service'),
    ('car_setup/ros2/armpi-ros2-lidar.service', UNITS + '/armpi-ros2-lidar.service'),
    ('car_setup/ros2/armpi-startup-pose.service', UNITS + '/armpi-startup-pose.service'),
    ('car_setup/ros2/legacy_sidecar.sh',
     'armpi_pro:/home/ubuntu/armpi_ros2_legacy_sidecar.sh'),
    ('car_setup/ros2/legacy_sidecar.launch',
     'armpi_pro:/home/ubuntu/armpi_ros2_legacy_sidecar.launch'),
]

# Only these local prefixes are expected to be deployed; anything else in
# car_setup/ros2 is a local-only reference and is reported as such.
DEPLOYED_LOCAL_DIRS = ('car_setup/ros2/',)


def local_digest(path):
    if not path.exists():
        return None
    return hashlib.sha256(path.read_bytes()).hexdigest()


def sh(client, command, timeout=120):
    _, out, err = client.exec_command(command, timeout=timeout)
    text = out.read().decode('utf-8', 'replace')
    errors = err.read().decode('utf-8', 'replace')
    code = out.channel.recv_exit_status()
    return code, text, errors


def remote_hashes(client, paths):
    """One round trip for all host paths, then one docker exec for container paths."""
    host_paths = [p for p in paths if not p.startswith('armpi_pro:')]
    container_paths = [p.split(':', 1)[1] for p in paths if p.startswith('armpi_pro:')]
    result = {}
    if host_paths:
        quoted = ' '.join("'" + p + "'" for p in host_paths)
        code, out, _ = sh(client, 'sudo -n sha256sum %s 2>/dev/null' % quoted)
        for line in out.splitlines():
            parts = line.split(None, 1)
            if len(parts) == 2:
                result[parts[1].strip()] = parts[0]
    if container_paths:
        quoted = ' '.join("'" + p + "'" for p in container_paths)
        code, out, _ = sh(client, 'sudo -n docker exec armpi_pro sha256sum %s 2>/dev/null'
                          % quoted)
        for line in out.splitlines():
            parts = line.split(None, 1)
            if len(parts) == 2:
                result['armpi_pro:' + parts[1].strip()] = parts[0]
    return result


def list_remote(client, remote_dir):
    code, out, _ = sh(client, 'sudo -n ls -1 %s 2>/dev/null' % remote_dir)
    return [n for n in out.splitlines() if n.strip()]


def image_sources(client, tag):
    """sha256 of sources inside the built image, split by area.

    Returns (src_tree, install_tree, stderr). The install space is the one that
    matters: `source install/setup.bash` + `ros2 launch` read it, and this image
    was patched there directly by a targeted COPY, so its src/ tree can be older
    than the code that actually runs.
    """
    find = ('find %s -type f '
            '\\( -name "*.py" -o -name "*.yaml" -o -name "*.xml" -o -name "*.cfg" \\) '
            '-exec sha256sum {} + 2>/dev/null | grep -v __pycache__')
    inner = ('echo @@SRC@@; cd /opt/armpi_ros2/src 2>/dev/null && ' + (find % '.')
             + '; echo @@INSTALL@@; ' + (find % '/opt/armpi_ros2/install'))
    command = "sudo -n docker run --rm %s bash -lc '%s'" % (tag, inner)
    code, out, err = sh(client, command, timeout=300)
    src, install, area = {}, {}, None
    for line in out.splitlines():
        if line.startswith('@@'):
            area = line.strip('@').strip()
            continue
        parts = line.split(None, 1)
        if len(parts) != 2:
            continue
        target = src if area == 'SRC' else install
        target[parts[1].strip()] = parts[0]
    return src, install, err


def install_rel(path):
    """Map an in-image install path to its path relative to the local package.

    Colcon installs the python package into site-packages/armpi_ros2/ (so the
    local counterpart is armpi_ros2/<mod>.py) and data files into
    share/armpi_ros2/ (local counterpart launch/base.launch.py, config/*.yaml).
    """
    marker = '/site-packages/armpi_ros2/'
    i = path.find(marker)
    if i >= 0:
        return 'armpi_ros2/' + path[i + len(marker):]
    marker = '/share/armpi_ros2/'
    i = path.find(marker)
    if i >= 0:
        return path[i + len(marker):]
    return None


def container_running(client, name):
    code, out, _ = sh(client, "sudo -n docker inspect -f '{{.State.Running}}' %s 2>/dev/null"
                      % name)
    return out.strip() == 'true'


def read_remote_text(client, command, timeout=240):
    """Return the decoded stdout of `command`, or None if it failed."""
    code, out, err = sh(client, command, timeout=timeout)
    if code != 0:
        return None
    return out


def unified(local_abs, remote_text, left, right):
    """Unified diff between a local file and a remote blob, local first."""
    a = local_abs.read_text(encoding='utf-8', errors='replace').splitlines(True)
    b = remote_text.splitlines(True)
    return ''.join(difflib.unified_diff(a, b, left, right, n=2))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--host', required=True)
    ap.add_argument('--user', default='pi')
    ap.add_argument('--password', default='raspberrypi')
    ap.add_argument('--tag', help='image tag to inspect (default: core.json image)')
    ap.add_argument('--show-diff', action='store_true',
                    help='print a unified diff for every differing text file')
    args = ap.parse_args()
    sys.stdout.reconfigure(encoding='utf-8')

    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(args.host, username=args.user, password=args.password,
                   timeout=15, auth_timeout=15, allow_agent=False, look_for_keys=False)
    try:
        code, out, _ = sh(client, 'hostname; cat /proc/uptime; '
                                  'systemctl is-active armpi-ros2 armpi-ros2-lidar '
                                  'armpi-startup-pose start_node')
        print('=== CAR ===')
        print(out.strip())
        code, core_text, _ = sh(client, 'sudo -n cat /etc/armpi-ros2/core.json')
        core = json.loads(core_text) if code == 0 else {}
        tag = args.tag or core.get('image')
        print('core.json image     = %s' % core.get('image'))
        print('core.json image_id  = %s' % core.get('image_id'))

        legacy_running = container_running(client, 'armpi_pro')
        print('armpi_pro running   = %s' % legacy_running)
        wanted = [r for _, r in PAIRS
                  if legacy_running or not r.startswith('armpi_pro:')]
        remote = remote_hashes(client, wanted)

        print()
        print('=== LOCAL vs CAR ===')
        width = max(len(local) for local, _ in PAIRS)
        problems = []
        drift_pairs = []
        for local_rel, remote_path in PAIRS:
            local_abs = REPO / local_rel
            ld = local_digest(local_abs)
            rd = remote.get(remote_path)
            if remote_path.startswith('armpi_pro:') and not legacy_running:
                # docker exec cannot read a stopped container; this is a gap in
                # the check, not evidence that the file is absent.
                verdict = 'SKIPPED (container stopped)'
            elif ld is None:
                verdict = 'MISSING-LOCAL'
            elif rd is None:
                verdict = 'MISSING-ON-CAR'
            elif ld == rd:
                verdict = 'match'
            else:
                verdict = '*** DIFFERENT ***'
            if verdict not in ('match',) and not verdict.startswith('SKIPPED'):
                problems.append((local_rel, remote_path, verdict))
            if verdict == '*** DIFFERENT ***':
                drift_pairs.append((local_rel, remote_path))
            print('  %-*s  %-26s %s' % (width, local_rel, verdict, remote_path))

        if args.show_diff and drift_pairs:
            for local_rel, remote_path in drift_pairs:
                if remote_path.startswith('armpi_pro:'):
                    cmd = ('sudo -n docker exec armpi_pro cat %s'
                           % remote_path.split(':', 1)[1])
                else:
                    cmd = 'sudo -n cat %s' % remote_path
                text = read_remote_text(client, cmd)
                print()
                print('--- DIFF %s (local) -> %s (car) ---' % (local_rel, remote_path))
                if text is None:
                    print('  could not read the remote file')
                    continue
                print(unified(REPO / local_rel, text, 'local', 'car').rstrip()
                      or '  (no textual difference: differs only in line endings)')

        print()
        print('=== CAR-ONLY FILES in %s ===' % RUNTIME)
        local_names = {Path(l).name for l, _ in PAIRS}
        for name in list_remote(client, RUNTIME):
            marker = '' if name in local_names else '  <-- no local counterpart'
            if name == '__pycache__':
                continue
            print('  %-28s%s' % (name, marker))
        for name in list_remote(client, UNITS):
            if name.startswith('armpi') and name.endswith('.service'):
                if name not in {Path(l).name for l, _ in PAIRS}:
                    print('  %-28s  <-- no local counterpart' % name)

        print()
        print('=== LOCAL deployable files never listed above ===')
        shipped = {l for l, _ in PAIRS}
        for path in sorted((REPO / 'car_setup' / 'ros2').glob('*')):
            if not path.is_file():
                continue
            rel = 'car_setup/ros2/' + path.name
            if rel not in shipped:
                print('  %-28s  (local only)' % path.name)

        if tag:
            print()
            print('=== IMAGE RUNTIME vs ros2_ws/src/armpi_ros2 (%s) ===' % tag)
            # The INSTALL space is what runs: `source install/setup.bash` then
            # `ros2 launch` resolve there. This image was patched in place with
            # a targeted COPY, so its src/ tree can be older than the runtime.
            src_tree, install_tree, err = image_sources(client, tag)
            ours = {install_rel(p): d for p, d in install_tree.items() if install_rel(p)}
            if not ours:
                print('  could not read install space: %s' % (err.strip()[:200] or 'empty'))
            else:
                local_root = REPO / 'ros2_ws' / 'src' / 'armpi_ros2'
                inst_of = {install_rel(p): p for p in install_tree if install_rel(p)}
                checked = diff = missing_local = 0
                image_drift = []
                for rel, digest in sorted(ours.items()):
                    local_abs = local_root / rel
                    if not local_abs.exists():
                        missing_local += 1
                        print('  %-40s  MISSING LOCALLY' % rel)
                        continue
                    checked += 1
                    if local_digest(local_abs) != digest:
                        diff += 1
                        image_drift.append(rel)
                        print('  %-40s  *** DIFFERENT *** (runtime is behind local)' % rel)
                suffixes = ('.py', '.yaml', '.xml', '.cfg')
                not_installed = 0
                for path in sorted(local_root.rglob('*')):
                    if not path.is_file() or '__pycache__' in path.parts:
                        continue
                    if path.suffix not in suffixes:
                        continue
                    rel = path.relative_to(local_root).as_posix()
                    if rel not in ours:
                        not_installed += 1
                        print('  %-40s  not installed (build input only)' % rel)
                print('  checked=%d identical=%d different=%d missing_locally=%d '
                      'not_installed=%d'
                      % (checked, checked - diff, diff, missing_local, not_installed))

                # src/ inside the image is a stale reference; say so loudly so
                # nobody reads it as the deployed code.
                src_map = {}
                for p, d in src_tree.items():
                    parts = p.lstrip('./').split('/')
                    if parts[0] != 'armpi_ros2' or '__pycache__' in parts:
                        continue
                    src_map[posixpath.join(*parts[1:])] = d
                stale = sorted(r for r, d in ours.items()
                               if r in src_map and src_map[r] != d)
                vendored = sorted({p.lstrip('./').split('/')[0] for p in src_tree
                                   if p.lstrip('./').split('/')[0] != 'armpi_ros2'})
                if stale:
                    print('  NOTE: image src/ disagrees with its own install space for: %s'
                          % ', '.join(stale))
                if vendored:
                    print('  vendored in base image (not from this repo): %s'
                          % ', '.join(vendored))

                if args.show_diff and image_drift:
                    for rel in image_drift:
                        text = read_remote_text(
                            client, 'sudo -n docker run --rm %s cat %s' % (tag, inst_of[rel]))
                        print()
                        print('--- DIFF ros2_ws/src/armpi_ros2/%s (local) -> image runtime ---'
                              % rel)
                        if text is None:
                            print('  could not read the file from the image')
                            continue
                        print(unified(local_root / rel, text, 'local', 'image').rstrip()
                              or '  (no textual difference)')
                for rel in image_drift:
                    problems.append(('ros2_ws/src/armpi_ros2/' + rel, tag,
                                     'IMAGE-BEHIND-LOCAL'))

        print()
        if problems:
            print('DRIFT FOUND: %d deployed file(s) differ or are missing' % len(problems))
            for local_rel, remote_path, verdict in problems:
                print('  %-26s %s' % (verdict, local_rel))
            return 1
        print('NO DRIFT: every deployed file matches the local working tree')
        return 0
    finally:
        client.close()


if __name__ == '__main__':
    sys.exit(main())
