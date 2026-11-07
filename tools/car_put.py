#!/usr/bin/env python3
"""Copy files to and from the car over SFTP, credentials kept off-repo.

``car_shell.py`` runs commands; this handles the other half.  The Pi answers
SFTP slowly when the servo bus is busy, so transfers report progress and retry
once before failing loudly.

Usage
    python3 tools/car_put.py put tools/perception/detectors.py /home/pi/
    python3 tools/car_put.py put models/yolo11n.onnx /home/pi/vision-models/
    python3 tools/car_put.py get /home/pi/out.json artifacts/
    python3 tools/car_put.py ls /home/pi/vision-models/
"""

import argparse
import json
import os
import posixpath
import sys
import time

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CREDS = os.path.join(REPO, 'tools', 'runtime', 'arm_creds.json')
DEFAULT_HOST = '100.102.57.19'
DEFAULT_USER = 'pi'


def credentials():
    if not os.path.exists(CREDS):
        raise SystemExit('missing credentials file: %s' % CREDS)
    with open(CREDS, 'r', encoding='utf-8') as handle:
        password = json.load(handle).get('password')
    if not password:
        raise SystemExit('credentials file has no password field')
    return password


def remote_path(value, user):
    """Normalise a remote POSIX path and refuse anything MSYS has mangled.

    Git Bash rewrites arguments that look like POSIX absolute paths into Windows
    ones, so ``/home/pi/vision`` reaches Python as
    ``D:/workbuddy/.../PortableGit/home/pi/vision`` and SFTP quite happily
    creates a directory called ``D:`` on the Pi.  Two defences: accept ``~/``
    (never rewritten), and reject a colon or backslash outright.
    """
    if ':' in value or '\\' in value:
        raise SystemExit(
            'remote path looks like a Windows path: %s\n'
            'Git Bash rewrote it. Use a path starting with ~/ instead, e.g.\n'
            '    python tools/car_put.py put <local> ~/vision/\n'
            'or prefix the command with MSYS_NO_PATHCONV=1' % value)
    if value.startswith('~'):
        value = '/home/' + user + value[1:]
    normalized = posixpath.normpath(value)
    # normpath eats the trailing slash, which is the only signal that the target
    # is a directory. Losing it makes `put file ~/vision/` overwrite the folder
    # with a regular file.
    if value.endswith('/') and not normalized.endswith('/'):
        normalized += '/'
    return normalized


def connect(host, user, timeout=20):
    import paramiko
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(host, username=user, password=credentials(),
                   timeout=timeout, banner_timeout=timeout, auth_timeout=timeout)
    return client


def ensure_remote_dir(client, remote_path):
    """Create the parent directory with a remote mkdir -p.

    Doing this over SFTP one level at a time walks through /home and /usr, which
    are not writable by `pi` and make paramiko raise EACCES on a path that
    already exists.  A single shell call is both simpler and accurate.
    """
    parent = posixpath.dirname(remote_path)
    if not parent or parent == '/':
        return
    escaped = parent.replace('"', '\\"')
    _, stdout, stderr = client.exec_command('mkdir -p "%s"' % escaped, timeout=20)
    code = stdout.channel.recv_exit_status()
    if code != 0:
        raise SystemExit('mkdir -p %s failed: %s'
                         % (parent, stderr.read().decode('utf-8', 'replace').strip()))


def stream_upload(client, local, remote):
    """Push a file through a plain exec channel instead of SFTP.

    Measured on this link: SFTP managed ~1.6 KB/s because its 32 KB window waits
    for an acknowledgement every chunk over a relayed path with 100-200 ms RTT.
    A raw channel just writes, so it is limited by bandwidth rather than latency.
    """
    channel = client.get_transport().open_session()
    channel.exec_command('cat > "%s"' % remote)
    written = 0
    with open(local, 'rb') as handle:
        while True:
            chunk = handle.read(65536)
            if not chunk:
                break
            channel.sendall(chunk)
            written += len(chunk)
    channel.shutdown_write()
    code = channel.recv_exit_status()
    if code != 0:
        error = channel.recv_stderr(4096).decode('utf-8', 'replace').strip()
        raise SystemExit('remote write failed (%d): %s' % (code, error))
    return written


def do_put(client, sftp, local, remote):
    """Upload one file; a trailing slash means the target is a directory."""
    if remote.endswith('/') or remote == '/':
        remote = posixpath.join(remote, os.path.basename(local))
    else:
        try:
            is_dir = sftp.stat(remote).st_mode & 0o040000 != 0
        except IOError:
            is_dir = False
        if is_dir:
            remote = posixpath.join(remote, os.path.basename(local))
    ensure_remote_dir(client, remote)

    size = os.path.getsize(local)
    started = time.time()
    stream_upload(client, local, remote)
    elapsed = time.time() - started

    actual = sftp.stat(remote).st_size
    if actual != size:
        raise SystemExit('size mismatch after upload: %d != %d' % (actual, size))
    print('%s -> %s  (%d bytes, %.0f KiB/s)'
          % (local, remote, actual, size / 1024.0 / max(elapsed, 1e-6)))


def do_get(sftp, remote, local):
    if os.path.isdir(local) or local.endswith(('/', '\\')):
        local = os.path.join(local, os.path.basename(remote))
    parent = os.path.dirname(os.path.abspath(local))
    if parent and not os.path.isdir(parent):
        os.makedirs(parent)
    sftp.get(remote, local)
    print('%s -> %s  (%d bytes)' % (remote, local, os.path.getsize(local)))


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('action', choices=('put', 'get', 'ls', 'rm'))
    parser.add_argument('source')
    parser.add_argument('target', nargs='?')
    parser.add_argument('--host', default=DEFAULT_HOST)
    parser.add_argument('--user', default=DEFAULT_USER)
    args = parser.parse_args()

    client = connect(args.host, args.user)
    try:
        sftp = client.open_sftp()
    except Exception as error:               # noqa: BLE001
        client.close()
        raise SystemExit('could not open SFTP: %s' % error)

    try:
        if args.action == 'put':
            if not args.target:
                raise SystemExit('put needs a remote target')
            do_put(client, sftp, args.source,
                   remote_path(args.target, args.user))
        elif args.action == 'get':
            if not args.target:
                raise SystemExit('get needs a local target')
            do_get(sftp, remote_path(args.source, args.user), args.target)
        elif args.action == 'rm':
            target = remote_path(args.source, args.user)
            sftp.remove(target)
            print('removed %s' % target)
        else:
            folder = remote_path(args.source, args.user)
            for entry in sorted(sftp.listdir_attr(folder),
                                key=lambda e: e.filename):
                kind = 'd' if entry.st_mode & 0o040000 else '-'
                print('%s %10d  %s' % (kind, entry.st_size, entry.filename))
    finally:
        sftp.close()
        client.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
