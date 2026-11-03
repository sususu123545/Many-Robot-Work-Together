#!/usr/bin/env python3
"""Run a shell command on the car over SSH, with the credentials kept off-repo.

The password lives in ``tools/runtime/arm_creds.json`` (git-ignored), which the
other tools in this directory already use. Nothing here hardcodes it.

Why a helper at all: the Pi answers SSH slowly under load, and a bare
``exec_command`` with the default timeout reports a pipe timeout instead of the
command's output, which reads like a connection failure. This retries the read
and separates "already connected but nothing came back" from "could not log in".

Usage:
    python3 tools/car_shell.py 'uptime -p'
    python3 tools/car_shell.py --host 192.168.86.178 'ros2 node list'
"""

import argparse
import json
import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CREDS = os.path.join(REPO, 'tools', 'runtime', 'arm_creds.json')
DEFAULT_HOST = '192.168.86.178'
DEFAULT_USER = 'pi'


def credentials():
    if not os.path.exists(CREDS):
        raise SystemExit('missing credentials file: %s' % CREDS)
    with open(CREDS, 'r', encoding='utf-8') as handle:
        payload = json.load(handle)
    password = payload.get('password')
    if not password:
        raise SystemExit('credentials file has no password field')
    return password


def connect(host=DEFAULT_HOST, user=DEFAULT_USER, timeout=15):
    import paramiko
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(host, username=user, password=credentials(),
                   timeout=timeout, banner_timeout=timeout, auth_timeout=timeout)
    return client


def run(client, command, timeout=60, quiet_on_timeout=True):
    """Execute one command; returns (exit_code, stdout, stderr).

    ``exit_code`` of ``None`` means the command was sent but no status came back
    before the timeout, which on this Pi usually means it is still running.
    """
    _, stdout, stderr = client.exec_command(command, timeout=timeout)
    try:
        out = stdout.read().decode('utf-8', 'replace')
        err = stderr.read().decode('utf-8', 'replace')
        code = stdout.channel.recv_exit_status()
        return code, out, err
    except Exception as error:  # noqa: BLE001 - reported to the caller as-is
        partial = ''
        try:
            partial = stdout.channel.recv(65536).decode('utf-8', 'replace')
        except Exception:  # noqa: BLE001
            pass
        if quiet_on_timeout:
            return None, partial, 'timeout after %ss: %s' % (timeout, error)
        raise


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('command', nargs='+', help='command to run on the car')
    parser.add_argument('--host', default=DEFAULT_HOST)
    parser.add_argument('--user', default=DEFAULT_USER)
    parser.add_argument('--timeout', type=float, default=60.0)
    args = parser.parse_args(argv)

    client = connect(args.host, args.user)
    try:
        code, out, err = run(client, ' '.join(args.command), args.timeout)
    finally:
        client.close()
    sys.stdout.write(out)
    if err.strip():
        sys.stderr.write(err)
    return 0 if code == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
