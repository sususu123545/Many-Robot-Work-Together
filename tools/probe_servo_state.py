#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""PC-side wrapper: run tools/servo_bus_probe.py INSIDE the armpi-ros2 container.

WHY A WRAPPER
-------------
The probe needs the `armpi_ros2` package (colcon install space) and /dev/rrc,
so it can only run inside the image. Uploading it as base64 and piping it into
`python3 -` avoids leaving a file on the Pi and avoids quoting hell.

READ-ONLY GUARANTEE
-------------------
The payload only ever issues read commands: 0x05 position, 0x09 temperature,
0x0D torque state, 0x07 supply voltage. It never sends move (0x01), torque
load/unload (0x0B/0x0C) or stop (0x03). This wrapper adds nothing of its own.

SAFETY PRE-CHECK
----------------
Before running, the wrapper asks the Pi who owns /dev/rrc. If ANY process holds
it, the run is refused: two serial owners on an exclusive Linux fd produce
corrupt replies, and a wrong reading is worse than no reading. Use
--no-busy-check only when you already know the owner is a benign reader.

Usage:
    python tools/probe_servo_state.py --host 10.18.115.178
        [--user pi] [--password raspberrypi] [--timeout 60]
        [--probe tools/servo_bus_probe.py] [--no-busy-check] [--json-only]
"""
import argparse
import base64
import os
import re
import sys
import threading
import time

import paramiko

T0 = time.time()
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

DEFAULT_PROBE = os.path.join(HERE, 'servo_bus_probe.py')
CORE_JSON = '/etc/armpi-ros2/core.json'
REMOTE_PROBE = '/tmp/servo_bus_probe.py'


def log(msg):
    print('[%7.1fs] %s' % (time.time() - T0, msg), flush=True)


def run_cmd(client, cmd, timeout):
    """Run a command with a real deadline; returns (rc, stdout, stderr)."""
    log('  exec: %s' % (cmd if len(cmd) < 110 else cmd[:107] + '...'))
    try:
        chan = client.get_transport().open_session(timeout=timeout)
        chan.settimeout(timeout)
        chan.exec_command(cmd)
    except Exception as e:  # noqa: BLE001
        return None, '', 'OPEN FAILED: %r' % (e,)

    out_b, err_b = b'', b''
    deadline = time.time() + timeout
    try:
        while True:
            if time.time() > deadline:
                return (None, out_b.decode('utf-8', 'replace'),
                        err_b.decode('utf-8', 'replace')
                        + '\nTIMEOUT: still running after %ss (partial output kept)' % timeout)
            got = False
            if chan.recv_ready():
                data = chan.recv(65536)
                if data:
                    out_b += data
                    got = True
            if chan.recv_stderr_ready():
                data = chan.recv_stderr(65536)
                if data:
                    err_b += data
                    got = True
            if not got:
                if chan.exit_status_ready():
                    while chan.recv_ready():
                        data = chan.recv(65536)
                        if not data:
                            break
                        out_b += data
                    while chan.recv_stderr_ready():
                        data = chan.recv_stderr(65536)
                        if not data:
                            break
                        err_b += data
                    break
                time.sleep(0.05)
        rc = chan.recv_exit_status()
    except Exception as e:  # noqa: BLE001
        return None, out_b.decode('utf-8', 'replace'), 'TIMEOUT/ERROR: %r' % (e,)
    return rc, out_b.decode('utf-8', 'replace'), err_b.decode('utf-8', 'replace')


def connect(args):
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    for attempt in range(1, args.retries + 1):
        log('connecting to %s@%s:%d (attempt %d/%d) ...'
            % (args.user, args.host, args.port, attempt, args.retries))
        try:
            client.connect(hostname=args.host, port=args.port,
                           username=args.user, password=args.password,
                           timeout=args.timeout, banner_timeout=args.timeout,
                           auth_timeout=args.timeout,
                           allow_agent=False, look_for_keys=False)
            log('CONNECTED + AUTH OK')
            return client
        except Exception as e:  # noqa: BLE001
            log('  attempt %d failed: %r' % (attempt, e))
            try:
                client.close()
            except Exception:  # noqa: BLE001
                pass
            if attempt < args.retries:
                time.sleep(args.retry_sleep)
    return None


def parse_result(stdout):
    """Pull the RESULT_JSON= payload out of the probe's mixed human output."""
    for line in stdout.splitlines():
        if line.startswith('RESULT_JSON='):
            import json
            return json.loads(line[len('RESULT_JSON='):])
    return None


def report(result):
    joints = result.get('joints') or {}
    failures = result.get('failures') or {}
    if not joints:
        print('  no joint rows at all -- the bus produced nothing usable', flush=True)
    order = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'r_joint']
    print('  %-9s %5s %6s %5s %11s %8s' % ('joint', 'servo', 'raw', 'temp', 'limits', 'inrange'),
          flush=True)
    for name in order:
        row = joints.get(name)
        if row is None:
            continue
        print('  %-9s %5d %6d %5d %11s %8s'
              % (name, row['id'], row['raw'], row['temperature'],
                 '%d..%d' % tuple(row['limits']), 'yes' if row['in_range'] else 'NO'),
              flush=True)
    for name, err in failures.items():
        print('  %-9s READ FAILED: %s' % (name, err), flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--host', required=True)
    ap.add_argument('--user', default='pi')
    ap.add_argument('--password', default='raspberrypi')
    ap.add_argument('--port', type=int, default=22)
    ap.add_argument('--timeout', type=float, default=90.0,
                    help='per-command deadline; the container start alone can take 10s+')
    ap.add_argument('--retries', type=int, default=5)
    ap.add_argument('--retry-sleep', type=float, default=6.0)
    ap.add_argument('--probe', default=DEFAULT_PROBE)
    ap.add_argument('--no-busy-check', action='store_true')
    ap.add_argument('--json-only', action='store_true')
    args = ap.parse_args()

    with open(args.probe, 'r', encoding='utf-8') as handle:
        source = handle.read()
    payload = base64.b64encode(source.encode('utf-8')).decode('ascii')
    log('probe source: %s (%d bytes -> %d b64)' % (args.probe, len(source), len(payload)))

    client = connect(args)
    if client is None:
        log('ERROR: could not connect')
        return 1
    rc_all = 0
    try:
        if not args.no_busy_check:
            rc, out, err = run_cmd(
                client,
                "sudo -n fuser -v /dev/rrc 2>&1; echo BUSY_RC=$?; "
                "systemctl is-active armpi-ros2 start_node 2>&1; echo SVC_RC=$?; "
                "cat /proc/uptime",
                args.timeout)
            log('busy check rc=%s' % rc)
            print(out, flush=True)
            if err.strip():
                print(err, flush=True)
            match = re.search(r'BUSY_RC=(\d+)', out or '')
            if match is None:
                print('REFUSED: could not read the bus-owner probe result', flush=True)
                return 1
            if match.group(1) == '0':
                print('REFUSED: /dev/rrc is held by another process (see table above). '
                      'Two owners on the exclusive fd corrupt replies. Stop the owner first, '
                      'or pass --no-busy-check if you know it is a benign reader.', flush=True)
                return 1

        upload = ("printf '%s' '" + payload + "' | base64 -d > " + REMOTE_PROBE
                  + " && wc -c < " + REMOTE_PROBE)
        rc, out, err = run_cmd(client, upload, args.timeout)
        log('upload rc=%s' % rc)
        if rc != 0:
            print(out, flush=True)
            print(err, flush=True)
            return 1

        rc, out, err = run_cmd(client, 'cat ' + CORE_JSON, args.timeout)
        image = 'armpi-ros2-migration:stage-4xhjqf'
        if rc == 0:
            import json as _json
            try:
                image = _json.loads(out.strip()).get('image', image)
            except Exception:  # noqa: BLE001
                pass
        log('image: %s' % image)

        inner = ('set -o pipefail; sudo -n docker run --rm -i --device /dev/rrc:/dev/rrc '
                 + image
                 + " bash -lc 'source /opt/armpi_ros2/install/setup.bash && exec python3 -'"
                 + ' < ' + REMOTE_PROBE)
        t_start = time.time()
        rc, out, err = run_cmd(client, inner, args.timeout)
        elapsed = time.time() - t_start
        log('probe rc=%s in %.1fs' % (rc, elapsed))
        rc_all = 0 if rc == 0 else 1
        if err.strip():
            print('--- STDERR ---', flush=True)
            print(err, flush=True)
        if not args.json_only or rc != 0:
            print('--- PROBE OUTPUT ---', flush=True)
            print(out, flush=True)

        result = parse_result(out or '')
        if result is None:
            print('NO_RESULT_JSON: the probe did not reach the end of its run', flush=True)
            rc_all = 1
        else:
            print('--- SUMMARY ---', flush=True)
            report(result)
            if result.get('failures'):
                rc_all = 1
            out_of_range = [n for n, r in (result.get('joints') or {}).items()
                            if not r.get('in_range')]
            if out_of_range:
                print('  OUT OF RANGE: %s' % ', '.join(sorted(out_of_range)), flush=True)
                rc_all = 1
    finally:
        try:
            t = threading.Thread(target=lambda: client.close(), daemon=True)
            t.start()
            t.join(timeout=4)
        except Exception:  # noqa: BLE001
            pass
    print('EXIT=%d' % rc_all, flush=True)
    return rc_all


if __name__ == '__main__':
    os._exit(main())
