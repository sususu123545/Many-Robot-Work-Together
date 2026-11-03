#!/usr/bin/env python3
"""Deploy and run the ball search on the car, end to end, in one command.

Doing this by hand costs six steps and three of them are easy to get wrong in a
way that looks like a perception bug:

* forgetting to rebuild after a source change (the node runs, old behaviour);
* granting motion and then starting the sweep before the parameter has actually
  propagated (the sweep silently holds);
* leaving `enable_motion` on afterwards, so the next person finds a robot that
  can move without having authorised it.

This script does those in order, prints what it sees at each step, and turns
motion back off at the end no matter how it exits.

It also recognises the failure this repository actually hit on 2026-11-03: the
Pi answering SSH authentication in about a second while every session channel
(exec, sftp) hangs forever. That is not a network problem and retrying does not
help -- no new process can be created on the board. The script says so instead of
timing out with a traceback.

Usage:
    python3 tools/car_search_once.py                 # full run
    python3 tools/car_search_once.py --preflight     # checks only, no motion
    python3 tools/car_search_once.py --no-build      # skip colcon
"""

import argparse
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)

from car_shell import connect, run  # noqa: E402

REMOTE_WS = '/opt/armpi_ros2'
ROS_SETUP = ('source /opt/ros/jazzy/setup.bash 2>/dev/null; '
             'source {ws}/install/setup.bash 2>/dev/null; ').format(ws=REMOTE_WS)


class CarError(RuntimeError):
    pass


def sh(client, command, timeout=60, wrap=True):
    """Run a shell command on the car, returning (code, stdout, stderr)."""
    full = (ROS_SETUP + command) if wrap else command
    code, out, err = run(client, full, timeout=timeout)
    if code is None:
        raise CarError(
            'the car accepted the SSH login but never answered %r.\n'
            'This is the "cannot fork" failure: no new process can be created on '
            'the board. Power-cycle the car; retrying will not help.'
            % command[:60])
    return code, out, err


def step(title):
    print('\n== %s' % title, flush=True)


def check_boot(client):
    step('boot and load')
    code, out, _ = sh(client, 'cat /proc/uptime; nproc; '
                              'awk \'{print $1" "$2" "$3}\' /proc/loadavg', wrap=False)
    if code != 0 or not out.strip():
        raise CarError('could not read /proc/uptime')
    fields = out.split()
    uptime = float(fields[0])
    print('   uptime %.0f s (%.1f h), load %s' % (uptime, uptime / 3600.0,
                                                  ' '.join(fields[3:6])))
    if uptime < 90:
        print('   ! board just booted; ROS 2 may still be starting')
    return uptime


def check_process_count(client):
    """A leaked-process count explains a fork failure before it happens again."""
    step('process table')
    code, out, _ = sh(client, 'ps -e --no-headers 2>/dev/null | wc -l; '
                              'cat /proc/sys/kernel/pid_max 2>/dev/null', wrap=False)
    numbers = [int(value) for value in out.split() if value.strip().isdigit()]
    if len(numbers) >= 2:
        count, limit = numbers[0], numbers[1]
        print('   %d processes, pid_max %d' % (count, limit))
        if count > limit * 0.8:
            print('   ! process table is nearly full; this is the failure that '
                  'makes SSH sessions hang')


def check_topics(client, topics):
    step('ros 2 graph')
    code, out, err = sh(client, 'ros2 topic list', timeout=90)
    if code != 0:
        raise CarError('ros2 topic list failed: %s' % (err.strip()[:200] or out.strip()[:200]))
    present = set(line.strip() for line in out.splitlines() if line.strip())
    print('   %d topics visible' % len(present))
    missing = [topic for topic in topics if topic not in present]
    for topic in topics:
        print('   %s %s' % ('OK  ' if topic in present else 'MISS', topic))
    return missing


def set_param(client, node, parameter, value):
    """Set one parameter on a running node and report what the node said back."""
    step('set %s %s = %s' % (node, parameter, value))
    code, out, err = sh(client, 'ros2 param set %s %s %s'
                        % (node, parameter, 'true' if value else 'false'), timeout=60)
    print('   %s' % (out.strip() or err.strip())[:160])
    return code == 0


def build(client, packages):
    step('colcon build')
    commands = 'cd {ws} && colcon build --packages-select {pkgs} 2>&1 | tail -12'.format(
        ws=REMOTE_WS, pkgs=' '.join(packages))
    code, out, err = sh(client, commands, timeout=600, wrap=False)
    print('   ' + (out.strip() or err.strip()).replace('\n', '\n   '))
    if code != 0:
        raise CarError('colcon build failed')


def poll_status(client, service_call, timeout_s):
    step('waiting for the sweep to finish')
    code, out, err = sh(client, service_call, timeout=60)
    print('   start: %s' % (out.strip() or err.strip())[:160])
    if code != 0:
        raise CarError('could not start the sweep')

    deadline = timeout_s
    elapsed = 0
    while elapsed < deadline:
        code, out, err = sh(client, 'ros2 topic echo --once /vla/search_status', timeout=90)
        payload = {}
        for line in out.splitlines():
            stripped = line.strip()
            if stripped.startswith('{'):
                try:
                    payload = json.loads(stripped)
                except ValueError:
                    pass
        state = payload.get('state', '?')
        print('   [%3ds] state=%-9s turned=%s deg blocked=%s'
              % (elapsed, state, payload.get('travelled_deg'),
                 (payload.get('blocked_reason') or '-')[:50]))
        if state in ('found', 'exhausted', 'aborted', 'idle'):
            return payload
        elapsed += 5
    return {}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--host', default='192.168.86.178')
    parser.add_argument('--user', default='pi')
    parser.add_argument('--preflight', action='store_true',
                        help='checks only; no build, no motion')
    parser.add_argument('--no-build', action='store_true')
    parser.add_argument('--timeout', type=float, default=300.0)
    args = parser.parse_args(argv)

    client = connect(args.host, args.user)
    granted = False
    try:
        check_boot(client)
        check_process_count(client)
        missing = check_topics(client, ['/rear_camera/image_raw',
                                        '/chassis_control/status',
                                        '/grasp/red_ball/detection',
                                        '/cmd_vel'])
        if missing:
            print('\n   ! missing topics above; the detector or chassis node is not '
                  'running. Start the core before searching.')

        if args.preflight:
            print('\npreflight only; nothing was moved')
            return 0 if not missing else 1

        if not args.no_build:
            build(client, ['robot_agent'])

        granted = set_param(client, '/chassis_control', 'enable_motion', True)
        if not granted:
            raise CarError('chassis refused motion permission; check estop and '
                           'hardware_fault in /chassis_control/status')

        result = poll_status(client, 'ros2 service call /vla/search/start '
                                     'std_srvs/srv/Trigger', args.timeout)
        print('\nfinal: %s' % json.dumps(result, ensure_ascii=False))
        if result.get('state') == 'found':
            print('FOUND at %s deg, camera point %s'
                  % (result.get('travelled_deg'), result.get('point_camera_m')))
            return 0
        print('not found: %s' % (result.get('abort_reason') or result.get('state')))
        return 2
    finally:
        try:
            sh(client, 'ros2 service call /vla/search/stop std_srvs/srv/Trigger',
               timeout=30)
        except Exception:  # noqa: BLE001 - best effort on the way out
            pass
        if granted:
            try:
                sh(client, 'ros2 param set /chassis_control enable_motion false',
                   timeout=30)
                print('\nmotion permission revoked')
            except Exception:  # noqa: BLE001
                print('\n! could not revoke motion permission; do it by hand')
        client.close()


if __name__ == '__main__':
    sys.exit(main())
