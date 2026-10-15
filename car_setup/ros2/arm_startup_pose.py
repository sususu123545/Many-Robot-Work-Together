#!/usr/bin/env python3
"""Boot-time guarded pose travel for the ArmPi Pro arm. Runs ON the Raspberry Pi.

Installed by tools/deploy_startup_pose.py as /opt/armpi-ros2/arm_startup_pose.py
and started by armpi-startup-pose.service after armpi-ros2.service.

This is an external client. It does NOT modify arm_node.py, run_native_core.py or
the armpi-ros2 unit, and it never takes /dev/rrc: the arm node stays the only
serial owner and this script only calls the documented services and topic.

It differs from the rest of this project's design on purpose, by explicit user
decision: the node deliberately refuses to move the arm until a human resets the
emergency latch, and this script resets that latch and authorizes arm motion
without a human present. Disable with:
    systemctl disable --now armpi-startup-pose.service

Unavoidable failure mode: if the arm settled outside joint3's 0..1000 range when
it lost torque, the node latches and refuses every reset, so this script can never
start. That is a physical precondition, not a bug - the arm must be lifted by hand.
"""
import json
import os
import subprocess
import sys
import time
from pathlib import Path

CONFIG = Path(os.environ.get('ARMPI_STARTUP_POSE_CONFIG',
                             '/opt/armpi-ros2/arm_startup_pose.json'))
ROS_SCRIPT = Path(os.environ.get('ARMPI_STARTUP_POSE_ROS',
                                 '/opt/armpi-ros2/arm_startup_pose_ros.py'))
RUNTIME = Path('/run/armpi-ros2')
CONTAINER_FILE = RUNTIME / 'container.json'
RESULT_FILE = RUNTIME / 'arm-startup-pose.json'
SETUP = 'source /opt/ros/jazzy/setup.bash && export ROS_DOMAIN_ID=42 && '

PROBE = '''
import time, rclpy
from rclpy.node import Node
from std_msgs.msg import String
rclpy.init()
node = Node('startup_pose_probe')
seen = []
node.create_subscription(String, '/arm_controller/status', lambda m: seen.append(1), 10)
end = time.monotonic() + 5
while time.monotonic() < end and not seen:
    rclpy.spin_once(node, timeout_sec=0.2)
print('READY' if seen else 'NOT_READY', flush=True)
node.destroy_node()
'''


def log(message):
    print('[startup-pose] ' + message, flush=True)


def run_in_container(container, source, env=None, timeout=900):
    setup = SETUP + ''.join('export %s=%s && ' % (k, v) for k, v in (env or {}).items())
    command = ['docker', 'exec', '-i', container, 'bash', '-lc', setup + 'python3 - ']
    done = subprocess.run(command, input=source, capture_output=True, text=True,
                          timeout=timeout)
    return done.returncode, done.stdout, done.stderr


def container_name():
    if not CONTAINER_FILE.exists():
        return None
    try:
        # Explicit UTF-8: a systemd unit runs with a minimal environment where
        # LANG is often unset, so the locale default can be plain ASCII.
        return json.loads(CONTAINER_FILE.read_text(encoding='utf-8'))['name']
    except (ValueError, KeyError, UnicodeDecodeError):
        return None


def container_running(name):
    done = subprocess.run(['docker', 'inspect', name, '--format', '{{.State.Running}}'],
                          capture_output=True, text=True)
    return done.stdout.strip() == 'true'


def wait_core(seconds):
    """The container starts early, but the arm node needs time to answer."""
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        name = container_name()
        if name and container_running(name):
            try:
                code, out, _ = run_in_container(name, PROBE, timeout=30)
            except subprocess.TimeoutExpired:
                code, out = 1, ''
            if code == 0 and 'READY' in out.splitlines():
                log('arm node answered after %.0fs' % (seconds - (deadline - time.monotonic())))
                return name
        time.sleep(3)
    return None


def write_result(payload):
    try:
        RUNTIME.mkdir(exist_ok=True)
        RESULT_FILE.write_text(json.dumps(payload, indent=2))
    except OSError as error:
        log('could not write %s: %s' % (RESULT_FILE, error))


def main():
    log('config=%s ros_script=%s' % (CONFIG, ROS_SCRIPT))
    if not CONFIG.exists() or not ROS_SCRIPT.exists():
        log('REFUSED missing config or ros script; nothing sent')
        write_result({'ok': False, 'reason': 'missing config or ros script'})
        return 2

    try:
        pose = json.loads(CONFIG.read_text(encoding='utf-8'))
        startup = pose['startup']
        targets = dict(startup['target_raw'])
    except (ValueError, KeyError, UnicodeDecodeError) as error:
        log('REFUSED unreadable config: %s' % error)
        write_result({'ok': False, 'reason': 'unreadable config: %s' % error})
        return 2

    if not startup.get('enabled'):
        log('startup pose disabled in config; nothing sent')
        write_result({'ok': True, 'skipped': 'disabled in config'})
        return 0
    if not targets:
        log('no joints configured; nothing sent')
        write_result({'ok': True, 'skipped': 'no joints configured'})
        return 0

    container = wait_core(float(startup.get('wait_core_seconds', 180)))
    if not container:
        log('REFUSED arm node never answered; arm left untouched')
        write_result({'ok': False, 'reason': 'arm node never answered'})
        return 4

    env = {'JOINTS': ','.join(targets),
           'STEP': startup.get('step_ticks', 100),
           'DURATION': startup.get('duration_seconds', 0.6),
           'GATE_TRIES': startup.get('gate_tries', 6),
           'MESSAGE_CAP': startup.get('message_cap', 60),
           'MAX_TRAVEL': startup.get('max_travel_ticks', 400),
           'TOLERANCE': startup.get('tolerance_ticks', 12)}
    for name, raw in targets.items():
        env['POSE_' + name] = raw
    log('container=%s env=%s' % (container, json.dumps(env)))

    source = ROS_SCRIPT.read_text(encoding='utf-8')
    if not source.isascii():
        log('REFUSED container script is not ASCII; refusing to pipe it')
        write_result({'ok': False, 'reason': 'container script not ASCII'})
        return 2
    try:
        code, out, err = run_in_container(container, source, env,
                                          timeout=float(startup.get('timeout_seconds', 600)))
    except subprocess.TimeoutExpired:
        log('REFUSED container call timed out; arm state unknown')
        write_result({'ok': False, 'reason': 'container call timed out'})
        return 5

    for line in out.splitlines():
        log(line)
    if err.strip():
        for line in err.strip().splitlines():
            log('stderr: ' + line)

    summary = {'ok': code == 0, 'exit_code': code, 'container': container, 'env': env}
    for line in out.splitlines():
        if line.startswith('RESULT_JSON='):
            summary['result'] = json.loads(line.split('=', 1)[1])
    write_result(summary)
    log('done exit=%d' % code)
    return code


if __name__ == '__main__':
    sys.exit(main())
