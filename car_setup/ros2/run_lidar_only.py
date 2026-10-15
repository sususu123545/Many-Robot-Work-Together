#!/usr/bin/env python3
"""Exclusive LD06 + loopback ROS2 bridge; no automatic whole-car recovery."""
import json
import os
from pathlib import Path
import signal
import socket
import stat
import subprocess
import uuid


def run(args, timeout=20):
    p = subprocess.run(args, capture_output=True, text=True, timeout=timeout)
    if p.returncode:
        raise RuntimeError('Command failed: ' + ' '.join(args[:3]) + ': ' + p.stderr[-1500:])
    return p.stdout.strip()


def device_owners(device):
    target = os.stat(device)
    if not stat.S_ISCHR(target.st_mode):
        raise RuntimeError('LD06 must be a character device')
    owners = set()
    for proc in Path('/proc').glob('[0-9]*'):
        try:
            for fd in (proc / 'fd').iterdir():
                try:
                    value = fd.stat()
                    if stat.S_ISCHR(value.st_mode) and value.st_rdev == target.st_rdev:
                        owners.add(int(proc.name))
                except OSError:
                    pass
        except OSError:
            pass
    return sorted(owners)


def validate_devices(devices):
    if devices != ['/dev/ld06']:
        raise ValueError('Sensor recovery may map only /dev/ld06')


def container_args(name, image, launch_file):
    devices = ['/dev/ld06']
    validate_devices(devices)
    return ['docker', 'run', '-d', '--rm', '--name', name,
            '--label', 'armpi.owner=ros2-lidar-only', '--network', 'host',
            '--log-driver', 'journald', '--log-opt', 'tag=armpi-ros2-lidar',
            '--stop-signal', 'SIGINT',
            '-e', 'ROS_DOMAIN_ID=42',
            '-e', 'ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST',
            '--device', devices[0] + ':' + devices[0],
            '-v', str(launch_file) + ':/opt/armpi_lidar_only.launch.py:ro',
            image, 'bash', '-lc',
            'source /opt/armpi_ros2/install/setup.bash && '
            'exec ros2 launch /opt/armpi_lidar_only.launch.py']


def main():
    cfg = json.loads(Path('/etc/armpi-ros2/core.json').read_text())
    if run(['docker', 'image', 'inspect', cfg['image'], '--format', '{{.Id}}']) != cfg['image_id']:
        raise RuntimeError('Pinned image changed')
    state = subprocess.run(['systemctl', 'is-active', 'armpi-ros2.service'],
                           capture_output=True, text=True).stdout.strip()
    if state not in ('inactive', 'failed'):
        raise RuntimeError('Whole-car core must remain stopped: ' + state)
    if device_owners('/dev/ld06'):
        raise RuntimeError('LD06 already owned; will not kill its driver')
    # Do not replace another bridge or change the ROS1 9090 listener.
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', 9091))
    name = 'armpi_ros2_lidar_recovery_' + uuid.uuid4().hex[:12]
    created = False
    stopping = False

    def stop(_signal=None, _frame=None):
        nonlocal stopping
        if not stopping:
            stopping = True
            if created:
                # --rm may already have removed an unexpectedly exited sensor
                # container. Driver logs remain in the bounded system journal.
                result = subprocess.run(['docker', 'stop', '-t', '15', name],
                                        capture_output=True, text=True, timeout=20)
                if result.returncode and 'No such container' not in result.stderr:
                    raise RuntimeError('Unable to stop sensor container: ' + result.stderr)

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    try:
        created = True
        run(container_args(name, cfg['image'], '/opt/armpi-ros2/lidar_only.launch.py'))
        runtime = Path('/run/armpi-ros2-lidar')
        runtime.mkdir(exist_ok=True)
        (runtime / 'container.json').write_text(json.dumps({
            'name': name, 'image_id': cfg['image_id'], 'devices': ['/dev/ld06'],
            'sensor_only': True, 'motion_enabled': False}))
        print('ROS2_LIDAR_ONLY_CONTAINER=' + name, flush=True)
        code = int(run(['docker', 'wait', name], timeout=None))
        if not stopping:
            # ros2 launch can exit 0 after a required-node Shutdown event.
            raise RuntimeError('Sensor container exited unexpectedly: ' + str(code))
    finally:
        if created:
            stop()


if __name__ == '__main__':
    main()
