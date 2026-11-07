#!/usr/bin/env python3
"""Stationary native ROS2 boot: pinned image, bounded devices, motion disabled.

Must be launched by armpi-ros2.service after legacy core services are gated.
Never restores ROS1 automatically. Motion stays disabled in the regular core;
the separately opted-in arm bootstrap/pose feature is an explicit exception.
"""
import hashlib
import json
import os
from pathlib import Path
import signal
import stat
import subprocess
import time
import uuid

ROOT = Path('/etc/armpi-ros2')
RUNTIME = Path('/run/armpi-ros2')
CALIBRATION = Path('/var/lib/armpi-ros2/calibration')
# Occupancy maps live on the host, not in the image.
#
# nav2_amcl needs a map file, and the map is the one thing on this car that gets
# re-surveyed regularly.  Baking it into the image would mean a `docker commit`
# + `core.json` edit + core restart every time the map improves -- exactly the
# ceremony we want to avoid.  Bind-mounting a host directory (same shape as
# CALIBRATION) lets us drop a new workshop.pgm/.yaml in place and just restart
# the localisation node.
MAPS = Path('/var/lib/armpi-ros2/maps')
LEGACY_SERVICES = ('ld06', 'odom', 'ch341-bmi270', 'auto-mapping', 'slam-control')
LEGACY_EXECUTABLES = (
    'chassis_control_node.py', 'bmi270_imu_node.py', 'ldlidar_stl_ros_node',
    'ros_robot_controller_node.py', 'controller_manager.py', 'mecanum_odom_node.py',
)


def run(args, timeout=20):
    p = subprocess.run(args, capture_output=True, text=True, timeout=timeout)
    if p.returncode:
        raise RuntimeError('Command failed: ' + ' '.join(args[:3]) + ': ' + p.stderr[-2000:])
    return p.stdout.strip()


def wait_container(name):
    # Linux poll() uses a signed 32-bit millisecond timeout. A 30-day timeout
    # overflows it and triggers finally/stop even when the core is healthy.
    # systemd/signal handlers supervise this intentionally long-lived wait.
    return int(run(['docker', 'wait', name], timeout=None))


def owners(devices):
    targets = {d: os.stat(d).st_rdev for d in devices}
    result = {d: set() for d in devices}
    for p in Path('/proc').glob('[0-9]*'):
        try:
            for f in (p / 'fd').iterdir():
                try:
                    s = f.stat()
                    if stat.S_ISCHR(s.st_mode):
                        for d, target in targets.items():
                            if s.st_rdev == target:
                                result[d].add(int(p.name))
                except OSError:
                    pass
        except OSError:
            pass
    return {d: sorted(v) for d, v in result.items()}


def legacy_processes():
    found = []
    for p in Path('/proc').glob('[0-9]*/cmdline'):
        try:
            args = p.read_bytes().split(b'\0')
            if any(Path(a.decode(errors='replace')).name in LEGACY_EXECUTABLES for a in args):
                found.append(int(p.parent.name))
        except OSError:
            pass
    return found


def library_digest(path):
    # Same ordering/format as tools/verify_ros2_imu.py's source verification.
    h = hashlib.sha256()
    for p in sorted(Path(path).rglob('*.py')):
        h.update(p.relative_to(path).as_posix().encode())
        h.update(b'\0')
        h.update(p.read_bytes())
        h.update(b'\0')
    return h.hexdigest()


def prepare_pose(image, root=Path('/opt/armpi-ros2')):
    """把机械臂恢复到记录的目标姿态；失败不阻塞启动，仅告警。"""
    script = root / 'pose_restore.py'
    if not script.exists():
        print('pose_restore.py 不存在，跳过姿态恢复', flush=True)
        return False
    name = 'armpi_pose_restore_' + uuid.uuid4().hex[:8]
    cmd = ['docker', 'run', '--rm', '--name', name, '--network', 'none',
           '--device', '/dev/rrc:/dev/rrc',
           '-v', str(script) + ':/opt/pose_restore.py:ro',
           image, 'bash', '-lc',
           'source /opt/armpi_ros2/install/setup.bash && python3 /opt/pose_restore.py']
    try:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=200)
        if p.stdout:
            print(p.stdout[-1500:], flush=True)
        if p.returncode != 0:
            print('姿态恢复返回码 %d: %s' % (p.returncode, p.stderr[-300:]), flush=True)
            return False
        print('姿态恢复完成', flush=True)
        return True
    except subprocess.TimeoutExpired:
        print('姿态恢复超时', flush=True)
        subprocess.run(['docker', 'rm', '-f', name], capture_output=True)
        return False


def main():
    assert (ROOT / 'active').is_file(), 'Native core profile is not active'
    cfg = json.loads((ROOT / 'core.json').read_text())
    assert run(['docker', 'image', 'inspect', cfg['image'], '--format', '{{.Id}}']) == cfg['image_id']
    assert library_digest(cfg['bmi_library']) == cfg['bmi_library_sha256'], 'BMI library changed'
    for unit in LEGACY_SERVICES:
        state = subprocess.run(['systemctl', 'is-active', unit], capture_output=True, text=True).stdout.strip()
        assert state not in ('active', 'activating', 'reloading'), 'Legacy service active: ' + unit
    deadline = time.monotonic() + 60
    while True:
        adapters = [p.parent.name for p in Path('/sys/class/i2c-adapter').glob('i2c-*/name')
                    if 'i2c-ch341-usb' in p.read_text().lower()]
        if len(adapters) == 1 and all(Path(d).exists() for d in ('/dev/rrc', '/dev/ld06', '/dev/i2c-1')):
            imu_dev = '/dev/' + adapters[0]
            if Path(imu_dev).exists():
                break
        if time.monotonic() >= deadline:
            raise RuntimeError('Required core devices unavailable/ambiguous; no hardware enabled')
        time.sleep(.5)
    required = ['/dev/rrc', '/dev/ld06', '/dev/i2c-1', imu_dev]
    # The rear camera is an optional UVC peripheral, not a controller. It is mapped
    # when the device exists and reported as absent when it does not, so an
    # unplugged camera can never stop the arm and chassis from coming up. It is
    # also excluded from the owner check below on purpose: a second holder of
    # /dev/video0 (a stale ROS 1 usb_cam, a manual v4l2 tool) should cost the video
    # feed, not the whole-car core.
    camera = '/dev/video0'
    camera_present = Path(camera).exists()
    devices = required + ([camera] if camera_present else [])
    launch_camera = 'camera:=true' if camera_present else 'camera:=false'
    assert not legacy_processes(), 'Legacy hardware process still running'
    for _ in range(3):
        assert not any(owners(required).values()), 'Device has another owner'
        time.sleep(.1)
    # Explicit opt-in, only before regular drivers and only once per boot.
    from arm_bootstrap import prepare_boot_arm
    prepare_boot_arm(cfg['image'])
    assert not any(owners(required).values()), 'Bootstrap has not released hardware'
    from lidar_preflight import prepare_lidar
    prepare_lidar()
    prepare_pose(cfg['image'])
    RUNTIME.mkdir(exist_ok=True)
    CALIBRATION.mkdir(parents=True, exist_ok=True)
    MAPS.mkdir(parents=True, exist_ok=True)
    seed_file = RUNTIME / 'cutover-seed.json'
    seed = json.loads(seed_file.read_text())['old_odom_pose'] if seed_file.exists() else [0, 0, 0]
    launch = ('source /opt/armpi_ros2/install/setup.bash && exec ros2 launch armpi_ros2 base.launch.py '
              'dry_run:=false enable_motion:=true enable_arm_motion:=false enable_gripper_motion:=false arm:=true lidar:=true imu:=true '
              + launch_camera + ' '
              'video_server:=true rosbridge_port:=9091 ' + ' '.join(k + ':=' + str(v) for k, v in
              zip(('initial_x', 'initial_y', 'initial_yaw'), seed)))
    # 固定容器名：启动前清掉同名残留，保证全局只有一个核心容器，不再堆积临时容器
    name = 'armpi_ros2_native'
    subprocess.run(['docker', 'rm', '-f', name], capture_output=True, text=True)
    args = ['docker', 'run', '-d', '--name', name, '--label', 'armpi.owner=ros2-native-core',
            '--network', 'host', '--stop-signal', 'SIGINT',
            # Fast DDS defaults to shared-memory transport for same-host peers,
            # and every container gets its own /dev/shm. A sidecar that reaches
            # this core over --network host therefore discovers the whole graph
            # (`ros2 node list` is complete) yet receives ZERO payload -- every
            # echo sits at its timeout. The fix is to let sidecars join THIS
            # container's IPC namespace, which requires the donor to be
            # shareable; a container created `private` (the default) cannot be
            # a donor once it is running. Perception probes need nothing else.
            '--ipc', 'shareable',
            # Honor the ROS environment even when the pinned image defaults to
            # an inert entrypoint such as /bin/true.
            '--entrypoint', '/ros_entrypoint.sh',
            '-e', 'ROS_DOMAIN_ID=42', '-e', 'ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST',
            '-v', '/sys/class/i2c-adapter:/sys/class/i2c-adapter:ro',
            '-v', str(CALIBRATION) + ':/opt/armpi-calibration',
            '-v', str(MAPS) + ':/opt/armpi-maps',
            '-v', cfg['bmi_library'] + ':/opt/bmi270_py:ro']
    for d in devices:
        args.extend(['--device', d + ':' + d])
    args.extend([cfg['image'], 'bash', '-lc', launch])
    created = False
    stopping = False

    def stop(_signal=None, _frame=None):
        nonlocal stopping
        if not stopping:
            stopping = True
            if created:
                run(['docker', 'stop', '-t', '20', name], timeout=25)

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    try:
        created = True  # A remote/docker operation can complete before its reply is lost.
        run(args)
        try:
            subprocess.run(['docker', 'exec', '-d', name, 'bash', '-c',
                            'source /opt/ros/jazzy/setup.bash && nohup python3 /opt/armpi_ros2/ready_beep.py '
                            '> /tmp/ready_beep.log 2>&1 &'],
                           capture_output=True, timeout=20)
            print('ready_beep 已启动', flush=True)
        except Exception as error:
            print('ready_beep 启动失败: %s' % error, flush=True)
        (RUNTIME / 'container.json').write_text(json.dumps({'name': name, 'image_id': cfg['image_id'],
            'devices': devices, 'motion_enabled': False, 'arm_motion_enabled': False,
            'gripper_motion_enabled': False}))
        print('ROS2_NATIVE_CONTAINER=' + name, flush=True)
        result = wait_container(name)
        if not stopping:
            raise RuntimeError('Required native hardware process exited: ' + str(result))
    finally:
        if created:
            stop()


if __name__ == '__main__':
    main()
