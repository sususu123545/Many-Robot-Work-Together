"""Opt-in, once-per-boot near-limit recovery; never a general homing override."""
import json
import os
from pathlib import Path
import subprocess
import signal
import time
import uuid


def prepare_boot_arm(image, root=Path('/opt/armpi-ros2'), runtime=Path('/run/armpi-ros2')):
    pose_path = root / 'arm_startup_pose.json'
    if not pose_path.exists():
        return
    startup = json.loads(pose_path.read_text(encoding='utf-8')).get('startup', {})
    if not startup.get('enabled') or not startup.get('near_limit_recovery', {}).get('enabled'):
        return
    final = startup.get('target_raw', {}).get('joint3')
    if type(final) is not int or not 900 <= final <= 950:
        raise RuntimeError('Bootstrap requires the validated joint3 target in 900..950')
    runtime.mkdir(exist_ok=True)
    marker = runtime / 'arm-bootstrap.json'
    try:
        fd = os.open(marker, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    except FileExistsError:
        previous = json.loads(marker.read_text())
        if previous.get('ok') is not True:
            raise RuntimeError('Earlier boot recovery failed; inspect before retrying')
        return
    with os.fdopen(fd, 'w') as handle:
        json.dump({'ok': False, 'state': 'running', 'started_unix': time.time()}, handle)
    name = 'armpi_arm_bootstrap_' + uuid.uuid4().hex[:12]
    command = ['docker', 'run', '--rm', '--name', name, '--network', 'none',
               '--device', '/dev/rrc:/dev/rrc',
               '-v', str(root / 'joint3_recover.py') + ':/opt/joint3_recover.py:ro',
               image, 'bash', '-lc',
               'source /opt/armpi_ros2/install/setup.bash && '
               'export MODE=run CONFIRM=AREA_CLEAR FINAL=' + str(final) + ' STEP=25 '
               'DURATION=1.5 SETTLE=1.8 MESSAGE_CAP=12 TEMP_ABORT=55 && '
               'exec python3 /opt/joint3_recover.py']
    result = {'ok': False, 'state': 'failed', 'started_unix': time.time()}
    old_term = signal.getsignal(signal.SIGTERM)

    def interrupted(_signal, _frame):
        raise InterruptedError('Bootstrap interrupted; do not launch the core')

    signal.signal(signal.SIGTERM, interrupted)
    try:
        done = subprocess.run(command, capture_output=True, text=True, timeout=60)
        print(done.stdout, flush=True)
        if done.stderr:
            print(done.stderr, flush=True)
        result['exit_code'] = done.returncode
        for line in done.stdout.splitlines():
            if line.startswith('RESULT_JSON='):
                result['result'] = json.loads(line.split('=', 1)[1])
        measured = result.get('result', {})
        result['ok'] = bool(done.returncode == 0 and measured.get('in_range') is True
                            and measured.get('messages', 99) <= 12
                            and measured.get('servo') == 4)
        result['state'] = 'complete' if result['ok'] else 'failed'
        if not result['ok']:
            raise RuntimeError('Near-limit arm recovery refused/failed; core left stopped')
    except BaseException as error:
        subprocess.run(['docker', 'stop', '-t', '3', name], capture_output=True, timeout=8)
        result['reason'] = str(error)
        raise
    finally:
        signal.signal(signal.SIGTERM, old_term)
        marker.write_text(json.dumps(result, indent=2))
