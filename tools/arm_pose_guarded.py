"""Ad-hoc guarded pose travel from this PC, without installing anything on the car.

This is the manual twin of the boot service. It pipes the SAME container script
(car_setup/ros2/arm_startup_pose_ros.py) that the installed service uses, so the
two paths can never drift apart.

It never takes the serial port: the arm node keeps /dev/rrc and stays the only
reader. It only calls the documented services and publishes to the documented
topic, and it is the only thing in this repo that can authorize arm motion, so
it always requires --confirm-arm-area-clear.

Targets come from a recorded pose file:
  --source startup   pose["startup"]["target_raw"]  (arm joints only, as decided)
  --source raw       pose["raw"]                    (all six, includes the gripper)
"""
import argparse
import json
import shlex
import sys
from pathlib import Path

import paramiko

REPO = Path(__file__).resolve().parents[1]
ROS_SCRIPT = REPO / 'car_setup' / 'ros2' / 'arm_startup_pose_ros.py'
ROS2_SETUP = 'source /opt/ros/jazzy/setup.bash && export ROS_DOMAIN_ID=42 && '


def run(client, container, source, env, timeout):
    setup = ROS2_SETUP + ''.join('export %s=%s && ' % (k, v)
                                for k, v in env.items()) + 'python3 - '
    command = ('sudo -n docker exec -i ' + shlex.quote(container) + ' bash -lc ' +
               shlex.quote(setup))
    stdin, out, err = client.exec_command(command, timeout=timeout)
    stdin.write(source)
    stdin.flush()
    stdin.channel.shutdown_write()
    text, errors = out.read().decode('utf-8', 'replace'), err.read().decode('utf-8', 'replace')
    if text:
        print(text, flush=True)
    if errors.strip():
        print(errors, flush=True)
    return out.channel.recv_exit_status()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', required=True)
    parser.add_argument('--user', default='pi')
    parser.add_argument('--password', default='raspberrypi')
    parser.add_argument('--pose', default=str(REPO / 'car_setup' / 'ros2'
                                              / 'arm_startup_pose.json'))
    parser.add_argument('--source', choices=('startup', 'raw'), default='startup')
    parser.add_argument('--joint3', type=int, help='Override the joint3 target')
    parser.add_argument('--gate-tries', type=int, default=6)
    parser.add_argument('--max-travel', type=int, default=400)
    parser.add_argument('--confirm-arm-area-clear', action='store_true', required=True)
    parser.add_argument('--plan-only', action='store_true')
    args = parser.parse_args()

    pose = json.loads(Path(args.pose).read_text(encoding='utf-8'))
    if args.source == 'startup':
        targets = dict(pose['startup']['target_raw'])
        limits = pose['startup']
    else:
        targets = {k: v for k, v in pose['raw'].items()}
        limits = pose['startup']
    if args.joint3 is not None:
        targets['joint3'] = args.joint3

    env = {'JOINTS': ','.join(targets),
           'STEP': limits.get('step_ticks', 100),
           'DURATION': limits.get('duration_seconds', 0.6),
           'GATE_TRIES': args.gate_tries,
           'MESSAGE_CAP': limits.get('message_cap', 60),
           'MAX_TRAVEL': args.max_travel}
    for name, raw in targets.items():
        env['POSE_' + name] = raw

    if args.plan_only:
        print(json.dumps({'source': args.source, 'pose': pose.get('name'),
                          'joints': list(targets), 'targets_raw': targets,
                          'recorded_raw': pose['raw'],
                          'headroom': {k: pose['hard_limits_raw'][k][1] - v
                                       for k, v in targets.items()},
                          'env': env,
                          'ros_script': str(ROS_SCRIPT),
                          'no_command_sent': True}, indent=2))
        return 0

    source = ROS_SCRIPT.read_text(encoding='utf-8')
    if not source.isascii():
        raise SystemExit('The container script must stay ASCII for safe piping')
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(args.host, username=args.user, password=args.password,
                   timeout=10, auth_timeout=10, allow_agent=False, look_for_keys=False)
    try:
        container = json.loads(
            client.exec_command('cat /run/armpi-ros2/container.json', timeout=10)[1]
            .read().decode())['name']
        print('container=%s joints=%s' % (container, list(targets)), flush=True)
        return run(client, container, source, env, timeout=900)
    finally:
        client.close()


if __name__ == '__main__':
    sys.stdout.reconfigure(encoding='utf-8')
    sys.exit(main())
