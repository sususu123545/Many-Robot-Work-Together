"""Validate the migration image in an ephemeral container with NO hardware/network.

Only creates/stops its own uniquely named test container. Does not stop ROS 1,
publish to its rosbridge, mount devices, change boot services or delete images.
"""
import argparse
import re
import shlex
import sys
import time
import uuid
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ros2_preflight import connect
from stage_ros2 import execute


BRIDGE_PROBE = '''
import json
from tornado.ioloop import IOLoop
from tornado.websocket import websocket_connect
from tornado import gen

async def probe():
    socket = await websocket_connect('ws://127.0.0.1:9091', connect_timeout=5)
    try:
        await socket.write_message(json.dumps({
            'op': 'subscribe', 'id': 'migration_readonly',
            'topic': '/chassis_control/status', 'type': 'std_msgs/msg/String'}))
        deadline = IOLoop.current().time() + 8
        while IOLoop.current().time() < deadline:
            raw = await gen.with_timeout(deadline, socket.read_message())
            if raw is None:
                raise RuntimeError('rosbridge disconnected')
            message = json.loads(raw)
            if message.get('op') == 'publish' and message.get('topic') == '/chassis_control/status':
                state = json.loads(message['msg']['data'])
                assert state['dry_run'] is True and state['enable_motion'] is False
                print(json.dumps({'websocket': 'OK', 'ROS2_message_received': True, 'state': state}))
                return
        raise RuntimeError('No status received through rosbridge')
    finally:
        socket.close()

IOLoop.current().run_sync(probe)
'''


def run_python(client, container, source):
    command = ('docker exec -i ' + shlex.quote(container) +
               " bash -lc 'source /opt/armpi_ros2/install/setup.bash && python3 -'")
    stdin, out, err = client.exec_command(command, timeout=30)
    stdin.write(source)
    stdin.flush()
    stdin.channel.shutdown_write()
    output, errors = out.read().decode(), err.read().decode()
    if out.channel.recv_exit_status() != 0:
        raise RuntimeError(errors or output)
    print(output, flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', required=True)
    parser.add_argument('--config', required=True)
    parser.add_argument('--image', required=True)
    args = parser.parse_args()
    if not re.fullmatch(r'armpi-ros2-migration:stage-[a-zA-Z0-9]+', args.image):
        parser.error('Use the exact migration staging tag, not an arbitrary image')
    client = connect(args.host, config_path=args.config)
    name = 'armpi_ros2_verify_' + uuid.uuid4().hex[:12]
    created = False
    try:
        print(execute(client, 'docker image inspect ' + shlex.quote(args.image) +
                      " --format '{{.Id}} {{.Architecture}}'"), flush=True)
        created = True  # A lost SSH reply must not leave our own container running.
        print(execute(client, 'docker run -d --network none --stop-signal SIGINT --name ' + name +
                      ' ' + shlex.quote(args.image)), flush=True)
        # No privileged/devices/volumes and network none are fixed, not CLI options.
        print(execute(client, 'docker inspect ' + name +
                      " --format 'Privileged={{.HostConfig.Privileged}} Devices={{json .HostConfig.Devices}} Mounts={{json .Mounts}} Network={{.HostConfig.NetworkMode}}'"), flush=True)
        time.sleep(5)
        print(execute(client, 'docker logs ' + name + ' 2>&1'), flush=True)
        smoke = Path(__file__).resolve().parents[1] / 'ros2_ws' / 'smoke_no_hardware.py'
        run_python(client, name, smoke.read_text(encoding='utf-8'))
        run_python(client, name, BRIDGE_PROBE)
        print('ISOLATED_ROS2_VERIFICATION=PASS', flush=True)
    finally:
        try:
            if created and name in execute(client, "docker ps -a --format '{{.Names}}'").splitlines():
                # Only the unique container created above; retain it stopped for logs.
                print(execute(client, 'docker stop -t 10 ' + name), flush=True)
                print('RETAINED_STOPPED_TEST_CONTAINER=' + name, flush=True)
        finally:
            client.close()


if __name__ == '__main__':
    main()
