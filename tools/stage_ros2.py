"""Stage an isolated ROS 2 build without stopping ROS 1 or granting device access.

Creates a NEW temporary directory/image tag. No destructive cleanup, startup
service changes, credential copies, device mounts, or robot motion commands.
"""
import argparse
import hashlib
import json
import posixpath
import shlex
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))  # Windows embedded Python uses isolated sys.path.
from ros2_preflight import connect


def execute(client, command):
    _, out, err = client.exec_command(command, timeout=30)
    channel = out.channel
    output_parts, error_parts = [], []
    deadline = time.monotonic() + 300
    while True:
        if channel.recv_ready():
            output_parts.append(channel.recv(16384).decode(errors='replace'))
        if channel.recv_stderr_ready():
            error_parts.append(channel.recv_stderr(16384).decode(errors='replace'))
        if channel.exit_status_ready() and not channel.recv_ready() and not channel.recv_stderr_ready():
            break
        if time.monotonic() > deadline:
            raise TimeoutError('Remote staging command exceeded 300 seconds; inspect state before retry')
        time.sleep(0.1)
    output, errors = ''.join(output_parts), ''.join(error_parts)
    if channel.recv_exit_status() != 0:
        raise RuntimeError(errors or output)
    return output.strip()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', required=True)
    parser.add_argument('--config', required=True)
    parser.add_argument('--build', action='store_true')
    parser.add_argument('--base-archive', help='Verified archive produced by download_ros2_base.py')
    parser.add_argument('--incremental-hardware', action='store_true',
                        help='Rebuild only native Python drivers on the exact verified runtime')
    args = parser.parse_args()
    client = connect(args.host, config_path=args.config)
    try:
        free = int(execute(client, "df -B1 --output=avail / | tail -n 1"))
        minimum = (2 if args.incremental_hardware else 5) * 1024**3
        if free < minimum:
            raise RuntimeError('Insufficient staging space; no automatic cleanup permitted')
        if args.incremental_hardware and args.base_archive:
            raise ValueError('Incremental build must not import another base image')
        runtime_id = 'sha256:c06d04049a9650aef1044e5811887a4fb6e459871ae7bbedc8cba9b65addca08'
        if args.incremental_hardware:
            actual = execute(client, "docker image inspect armpi-ros2-migration:stage-encoders1 --format '{{.Id}}'")
            if actual != runtime_id:
                raise ValueError('Incremental runtime image does not match verified ID')
        stage = execute(client, 'mktemp -d /home/pi/armpi-ros2-stage.XXXXXX')
        if not stage.startswith('/home/pi/armpi-ros2-stage.') or '/' in stage[len('/home/pi/'):]:
            raise RuntimeError('Unexpected staging path')
        tag = 'armpi-ros2-migration:stage-' + stage.rsplit('.', 1)[1].lower()
        root = Path(__file__).resolve().parents[1] / 'ros2_ws'
        with client.open_sftp() as sftp:
            for path in sorted(root.rglob('*')):
                relative = path.relative_to(root)
                if any(p in ('build', 'install', 'log', '__pycache__') for p in relative.parts):
                    continue
                if path.suffix == '.pyc' or not path.is_file():
                    continue
                target = posixpath.join(stage, relative.as_posix())
                parent = stage
                for part in relative.parts[:-1]:
                    parent = posixpath.join(parent, part)
                    try:
                        sftp.mkdir(parent)
                    except OSError:
                        sftp.stat(parent)  # Do not swallow permission/missing-directory failures.
                sftp.put(str(path), target)
        print(f'STAGED_DIRECTORY={stage}\nIMAGE_TAG={tag}', flush=True)
        build_args = ''
        if args.incremental_hardware:
            # Docker FROM interprets a bare sha256:... as a registry name/tag.
            # Use the locally present tag, pin it by inspect before and after build.
            build_args = ' -f Dockerfile.hardware --pull=false --build-arg VERIFIED_RUNTIME=armpi-ros2-migration:stage-encoders1'
        if args.base_archive:
            archive = Path(args.base_archive).resolve()
            record = json.loads((archive.parent / 'verification.json').read_text(encoding='utf-8'))
            with archive.open('rb') as handle:
                digest = hashlib.file_digest(handle, 'sha256').hexdigest()
            if record.get('repository') != 'library/ros' or record.get('platform') != 'linux/arm64/v8' or record.get('archive_sha256') != digest:
                raise RuntimeError('Base archive verification record does not match')
            remote_archive = stage + '/official-ros-jazzy-arm64.tar.gz'
            try:
                present_id = execute(client, "docker image inspect armpi-ros2-base:jazzy-arm64 --format '{{.Id}}'")
            except RuntimeError:
                present_id = None
            if present_id == record.get('config_digest'):
                print('VERIFIED_BASE_ALREADY_IMPORTED', flush=True)
            else:
                print('UPLOADING_VERIFIED_BASE_ARCHIVE', flush=True)
                with client.open_sftp() as sftp:
                    sftp.put(str(archive), remote_archive)
                if execute(client, f'sha256sum {shlex.quote(remote_archive)}').split()[0] != digest:
                    raise RuntimeError('Remote archive integrity check failed')
                print(execute(client, f'docker load -i {shlex.quote(remote_archive)}'), flush=True)
            build_args = ' --build-arg ROS_BASE_IMAGE=armpi-ros2-base:jazzy-arm64'
        if not args.build:
            return
        command = (f'cd {shlex.quote(stage)} && '
                   f'timeout 1800 docker build --network=host{build_args} -t {shlex.quote(tag)} .')
        _, out, _err = client.exec_command(command, timeout=30)
        channel = out.channel
        channel.settimeout(30)
        while True:
            if channel.recv_ready():
                print(channel.recv(16384).decode(errors='replace'), end='', flush=True)
            if channel.recv_stderr_ready():
                print(channel.recv_stderr(16384).decode(errors='replace'), end='', flush=True)
            if channel.exit_status_ready() and not channel.recv_ready() and not channel.recv_stderr_ready():
                break
            time.sleep(0.1)
        result = channel.recv_exit_status()
        if args.incremental_hardware:
            actual = execute(client, "docker image inspect armpi-ros2-migration:stage-encoders1 --format '{{.Id}}'")
            if actual != runtime_id:
                raise RuntimeError('Verified runtime tag changed during build; refuse hardware use')
        print(f'BUILD_EXIT_CODE={result}', flush=True)
        if result:
            raise SystemExit(result)
    finally:
        client.close()


if __name__ == '__main__':
    main()
