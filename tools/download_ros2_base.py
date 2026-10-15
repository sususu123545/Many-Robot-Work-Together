"""Fetch ONLY official library/ros Jazzy ARM64; verify every content digest.

Creates a new generated Docker-load archive under ignored tools/runtime.
Does not install Docker, change DNS/proxy settings, or execute image contents.
"""
import gzip
import hashlib
import io
import json
import shutil
import tarfile
import tempfile
import urllib.parse
import urllib.request
from pathlib import Path

REPOSITORY = 'library/ros'
REGISTRY = 'https://registry-1.docker.io/v2/' + REPOSITORY
ACCEPT = ', '.join(('application/vnd.oci.image.index.v1+json',
                    'application/vnd.docker.distribution.manifest.list.v2+json',
                    'application/vnd.oci.image.manifest.v1+json',
                    'application/vnd.docker.distribution.manifest.v2+json'))


class SafeRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        if urllib.parse.urlsplit(newurl).scheme != 'https':
            raise RuntimeError('Refusing non-HTTPS blob redirect')
        redirected = super().redirect_request(req, fp, code, msg, headers, newurl)
        if urllib.parse.urlsplit(newurl).netloc != urllib.parse.urlsplit(req.full_url).netloc:
            redirected.remove_header('Authorization')
        return redirected


def verify(data, digest):
    if digest != 'sha256:' + hashlib.sha256(data).hexdigest():
        raise RuntimeError(f'Content digest mismatch: {digest}')


def main():
    root = Path(__file__).resolve().parent / 'runtime' / 'ros2_images'
    root.mkdir(parents=True, exist_ok=True)
    if shutil.disk_usage(root).free < 8 * 1024**3:
        raise RuntimeError('At least 8 GiB local free space required')
    stage = Path(tempfile.mkdtemp(prefix='official-jazzy-', dir=root))
    opener = urllib.request.build_opener(SafeRedirect())
    token_url = 'https://auth.docker.io/token?service=registry.docker.io&scope=repository:library/ros:pull'
    with opener.open(token_url, timeout=20) as response:
        token = json.load(response)['token']
    headers = {'Authorization': 'Bearer ' + token, 'Accept': ACCEPT}
    def fetch(path):
        with opener.open(urllib.request.Request(REGISTRY + path, headers=headers), timeout=30) as response:
            return response.read()
    index = json.loads(fetch('/manifests/jazzy-ros-base'))
    descriptors = [m for m in index['manifests']
                   if m.get('platform', {}).get('os') == 'linux'
                   and m.get('platform', {}).get('architecture') == 'arm64'
                   and m.get('platform', {}).get('variant', 'v8') == 'v8']
    if len(descriptors) != 1:
        raise RuntimeError('Ambiguous official ARM64 manifest')
    manifest_digest = descriptors[0]['digest']
    raw_manifest = fetch('/manifests/' + manifest_digest)
    verify(raw_manifest, manifest_digest)
    manifest = json.loads(raw_manifest)
    config_data = fetch('/blobs/' + manifest['config']['digest'])
    verify(config_data, manifest['config']['digest'])
    config = json.loads(config_data)
    if config.get('architecture') != 'arm64' or config.get('os') != 'linux':
        raise RuntimeError('Wrong image platform')
    diff_ids = config['rootfs']['diff_ids']
    if len(diff_ids) != len(manifest['layers']):
        raise RuntimeError('Unexpected image layer count')
    config_name = manifest['config']['digest'].split(':')[1] + '.json'
    (stage / config_name).write_bytes(config_data)
    print(f'OFFICIAL_ARM64_MANIFEST={manifest_digest}', flush=True)
    layer_names, unpacked_total = [], 0
    for i, (descriptor, diff_id) in enumerate(zip(manifest['layers'], diff_ids)):
        digest = descriptor['digest']
        if not digest.startswith('sha256:') or not descriptor['mediaType'].endswith('+gzip'):
            raise RuntimeError('Only sha256/gzip OCI or Docker layers are accepted')
        compressed = stage / f'layer-{i}.gz'
        hasher = hashlib.sha256()
        size = 0
        print(f'DOWNLOADING_LAYER={i + 1}/{len(diff_ids)} bytes={descriptor["size"]}', flush=True)
        with opener.open(urllib.request.Request(REGISTRY + '/blobs/' + digest, headers=headers), timeout=30) as source:
            with compressed.open('wb') as output:
                while chunk := source.read(512 * 1024):
                    output.write(chunk)
                    hasher.update(chunk)
                    size += len(chunk)
        if size != descriptor['size'] or 'sha256:' + hasher.hexdigest() != digest:
            raise RuntimeError('Compressed layer integrity check failed')
        name = f'{i}/layer.tar'
        layer = stage / name
        layer.parent.mkdir()
        diff_hash = hashlib.sha256()
        with gzip.open(compressed, 'rb') as source, layer.open('wb') as output:
            while chunk := source.read(512 * 1024):
                unpacked_total += len(chunk)
                if unpacked_total > 4 * 1024**3:
                    raise RuntimeError('Image exceeds 4 GiB unpacked size budget')
                output.write(chunk)
                diff_hash.update(chunk)
        if 'sha256:' + diff_hash.hexdigest() != diff_id:
            raise RuntimeError('Uncompressed layer integrity check failed')
        layer_names.append(name)
        print(f'VERIFIED_LAYER={i + 1}', flush=True)
    docker_manifest = [{'Config': config_name,
                        'RepoTags': ['armpi-ros2-base:jazzy-arm64'], 'Layers': layer_names}]
    archive = stage / 'official-ros-jazzy-arm64.tar.gz'
    with tarfile.open(archive, 'w:gz', compresslevel=1) as tar:
        tar.add(stage / config_name, arcname=config_name)
        payload = json.dumps(docker_manifest).encode()
        info = tarfile.TarInfo('manifest.json')
        info.size = len(payload)
        tar.addfile(info, io.BytesIO(payload))
        for name in layer_names:
            tar.add(stage / name, arcname=name)
    with archive.open('rb') as handle:
        archive_hash = hashlib.file_digest(handle, 'sha256').hexdigest()
    record = {'repository': REPOSITORY, 'tag': 'jazzy-ros-base',
              'platform': 'linux/arm64/v8', 'manifest_digest': manifest_digest,
              'config_digest': manifest['config']['digest'],
              'archive_sha256': archive_hash, 'archive': str(archive)}
    (stage / 'verification.json').write_text(json.dumps(record, indent=2), encoding='utf-8')
    print(f'VERIFIED_ARCHIVE={archive}\nARCHIVE_SHA256={archive_hash}', flush=True)


if __name__ == '__main__':
    main()
