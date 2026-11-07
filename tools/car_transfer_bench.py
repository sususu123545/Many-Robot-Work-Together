#!/usr/bin/env python3
"""Compare SFTP and raw SSH-channel throughput to the car.

Why bother: SFTP moved about 1.6 KB/s to this Pi while the rosbridge tunnel
carried 70 KB images at 8 Hz over the same link, so the bottleneck is the SFTP
window (32 KB per round trip) against a relayed path with 100-200 ms RTT, not
bandwidth.  A raw exec channel streams without waiting for acknowledgements.

Run this once whenever transfers feel slow; it prints both numbers so the choice
in car_put.py stays evidence-based.
"""

import os
import sys
import time

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, 'tools'))

import car_put  # noqa: E402

HOST = '100.102.57.19'
USER = 'pi'
SIZE = 2 * 1024 * 1024        # 2 MiB of random bytes: no compression can help


def stream_put(client, payload, remote):
    """Write bytes through a plain exec channel, no SFTP handshake."""
    channel = client.get_transport().open_session()
    channel.exec_command('cat > %s' % remote)
    view = memoryview(payload)
    sent = 0
    while sent < len(payload):
        chunk = view[sent:sent + 65536]
        channel.sendall(chunk)
        sent += len(chunk)
    channel.shutdown_write()
    code = channel.recv_exit_status()
    if code != 0:
        raise SystemExit('remote cat failed with %d' % code)


def main():
    payload = os.urandom(SIZE)
    local = os.path.join(REPO, 'tools', 'runtime', 'speedtest.bin')
    if not os.path.isdir(os.path.dirname(local)):
        os.makedirs(os.path.dirname(local))
    with open(local, 'wb') as handle:
        handle.write(payload)

    client = car_put.connect(HOST, USER)
    try:
        sftp = client.open_sftp()

        started = time.time()
        sftp.put(local, '/home/pi/speedtest_sftp.bin')
        sftp_elapsed = time.time() - started

        started = time.time()
        stream_put(client, payload, '/home/pi/speedtest_stream.bin')
        stream_elapsed = time.time() - started

        sftp_size = sftp.stat('/home/pi/speedtest_sftp.bin').st_size
        stream_size = sftp.stat('/home/pi/speedtest_stream.bin').st_size

        sftp.remove('/home/pi/speedtest_sftp.bin')
        sftp.remove('/home/pi/speedtest_stream.bin')
        sftp.close()
    finally:
        client.close()
    os.remove(local)

    print('%d KiB test' % (SIZE // 1024))
    print('  sftp   %6.1f s  %8.1f KiB/s   (%d bytes landed)'
          % (sftp_elapsed, SIZE / 1024.0 / sftp_elapsed, sftp_size))
    print('  stream %6.1f s  %8.1f KiB/s   (%d bytes landed)'
          % (stream_elapsed, SIZE / 1024.0 / stream_elapsed, stream_size))
    return 0


if __name__ == '__main__':
    sys.exit(main())
