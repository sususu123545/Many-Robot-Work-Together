"""Bounded, read-only SSH command/EOF diagnostics; never print credentials."""
import argparse
import json
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ros2_preflight import connect


def check(client, label, command, eof=True, seconds=8):
    start = time.monotonic()
    print('BEGIN ' + label, flush=True)
    watchdog = threading.Timer(seconds + 3, client.close)
    watchdog.daemon = True
    watchdog.start()
    try:
        stdin, out, err = client.exec_command(command, timeout=seconds)
        print('EXEC_ACK ' + label, flush=True)
        channel = out.channel
        if eof:
            channel.shutdown_write()
        stdout = bytearray()
        stderr = bytearray()
        until = time.monotonic() + seconds
        while True:
            while channel.recv_ready():
                stdout.extend(channel.recv(65536))
            while channel.recv_stderr_ready():
                stderr.extend(channel.recv_stderr(65536))
            if channel.exit_status_ready() and not channel.recv_ready() and not channel.recv_stderr_ready():
                code = channel.recv_exit_status()
                break
            if time.monotonic() > until:
                code = 'TIMEOUT'
                break
            time.sleep(.02)
        channel.close()
        print(json.dumps({'check': label, 'eof_sent': eof, 'seconds': round(time.monotonic()-start, 3),
                          'exit': code, 'stdout': stdout.decode(errors='replace'),
                          'stderr': stderr.decode(errors='replace')}, ensure_ascii=False), flush=True)
    except Exception as exc:
        print(json.dumps({'check': label, 'seconds': round(time.monotonic()-start, 3),
                          'error': type(exc).__name__ + ': ' + str(exc)}), flush=True)
    finally:
        watchdog.cancel()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--host', required=True)
    p.add_argument('--config', required=True)
    p.add_argument('--logs-only', action='store_true')
    args = p.parse_args()
    sys.stdout.reconfigure(encoding='utf-8')
    cmd = 'timeout 5 docker exec armpi_pro sed -n 1,160p /home/ubuntu/ld06_ws/src/ldlidar_stl_ros/launch/ld06.launch'
    tests = [('echo_without_stdin_EOF', 'printf ssh_command_ok', False, 3),
             ('echo_with_stdin_EOF', 'printf ssh_command_ok', True, 3),
             ('host', 'hostname; uptime; getent passwd pi; free -m', True, 8),
             ('config_with_stdin_EOF', cmd, True, 7),
             ('services', 'systemctl is-active ssh docker; timeout 5 docker ps; ps -eo pid,ppid,stat,etime,comm | awk '+
              "'$5 ~ /sshd|docker|containerd|zsh/ {print}'", True, 8)]
    if args.logs_only:
        tests = [
            ('ssh_recent_logs', "sudo -n journalctl -u ssh --since '-20 min' --no-pager -n 45", True, 10),
            ('wifi_tcp', "ip -s link show wlan0; iw dev wlan0 link; ss -tin '( sport = :22 )'", True, 8),
            ('kernel_recent_faults', "sudo -n journalctl -k --since '-20 min' --no-pager | grep -Ei 'brcmfmac|wlan0|under.?voltage|out of memory|oom|mmc.*error' | tail -n 30; vcgencmd get_throttled", True, 10),
            ('ssh_limits', "sudo -n /usr/sbin/sshd -T 2>/dev/null | awk '$1 ~ /maxsessions|maxstartups|clientalive|logingrace/ {print}'", True, 8),
        ]
    for label, command, eof, seconds in tests:
        c = connect(args.host, config_path=args.config)
        try:
            print('CONNECTED ' + label, flush=True)
            check(c, label, command, eof, seconds)
        finally:
            c.close()


if __name__ == '__main__':
    main()
