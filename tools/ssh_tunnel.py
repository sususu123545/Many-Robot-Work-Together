#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Forward a local TCP port to a host-side service through SSH.

Used by the local ArmPi console when the robot's SSH port is reachable but
the ROS WebSocket port is filtered by the Wi-Fi/AP network.
"""

import argparse
import os
import select
import socket
import sys
import threading

import paramiko


def relay(local_sock, channel):
    """Copy bytes in both directions until either side closes."""
    try:
        while True:
            readable, _, _ = select.select([local_sock, channel], [], [], 1.0)
            if not readable:
                if channel.closed or channel.eof_received:
                    break
                continue
            for source in readable:
                data = source.recv(65536)
                if not data:
                    return
                target = channel if source is local_sock else local_sock
                target.sendall(data)
    except (OSError, EOFError, paramiko.SSHException):
        pass
    finally:
        try:
            local_sock.close()
        except OSError:
            pass
        try:
            channel.close()
        except OSError:
            pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", required=True)
    parser.add_argument("--user", default="pi")
    parser.add_argument("--password", default=os.environ.get("ARMPI_SSH_PASSWORD"))
    parser.add_argument("--local-port", type=int, default=9090)
    parser.add_argument("--remote-host", default="127.0.0.1")
    parser.add_argument("--remote-port", type=int, default=9090)
    args = parser.parse_args()
    if not args.password:
        parser.error("--password or ARMPI_SSH_PASSWORD is required")

    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(
        hostname=args.host,
        username=args.user,
        password=args.password,
        timeout=15,
        banner_timeout=15,
        auth_timeout=15,
        allow_agent=False,
        look_for_keys=False,
    )
    transport = ssh.get_transport()
    if transport is None or not transport.is_active():
        raise RuntimeError("SSH transport is not active")
    # Detect a dropped SSH connection even when no browser client is currently
    # connected.  Without a timeout, accept() could keep this process alive
    # forever while the local 9090 port points at a dead SSH transport.
    transport.set_keepalive(10)

    listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    listener.bind(("127.0.0.1", args.local_port))
    listener.listen(16)
    listener.settimeout(1.0)
    print(
        "SSH tunnel listening on 127.0.0.1:%d -> %s:%d"
        % (args.local_port, args.remote_host, args.remote_port),
        flush=True,
    )

    try:
        while transport.is_active():
            try:
                local_sock, client_addr = listener.accept()
            except socket.timeout:
                # Re-check transport.is_active() regularly so the supervisor
                # can restart us after Wi-Fi/SSH/car restarts.
                continue
            try:
                channel = transport.open_channel(
                    "direct-tcpip",
                    (args.remote_host, args.remote_port),
                    client_addr,
                )
            except Exception:
                local_sock.close()
                continue
            threading.Thread(
                target=relay,
                args=(local_sock, channel),
                daemon=True,
            ).start()
    finally:
        listener.close()
        ssh.close()
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(0)
    except Exception as exc:
        print("SSH tunnel failed: %s" % exc, file=sys.stderr, flush=True)
        raise SystemExit(1)
