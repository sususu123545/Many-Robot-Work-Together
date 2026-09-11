#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Upload a local file to the car.

Strategy:
  1) try SFTP (fast link -> usually fine), with a hard deadline
  2) fall back to piping bytes through an exec'd `cat > dest`

Usage:
    python pi_upload.py --host 10.120.150.178 --local <path> --remote /home/pi/x.pkg
"""
import argparse
import hashlib
import os
import threading
import time

import paramiko

T0 = time.time()


def log(msg):
    print("[%7.1fs] %s" % (time.time() - T0, msg), flush=True)


def sha256_of(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def sftp_put(client, local, remote, timeout):
    """Returns True on success."""
    tr = client.get_transport()
    dest = {}

    def work():
        try:
            sf = paramiko.SFTPClient.from_transport(tr)
            sf.get_channel().settimeout(timeout)
            sf.put(local, remote)
            sf.close()
            dest["ok"] = True
        except Exception as e:  # noqa: BLE001
            dest["err"] = repr(e)

    t = threading.Thread(target=work, daemon=True)
    t.start()
    t.join(timeout + 20)
    if t.is_alive():
        log("  SFTP: still running after %.0fs -> giving up" % (timeout + 20))
        return False
    if dest.get("ok"):
        log("  SFTP: OK")
        return True
    log("  SFTP failed: %s" % dest.get("err"))
    return False


def pipe_put(client, local, remote, timeout):
    """Fallback: exec `cat > dest` and push bytes down stdin."""
    chan = client.get_transport().open_session(timeout=timeout)
    chan.settimeout(timeout)
    chan.exec_command("cat > %s" % remote)
    size = os.path.getsize(local)
    sent = 0
    deadline = time.time() + max(timeout, size / 20000.0)
    with open(local, "rb") as f:
        while True:
            if time.time() > deadline:
                log("  PIPE: deadline hit at %d/%d bytes" % (sent, size))
                return False
            chunk = f.read(32768)
            if not chunk:
                break
            view = memoryview(chunk)
            while len(view):
                if not chan.send_ready():
                    time.sleep(0.005)
                    if time.time() > deadline:
                        log("  PIPE: send blocked at %d/%d bytes" % (sent, size))
                        return False
                    continue
                n = chan.send(view)
                view = view[n:]
                sent += n
    chan.shutdown_write()
    # wait for cat to exit
    end = time.time() + 30
    while time.time() < end:
        if chan.exit_status_ready():
            break
        time.sleep(0.1)
    log("  PIPE: sent %d/%d bytes" % (sent, size))
    return sent == size


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", required=True)
    ap.add_argument("--user", default="pi")
    ap.add_argument("--password", default="raspberrypi")
    ap.add_argument("--local", required=True)
    ap.add_argument("--remote", required=True)
    ap.add_argument("--timeout", type=float, default=60.0)
    ap.add_argument("--mode", default="auto", choices=["auto", "sftp", "pipe"])
    args = ap.parse_args()

    if not os.path.isfile(args.local):
        log("ERROR: local not found: %s" % args.local)
        return 2
    local_sha = sha256_of(args.local)
    size = os.path.getsize(args.local)
    log("local: %s  (%d bytes)  sha256=%s" % (args.local, size, local_sha[:16]))

    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    log("connecting to %s@%s ..." % (args.user, args.host))
    client.connect(hostname=args.host, username=args.user, password=args.password,
                   timeout=15, banner_timeout=15, auth_timeout=15,
                   allow_agent=False, look_for_keys=False)
    client.get_transport().set_keepalive(10)
    log("CONNECTED")

    ok = False
    if args.mode in ("auto", "sftp"):
        ok = sftp_put(client, args.local, args.remote, args.timeout)
    if not ok and args.mode != "sftp":
        log("  falling back to exec-pipe upload ...")
        ok = pipe_put(client, args.local, args.remote, args.timeout)

    if not ok:
        log("UPLOAD FAILED")
        return 1

    # verify remotely
    chan = client.get_transport().open_session(timeout=30)
    chan.settimeout(30)
    chan.exec_command("sha256sum %s; stat -c %%s %s" % (args.remote, args.remote))
    time.sleep(1.0)
    out = b""
    while chan.recv_ready():
        out += chan.recv(65536)
    txt = out.decode("utf-8", "replace")
    log("remote: %s" % txt.strip().replace("\n", " | "))
    remote_sha = txt.split()[0] if txt.split() else ""
    log("SHA MATCH: %s" % (remote_sha == local_sha))
    log("EXIT=%d" % (0 if remote_sha == local_sha else 1))
    os._exit(0 if remote_sha == local_sha else 1)


if __name__ == "__main__":
    try:
        main()
    except SystemExit:
        raise
    except Exception as e:  # noqa: BLE001
        log("UNEXPECTED: %r" % (e,))
        print("EXIT=1", flush=True)
        os._exit(1)
