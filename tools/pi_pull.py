#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Download a file from the car.

Strategy:
  1) try SFTP, with a hard deadline
  2) fall back to exec'ing `base64 -w0 <remote>` and decoding locally

Usage:
    python pi_pull.py --host 10.120.150.178 --remote /home/pi/x.jpg --local x.jpg
"""
import argparse
import base64
import hashlib
import os
import threading
import time

import paramiko

T0 = time.time()


def log(msg):
    print("[%7.1fs] %s" % (time.time() - T0, msg), flush=True)


def sftp_get(client, remote, local, timeout):
    """Returns True on success."""
    tr = client.get_transport()
    dest = {}

    def work():
        try:
            sf = paramiko.SFTPClient.from_transport(tr)
            sf.get_channel().settimeout(timeout)
            sf.get(remote, local)
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


def base64_get(client, remote, local, timeout):
    """Fallback: exec `base64 -w0 <remote>` and decode locally."""
    chan = client.get_transport().open_session(timeout=timeout)
    chan.settimeout(timeout)
    chan.exec_command("base64 -w0 %s" % remote)
    buf = b""
    deadline = time.time() + max(timeout, 120)
    while True:
        if time.time() > deadline:
            log("  B64: deadline hit at %d bytes" % len(buf))
            break
        try:
            chunk = chan.recv(65536)
        except Exception:  # noqa: BLE001
            chunk = b""
        if not chunk:
            if chan.exit_status_ready():
                break
            time.sleep(0.05)
            continue
        buf += chunk
    txt = buf.decode("ascii", "ignore").strip()
    if not txt:
        log("  B64: empty response")
        return False
    try:
        data = base64.b64decode(txt)
    except Exception as e:  # noqa: BLE001
        log("  B64 decode failed: %r" % (e,))
        return False
    with open(local, "wb") as f:
        f.write(data)
    log("  B64: OK %d bytes" % len(data))
    return True


def sha256_of(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", required=True)
    ap.add_argument("--user", default="pi")
    ap.add_argument("--password", default="raspberrypi")
    ap.add_argument("--remote", required=True)
    ap.add_argument("--local", required=True)
    ap.add_argument("--timeout", type=float, default=60.0)
    ap.add_argument("--mode", default="auto", choices=["auto", "sftp", "base64"])
    args = ap.parse_args()

    outdir = os.path.dirname(os.path.abspath(args.local))
    if outdir and not os.path.isdir(outdir):
        os.makedirs(outdir, exist_ok=True)

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
        ok = sftp_get(client, args.remote, args.local, args.timeout)
    if not ok and args.mode != "sftp":
        log("  falling back to base64-over-exec ...")
        ok = base64_get(client, args.remote, args.local, args.timeout)

    if not ok:
        log("DOWNLOAD FAILED")
        os._exit(1)

    # verify against the remote sha256
    chan = client.get_transport().open_session(timeout=30)
    chan.settimeout(30)
    chan.exec_command("sha256sum %s" % args.remote)
    time.sleep(0.8)
    out = b""
    while chan.recv_ready():
        out += chan.recv(65536)
    txt = out.decode("utf-8", "replace")
    remote_sha = txt.split()[0] if txt.split() else ""
    local_sha = sha256_of(args.local)
    match = remote_sha == local_sha
    log("size=%d  remote_sha=%s  local_sha=%s  MATCH=%s" % (
        os.path.getsize(args.local), remote_sha[:16], local_sha[:16], match))
    log("EXIT=%d" % (0 if match else 1))
    os._exit(0 if match else 1)


if __name__ == "__main__":
    try:
        main()
    except SystemExit:
        raise
    except Exception as e:  # noqa: BLE001
        log("UNEXPECTED: %r" % (e,))
        print("EXIT=1", flush=True)
        os._exit(1)
