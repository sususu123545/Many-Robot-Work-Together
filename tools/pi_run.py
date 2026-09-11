#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Reusable one-shot SSH runner for the ArmPi Pro car.

Usage:
    python pi_run.py --host 10.120.150.178 --user pi --password raspberrypi \
        --cmd "hostname" --cmd "uname -m" [--timeout 15]

Design notes (learned the hard way on a flaky WiFi link):
  * never trust the bash tool's stdout -> caller redirects to a .log and Reads it
  * exec_command(timeout=) only bounds channel *open*
  * return PARTIAL output on deadline
  * os._exit() to avoid paramiko thread hangs
"""
import argparse
import os
import sys
import threading
import time

import paramiko

T0 = time.time()


def log(msg):
    print("[%7.1fs] %s" % (time.time() - T0, msg), flush=True)


def run_cmd(client, cmd, timeout):
    """Run a command with a real deadline; returns (rc, stdout, stderr)."""
    log("  exec: %s" % (cmd if len(cmd) < 100 else cmd[:97] + "..."))
    try:
        chan = client.get_transport().open_session(timeout=timeout)
        chan.settimeout(timeout)
        chan.exec_command(cmd)
    except Exception as e:  # noqa: BLE001
        return None, "", "OPEN FAILED: %r" % (e,)

    out_b = b""
    err_b = b""
    deadline = time.time() + timeout
    try:
        while True:
            if time.time() > deadline:
                return (None,
                        out_b.decode("utf-8", "replace"),
                        (err_b.decode("utf-8", "replace")
                         + "\nTIMEOUT: still running after %ss (partial output kept)" % timeout))

            got = False
            if chan.recv_ready():
                data = chan.recv(65536)
                if data:
                    out_b += data
                    got = True
            if chan.recv_stderr_ready():
                data = chan.recv_stderr(65536)
                if data:
                    err_b += data
                    got = True
            if not got:
                if chan.exit_status_ready():
                    while chan.recv_ready():
                        data = chan.recv(65536)
                        if not data:
                            break
                        out_b += data
                    while chan.recv_stderr_ready():
                        data = chan.recv_stderr(65536)
                        if not data:
                            break
                        err_b += data
                    break
                time.sleep(0.05)
        rc = chan.recv_exit_status()
    except Exception as e:  # noqa: BLE001
        return None, out_b.decode("utf-8", "replace"), "TIMEOUT/ERROR: %r" % (e,)
    return rc, out_b.decode("utf-8", "replace"), err_b.decode("utf-8", "replace")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", required=True)
    ap.add_argument("--user", default="pi")
    ap.add_argument("--password", default="raspberrypi")
    ap.add_argument("--port", type=int, default=22)
    ap.add_argument("--timeout", type=float, default=15.0)
    ap.add_argument("--cmd", action="append", default=[])
    ap.add_argument("--global-timeout", type=float, default=0.0)
    ap.add_argument("--retries", type=int, default=5,
                    help="connect attempts (sshd drops the banner under load / "
                         "MaxStartups throttling)")
    ap.add_argument("--retry-sleep", type=float, default=6.0)
    args = ap.parse_args()

    client = None
    for attempt in range(1, args.retries + 1):
        client = paramiko.SSHClient()
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        log("connecting to %s@%s:%d (attempt %d/%d) ..."
            % (args.user, args.host, args.port, attempt, args.retries))
        try:
            client.connect(
                hostname=args.host, port=args.port,
                username=args.user, password=args.password,
                timeout=args.timeout, banner_timeout=args.timeout,
                auth_timeout=args.timeout,
                allow_agent=False, look_for_keys=False,
            )
            break
        except Exception as e:  # noqa: BLE001
            log("  attempt %d failed: %r" % (attempt, e))
            try:
                client.close()
            except Exception:  # noqa: BLE001
                pass
            client = None
            if attempt < args.retries:
                time.sleep(args.retry_sleep)
    if client is None:
        log("ERROR: all %d connect attempts failed" % args.retries)
        return 1
    log("CONNECTED + AUTH OK  (%s@%s)" % (args.user, args.host))

    for c in args.cmd:
        rc, out, err = run_cmd(client, c, args.timeout)
        print("### CMD: %s" % c, flush=True)
        print("### RC: %s" % rc, flush=True)
        if out.strip():
            print("--- STDOUT ---", flush=True)
            print(out, flush=True)
        if err.strip():
            print("--- STDERR ---", flush=True)
            print(err, flush=True)
        print("### END", flush=True)

    try:
        t = threading.Thread(target=lambda: client.close(), daemon=True)
        t.start()
        t.join(timeout=4)
    except Exception:  # noqa: BLE001
        pass
    return 0


def _entry():
    try:
        rc = main()
    except Exception as e:  # noqa: BLE001
        log("UNEXPECTED: %r" % (e,))
        rc = 1
    print("EXIT=%d" % rc, flush=True)
    os._exit(rc)


if __name__ == "__main__":
    pre = argparse.ArgumentParser(add_help=False)
    pre.add_argument("--timeout", type=float, default=15.0)
    pre.add_argument("--global-timeout", type=float, default=0.0)
    pre.add_argument("--retries", type=int, default=5)
    pre.add_argument("--retry-sleep", type=float, default=6.0)
    ns, _ = pre.parse_known_args()
    total = ns.global_timeout or (ns.timeout * 10.0 + 60.0 + ns.retries * ns.retry_sleep)
    th = threading.Thread(target=_entry, daemon=True)
    th.start()
    th.join(total)
    if th.is_alive():
        print("[%7.1fs] GLOBAL TIMEOUT after %.0fs -- process killed" % (time.time() - T0, total), flush=True)
        os._exit(1)
