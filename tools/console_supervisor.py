#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Keep the local ArmPi console proxy and SSH ROS tunnel alive.

The web page is served on localhost:8000, while roslibjs connects to
localhost:9090.  They are deliberately managed as two children so a failed
SSH connection cannot take the web UI down with it.  The supervisor restarts
either child when it exits or its listening port disappears.
"""

import argparse
import base64
import concurrent.futures
import hashlib
import ipaddress
import json
import logging
import os
from pathlib import Path
import re
import signal
import socket
import subprocess
import sys
import time


ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / "tools"
WEB_CONSOLE = ROOT / "web_console"
PROXY = WEB_CONSOLE / "proxy_server(1).py"
TUNNEL = TOOLS / "ssh_tunnel.py"
DEFAULT_CONFIG = TOOLS / "console_supervisor.local.json"
LOG_DIR = TOOLS / "runtime"
STATUS_FILE = LOG_DIR / "robot_status.json"
REPAIR_REQUEST_FILE = LOG_DIR / "repair_tunnel.request.json"
DEFAULT_ROBOT_MAC = "88-a2-9e-29-e1-e0"  # tools/net_scan.py 中识别的小车 Wi-Fi MAC
DISCOVERY_INTERVAL = 12.0
DISCOVERY_MISS_LIMIT = 3
DISCOVERY_AMBIGUOUS = "同一小车 MAC 对应多个可用 IP，暂不连接以避免误控"
WEBSOCKET_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"


def port_open(port):
    try:
        with socket.create_connection(("127.0.0.1", port), timeout=0.4):
            return True
    except OSError:
        return False


def websocket_handshake_ready(host="127.0.0.1", port=9090, timeout=1.2):
    """Verify a real WebSocket upgrade, not merely a listening TCP socket."""
    key = base64.b64encode(os.urandom(16)).decode("ascii")
    expected = base64.b64encode(
        hashlib.sha1((key + WEBSOCKET_GUID).encode("ascii")).digest()
    ).decode("ascii")
    request = (
        "GET / HTTP/1.1\r\n"
        "Host: %s:%d\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        "Sec-WebSocket-Key: %s\r\n"
        "Sec-WebSocket-Version: 13\r\n\r\n"
    ) % (host, port, key)
    try:
        with socket.create_connection((host, port), timeout=timeout) as conn:
            conn.settimeout(timeout)
            conn.sendall(request.encode("ascii"))
            response = bytearray()
            while b"\r\n\r\n" not in response and len(response) < 8192:
                chunk = conn.recv(1024)
                if not chunk:
                    break
                response.extend(chunk)
    except OSError:
        return False

    try:
        headers = bytes(response).split(b"\r\n\r\n", 1)[0].decode("latin1")
        lines = headers.split("\r\n")
        fields = {}
        for line in lines[1:]:
            if ":" in line:
                name, value = line.split(":", 1)
                fields[name.strip().lower()] = value.strip()
        return (
            lines[0].startswith("HTTP/1.1 101")
            and fields.get("upgrade", "").lower() == "websocket"
            and "upgrade" in fields.get("connection", "").lower()
            and fields.get("sec-websocket-accept") == expected
        )
    except (UnicodeDecodeError, IndexError):
        return False


def take_repair_request(path=None):
    """Atomically claim one queued repair request so it cannot run twice."""
    path = Path(path or REPAIR_REQUEST_FILE)
    claimed = path.with_name(path.name + ".processing." + str(os.getpid()))
    try:
        os.replace(str(path), str(claimed))
    except FileNotFoundError:
        return None
    except OSError:
        return None
    try:
        with claimed.open("r", encoding="utf-8") as handle:
            return json.load(handle)
    except (OSError, ValueError):
        return {}
    finally:
        try:
            claimed.unlink()
        except OSError:
            pass


def validate_repair_request(request, active_host, tunnel, now=None):
    """Only allow a fresh request for the currently MAC-verified managed tunnel."""
    now = time.time() if now is None else now
    if not isinstance(request, dict) or not request.get("request_id"):
        return False, "修复请求格式无效"
    try:
        age = now - float(request.get("requested_at", 0))
    except (TypeError, ValueError):
        return False, "修复请求时间无效"
    if age < 0 or age > 30:
        return False, "修复请求已过期，请重新点击"
    if not active_host or request.get("host") != active_host:
        return False, "小车 IP 已变化或尚未确认，拒绝重建隧道"
    if tunnel.last_state == "external" or (tunnel.proc is None and port_open(9090)):
        return False, "本机 9090 被未受监督的程序占用，未触碰该程序"
    if not tunnel.command:
        return False, "当前没有已确认的小车 SSH 隧道目标"
    return True, ""


def normalize_mac(value):
    return re.sub(r"[^0-9a-f]", "", str(value or "").lower())


def parse_network_records(payload):
    """Parse Get-NetIPAddress JSON into (IPv4, prefix length) pairs."""
    if not payload or not payload.strip():
        return []
    try:
        rows = json.loads(payload)
    except (TypeError, ValueError):
        return []
    if isinstance(rows, dict):
        rows = [rows]
    result = []
    for row in rows:
        try:
            address = ipaddress.ip_address(str(row.get("address", "")))
            prefix = int(row.get("prefix", -1))
        except (ValueError, TypeError):
            continue
        if (address.version == 4 and address.is_private and
                not address.is_loopback and not address.is_link_local):
            if 0 <= prefix <= 32:
                result.append((str(address), prefix))
    return result


def parse_ipconfig_networks(payload):
    """Fallback parser for English/Chinese ipconfig IPv4 and mask labels."""
    address_re = re.compile(
        r"^\s*(?:IPv4\s+Address|IPv4\s*地址)[^\r\n:]*:\s*([0-9.]+)",
        re.IGNORECASE | re.MULTILINE,
    )
    mask_re = re.compile(
        r"^\s*(?:Subnet\s+Mask|子网掩码)[^\r\n:]*:\s*([0-9.]+)",
        re.IGNORECASE | re.MULTILINE,
    )
    addresses = list(address_re.finditer(payload or ""))
    masks = list(mask_re.finditer(payload or ""))
    result = []
    for index, match in enumerate(addresses):
        end = addresses[index + 1].start() if index + 1 < len(addresses) else len(payload)
        mask = next((item for item in masks if match.end() <= item.start() < end), None)
        if not mask:
            continue
        try:
            prefix = ipaddress.IPv4Network("0.0.0.0/" + mask.group(1)).prefixlen
            address = ipaddress.IPv4Address(match.group(1))
        except ValueError:
            continue
        if address.is_private and not address.is_loopback and not address.is_link_local:
            result.append((str(address), prefix))
    return result


def parse_arp_table(payload):
    """Return an IPv4-to-normalized-MAC map from Windows' localized arp output."""
    found = {}
    pattern = re.compile(
        r"(?P<ip>(?:\d{1,3}\.){3}\d{1,3})\s+"
        r"(?P<mac>(?:[0-9a-fA-F]{2}[-:]){5}[0-9a-fA-F]{2})"
    )
    for match in pattern.finditer(payload or ""):
        found[match.group("ip")] = normalize_mac(match.group("mac"))
    return found


def connected_physical_networks():
    """Read active physical-adapter IPv4 subnets (typically the phone hotspot)."""
    script = r"""
$up = @(Get-NetAdapter -Physical -ErrorAction SilentlyContinue |
  Where-Object { $_.Status -eq 'Up' } | ForEach-Object { [int]$_.ifIndex })
$rows = @(Get-NetIPAddress -AddressFamily IPv4 -ErrorAction SilentlyContinue |
  Where-Object { $up -contains [int]$_.InterfaceIndex -and
    $_.IPAddress -notlike '127.*' -and $_.IPAddress -notlike '169.254.*' } |
  ForEach-Object { [pscustomobject]@{ address=$_.IPAddress; prefix=[int]$_.PrefixLength } })
$rows | ConvertTo-Json -Compress
"""
    try:
        result = subprocess.run(
            ["powershell.exe", "-NoProfile", "-NonInteractive", "-Command", script],
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=8,
            creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
        )
        networks = parse_network_records(result.stdout) if result.returncode == 0 else []
    except (OSError, subprocess.SubprocessError) as exc:
        logging.info("PowerShell adapter query unavailable; falling back to ipconfig: %s", exc)
        networks = []
    if networks:
        return networks

    try:
        result = subprocess.run(
            ["ipconfig"], capture_output=True, text=True, encoding="mbcs",
            errors="replace", timeout=8,
            creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
        )
        networks = parse_ipconfig_networks(result.stdout)
        if networks:
            logging.info("using ipconfig fallback for local IPv4 subnet discovery")
        return networks
    except (OSError, subprocess.SubprocessError) as exc:
        logging.warning("could not enumerate local IPv4 networks: %s", exc)
        return []


def _ping_host(host):
    try:
        result = subprocess.run(
            ["ping", "-n", "1", "-w", "350", host],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=3,
            creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
        )
        return host if result.returncode == 0 else None
    except (OSError, subprocess.SubprocessError):
        return None


def _tcp_host(host, timeout=0.6):
    """Check the services the console actually needs, without relying on ICMP."""
    for port in (9090, 22):
        try:
            with socket.create_connection((host, port), timeout=timeout):
                return True
        except OSError:
            continue
    return False


def read_arp_table():
    try:
        result = subprocess.run(
            ["arp", "-a"], capture_output=True, text=True,
            encoding="mbcs", errors="replace", timeout=5,
            creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
        )
        return parse_arp_table(result.stdout) if result.returncode == 0 else {}
    except (OSError, subprocess.SubprocessError):
        return {}


def discover_robot_ip(robot_mac, preferred_host=None):
    """Find the configured robot MAC on the active physical LAN and refresh its ARP entry."""
    networks = connected_physical_networks()
    targets = set()
    for address, prefix in networks:
        try:
            network = ipaddress.ip_network("%s/%s" % (address, prefix), strict=False)
        except ValueError:
            continue
        # Avoid unexpectedly sweeping a large office/VPN subnet. Phone hotspots
        # and ordinary robot LANs are normally /24 or smaller.
        if network.num_addresses > 512:
            logging.warning("skip oversized adapter subnet %s", network)
            continue
        targets.update(str(host) for host in network.hosts() if str(host) != address)
    if not targets:
        return None, "未找到可扫描的物理 IPv4 网段"

    target_mac = normalize_mac(robot_mac)

    # Refresh a known cached robot address first; the full sweep is only needed
    # after the hotspot has assigned a different address or the cache is empty.
    cached = [host for host, mac in read_arp_table().items()
              if host in targets and mac == target_mac]
    if preferred_host in targets and preferred_host not in cached:
        cached.insert(0, preferred_host)
    responsive = {host for host in cached if _tcp_host(host)}
    refreshed = read_arp_table()
    matches = sorted(host for host in responsive
                     if refreshed.get(host) == target_mac)
    if not matches:
        # Retry cached hosts too: one failed probe must not permanently exclude
        # the known robot from this discovery cycle.
        with concurrent.futures.ThreadPoolExecutor(max_workers=64) as pool:
            responsive = {host for host in pool.map(_ping_host, targets) if host}
        refreshed = read_arp_table()
        matches = sorted(host for host in responsive
                         if refreshed.get(host) == target_mac and _tcp_host(host))
    if preferred_host in matches:
        return preferred_host, "已通过小车 MAC 与实时 TCP 确认"
    if len(matches) == 1:
        return matches[0], "已通过小车 MAC 与实时 TCP 确认"
    if len(matches) > 1:
        return None, DISCOVERY_AMBIGUOUS
    return None, "热点内未发现 MAC 匹配且在线的小车"


def stabilize_discovery(found_host, active_host, misses, unsafe=False):
    """Keep a verified connection across brief probe failures, not identity conflicts."""
    if found_host is not None:
        return found_host, 0
    if active_host is None or unsafe:
        return None, 0
    misses += 1
    if misses >= DISCOVERY_MISS_LIMIT:
        return None, misses
    return active_host, misses


def write_robot_status(state, host=None, message="", repair=None):
    payload = {
        "state": state,
        "ip": host,
        "message": message,
        "updated_at": int(time.time()),
        "supervisor_heartbeat": time.time(),
    }
    if repair:
        payload["repair"] = repair
    try:
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        temporary = STATUS_FILE.with_suffix(".tmp")
        temporary.write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")
        os.replace(str(temporary), str(STATUS_FILE))
    except OSError:
        logging.exception("could not publish robot discovery status")


def child_flags():
    flags = getattr(subprocess, "CREATE_NO_WINDOW", 0)
    flags |= getattr(subprocess, "CREATE_NEW_PROCESS_GROUP", 0)
    return flags


class ManagedChild:
    def __init__(self, name, command, cwd, port, log_path, extra_env=None):
        self.name = name
        self.command = command
        self.cwd = cwd
        self.port = port
        self.log_path = log_path
        self.extra_env = extra_env or {}
        self.proc = None
        self.log_file = None
        self.started_at = 0.0
        self.next_start = 0.0
        self.failures = 0
        self.last_state = None

    def set_command(self, command, extra_env=None):
        extra_env = extra_env or {}
        if command == self.command and extra_env == self.extra_env:
            return
        self.stop()
        self.command = command
        self.extra_env = extra_env
        self.failures = 0
        self.next_start = 0.0
        self.last_state = None
        if command:
            logging.info("updated %s target; restarting managed process", self.name)

    def _close_log(self):
        if self.log_file:
            try:
                self.log_file.close()
            except OSError:
                pass
            self.log_file = None

    def stop(self):
        if self.proc and self.proc.poll() is None:
            try:
                self.proc.terminate()
                self.proc.wait(timeout=3)
            except (OSError, subprocess.TimeoutExpired):
                try:
                    self.proc.kill()
                except OSError:
                    pass
        self.proc = None
        self._close_log()

    def start_if_needed(self, now):
        if not self.command:
            return
        if self.proc and self.proc.poll() is None:
            # A child that is alive but never opened its port is stuck or
            # failed to bind. Give it a short grace period, then restart it.
            if now - self.started_at > 8 and not port_open(self.port):
                logging.warning("%s alive but port %d is closed; restarting", self.name, self.port)
                self.stop()
            else:
                return

        if self.proc is not None:
            code = self.proc.poll()
            self.proc = None
            self._close_log()
            self.failures += 1
            delay = min(60, max(2, 2 ** min(self.failures, 5)))
            self.next_start = now + delay
            logging.warning("%s exited with code %s; retry in %ss", self.name, code, delay)

        # An already-running manual instance is healthy; do not create a
        # duplicate listener. The supervisor will take ownership once it dies.
        if port_open(self.port):
            if self.last_state != "external":
                logging.info("%s port %d already listening; leaving it alone", self.name, self.port)
                self.last_state = "external"
            return
        if now < self.next_start:
            return

        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        self.log_file = self.log_path.open("ab")
        env = os.environ.copy()
        env.update(self.extra_env)
        try:
            self.proc = subprocess.Popen(
                self.command,
                cwd=str(self.cwd),
                stdin=subprocess.DEVNULL,
                stdout=self.log_file,
                stderr=subprocess.STDOUT,
                creationflags=child_flags(),
                close_fds=False,
                env=env,
            )
        except OSError:
            self._close_log()
            self.failures += 1
            self.next_start = now + min(60, max(2, 2 ** min(self.failures, 5)))
            logging.exception("failed to start %s", self.name)
            return
        self.started_at = now
        self.last_state = "starting"
        logging.info("started %s (pid=%s)", self.name, self.proc.pid)


def load_config(path):
    config = {
        "host": "192.168.19.178",
        "robot_mac": DEFAULT_ROBOT_MAC,
        "user": "pi",
        "password": os.environ.get("ARMPI_SSH_PASSWORD", ""),
    }
    if path.exists():
        with path.open("r", encoding="utf-8-sig") as handle:
            data = json.load(handle)
        config.update({key: value for key, value in data.items() if value is not None})
    if not config.get("password"):
        raise RuntimeError("SSH password missing: set ARMPI_SSH_PASSWORD or configure the local JSON file")
    return config


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    args = parser.parse_args()
    config = load_config(args.config.expanduser().resolve())

    LOG_DIR.mkdir(parents=True, exist_ok=True)
    logging.basicConfig(
        filename=str(LOG_DIR / "console_supervisor.log"),
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
    )
    logging.info("supervisor starting for %s; target discovery by MAC %s",
                 config["user"], normalize_mac(config.get("robot_mac")))

    python = sys.executable
    proxy = ManagedChild(
        "web proxy",
        [python, str(PROXY)],
        WEB_CONSOLE,
        8000,
        LOG_DIR / "proxy.log",
    )
    tunnel = ManagedChild(
        "SSH ROS tunnel",
        None,
        ROOT,
        9090,
        LOG_DIR / "tunnel.log",
        extra_env={"ARMPI_SSH_PASSWORD": str(config["password"])},
    )
    children = [proxy, tunnel]
    stopping = False
    active_host = None
    discovery_misses = 0
    next_discovery = 0.0
    last_status = None
    last_status_write = 0.0
    repair = None
    repair_deadline = 0.0
    next_ws_check = 0.0

    def publish_status(state, host, message):
        nonlocal last_status, last_status_write
        signature = (state, host, message)
        now = time.monotonic()
        if signature != last_status or now - last_status_write >= 3.0:
            write_robot_status(state, host, message, repair=repair)
            last_status = signature
            last_status_write = now

    def set_repair(request_id, state, message):
        nonlocal repair
        repair = {
            "request_id": request_id,
            "state": state,
            "message": message,
            "updated_at": int(time.time()),
        }
        logging.info("connection repair %s: %s (%s)", request_id, state, message)

    def tunnel_command(host):
        return [
            python, str(TUNNEL),
            "--host", str(host),
            "--user", str(config.get("user", "pi")),
            "--local-port", "9090",
            "--remote-host", "127.0.0.1",
            "--remote-port", "9090",
        ]

    def shutdown(_signum=None, _frame=None):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, shutdown)
    if hasattr(signal, "SIGTERM"):
        signal.signal(signal.SIGTERM, shutdown)

    try:
        while not stopping:
            now = time.monotonic()
            request = take_repair_request()
            if request is not None:
                request_id = str(request.get("request_id", ""))[:80] or "unknown"
                allowed, reason = validate_repair_request(request, active_host, tunnel)
                if not allowed:
                    set_repair(request_id, "failed", reason)
                else:
                    # This child owns only the local SSH port forward. Restarting
                    # it never restarts ROS nodes or publishes a motion command.
                    tunnel.stop()
                    tunnel.failures = 0
                    tunnel.next_start = now
                    tunnel.last_state = None
                    repair_deadline = now + 30.0
                    next_ws_check = now + 0.5
                    set_repair(request_id, "restarting", "正在重建本机 SSH 隧道…")

            if now >= next_discovery:
                found_host, message = discover_robot_ip(
                    config.get("robot_mac", DEFAULT_ROBOT_MAC),
                    preferred_host=active_host,
                )
                observed_mac = read_arp_table().get(active_host) if active_host else None
                unsafe = (message == DISCOVERY_AMBIGUOUS or
                          (observed_mac is not None and
                           observed_mac != normalize_mac(config.get("robot_mac", DEFAULT_ROBOT_MAC))))
                host, discovery_misses = stabilize_discovery(
                    found_host, active_host, discovery_misses, unsafe=unsafe,
                )
                if found_host is None and host == active_host and active_host:
                    logging.warning("robot discovery missed %d/%d times; keeping verified tunnel to %s",
                                    discovery_misses, DISCOVERY_MISS_LIMIT, active_host)
                    message = "小车探测短暂失败，保留已确认的连接"
                if host != active_host:
                    active_host = host
                    tunnel.set_command(
                        tunnel_command(host) if host else None,
                        {"ARMPI_SSH_PASSWORD": str(config["password"])} if host else {},
                    )
                    logging.info("robot discovery: %s (%s)", host or "not found", message)
                elif not host:
                    logging.debug("robot discovery: %s", message)

                if host:
                    state = "tunnel_listening" if tunnel.proc and port_open(9090) else "connecting"
                    if tunnel.last_state == "external":
                        state = "port_conflict"
                        message = "本机 9090 被未受监督的进程占用；为避免误连，不自动连接"
                    publish_status(state, host, message)
                else:
                    publish_status("searching", None, message)
                next_discovery = now + DISCOVERY_INTERVAL

            for child in children:
                child.start_if_needed(now)

            if repair and repair.get("state") in ("restarting", "verifying"):
                if tunnel.proc and tunnel.proc.poll() is None and port_open(9090):
                    if repair.get("state") != "verifying":
                        set_repair(repair["request_id"], "verifying",
                                   "隧道已建立，正在验证 rosbridge WebSocket…")
                        next_ws_check = now
                    if now >= next_ws_check:
                        if websocket_handshake_ready():
                            set_repair(repair["request_id"], "complete",
                                       "连接修复完成，rosbridge WebSocket 握手成功")
                        else:
                            next_ws_check = now + 1.5
                if repair.get("state") in ("restarting", "verifying") and now >= repair_deadline:
                    set_repair(repair["request_id"], "failed",
                               "隧道未能在 30 秒内通过 rosbridge 握手；请检查小车端 SSH/9090 服务")

            if active_host:
                if repair and repair.get("state") in ("restarting", "verifying"):
                    publish_status("repairing", active_host, repair["message"])
                elif tunnel.last_state == "external":
                    publish_status("port_conflict", active_host,
                                   "本机 9090 被未受监督的进程占用；为避免误连，不自动连接")
                elif tunnel.proc and tunnel.proc.poll() is None and port_open(9090):
                    publish_status("tunnel_listening", active_host, "小车已发现，SSH 隧道已就绪")
                else:
                    if tunnel.proc is None and now < tunnel.next_start:
                        retry_in = max(1, int(tunnel.next_start - now))
                        message = "小车 IP 已确认，但 SSH 隧道连接失败；%d 秒后重试" % retry_in
                        publish_status("ssh_retry", active_host, message)
                    else:
                        publish_status("connecting", active_host, "小车已发现，正在建立 SSH 隧道")
            time.sleep(1.0)
    finally:
        for child in children:
            child.stop()
        publish_status("stopped", None, "本地控制台监督进程已停止")
        logging.info("supervisor stopped")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception:
        logging.exception("supervisor failed")
        raise
