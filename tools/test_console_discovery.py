import base64
import hashlib
import json
import socket
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest import mock

import console_supervisor as supervisor

from console_supervisor import (
    DISCOVERY_MISS_LIMIT,
    discover_robot_ip,
    normalize_mac,
    parse_arp_table,
    parse_ipconfig_networks,
    parse_network_records,
    stabilize_discovery,
    take_repair_request,
    validate_repair_request,
    websocket_handshake_ready,
)


class ConsoleDiscoveryParsingTests(unittest.TestCase):
    def test_network_json_accepts_object_or_array(self):
        self.assertEqual(parse_network_records(
            '{"address":"192.168.43.12","prefix":24}'
        ), [("192.168.43.12", 24)])
        self.assertEqual(parse_network_records(
            '[{"address":"10.0.0.5","prefix":24},'
            '{"address":"169.254.1.2","prefix":16},'
            '{"address":"::1","prefix":128}]'
        ), [("10.0.0.5", 24)])

    def test_arp_parser_handles_windows_separators_and_noise(self):
        text = """
Interface: 192.168.43.12 --- 0x7
  192.168.43.1       aa-bb-cc-dd-ee-ff     dynamic
  192.168.43.9       88-a2-9e-29-e1-e0     dynamic
  192.168.43.10      88:a2:9e:29:e1:e0     dynamic
"""
        self.assertEqual(parse_arp_table(text), {
            "192.168.43.1": "aabbccddeeff",
            "192.168.43.9": "88a29e29e1e0",
            "192.168.43.10": "88a29e29e1e0",
        })

    def test_ipconfig_fallback_handles_english_and_chinese_labels(self):
        text = """
Wireless LAN adapter WLAN:
   IPv4 Address. . . . . . . . . . . : 10.199.168.87
   Subnet Mask . . . . . . . . . . . : 255.255.255.0
Ethernet adapter Ethernet:
   IPv4 地址. . . . . . . . . . . . : 192.168.43.10
   子网掩码. . . . . . . . . . . . . : 255.255.255.0
"""
        self.assertEqual(parse_ipconfig_networks(text), [
            ("10.199.168.87", 24), ("192.168.43.10", 24),
        ])

    def test_mac_normalization(self):
        self.assertEqual(normalize_mac("88-A2-9E-29-E1-E0"), "88a29e29e1e0")

    def test_cached_robot_uses_tcp_when_icmp_is_unreliable(self):
        robot = "10.0.0.2"
        with (mock.patch.object(supervisor, "connected_physical_networks",
                                return_value=[("10.0.0.1", 30)]),
              mock.patch.object(supervisor, "read_arp_table",
                                return_value={robot: normalize_mac(supervisor.DEFAULT_ROBOT_MAC)}),
              mock.patch.object(supervisor, "_tcp_host", return_value=True) as tcp,
              mock.patch.object(supervisor, "_ping_host") as ping):
            found, _ = discover_robot_ip(supervisor.DEFAULT_ROBOT_MAC, robot)
        self.assertEqual(found, robot)
        tcp.assert_called_once_with(robot)
        ping.assert_not_called()

    def test_full_sweep_retries_cached_robot_after_a_failed_tcp_probe(self):
        robot = "10.0.0.2"
        with (mock.patch.object(supervisor, "connected_physical_networks",
                                return_value=[("10.0.0.1", 30)]),
              mock.patch.object(supervisor, "read_arp_table",
                                return_value={robot: normalize_mac(supervisor.DEFAULT_ROBOT_MAC)}),
              mock.patch.object(supervisor, "_tcp_host", side_effect=[False, True]) as tcp,
              mock.patch.object(supervisor, "_ping_host",
                                side_effect=lambda host: host if host == robot else None) as ping):
            found, _ = discover_robot_ip(supervisor.DEFAULT_ROBOT_MAC, robot)
        self.assertEqual(found, robot)
        self.assertEqual(tcp.call_count, 2)
        self.assertIn(mock.call(robot), ping.call_args_list)

    def test_new_ip_must_match_robot_mac_even_when_old_ip_has_open_port(self):
        old_host, new_host = "10.0.0.2", "10.0.0.3"
        arp = {old_host: "001122334455",
               new_host: normalize_mac(supervisor.DEFAULT_ROBOT_MAC)}
        with (mock.patch.object(supervisor, "connected_physical_networks",
                                return_value=[("10.0.0.1", 29)]),
              mock.patch.object(supervisor, "read_arp_table", return_value=arp),
              mock.patch.object(supervisor, "_tcp_host", return_value=True),
              mock.patch.object(supervisor, "_ping_host") as ping):
            found, _ = discover_robot_ip(supervisor.DEFAULT_ROBOT_MAC, old_host)
        self.assertEqual(found, new_host)
        ping.assert_not_called()

    def test_transient_misses_keep_verified_host_but_repeated_misses_drop_it(self):
        robot = "10.0.0.2"
        misses = 0
        for expected in range(1, DISCOVERY_MISS_LIMIT):
            host, misses = stabilize_discovery(None, robot, misses)
            self.assertEqual((host, misses), (robot, expected))
        host, misses = stabilize_discovery(None, robot, misses)
        self.assertEqual((host, misses), (None, DISCOVERY_MISS_LIMIT))
        self.assertEqual(stabilize_discovery(robot, robot, misses), (robot, 0))

    def test_identity_conflict_drops_host_immediately(self):
        self.assertEqual(stabilize_discovery(None, "10.0.0.2", 0, unsafe=True),
                         (None, 0))


class ConsoleRepairTests(unittest.TestCase):
    def test_take_repair_request_claims_file_once(self):
        with tempfile.TemporaryDirectory() as directory:
            request_path = Path(directory) / "repair.json"
            request = {"request_id": "abc", "host": "10.0.0.2",
                       "requested_at": time.time()}
            request_path.write_text(json.dumps(request), encoding="utf-8")
            self.assertEqual(take_repair_request(request_path), request)
            self.assertIsNone(take_repair_request(request_path))
            self.assertFalse(request_path.with_name("repair.json.processing").exists())

    def test_repair_requires_fresh_request_for_managed_current_host(self):
        tunnel = type("Tunnel", (), {"last_state": None, "proc": object(),
                                      "command": ["ssh"]})()
        payload = {"request_id": "abc", "host": "10.0.0.2", "requested_at": 100}
        with mock.patch.object(supervisor, "port_open", return_value=False):
            self.assertEqual(validate_repair_request(payload, "10.0.0.2", tunnel, 105),
                             (True, ""))
            self.assertFalse(validate_repair_request(payload, "10.0.0.3", tunnel, 105)[0])
            self.assertFalse(validate_repair_request(payload, "10.0.0.2", tunnel, 140)[0])
        tunnel.last_state = "external"
        self.assertFalse(validate_repair_request(payload, "10.0.0.2", tunnel, 105)[0])

    def test_websocket_check_validates_upgrade_accept_key(self):
        listener = socket.socket()
        listener.bind(("127.0.0.1", 0))
        listener.listen(1)
        port = listener.getsockname()[1]

        def serve_one():
            connection, _ = listener.accept()
            with connection:
                request = bytearray()
                while b"\r\n\r\n" not in request:
                    request.extend(connection.recv(1024))
                key_line = next(line for line in bytes(request).split(b"\r\n")
                                if line.lower().startswith(b"sec-websocket-key:"))
                key = key_line.split(b":", 1)[1].strip().decode("ascii")
                accept = base64.b64encode(hashlib.sha1(
                    (key + supervisor.WEBSOCKET_GUID).encode("ascii")
                ).digest()).decode("ascii")
                response = (
                    "HTTP/1.1 101 Switching Protocols\r\n"
                    "Upgrade: websocket\r\nConnection: Upgrade\r\n"
                    "Sec-WebSocket-Accept: %s\r\n\r\n" % accept
                )
                connection.sendall(response.encode("ascii"))

        worker = threading.Thread(target=serve_one, daemon=True)
        worker.start()
        try:
            self.assertTrue(websocket_handshake_ready(port=port, timeout=1))
        finally:
            listener.close()
            worker.join(timeout=1)


if __name__ == "__main__":
    unittest.main()
