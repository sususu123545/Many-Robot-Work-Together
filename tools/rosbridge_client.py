"""A minimal rosbridge client with no third-party dependencies.

Why this exists
---------------
On 2026-11-03 the robot's sshd accepted authentication in about a second but
never confirmed an ``exec`` request, so no command could be started on the board
``ssh -L`` port forwarding still worked, and rosbridge listens on
``127.0.0.1:9091``. Forwarding does not require the server to fork anything, so
tunnelling to rosbridge was the only way left to talk to the robot.

There was no WebSocket library on the workstation, so the protocol is
implemented here: RFC 6455 client handshake, masked text frames, ping/pong, and
just enough reassembly for rosbridge's message sizes. It is not a general
WebSocket library and does not try to be.

rosbridge's protocol is JSON over text frames::

    {"op": "subscribe", "topic": "/x", "type": "std_msgs/String"}
    {"op": "publish",   "topic": "/cmd_vel", "msg": {...}}
    {"op": "call_service", "service": "/x", "args": [], "id": "req-1"}
"""

import base64
import json
import time
import os
import socket
import struct


class RosbridgeError(RuntimeError):
    pass


class WebSocket:
    """Client-side WebSocket, text only."""

    def __init__(self, host, port, path='/', timeout=5.0, attempts=3):
        # rosbridge is single-threaded and queues; after a burst it can take a
        # few seconds to answer a new handshake. Failing on the first try made a
        # congested bridge look like a dead one.
        last = None
        for attempt in range(attempts):
            try:
                self.socket = socket.create_connection((host, port), timeout=timeout)
                self.socket.settimeout(timeout)
                self._buffer = b''
                self._handshake(host, port, path)
                return
            except (OSError, RosbridgeError) as error:
                last = error
                try:
                    self.socket.close()
                except Exception:                  # noqa: BLE001
                    pass
                time.sleep(1.5 * (attempt + 1))
        raise RosbridgeError('could not open rosbridge at %s:%d after %d attempts: %s'
                             % (host, port, attempts, last))

    def _handshake(self, host, port, path):
        key = base64.b64encode(os.urandom(16)).decode()
        request = (
            'GET %s HTTP/1.1\r\n'
            'Host: %s:%d\r\n'
            'Upgrade: websocket\r\n'
            'Connection: Upgrade\r\n'
            'Sec-WebSocket-Key: %s\r\n'
            'Sec-WebSocket-Version: 13\r\n\r\n'
            % (path, host, port, key))
        self.socket.sendall(request.encode())
        response = b''
        while b'\r\n\r\n' not in response:
            chunk = self.socket.recv(4096)
            if not chunk:
                raise RosbridgeError('connection closed during handshake')
            response += chunk
        head, _, rest = response.partition(b'\r\n\r\n')
        self._buffer = rest
        status = head.split(b'\r\n', 1)[0].decode('latin-1')
        if '101' not in status:
            raise RosbridgeError('websocket upgrade refused: ' + status)

    # -- framing -----------------------------------------------------------

    def _send_frame(self, opcode, payload):
        header = bytearray([0x80 | opcode])
        length = len(payload)
        if length < 126:
            header.append(0x80 | length)
        elif length < 65536:
            header.append(0x80 | 126)
            header += struct.pack('>H', length)
        else:
            header.append(0x80 | 127)
            header += struct.pack('>Q', length)
        mask = os.urandom(4)
        header += mask
        masked = bytes(byte ^ mask[index % 4] for index, byte in enumerate(payload))
        self.socket.sendall(bytes(header) + masked)

    def send_text(self, text):
        self._send_frame(0x1, text.encode('utf-8'))

    def _read_exact(self, count, deadline):
        while len(self._buffer) < count:
            remaining = deadline
            if remaining <= 0:
                raise RosbridgeError('timed out waiting for frame')
            self.socket.settimeout(max(0.05, remaining))
            try:
                chunk = self.socket.recv(65536)
            except socket.timeout:
                raise RosbridgeError('timed out waiting for frame')
            if not chunk:
                raise RosbridgeError('connection closed')
            self._buffer += chunk
        data, self._buffer = self._buffer[:count], self._buffer[count:]
        return data

    def recv_text(self, timeout=5.0):
        """Return the next text message payload, answering pings on the way."""
        import time
        deadline = time.monotonic() + timeout
        while True:
            header = self._read_exact(2, deadline - time.monotonic())
            fin = bool(header[0] & 0x80)
            opcode = header[0] & 0x0F
            masked = bool(header[1] & 0x80)
            length = header[1] & 0x7F
            if length == 126:
                length = struct.unpack('>H', self._read_exact(2, deadline - time.monotonic()))[0]
            elif length == 127:
                length = struct.unpack('>Q', self._read_exact(8, deadline - time.monotonic()))[0]
            mask = self._read_exact(4, deadline - time.monotonic()) if masked else None
            payload = self._read_exact(length, deadline - time.monotonic()) if length else b''
            if mask:
                payload = bytes(byte ^ mask[index % 4] for index, byte in enumerate(payload))
            if opcode == 0x9:                      # ping -> pong
                self._send_frame(0xA, payload)
                continue
            if opcode == 0x8:                      # close
                raise RosbridgeError('server closed the connection')
            if opcode in (0x1, 0x0) and fin:
                return payload.decode('utf-8', 'replace')
            # Fragmented or binary frames are not used by rosbridge here.

    def close(self):
        try:
            self._send_frame(0x8, b'')
        except Exception:                          # noqa: BLE001 - best effort
            pass
        try:
            self.socket.close()
        except Exception:                          # noqa: BLE001
            pass


class Rosbridge:
    """Thin request/response wrapper over the rosbridge JSON protocol."""

    def __init__(self, host='127.0.0.1', port=19091, timeout=8.0):
        self.host = host
        self.port = port
        self.ws = WebSocket(host, port, timeout=timeout)
        self.timeout = timeout
        self._pending = {}
        self._counter = 0
        self.inbox = []          # messages nobody claimed, newest last
        self.subscriptions = {}
        self.reconnects = 0

    # -- plumbing ----------------------------------------------------------

    def reconnect(self):
        """Rebuild the socket after the SSH forward was reset, replaying subs.

        The tunnel to the robot is torn down by the far end every few minutes
        under load -- "远程主机强迫关闭了一个现有的连接" / WSAECONNRESET -- which
        kills rosbridge in the middle of a sequence.  A grasp run takes about
        three and a half minutes and died at almost exactly that point on three
        consecutive attempts, always just before the gripper closed.  Retrying
        the read on a fresh socket turns that from a hard failure into a hiccup.
        """
        try:
            self.ws.close()
        except Exception:                       # noqa: BLE001 - closing is best effort
            pass
        self.ws = WebSocket(self.host, self.port, timeout=self.timeout)
        for topic, message_type in list(self.subscriptions.items()):
            payload = {'op': 'subscribe', 'topic': topic}
            if message_type:
                payload['type'] = message_type
            # write straight to the socket: going through send() would call
            # reconnect() again on failure and loop forever
            self.ws.send_text(json.dumps(payload))
        self.reconnects += 1

    def _next_id(self):
        self._counter += 1
        return 'req-%d' % self._counter

    def _drain(self, match, timeout):
        """Read until ``match(message)`` is true, or time out."""
        import time
        deadline = time.monotonic() + timeout
        retried = False
        while time.monotonic() < deadline:
            try:
                text = self.ws.recv_text(timeout=max(0.05, deadline - time.monotonic()))
            except RosbridgeError as error:
                if 'timed out' in str(error):
                    return None
                if not retried:
                    retried = True
                    self.reconnect()
                    deadline = time.monotonic() + timeout
                    continue
                raise
            try:
                message = json.loads(text)
            except ValueError:
                continue
            if match(message):
                return message
            self.inbox.append(message)
            if len(self.inbox) > 500:
                del self.inbox[:250]
        return None

    def send(self, payload):
        try:
            self.ws.send_text(json.dumps(payload))
        except RosbridgeError:
            self.reconnect()
            self.ws.send_text(json.dumps(payload))

    # -- operations --------------------------------------------------------

    def topics(self, timeout=None):
        """Topic names and types, assembled from rosapi's two list services."""
        names = self.call_service('/rosapi/topics', [], timeout=timeout)
        types = self.call_service('/rosapi/topics_and_raw_types', [], timeout=timeout)
        result = {}
        if types and types.get('topics'):
            for name, type_name in zip(types['topics'], types.get('types', [])):
                result[name] = type_name
        elif names and names.get('topics'):
            result = {name: '?' for name in names['topics']}
        return result

    def call_service(self, service, args=(), timeout=None):
        request_id = self._next_id()
        self.send({'op': 'call_service', 'service': service,
                   'args': list(args), 'id': request_id})
        message = self._drain(lambda m: m.get('op') == 'service_response'
                              and m.get('id') == request_id,
                              timeout or self.timeout)
        if message is None:
            raise RosbridgeError('no response from %s within %.1fs'
                                 % (service, timeout or self.timeout))
        if not message.get('result', False):
            raise RosbridgeError('%s refused: %s'
                                 % (service, message.get('values')))
        return message.get('values') or {}

    def subscribe(self, topic, message_type=None, throttle_rate=0):
        payload = {'op': 'subscribe', 'topic': topic}
        if message_type:
            payload['type'] = message_type
        if throttle_rate:
            payload['throttle_rate'] = int(throttle_rate)
        self.send(payload)
        self.subscriptions[topic] = message_type

    def read_fresh(self, topic, msg_type, timeout=2.0, drain_s=0.4):
        """Subscribe, drain the stale backlog, then read the newest sample.

        This is the ONLY safe way to read state (odom, IMU, scan, arm status).
        Four separate failures on 2026-11-05/06 all came from skipping the
        drain: a rotating car read as stationary, a reversing car read as
        motionless, the arm's gyro integral reading zero, and a table corner
        vanishing from the scan.  wait_for_topic() consumes oldest-first from
        the socket buffer, so without a drain it hands back whatever was
        queued BEFORE the event you are asking about.
        """
        self.subscribe(topic, msg_type)
        self.drain(drain_s)
        return self.wait_for_topic(topic, timeout)


    def publish(self, topic, message, message_type=None):
        payload = {'op': 'publish', 'topic': topic, 'msg': message}
        if message_type:
            payload['type'] = message_type
        self.send(payload)

    def advertise(self, topic, message_type):
        self.send({'op': 'advertise', 'topic': topic, 'type': message_type})

    def unadvertise(self, topic):
        self.send({'op': 'unadvertise', 'topic': topic})

    def drain(self, timeout=0.3):
        """Read and discard everything already queued on the socket.

        Without this, ``newest=True`` is a lie: messages still sitting unread in
        the socket are not in ``self.inbox``, so a "latest state" read returns
        whatever was oldest. That made a moving robot look stationary twice.
        """
        import time
        deadline = time.monotonic() + timeout
        discarded = 0
        while time.monotonic() < deadline:
            try:
                text = self.ws.recv_text(timeout=max(0.01, deadline - time.monotonic()))
            except RosbridgeError as error:
                if 'timed out' in str(error):
                    break
                raise
            if text:
                discarded += 1
        self.inbox.clear()
        return discarded

    def wait_for_topic(self, topic, timeout, predicate=None, newest=False):
        """Block until a message on ``topic`` satisfies ``predicate``.

        ``newest=False`` consumes buffered messages oldest-first, which is what a
        stream reader wants. ``newest=True`` takes the freshest buffered message
        and throws the older ones away, which is what a *state* check wants: a
        pose read must not be answered from a buffer filled before the command
        under test was issued. Getting this wrong once made a moving robot look
        stationary, because a pre-command odometry sample satisfied a
        post-command query.
        """
        def match(message):
            return (message.get('op') == 'publish' and message.get('topic') == topic
                    and (predicate is None or predicate(message.get('msg') or {})))

        # Auto-subscribe if this topic was never subscribed. Callers used to
        # assume wait_for_topic subscribed on its own; without this the read
        # just times out on a connection that never asked for the topic
        # (2026-11-03: a healthy robot looked fully dead because of it).
        if topic not in self.subscriptions:
            self.subscribe(topic)

        candidates = [index for index, message in enumerate(self.inbox) if match(message)]
        if candidates:
            index = candidates[-1] if newest else candidates[0]
            found = self.inbox[index].get('msg')
            if newest:
                del self.inbox[:index + 1]
            else:
                del self.inbox[index]
            return found
        message = self._drain(match, timeout)
        return None if message is None else message.get('msg')

    def close(self):
        self.ws.close()


def set_bool_parameter(bridge, node, name, value, timeout=8.0):
    """Set one boolean parameter on a running node and report what it said."""
    values = bridge.call_service(
        node.rstrip('/') + '/set_parameters',
        [{'name': name, 'value': {'type': 1, 'bool_value': bool(value)}}],
        timeout=timeout)
    results = values.get('results') or []
    if not results:
        raise RosbridgeError('set_parameters returned nothing')
    first = results[0]
    return bool(first.get('successful')), str(first.get('reason', ''))
