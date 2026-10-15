"""Arm-only RRC protocol, derived from the checked existing board SDK.

Never sends motor, torque-enable, offset, limit-write, or startup-position packets.
All feedback is physical; no fake position cache.
"""
import math
import copy
from collections import Counter, deque
import queue
import struct
import threading
import time

TICK_RAD = math.radians(240.0) / 1000
JOINTS = {'joint1': (6, 500, 1, 0, 1000),
          'joint2': (5, 500, 1, 125, 875),
          'joint3': (4, 500, -1, 0, 1000),
          'joint4': (3, 500, 1, 0, 1000),
          'joint5': (2, 500, 1, 0, 1000),
          'r_joint': (1, 700, 1, 0, 700)}

# RRC host-side command codes, NOT the downstream servo-bus command codes.
# Formats match ros_robot_controller_sdk.py's physical read methods. Voltage
# and torque reads are diagnostic-only APIs; the regular poll does not use them.
READ_FORMATS = {0x05: '<BBbh', 0x07: '<BBbH', 0x09: '<BBbB',
                0x0D: '<BBbb', 0x32: '<BBbHH'}


def tick_radians(name):
    # Existing manager explicitly initializes the gripper with 180 degrees,
    # whereas the other five controllers use 240 degrees per 1000 ticks.
    return math.radians(180.0) / 1000 if name == 'r_joint' else TICK_RAD


def crc8(data):
    crc = 0
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = (crc >> 1) ^ (0x8C if crc & 1 else 0)
    return crc


def packet(payload):
    body = bytes((5, len(payload))) + bytes(payload)
    return b'\xaa\x55' + body + bytes((crc8(body),))


def to_raw(name, radians):
    if name not in JOINTS or not math.isfinite(radians):
        raise ValueError('Unknown joint or nonfinite angle')
    ident, centre, sign, low, high = JOINTS[name]
    raw = round(centre + sign * radians / tick_radians(name))
    if not low <= raw <= high:
        raise ValueError('Target exceeds measured hardware limits: ' + name)
    return ident, raw


def to_radians(name, raw):
    _, centre, sign, _, _ = JOINTS[name]
    return sign * (raw - centre) * tick_radians(name)


def movement_payload(duration, targets):
    if not math.isfinite(duration) or not .3 <= duration <= 5:
        raise ValueError('Duration must be 0.3..5 seconds')
    if not targets or len(targets) > 6 or len({i for i, _ in targets}) != len(targets):
        raise ValueError('Empty/duplicate/oversized target set')
    result = struct.pack('<BHB', 1, round(duration * 1000), len(targets))
    for ident, raw in targets:
        limits = {v[0]: v[3:] for v in JOINTS.values()}
        if ident not in limits or not limits[ident][0] <= raw <= limits[ident][1]:
            raise ValueError('Invalid servo ID or raw hardware position')
        result += struct.pack('<BH', ident, raw)
    return result


def validated_targets(names, positions, samples, now, max_step=100, max_temperature=65):
    if not names or len(set(names)) != len(names) or len(positions) != len(names):
        raise ValueError('Invalid joint/position dimensions')
    targets = []
    for name, radians in zip(names, positions):
        ident, raw = to_raw(name, radians)
        sample = samples.get(name)
        if not sample or not 0 <= now - sample['time'] <= 1.5:
            raise ValueError('No fresh physical feedback: ' + name)
        if not 0 <= sample['temperature'] < max_temperature:
            raise ValueError('Servo temperature guard: ' + name)
        if abs(raw - sample['raw']) > max_step:
            raise ValueError('Step too large; use incremental targets: ' + name)
        targets.append((ident, raw))
    return targets


class ServoBoard:
    def __init__(self, device='/dev/rrc', baud=1000000, port=None):
        if port is None:
            import serial
            # Exclusive Linux fd; do not allow a second serial owner.
            port = serial.Serial(None, baudrate=baud, timeout=.01, write_timeout=.1, exclusive=True)
            port.rts = False
            port.dtr = False
            port.port = device
            port.open()
        self.port = port
        self.lock = threading.RLock()
        self.read_lock = threading.Lock()
        self.last_read_tx = 0.0
        self.buffer = bytearray()
        self.replies = queue.Queue(maxsize=64)
        self.receive_stop = threading.Event()
        self.receive_error = None
        self.diagnostic_lock = threading.Lock()
        self.counters = Counter()
        self.events = deque(maxlen=32)
        self.query_sequence = 0
        self.last_query = None
        self.last_failed_query = None
        self.last_failure_events = []
        self.reader = threading.Thread(target=self._receive, daemon=True)
        self.reader.start()

    def _write(self, payload):
        data = packet(payload)
        if self.port.write(data) != len(data):
            self._record('tx_error', payload_hex=data.hex())
            raise OSError('Incomplete serial write')
        self._record('tx', frame_hex=data.hex())

    def _record(self, event, **details):
        with self.diagnostic_lock:
            self.counters[event] += 1
            self.events.append({'event': event, 'monotonic_time': time.monotonic(), **details})

    def diagnostics(self):
        with self.diagnostic_lock:
            return {'counters': dict(self.counters), 'last_query': copy.deepcopy(self.last_query),
                    'last_failed_query': copy.deepcopy(self.last_failed_query),
                    'last_failure_events': copy.deepcopy(self.last_failure_events),
                    'recent_events': copy.deepcopy(list(self.events)),
                    'receiver_error': str(self.receive_error) if self.receive_error else None}

    def _query_result(self, **details):
        with self.diagnostic_lock:
            self.last_query.update(details)
            if details.get('result') not in (None, 'OK', 'WAITING'):
                self.last_failed_query = copy.deepcopy(self.last_query)
                self.last_failure_events = copy.deepcopy(list(self.events))

    def _frames(self):
        while len(self.buffer) >= 5:
            start = self.buffer.find(b'\xaa\x55')
            if start < 0:
                self._record('resync', discarded_bytes=max(0, len(self.buffer)-1))
                self.buffer[:] = self.buffer[-1:]
                return
            del self.buffer[:start]
            if len(self.buffer) < 5:
                return
            size = self.buffer[3] + 5
            if len(self.buffer) < size:
                return
            data = bytes(self.buffer[:size])
            if crc8(data[2:-1]) != data[-1]:
                self._record('crc_error', frame_hex=data.hex())
                del self.buffer[0]
                continue
            del self.buffer[:size]
            yield data[2], data[4:-1]

    def _receive(self):
        # The RRC also streams unsolicited telemetry. Always drain UART and
        # preserve partial frames, rather than flushing in the middle of packets.
        try:
            while not self.receive_stop.is_set():
                data = self.port.read(max(1, min(self.port.in_waiting, 256)))
                if not data:
                    self.receive_stop.wait(.001)
                    continue
                with self.diagnostic_lock:
                    self.counters['rx_bytes'] += len(data)
                self.buffer.extend(data)
                for function, payload in self._frames():
                    with self.diagnostic_lock:
                        self.counters['function_' + str(function) + '_frames'] += 1
                    if function != 5:
                        continue
                    self._record('rx_servo', payload_hex=payload.hex())
                    record = (time.monotonic(), payload)
                    try:
                        self.replies.put_nowait(record)
                    except queue.Full:
                        self._record('reply_queue_overflow')
                        try:
                            self.replies.get_nowait()
                        except queue.Empty:
                            pass
                        try:
                            self.replies.put_nowait(record)
                        except queue.Full:
                            pass
        except (OSError, ValueError) as error:
            if not self.receive_stop.is_set():
                self.receive_error = error
                self._record('receiver_error', message=str(error))

    def read(self, ident, command, fmt, timeout=.2, attempts=3):
        # Only read-only requests may be retried. Never replay movement/stop.
        if (READ_FORMATS.get(command) != fmt or ident not in {v[0] for v in JOINTS.values()}
                or not 1 <= attempts <= 3):
            raise ValueError('Only captured IDs, exact read formats and 1..3 attempts allowed')
        # Only serialize READ transactions. A stop must not wait through an
        # entire missing-response timeout; serial writes use the brief IO lock.
        with self.read_lock:
            started = time.monotonic()
            with self.diagnostic_lock:
                self.query_sequence += 1
                query_id = self.query_sequence
                self.counters['requests'] += 1
                self.last_query = {'query_id': query_id, 'servo_id': ident, 'command': command,
                                   'started': started, 'result': 'WAITING'}
            for attempt in range(1, attempts+1):
                gap = .02 - (time.monotonic() - self.last_read_tx)
                if gap > 0:
                    self.receive_stop.wait(gap)
                if self.receive_stop.is_set():
                    self._query_result(result='CLOSED')
                    raise OSError('Serial reader is closed')
                # Clear only complete old replies, NOT UART/partial RX frames.
                while True:
                    try:
                        self.replies.get_nowait()
                        self._record('old_reply_discarded', query_id=query_id)
                    except queue.Empty:
                        break
                with self.lock:
                    sent = self.last_read_tx = time.monotonic()
                    try:
                        self._write(bytes((command, ident)))
                    except OSError as error:
                        self._query_result(result='TX_ERROR', error=str(error))
                        raise
                self._record('read_attempt', query_id=query_id, servo_id=ident,
                             command=command, attempt=attempt)
                deadline = sent + timeout
                while time.monotonic() < deadline:
                    if self.receive_error:
                        self._query_result(result='RECEIVER_ERROR', error=str(self.receive_error))
                        raise OSError('Serial receiver failed: ' + str(self.receive_error))
                    try:
                        received, payload = self.replies.get(timeout=min(.02, max(.001, deadline-time.monotonic())))
                    except queue.Empty:
                        continue
                    if received < sent:
                        self._record('late_reply_discarded', query_id=query_id, payload_hex=payload.hex())
                        continue
                    if len(payload) != struct.calcsize(fmt):
                        self._record('length_mismatch', query_id=query_id, payload_hex=payload.hex())
                        continue
                    servo, cmd, error, *values = struct.unpack(fmt, payload)
                    if servo == ident and cmd == command:
                        if error:
                            self._record('servo_read_error', query_id=query_id, error=error)
                            self._query_result(result='SERVO_ERROR', error=error)
                            raise OSError(f'Servo {ident} read error {error}')
                        self._record('read_success', query_id=query_id, attempt=attempt,
                                     reply_ms=round((received-sent)*1000, 3))
                        self._query_result(result='OK', attempts=attempt,
                                           elapsed_ms=round((time.monotonic()-started)*1000, 3))
                        return values
                    self._record('id_command_mismatch', query_id=query_id, payload_hex=payload.hex())
                self._record('attempt_timeout', query_id=query_id, attempt=attempt)
            self._query_result(result='TIMEOUT', attempts=attempts,
                               elapsed_ms=round((time.monotonic()-started)*1000, 3))
            self._record('read_timeout', query_id=query_id, servo_id=ident, command=command)
            raise TimeoutError(f'No physical reply from servo {ident}, command {command} after {attempts} reads')

    def position(self, ident):
        return self.read(ident, 0x05, '<BBbh')[0]

    def temperature(self, ident):
        return self.read(ident, 0x09, '<BBbB')[0]

    def voltage_mv(self, ident):
        # A measurement, not proof that the supply/holding torque is healthy.
        return self.read(ident, 0x07, '<BBbH')[0]

    def torque_enabled(self, ident):
        # Read only. NEVER turn torque on/off to resolve an unknown arm state.
        state = self.read(ident, 0x0D, '<BBbb')[0]
        if state not in (0, 1):
            raise OSError(f'Invalid physical torque feedback: servo {ident}: {state}')
        return bool(state)

    def limits(self, ident):
        return self.read(ident, 0x32, '<BBbHH')

    def move(self, duration, targets):
        data = movement_payload(duration, targets)
        with self.lock:
            self._write(data)

    def stop(self):
        # SDK stop request only: do NOT unload or enable torque. A successful
        # UART write does not prove the servo stopped or can hold its weight.
        with self.lock:
            self._write(bytes((3, 6, 1, 2, 3, 4, 5, 6)))

    def close(self):
        self.receive_stop.set()
        self.reader.join(timeout=.2)
        self.port.close()
