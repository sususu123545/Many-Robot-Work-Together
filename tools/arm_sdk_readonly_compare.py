"""Isolated UART read comparison. No ROS nodes, movement, reset or torque writes.

The normal core MUST be stopped, homing inhibited and arm supported by the
host runner. Stock SDK sources are imported, never run as __main__.
"""
import importlib.util
import json
import os
from pathlib import Path
import queue
import struct
import time
from types import SimpleNamespace

from servo_protocol import ServoBoard
from arm_transport_readonly_ab import check_read_packet


class GuardedPort:
    def __init__(self, port):
        self.port = port
        self.tx = []
        self.last_tx = 0.0
        self.closed = False

    def __getattr__(self, name):
        return getattr(self.port, name)

    @property
    def rts(self): return self.port.rts
    @rts.setter
    def rts(self, value):
        if value: raise ValueError('RTS assertion refused')
        self.port.rts = False

    @property
    def dtr(self): return self.port.dtr
    @dtr.setter
    def dtr(self, value):
        if value: raise ValueError('DTR assertion refused')
        self.port.dtr = False

    def setPort(self, device):
        if device != '/dev/rrc': raise ValueError('Unexpected UART')
        self.port.port = device

    def open(self): return self.port.open()
    def read(self, count=1):
        if self.closed: return b''
        return self.port.read(count)

    def write(self, data):
        data = bytes(data)  # SDK passes a list; native driver passes bytes.
        check_read_packet(data)
        wait = .02 - (time.monotonic() - self.last_tx)
        if wait > 0: time.sleep(wait)
        self.last_tx = time.monotonic()
        self.tx.append((self.last_tx, bytes(data)))
        return self.port.write(data)

    def close(self):
        self.closed = True
        self.port.close()


class ObservedQueue:
    """Preserve SDK queue behavior, record only actually consumed replies."""
    def __init__(self, original):
        self.original = original
        self.consumed = []
        self.timeouts = 0
        self.arrivals = {}

    def __getattr__(self, name): return getattr(self.original, name)

    def put_nowait(self, value):
        self.arrivals[id(value)] = time.monotonic()
        try:
            return self.original.put_nowait(value)
        except queue.Full:
            self.arrivals.pop(id(value), None)
            raise

    def get(self, *args, **kwargs):
        try:
            value = self.original.get(*args, **kwargs)
        except queue.Empty:
            self.timeouts += 1
            raise
        self.consumed.append((self.arrivals.pop(id(value), -1.0), value))
        return value


def validate_sdk_reply(observed, start_index, ident, command, value, sent=None):
    if value is None:
        raise TimeoutError('SDK returned no value')
    replies = observed.consumed[start_index:]
    if len(replies) != 1:
        raise ValueError('SDK result has no unique consumed reply')
    if sent is not None and replies[0][0] < sent:
        raise ValueError('SDK consumed a pre-query stale reply')
    payload = replies[0][1]
    fmt = '<BBbh' if command == 5 else '<BBbB'
    if len(payload) != struct.calcsize(fmt):
        raise ValueError('SDK reply length mismatch')
    actual_id, actual_cmd, status, actual_value = struct.unpack(fmt, payload)
    if (actual_id, actual_cmd, status) != (ident, command, 0):
        raise ValueError('SDK consumed mismatched or error reply: '+payload.hex())
    if list(value) != [actual_value]:
        raise ValueError('SDK returned value differs from physical reply')
    return actual_value


def new_port():
    import serial
    port = serial.Serial(None, baudrate=1000000, timeout=.01,
                         write_timeout=.1, exclusive=True)
    port.rts = False
    port.dtr = False
    return GuardedPort(port)


def main():
    cycles = int(os.environ.get('AB_CYCLES', '120'))
    if not 20 <= cycles <= 180: raise ValueError('20..180 cycles required')
    profiles = ('native_100ms_3', 'sdk_100ms_3', 'sdk_stock_100ms_11')
    results = []
    first = {}
    for profile in profiles:
        port = new_port()
        board = None
        sdk = profile.startswith('sdk_')
        observed = None
        errors = []
        latest = {}
        drift = {}
        temps = []
        slow = 0
        max_ms = 0.0
        started = time.monotonic()
        try:
            if sdk:
                spec = importlib.util.spec_from_file_location('diagnostic_stock_sdk', '/opt/probe/stock_sdk.py')
                module = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(module)
                # Only redirect serial construction to an exclusive, read-only
                # port. SDK parser/read methods themselves remain unchanged.
                module.serial = SimpleNamespace(Serial=lambda *a, **kw: port)
                board = module.Board(device='/dev/rrc')
                board.retry_times = 10 if profile == 'sdk_stock_100ms_11' else 2
                observed = ObservedQueue(board.bus_servo_queue)
                board.bus_servo_queue = observed
                board.enable_reception(True)
                time.sleep(.1)
            else:
                port.setPort('/dev/rrc')
                port.open()
                board = ServoBoard(port=port)
            for cycle in range(cycles):
                cycle_start = time.monotonic()
                for ident in range(1, 7):
                    latest.setdefault(ident, {})
                    for command, label in ((5, 'position'), (9, 'temperature')):
                        begin = time.monotonic()
                        tx_before = len(port.tx)
                        index = len(observed.consumed) if sdk else 0
                        try:
                            if sdk:
                                method = board.bus_servo_read_position if command == 5 else board.bus_servo_read_temp
                                returned = method(ident)
                                value = validate_sdk_reply(observed, index, ident, command,
                                                           returned, sent=port.tx[tx_before][0])
                            else:
                                fmt = '<BBbh' if command == 5 else '<BBbB'
                                value = board.read(ident, command, fmt, timeout=.1, attempts=3)[0]
                            latest[ident][label] = value
                            if command == 9:
                                temps.append(value)
                                if not 0 <= value < 55: raise RuntimeError('Temperature stop guard')
                            else:
                                first.setdefault(ident, value)
                                delta = abs(value-first[ident])
                                drift[ident] = max(drift.get(ident, 0), delta)
                                if delta > 20: raise RuntimeError('Position drift stop guard')
                        except (OSError, TimeoutError, ValueError, struct.error) as error:
                            errors.append({'cycle': cycle, 'servo': ident, 'command': command,
                                           'error': str(error), 'attempts': len(port.tx)-tx_before,
                                           'seconds_since_start': round(time.monotonic()-started, 3)})
                        elapsed = (time.monotonic()-begin)*1000
                        max_ms = max(max_ms, elapsed)
                        if elapsed >= 100: slow += 1
                wait = .5-(time.monotonic()-cycle_start)
                if wait > 0: time.sleep(wait)
                if (cycle+1) % 30 == 0:
                    print('SDK_COMPARE_PROGRESS='+json.dumps({'profile': profile, 'cycles': cycle+1,
                          'errors': len(errors)}), flush=True)
            row = {'profile': profile, 'cycles': cycles, 'logical_reads': cycles*12,
                   'seconds': round(time.monotonic()-started, 2), 'error_count': len(errors),
                   'errors': errors, 'tx_attempts': len(port.tx), 'queries_over_100ms': slow,
                   'max_query_ms': round(max_ms, 2), 'latest': latest, 'max_drift': drift,
                   'temperature_range': [min(temps), max(temps)] if temps else None,
                   'attempt_timeouts': observed.timeouts if sdk else board.diagnostics()['counters'].get('attempt_timeout', 0)}
            results.append(row)
            print('SDK_COMPARE_RESULT='+json.dumps(row), flush=True)
        finally:
            if board is not None:
                if sdk:
                    board.enable_reception(False)
                    time.sleep(.05)
                    port.close()
                else: board.close()
            else: port.close()
    report = {'results': results, 'read_only_uart_enforced': True,
              'note': 'Native matched profile uses 100ms, unlike deployed 200ms. SDK stock allows 11 attempts. All profiles enforce >=20ms TX gap; sequential windows are not proof of hardware health.'}
    Path('/results/report.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print('AB_REPORT_COMPLETE', flush=True)


if __name__ == '__main__': main()
