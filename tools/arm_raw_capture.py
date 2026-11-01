"""Isolated read-only host UART capture; requires supported arm/core stopped.

Capture is at Python serial read/write boundaries, NOT an electrical bus trace.
Never reset, authorize, move, stop or change torque/limits.
"""
import json
import os
from pathlib import Path
import time

from servo_protocol import ServoBoard
from arm_transport_readonly_ab import ReadOnlyPort, check_read_packet


class CapturePort(ReadOnlyPort):
    def __init__(self, port):
        super().__init__(port)
        self.events = []

    def read(self, count):
        data = self.port.read(count)
        if data:
            self.events.append({'kind': 'rx', 'time': time.monotonic(), 'hex': bytes(data).hex()})
        return data

    def write(self, data):
        data = bytes(data)
        check_read_packet(data)
        event = {'kind': 'tx', 'time': time.monotonic(), 'hex': data.hex()}
        self.events.append(event)
        count = self.port.write(data)
        event.update(returned_count=count, completed=time.monotonic())
        return count


def main():
    import serial
    cycles = int(os.environ.get('AB_CYCLES', '180'))
    if not 20 <= cycles <= 240: raise ValueError('20..240 cycles required')
    port = serial.Serial(None, baudrate=1000000, timeout=.01, write_timeout=.1, exclusive=True)
    port.rts = False; port.dtr = False; port.port = '/dev/rrc'; port.open()
    capture = CapturePort(port)
    board = ServoBoard(port=capture)
    queries = []; initial = {}; drift = {}; temperatures = []; errors = []
    begun = time.monotonic(); fatal = None
    try:
        for cycle in range(cycles):
            cycle_start = time.monotonic()
            for ident in range(1, 7):
                for command, fmt in ((5, '<BBbh'), (9, '<BBbB')):
                    query = {'servo': ident, 'command': command, 'cycle': cycle,
                             'start': time.monotonic(), 'ok': False}
                    queries.append(query)
                    try:
                        value = board.read(ident, command, fmt, timeout=.2, attempts=3)[0]
                        query.update(ok=True, value=value)
                        if command == 9:
                            temperatures.append(value)
                            if not 0 <= value < 55: raise RuntimeError('Temperature stop guard')
                        else:
                            initial.setdefault(ident, value)
                            delta = abs(value-initial[ident]); drift[ident] = max(drift.get(ident, 0), delta)
                            if delta > 20: raise RuntimeError('Position drift stop guard')
                    except (OSError, TimeoutError) as error:
                        query['error'] = str(error)
                        errors.append(query)
                    finally:
                        query['end'] = time.monotonic()
                        if not query['ok']: query['diagnostics'] = board.diagnostics()
            wait = .5-(time.monotonic()-cycle_start)
            if wait > 0: time.sleep(wait)
            if (cycle+1) % 30 == 0:
                print('RAW_CAPTURE_PROGRESS='+json.dumps({'cycles': cycle+1, 'errors': len(errors),
                      'seconds': round(time.monotonic()-begun, 1)}), flush=True)
        # Read only, no new TX: retain a short tail to observe late replies.
        time.sleep(.7)
    except BaseException as error:
        fatal = repr(error)
        raise
    finally:
        board.close()
        report = {'cycles_requested': cycles, 'logical_reads': len(queries), 'error_count': len(errors),
                  'failed_queries': [{k: v for k, v in q.items() if k != 'diagnostics'} for q in errors],
                  'seconds': round(time.monotonic()-begun, 2), 'max_drift': drift,
                  'temperature_range': [min(temperatures), max(temperatures)] if temperatures else None,
                  'counters': board.diagnostics()['counters'], 'fatal': fatal,
                  'read_only_uart_enforced': True, 'capture_kind': 'host serial API boundaries, not wire timestamps',
                  'timeout': .2, 'attempts': 3, 'servo_order': list(range(1, 7))}
        Path('/results/capture.json').write_text(json.dumps({'metadata': report, 'events': capture.events,
                                                           'queries': queries}), encoding='utf-8')
        Path('/results/report.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
        print('RAW_CAPTURE_RESULT='+json.dumps(report), flush=True)


if __name__ == '__main__': main()
