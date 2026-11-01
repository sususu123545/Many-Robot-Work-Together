"""Isolated supported-arm voltage/position/temperature diagnosis, no motion.

Requires the host runner's exclusive UART and homing inhibit. Never use beside
the normal driver. This is a changed diagnostic workload, not a fix trial.
"""
import json
import os
from pathlib import Path
import time

from servo_protocol import ServoBoard
from arm_transport_readonly_ab import ReadOnlyPort

READS = (('position', 5, '<BBbh'), ('millivolts', 7, '<BBbH'),
         ('temperature', 9, '<BBbB'))


def summarize(rows):
    result = {}
    for ident in range(1, 7):
        samples = [r for r in rows if r['servo'] == ident]
        fields = {}
        for label, _, _ in READS:
            values = [r[label] for r in samples if label in r]
            fields[label] = ({'samples': len(values), 'min': min(values),
                              'max': max(values), 'latest': values[-1]} if values else {'samples': 0})
        result[ident] = fields
    return result


def main():
    import serial
    cycles = int(os.environ.get('AB_CYCLES', '60'))
    if not 20 <= cycles <= 120: raise ValueError('20..120 cycles required')
    port = serial.Serial(None, baudrate=1000000, timeout=.01, write_timeout=.1, exclusive=True)
    port.rts = False; port.dtr = False; port.port = '/dev/rrc'; port.open()
    board = ServoBoard(port=ReadOnlyPort(port))
    rows = []; errors = []; initial = {}; begun = time.monotonic(); fatal = None
    try:
        for cycle in range(cycles):
            cycle_start = time.monotonic()
            for ident in range(1, 7):
                row = {'servo': ident, 'cycle': cycle, 'times': {}}
                rows.append(row)
                for label, command, fmt in READS:
                    started = time.monotonic()
                    try:
                        value = board.read(ident, command, fmt, timeout=.2, attempts=3)[0]
                        row[label] = value
                        if label == 'position':
                            initial.setdefault(ident, value)
                            if abs(value-initial[ident]) > 20:
                                raise RuntimeError('Physical drift >20 ticks; stop diagnosis')
                        if label == 'temperature' and not 0 <= value < 55:
                            raise RuntimeError('Diagnostic temperature stop guard')
                    except (OSError, TimeoutError) as error:
                        errors.append({'servo': ident, 'cycle': cycle, 'command': command,
                                       'query': label, 'error': str(error), 'start': started,
                                       'end': time.monotonic(),
                                       'voltage_before_temperature_mV': row.get('millivolts') if label == 'temperature' else None,
                                       'diagnostics': board.diagnostics()})
                    finally:
                        row['times'][label] = {'start': started, 'end': time.monotonic()}
            wait = 1.0-(time.monotonic()-cycle_start)
            if wait > 0: time.sleep(wait)
            if (cycle+1) % 15 == 0:
                print('VOLTAGE_PROGRESS='+json.dumps({'cycles': cycle+1, 'errors': len(errors),
                      'seconds': round(time.monotonic()-begun, 1)}), flush=True)
    except BaseException as error:
        fatal = repr(error)
        raise
    finally:
        board.close()
        report = {'cycles_requested': cycles, 'seconds': round(time.monotonic()-begun, 2),
                  'read_order': [command for _, command, _ in READS], 'cycle_period_min_seconds': 1.0,
                  'timeout_seconds': .2, 'attempts': 3, 'servo_order': list(range(1, 7)),
                  'by_servo': summarize(rows), 'error_count': len(errors), 'errors': errors,
                  'rows': rows, 'counters': board.diagnostics()['counters'], 'fatal': fatal,
                  'read_only_uart_enforced': True,
                  'limitations': 'Changed diagnostic workload: extra voltage reads and 1s cycles. Reported voltage is sampled at the servo, not an oscilloscope trace. No-error windows do not prove a fix; model/ID mapping is not inferred from photos.'}
        Path('/results/report.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
        summary = {k: v for k, v in report.items() if k not in ('rows', 'errors')}
        summary['failed_queries'] = [{k: v for k, v in e.items() if k != 'diagnostics'} for e in errors]
        print('VOLTAGE_RESULT='+json.dumps(summary), flush=True)


if __name__ == '__main__': main()
