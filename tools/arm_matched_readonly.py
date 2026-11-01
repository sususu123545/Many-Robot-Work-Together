"""Supported-arm isolated matched-cycle query comparison. No motion/reset.

All six segments use the same 1s minimum cycle and 200ms x3 read budget.
ABC/CBA order reduces, but cannot eliminate, order/time confounding.
"""
from collections import Counter
import json
import os
from pathlib import Path
import statistics
import time

from servo_protocol import ServoBoard
from arm_transport_readonly_ab import ReadOnlyPort, check_read_packet

ORDER = ('A_pair', 'B_voltage', 'C_wait40', 'C_wait40', 'B_voltage', 'A_pair')
FORMATS = {5: '<BBbh', 7: '<BBbH', 9: '<BBbB'}


def commands(profile):
    if profile not in ORDER: raise ValueError('Unknown profile')
    return (5, 7, 9) if profile == 'B_voltage' else (5, 9)


def temperature_wait(profile, last_position_tx, now):
    return max(0.0, .04-(now-last_position_tx)) if profile == 'C_wait40' else 0.0


class TraceReadPort(ReadOnlyPort):
    def __init__(self, port):
        super().__init__(port)
        self.tx = []

    def write(self, data):
        data = bytes(data); check_read_packet(data)
        event = {'time': time.monotonic(), 'servo': data[5], 'command': data[4]}
        self.tx.append(event)
        count = self.port.write(data)
        event['returned_count'] = count
        return count


def summarize(queries):
    result = {}
    for command in sorted({q['command'] for q in queries}):
        rows = [q for q in queries if q['command'] == command]
        result[str(command)] = {
            'queries': len(rows), 'success': sum(q['ok'] for q in rows),
            'failed': sum(not q['ok'] for q in rows),
            'tx_attempts': sum(q['attempts'] for q in rows),
            'retried_queries': sum(q['attempts'] > 1 for q in rows),
            'failure_ids': dict(Counter(str(q['servo']) for q in rows if not q['ok']))}
    return result


def gap_summary(queries):
    values = [q['position_to_temperature_first_tx_ms'] for q in queries
              if q['command'] == 9 and q.get('position_to_temperature_first_tx_ms') is not None]
    return {'samples': len(values), 'min_ms': round(min(values), 3),
            'median_ms': round(statistics.median(values), 3), 'max_ms': round(max(values), 3)} if values else None


def main():
    import serial
    cycles = int(os.environ.get('AB_CYCLES', '60'))
    if not 20 <= cycles <= 90: raise ValueError('Bounded 20..90 cycles per segment required')
    port = serial.Serial(None, baudrate=1000000, timeout=.01, write_timeout=.1, exclusive=True)
    port.rts = False; port.dtr = False; port.port = '/dev/rrc'; port.open()
    trace = TraceReadPort(port); board = ServoBoard(port=trace)
    results = []; queries = []; initial = {}; fatal = None; begun = time.monotonic()
    report = {'order': ORDER, 'cycles_per_segment': cycles, 'cycle_min_seconds': 1.0,
              'timeout_seconds': .2, 'attempts': 3, 'servo_order': list(range(1, 7)),
              'read_only_uart_enforced': True, 'results': results, 'queries': queries,
              'limitations': 'Different profiles change bus load; C targets >=40ms after last position TX, not a hard wire-time guarantee. Sequential bounded windows do not identify firmware/electrical root cause or prove a fix.'}

    def save():
        report.update(seconds=round(time.monotonic()-begun, 2), fatal=fatal)
        Path('/results/report.json').write_text(json.dumps(report, indent=2), encoding='utf-8')

    try:
        for segment, profile in enumerate(ORDER):
            start = time.monotonic(); before = board.diagnostics()['counters']; index = len(queries)
            overruns = 0
            print('MATCHED_SEGMENT_START='+json.dumps({'segment': segment+1, 'profile': profile}), flush=True)
            for cycle in range(cycles):
                cycle_start = time.monotonic()
                for ident in range(1, 7):
                    position_tx = None
                    for command in commands(profile):
                        if command == 9 and position_tx is not None:
                            wait = temperature_wait(profile, position_tx, time.monotonic())
                            if wait > 0: time.sleep(wait)
                        tx_index = len(trace.tx)
                        q = {'segment': segment+1, 'profile': profile, 'cycle': cycle,
                             'servo': ident, 'command': command, 'start': time.monotonic(), 'ok': False}
                        queries.append(q)
                        try:
                            value = board.read(ident, command, FORMATS[command], timeout=.2, attempts=3)[0]
                            q.update(ok=True, value=value)
                            if command == 5:
                                initial.setdefault(ident, value)
                                if abs(value-initial[ident]) > 20: raise RuntimeError('Position drift stop guard')
                            elif command == 9 and not 0 <= value < 55:
                                raise RuntimeError('Temperature stop guard')
                        except (OSError, TimeoutError) as error:
                            q.update(error=str(error), error_type=type(error).__name__,
                                     diagnostics=board.diagnostics())
                        finally:
                            q['end'] = time.monotonic(); sent = trace.tx[tx_index:]
                            q['attempts'] = len(sent); q['transmissions'] = sent
                            if command == 5 and sent: position_tx = sent[-1]['time']
                            if command == 9 and position_tx is not None and sent:
                                q['position_to_temperature_first_tx_ms'] = (sent[0]['time']-position_tx)*1000
                elapsed = time.monotonic()-cycle_start
                if elapsed > 1.0: overruns += 1
                else: time.sleep(1.0-elapsed)
                if (cycle+1) % 15 == 0:
                    print('MATCHED_PROGRESS='+json.dumps({'segment': segment+1, 'profile': profile,
                          'cycles': cycle+1, 'failures': sum(not q['ok'] for q in queries[index:])}), flush=True)
            after = board.diagnostics()['counters']; rows = queries[index:]
            result = {'segment': segment+1, 'profile': profile, 'seconds': round(time.monotonic()-start, 2),
                      'cycles': cycles, 'cycles_over_1s': overruns, 'by_command': summarize(rows),
                      'temperature_gap': gap_summary(rows),
                      'counter_delta': {k: v-before.get(k, 0) for k, v in after.items() if v != before.get(k, 0)}}
            results.append(result); save()
            print('MATCHED_SEGMENT_RESULT='+json.dumps(result), flush=True)
        report['by_profile'] = {p: summarize([q for q in queries if q['profile'] == p]) for p in set(ORDER)}
    except BaseException as error:
        fatal = repr(error)
        raise
    finally:
        board.close(); save()
        print('MATCHED_REPORT_COMPLETE='+json.dumps({'fatal': fatal, 'segments_completed': len(results),
              'by_profile': report.get('by_profile')}), flush=True)


if __name__ == '__main__': main()
