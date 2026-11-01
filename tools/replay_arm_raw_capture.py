"""Offline host RX analysis and real native/SDK parser replay. No serial open."""
import argparse
from collections import Counter
import contextlib
import importlib.util
import io
import json
from pathlib import Path
import struct
import sys
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'ros2_ws/src/armpi_ros2'))
from armpi_ros2.servo_protocol import ServoBoard, crc8


def scan_valid_frames(events):
    """Independent scanner checks every AA55 offset, even inside bad frames."""
    data = bytearray(); times = []
    for event in events:
        if event['kind'] == 'rx':
            chunk = bytes.fromhex(event['hex']); data.extend(chunk)
            times.extend([event['time']]*len(chunk))
    frames = []
    for offset in range(len(data)-4):
        if data[offset:offset+2] != b'\xaa\x55': continue
        end = offset+data[offset+3]+5
        if end > len(data): continue
        frame = bytes(data[offset:end])
        if frame[2] >= 12 or crc8(frame[2:-1]) != frame[-1]: continue
        frames.append({'offset': offset, 'time': times[end-1], 'function': frame[2],
                       'payload': frame[4:-1].hex()})
    return frames


def replay_native(events):
    board = ServoBoard.__new__(ServoBoard)
    board.buffer = bytearray(); counters = Counter(); frames = []
    board._record = lambda name, **kw: counters.update([name])
    for event in events:
        if event['kind'] != 'rx': continue
        board.buffer.extend(bytes.fromhex(event['hex']))
        for function, payload in board._frames():
            frames.append({'time': event['time'], 'function': function, 'payload': payload.hex()})
    return frames, dict(counters)


class ReplayEnd(Exception): pass


def replay_sdk(events):
    # SDK imports serial, but replay never constructs Board or calls Serial.
    spec = importlib.util.spec_from_file_location('offline_stock_sdk', ROOT/'armpi_pro/src/ros_robot_controller/src/ros_robot_controller/ros_robot_controller_sdk.py')
    sdk = importlib.util.module_from_spec(spec)
    absent = 'serial' not in sys.modules
    if absent: sys.modules['serial'] = SimpleNamespace()
    try: spec.loader.exec_module(sdk)
    finally:
        if absent: del sys.modules['serial']
    chunks = iter(e for e in events if e['kind'] == 'rx'); frames = []; clock = [0.0]

    class ReplayPort:
        def read(self):
            try: event = next(chunks)
            except StopIteration: raise ReplayEnd()
            clock[0] = event['time']
            return bytes.fromhex(event['hex'])

    board = sdk.Board.__new__(sdk.Board)
    board.enable_recv = True; board.frame = []; board.recv_count = 0
    board.state = sdk.PacketControllerState.PACKET_CONTROLLER_STATE_STARTBYTE1
    board.port = ReplayPort()
    board.parsers = {sdk.PacketFunction.PACKET_FUNC_BUS_SERVO:
                     lambda payload: frames.append({'time': clock[0], 'function': 5, 'payload': payload.hex()})}
    printed = io.StringIO()
    with contextlib.redirect_stdout(printed):
        try: board.recv_task()
        except ReplayEnd: pass
    return frames, {'crc_error_messages': printed.getvalue().count('校验失败')}


def matching(frame, query):
    if frame['function'] != 5: return False
    payload = bytes.fromhex(frame['payload'])
    fmt = '<BBbh' if query['command'] == 5 else '<BBbB'
    if len(payload) != struct.calcsize(fmt): return False
    ident, command, status, _ = struct.unpack(fmt, payload)
    return (ident, command, status) == (query['servo'], query['command'], 0)


def analyze(capture):
    events = capture['events']
    scanned = scan_valid_frames(events); native, native_errors = replay_native(events)
    sdk, sdk_errors = replay_sdk(events)
    signature = lambda frames: Counter((f['function'], f['payload']) for f in frames if f['function'] == 5)
    sequence = lambda frames: [f['payload'] for f in frames if f['function'] == 5]
    independent = signature(scanned)
    failed = []
    for query in capture['queries']:
        if query['ok']: continue
        transmissions = [e for e in events if e['kind'] == 'tx'
                         and query['start'] <= e['time'] <= query['end']]
        start = transmissions[0]['time'] if transmissions else query['start']
        during = [f for f in scanned if start <= f['time'] <= query['end'] and matching(f, query)]
        late = [f for f in scanned if query['end'] < f['time'] <= query['end']+.5 and matching(f, query)]
        same_tx = [e['time'] for e in events if e['kind'] == 'tx' and e['time'] > query['end']
                   and len(bytes.fromhex(e['hex'])) == 7
                   and bytes.fromhex(e['hex'])[4:6] == bytes([query['command'], query['servo']])]
        next_same_tx = min(same_tx) if same_tx else None
        before_next = [f for f in late if next_same_tx is None or f['time'] < next_same_tx]
        failed.append({'servo': query['servo'], 'command': query['command'], 'cycle': query['cycle'],
                       'start': start, 'end': query['end'], 'tx_attempts': len(transmissions),
                       'matching_valid_rx_in_query': len(during), 'matching_valid_rx_next_500ms': len(late),
                       'matching_valid_rx_next_500ms_before_same_requery': len(before_next),
                       'next_same_requery_after_end_ms': round((next_same_tx-query['end'])*1000, 3) if next_same_tx else None,
                       'native_matching_in_query': sum(start <= f['time'] <= query['end'] and matching(f, query) for f in native),
                       'sdk_matching_in_query': sum(start <= f['time'] <= query['end'] and matching(f, query) for f in sdk),
                       'rx_bytes_in_query': sum(len(bytes.fromhex(e['hex'])) for e in events
                                                if e['kind'] == 'rx' and start <= e['time'] <= query['end'])})
    return {'metadata': capture['metadata'], 'rx_chunks': sum(e['kind'] == 'rx' for e in events),
            'rx_bytes': sum(len(bytes.fromhex(e['hex'])) for e in events if e['kind'] == 'rx'),
            'independent_servo_frames': sum(independent.values()),
            'native_servo_frames': sum(signature(native).values()), 'sdk_servo_frames': len(sdk),
            'native_equals_independent_servo_multiset': signature(native) == independent,
            'sdk_equals_independent_servo_multiset': signature(sdk) == independent,
            'native_equals_independent_servo_sequence': sequence(native) == sequence(scanned),
            'sdk_equals_independent_servo_sequence': sequence(sdk) == sequence(scanned),
            'native_parser_diagnostics': native_errors, 'sdk_parser_diagnostics': sdk_errors,
            'failed_queries': failed,
            'limitations': 'Host API timestamps, not wire timing. No transaction sequence in protocol; later matching replies cannot be assigned uniquely. No matching host RX does not distinguish controller firmware, downstream bus, electrical faults, UART driver loss or RX scheduling.'}


def main():
    p = argparse.ArgumentParser(description=__doc__); p.add_argument('capture')
    a = p.parse_args()
    print(json.dumps(analyze(json.loads(Path(a.capture).read_text(encoding='utf-8'))), indent=2))


if __name__ == '__main__': main()
