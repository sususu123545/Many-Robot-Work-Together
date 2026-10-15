#!/usr/bin/env python3
"""Read-only servo bus probe. Runs INSIDE the armpi-ros2 container, stdin-piped.

Never writes to the bus. Only these READ commands are issued:
    0x05 position, 0x09 temperature, 0x0D torque enabled, 0x07 supply voltage.
No move (0x01), no torque load/unload (0x0B/0x0C), no stop (0x03).
Safe to run when nothing else owns /dev/rrc.

0x0D is the ONLY way to learn whether a servo is holding torque; the bus exposes
no torque/current/load magnitude, so "how much holding force" stays unreadable.
This probe reports state only and never changes it.

Usage (from the PC, see tools/probe_servo_state.py for the wrapper):

    sudo -n docker run --rm -i --device /dev/rrc:/dev/rrc \
        armpi-ros2-migration:stage-4xhjqf python3 - < servo_bus_probe.py

Emits one human line per joint plus a RESULT_JSON= line. Exit 0 always unless
the port cannot be opened at all.
"""
import json
import sys

from armpi_ros2.servo_protocol import ServoBoard, JOINTS

DEVICE = '/dev/rrc'
BAUD = 1000000
TRIES = 4


def read(board, fn, ident):
    """The bus tolerates 0x09 (temperature) poorly; retry before believing a failure."""
    last = None
    for _ in range(TRIES):
        try:
            return fn(ident)
        except (TimeoutError, OSError) as error:  # noqa: PERF203
            last = error
    raise last


def torque_state(board, ident):
    """0x0D -> True/False. Uses the generic read() because the deployed image's
    ServoBoard predates the torque_enabled() helper (only close/diagnostics/
    limits/move/position/read/stop/temperature exist there)."""
    state = board.read(ident, 0x0D, '<BBbb')[0]
    if state not in (0, 1):
        raise OSError('servo %d reports invalid torque state %r' % (ident, state))
    return bool(state)


def supply_mv(board, ident):
    """0x07 -> supply voltage in mV. A measurement, not proof of holding torque."""
    return board.read(ident, 0x07, '<BBbH')[0]


def main():
    board = ServoBoard(DEVICE, BAUD)
    rows = {}
    failures = {}
    try:
        for name, (ident, centre, sign, low, high) in JOINTS.items():
            try:
                raw = read(board, board.position, ident)
                temp = read(board, board.temperature, ident)
                ok = bool(low <= raw <= high)
                row = {'id': ident, 'raw': raw, 'temperature': temp,
                       'centre': centre, 'sign': sign,
                       'limits': [low, high], 'in_range': ok}
                # 0x0D / 0x07 are supplementary: a failure here must not hide the
                # position evidence, so they are reported as None rather than fatal.
                try:
                    row['torque_enabled'] = read(board, lambda i: torque_state(board, i), ident)
                except (TimeoutError, OSError, ValueError) as error:
                    row['torque_enabled'] = None
                    row['torque_error'] = str(error)
                try:
                    row['supply_mv'] = read(board, lambda i: supply_mv(board, i), ident)
                except (TimeoutError, OSError, ValueError) as error:
                    row['supply_mv'] = None
                    row['voltage_error'] = str(error)
                rows[name] = row
                torque = row['torque_enabled']
                torque_text = 'unknown' if torque is None else ('ENABLED' if torque else 'DISABLED')
                print('%-9s servo=%d raw=%-5s temp=%-4s limits=[%d,%d] centre=%d sign=%+d %s | torque=%s vin=%smV'
                      % (name, ident, raw, temp, low, high, centre, sign,
                         'ok' if ok else '*** OUT OF RANGE ***', torque_text,
                         row['supply_mv']), flush=True)
            except (TimeoutError, OSError) as error:
                failures[name] = str(error)
                print('%-9s servo=%d READ FAILED: %s' % (name, ident, error), flush=True)
    finally:
        try:
            board.close()
        except Exception:  # noqa: BLE001
            pass
    print('RESULT_JSON=' + json.dumps({'joints': rows, 'failures': failures},
                                      sort_keys=True))
    return 0


if __name__ == '__main__':
    sys.exit(main())
