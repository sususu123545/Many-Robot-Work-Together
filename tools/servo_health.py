#!/usr/bin/env python3
"""Servo health: turn /arm_controller/status into something a caller can act on.

Why this exists
---------------
On 2026-11-06 the wrist jammed against the lidar, servo 5 reached 65 C, the
temperature protection tripped, and the arm refused to move.  Every one of those
facts was sitting in /arm_controller/status the whole time -- ``fault_count`` had
climbed to 7 and ``last_fault`` named the servo -- and nothing in the pipeline
ever looked.  The first sign of trouble was a jog quietly reporting "moved 23 of
30" and a move that did not happen.  A human noticed the arm touching the lidar
before the software noticed anything.

Two sources, because one is not enough:

* ``transport.recent_events`` carries raw bus replies, and a 4-byte reply with
  command 0x09 is a temperature reading: ``[id, 0x09, 0x00, celsius]``.  But the
  controller only polls servos 1-4 -- measured 2026-11-06, eight samples over
  four seconds never contained servo 5.  The servo that overheated is exactly
  the one whose temperature cannot be read this way.
* ``last_fault`` / ``fault_count`` are the controller's own post-hoc record.
  They cover all six servos, they are always present, and ``sequence`` lets a
  caller distinguish "a fault that was already there" from "a fault that just
  happened on my watch".

Run ``python3 tools/servo_health.py`` for the offline self-test (no ROS needed).
"""

WARN_TEMP_C = 55
CRITICAL_TEMP_C = 62

# Command byte that asks a servo for its temperature.
TEMP_COMMAND = 0x09

SERVOS = (1, 2, 3, 4, 5)


def decode_temperatures(status):
    """{servo_id: celsius} from the raw bus replies in ``transport.recent_events``.

    Only servos 1-4 ever appear -- see the module docstring.  A missing servo is
    absent from the dict, never defaulted to a comfortable value: a fake 0 C is
    exactly the kind of silence that let this run to 65 C.
    """
    found = {}
    transport = (status or {}).get('transport') or {}
    for event in transport.get('recent_events') or []:
        payload = event.get('payload_hex')
        if not payload or len(payload) < 8:
            continue
        try:
            raw = bytes.fromhex(payload)
        except ValueError:
            continue
        if len(raw) == 4 and raw[1] == TEMP_COMMAND:
            found[raw[0]] = raw[3]
    return found


def read_health(status):
    """Summarise a status dict into something a caller can branch on."""
    status = status or {}
    temps = decode_temperatures(status)
    fault = status.get('last_fault') or None
    warnings = []

    hot = {sid: value for sid, value in temps.items() if value >= WARN_TEMP_C}
    for sid, value in sorted(hot.items()):
        warnings.append('servo %d at %d C' % (sid, value))
    unreadable = [sid for sid in SERVOS if sid not in temps]
    if unreadable:
        warnings.append('temperature unreadable for servo(s) %s'
                        % ', '.join(str(s) for s in unreadable))
    if fault:
        warnings.append('%s: %s' % (fault.get('kind', 'fault'),
                                    fault.get('message', '')))
    if status.get('estop'):
        warnings.append('estop is latched (%s)' % status.get('health_state'))

    critical = bool(fault) or any(value >= CRITICAL_TEMP_C for value in temps.values())
    return {'temps': temps,
            'unreadable': unreadable,
            'fault_count': status.get('fault_count'),
            'last_fault': fault,
            'warnings': warnings,
            'critical': critical}


def describe(health):
    """One-line-per-item text, for logs and for arm_boot_check."""
    lines = []
    if health['temps']:
        lines.append('servo temperatures: ' + '  '.join(
            '%d=%dC' % (sid, health['temps'][sid]) for sid in sorted(health['temps'])))
    missing = [s for s in SERVOS if s not in health['temps']]
    if missing:
        lines.append('servo %s: temperature not polled by the controller'
                     % ', '.join(str(s) for s in missing))
    if health['last_fault']:
        lines.append('last fault [%s] %s'
                     % (health['last_fault'].get('kind'),
                        health['last_fault'].get('message')))
    else:
        lines.append('no fault recorded (fault_count=%s)' % health['fault_count'])
    return lines


class Monitor:
    """Watches for faults that appear AFTER the caller started.

    A fault that was already recorded before this run must not abort it forever
    -- after a power cycle it is cleared, but after a mere estop reset it is not.
    Comparing ``sequence`` draws that line without needing history.
    """

    def __init__(self, status=None):
        self.baseline_sequence = None
        self.baseline_count = None
        if status is not None:
            self.adopt(status)

    def adopt(self, status):
        fault = (status or {}).get('last_fault') or {}
        self.baseline_sequence = fault.get('sequence')
        self.baseline_count = (status or {}).get('fault_count')

    def new_fault(self, status):
        """The newest fault if it is newer than the baseline, else None."""
        fault = (status or {}).get('last_fault') or None
        if not fault:
            return None
        sequence = fault.get('sequence')
        if self.baseline_sequence is None:
            return fault                     # adopt() was never called
        try:
            return fault if int(sequence) > int(self.baseline_sequence) else None
        except (TypeError, ValueError):
            return fault

    def check(self, status):
        """Return a list of problems worth stopping for.  Empty means carry on."""
        problems = []
        health = read_health(status)
        for sid, value in sorted(health['temps'].items()):
            if value >= CRITICAL_TEMP_C:
                problems.append('servo %d is %d C (>= %d C)' % (sid, value, CRITICAL_TEMP_C))
        fault = self.new_fault(status)
        if fault:
            problems.append('new fault [%s] %s' % (fault.get('kind'),
                                                    fault.get('message')))
        return problems


def _self_test():
    failures = []
    total = [0]

    def check(name, condition, detail=''):
        total[0] += 1
        if not condition:
            failures.append('%s %s' % (name, detail))
        print('%-56s %s %s' % (name, 'PASS' if condition else 'FAIL', detail))

    def status(temps=None, fault=None, count=0, estop=False):
        events = [{'payload_hex': '%02x0900%02x' % (sid, value)}
                  for sid, value in (temps or {}).items()]
        return {'transport': {'recent_events': events},
                'last_fault': fault, 'fault_count': count, 'estop': estop,
                'health_state': 'READY' if not estop else 'RECOVERED_AWAIT_RESET'}

    check('decodes a temperature reply',
          decode_temperatures(status({1: 31, 4: 44})) == {1: 31, 4: 44})
    check('ignores position replies',
          decode_temperatures({'transport': {'recent_events': [
              {'payload_hex': '010500b2'}]}}) == {})
    check('survives a malformed payload',
          decode_temperatures({'transport': {'recent_events': [
              {'payload_hex': 'zzzz'}, {'payload_hex': None}]}}) == {})

    health = read_health(status({1: 31, 2: 63}))
    check('a hot servo is critical', health['critical'])
    check('a hot servo is named', any('servo 2' in w for w in health['warnings']),
          str(health['warnings']))
    check('an unread servo is reported, not defaulted',
          5 in health['unreadable'] and 5 not in health['temps'])

    quiet = read_health(status({1: 31, 2: 32, 3: 30, 4: 33}))
    check('a cool arm is not critical', not quiet['critical'])

    recorded = status({1: 30}, fault={'kind': 'overtemperature',
                                      'message': 'Servo 5 is 65 C', 'sequence': 7},
                      count=7)
    check('a recorded fault is critical', read_health(recorded)['critical'])

    monitor = Monitor(status({1: 30}, count=0))
    check('no problem when nothing changed', monitor.check(status({1: 30}, count=0)) == [])
    appeared = status({1: 30}, fault={'kind': 'overtemperature',
                                      'message': 'Servo 5 is 65 C', 'sequence': 1}, count=1)
    problems = monitor.check(appeared)
    check('a NEW fault is reported', len(problems) == 1, str(problems))
    check('the new fault names the servo', 'Servo 5' in problems[0], problems[0] if problems else '')

    stale = Monitor(recorded)
    check('a fault older than the baseline is not re-reported',
          stale.check(recorded) == [])

    hot = Monitor(status({1: 30}))
    check('a servo crossing the critical line is reported',
          len(hot.check(status({1: 63}))) == 1)

    print('')
    print('%d checks, %d failed' % (total[0], len(failures)))
    for item in failures:
        print('  FAILED: %s' % item)
    return 1 if failures else 0


if __name__ == '__main__':
    raise SystemExit(_self_test())
