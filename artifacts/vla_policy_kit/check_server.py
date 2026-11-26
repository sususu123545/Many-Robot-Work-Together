"""Check a policy server against the contract before the robot ever sees it.

Sends one real observation (a frame + state taken from the training dataset) and
applies exactly the rules the robot applies.  If this passes, the remaining
failure modes on the robot side are mechanical, not protocol.

    python check_server.py --endpoint http://127.0.0.1:8000/act
    python check_server.py --endpoint http://100.x.y.z:8000/act --rounds 5
"""

from __future__ import annotations

import argparse
import json
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

KIT = Path(__file__).resolve().parent
SAMPLE = KIT / 'samples' / 'sample_observation.json'

JOINT_ORDER = ('joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'r_joint')
CONTROL_DT_RANGE = (0.3, 5.0)


def load_observation() -> dict:
    if not SAMPLE.is_file():
        raise SystemExit('missing %s -- copy it from the kit' % SAMPLE)
    return json.loads(SAMPLE.read_text(encoding='utf-8'))


def call(endpoint: str, observation: dict, timeout: float) -> tuple[dict, float]:
    body = json.dumps(observation).encode('utf-8')
    request = urllib.request.Request(
        endpoint, data=body, method='POST',
        headers={'Content-Type': 'application/json'})
    started = time.monotonic()
    with urllib.request.urlopen(request, timeout=timeout) as response:
        payload = json.loads(response.read().decode('utf-8'))
    return payload, (time.monotonic() - started) * 1000


def check(payload: dict) -> list[str]:
    """The same checks tools/vla_rollout.py runs before touching the arm."""
    problems = []
    actions = payload.get('actions')
    if not isinstance(actions, list) or not actions:
        problems.append('actions missing or empty')
        return problems
    kind = payload.get('output_kind')
    if kind != 'physical_radians':
        problems.append('output_kind is %r, must be "physical_radians" '
                        '(de-normalise on the server)' % (kind,))
    control_dt = payload.get('control_dt')
    if not isinstance(control_dt, (int, float)):
        problems.append('control_dt missing')
    elif not CONTROL_DT_RANGE[0] <= float(control_dt) <= CONTROL_DT_RANGE[1]:
        problems.append('control_dt %.3f outside 0.3..5.0 s' % control_dt)
    elif abs(float(control_dt) - 0.5) > 1e-6:
        problems.append('control_dt is %.3f s, expected 0.5 (2 Hz)' % control_dt)
    for index, action in enumerate(actions):
        if not isinstance(action, (list, tuple)) or len(action) != len(JOINT_ORDER):
            problems.append('action %d is not 6 numbers' % index)
            break
        if not all(isinstance(v, (int, float)) for v in action):
            problems.append('action %d has non-numeric values' % index)
            break
        if any(v != v or abs(v) == float('inf') for v in action):
            problems.append('action %d has non-finite values' % index)
            break
        if any(abs(v) > 4.0 for v in action):
            problems.append('action %d has |value| > 4 rad -- looks normalised, '
                            'not physical' % index)
            break
    if len(actions) != 10:
        problems.append('chunk has %d actions, expected 10 (5 s at 2 Hz)'
                        % len(actions))
    return problems


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--endpoint', required=True)
    parser.add_argument('--rounds', type=int, default=3)
    parser.add_argument('--timeout', type=float, default=2.0)
    args = parser.parse_args(argv)

    observation = load_observation()
    print('观测: state=%s' % ['%+.3f' % v for v in observation['state']])
    print('      image %d bytes (base64)  task=%r'
          % (len(observation['image_b64']), observation.get('task')))

    failures = 0
    for round_index in range(args.rounds):
        try:
            payload, elapsed = call(args.endpoint, observation, args.timeout)
        except urllib.error.URLError as error:
            print('第 %d 轮: 连不上 (%s)' % (round_index + 1, error))
            failures += 1
            continue
        problems = check(payload)
        status = 'OK' if not problems else 'FAIL'
        print('第 %d 轮: %s  %.0f ms  %d 条动作%s'
              % (round_index + 1, status, elapsed, len(payload.get('actions') or []),
                 '' if elapsed <= 500 else '  ← 超出 500 ms 预算'))
        for problem in problems:
            print('    - %s' % problem)
        if problems:
            failures += 1
        elif payload.get('actions'):
            first = payload['actions'][0]
            print('    首条: %s'
                  % ' '.join('%+.3f' % v for v in first))

    print()
    print('结论: %s' % ('全部通过，可以上真机' if not failures
                        else '%d/%d 轮不合格' % (failures, args.rounds)))
    return 0 if not failures else 1


if __name__ == '__main__':
    sys.exit(main())
