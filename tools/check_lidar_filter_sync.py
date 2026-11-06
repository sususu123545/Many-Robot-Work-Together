#!/usr/bin/env python3
"""Fail if the web console's lidar filter drifts away from scan_clean.py.

Why this exists
---------------
The console and the robot answer the same question -- "what is around the car?"
-- and on 2026-11-06 they answered differently.  `scan_clean.py` had already been
corrected to a sector-based filter after a collision, while the console still
carried the old "drop everything under 30 cm" rule and a comment claiming the two
were "kept consistent".  The comment had been true once.  Nothing checked it.

A comment is not a guard.  This script is the guard: it reads both files and
compares the numbers that have to agree.

    python3 tools/check_lidar_filter_sync.py        # exit 0 = in sync

Run it after touching either file.  It is cheap, offline, and needs no robot.
"""

import ast
import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
PYTHON_SIDE = os.path.join(HERE, 'scan_clean.py')
JS_SIDE = os.path.join(REPO, 'web_console', 'sensor_console.html')


def python_filter():
    """(min_return_m, [(lo, hi, max_m), ...]) as scan_clean actually runs them."""
    with open(PYTHON_SIDE, 'r', encoding='utf-8') as handle:
        tree = ast.parse(handle.read())
    values = {}
    for node in tree.body:
        if not isinstance(node, ast.Assign):
            continue
        for target in node.targets:
            if isinstance(target, ast.Name) and target.id in ('MIN_RETURN_M', 'SELF_SECTORS'):
                values[target.id] = ast.literal_eval(node.value)
    if 'MIN_RETURN_M' not in values or 'SELF_SECTORS' not in values:
        raise SystemExit('%s no longer defines MIN_RETURN_M and SELF_SECTORS '
                         'as module-level literals; this checker needs updating'
                         % PYTHON_SIDE)
    return float(values['MIN_RETURN_M']), [tuple(float(x) for x in row)
                                           for row in values['SELF_SECTORS']]


def js_filter():
    """(min_range_m, [(lo, hi, max_m), ...]) as the console actually applies them."""
    with open(JS_SIDE, 'r', encoding='utf-8') as handle:
        html = handle.read()
    sectors = re.search(r'const\s+SELF_SECTORS\s*=\s*(\[\[.*?\]\])', html, re.S)
    minimum = re.search(r'const\s+MIN_RANGE_M\s*=\s*([0-9.]+)', html)
    if sectors is None or minimum is None:
        raise SystemExit(
            '%s does not define SELF_SECTORS / MIN_RANGE_M.\n'
            'Either the console has not been migrated to the shared filter yet, or '
            'the names changed.  Until it is migrated the console applies its own '
            'rule, which is exactly the situation this checker exists to catch.'
            % JS_SIDE)
    rows = json.loads(sectors.group(1))
    return float(minimum.group(1)), [tuple(float(x) for x in row) for row in rows]


def main():
    py_min, py_sectors = python_filter()
    js_min, js_sectors = js_filter()
    problems = []

    if abs(py_min - js_min) > 1e-9:
        problems.append('MIN_RETURN_M differs: scan_clean=%g console=%g'
                        % (py_min, js_min))

    if len(py_sectors) != len(js_sectors):
        problems.append('different number of self-occlusion sectors: '
                        'scan_clean=%d console=%d' % (len(py_sectors), len(js_sectors)))
    else:
        for index, (a, b) in enumerate(zip(py_sectors, js_sectors)):
            if any(abs(x - y) > 1e-9 for x, y in zip(a, b)):
                problems.append('sector %d differs: scan_clean=%s console=%s'
                                % (index, a, b))

    # The old blanket rule is the specific thing that hid real table legs.  If
    # anyone reintroduces a bare distance cut-off, say so by name.  Comments are
    # stripped first: the fix itself explains the old rule in a comment, and a
    # checker that fails on its own documentation is a checker people disable.
    with open(JS_SIDE, 'r', encoding='utf-8') as handle:
        code = handle.read()
    code = re.sub(r'/\*.*?\*/', '', code, flags=re.S)
    code = re.sub(r'//[^\n]*', '', code)
    if re.search(r'r\s*>=\s*0\.3\b', code):
        problems.append('the console still contains a bare "r >= 0.3" distance '
                        'cut-off -- that is the rule that hid the table leg')

    print('scan_clean.py   min=%.3f m  sectors=%s' % (py_min, py_sectors))
    print('sensor_console  min=%.3f m  sectors=%s' % (js_min, js_sectors))
    print('')
    if problems:
        for problem in problems:
            print('MISMATCH: %s' % problem)
        print('')
        print('FAILED: the console and the robot do not agree on what is around the car.')
        return 1
    print('OK: both sides apply the same filter.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
