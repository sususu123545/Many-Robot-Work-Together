#!/usr/bin/env python3
"""One place to work out the car's SSH password.

Why this exists
---------------
Two tools shipped the password as a literal default::

    parser.add_argument('--password', default='raspberrypi')

The repository is public, so that line published the credential to anyone who
could read GitHub.  A default argument is also the worst possible place for a
secret: it looks like documentation, `--help` prints it, and removing it later
breaks every script that relied on it.

Resolution order -- first hit wins, and every source is explicit:

  1. ``--password`` on the command line (unchanged; still the escape hatch)
  2. ``ARM_PASSWORD`` in the environment
  3. ``tools/runtime/arm_creds.json`` -> ``{"password": "..."}``  (git-ignored)

Nothing is read at import time.  Importing this module has no side effects and
never fails; only ``resolve_password()`` can raise, and it raises with a message
that says how to supply the password rather than just "missing".
"""

import json
import os

DEFAULT_CREDS = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             'runtime', 'arm_creds.json')

ENV_VAR = 'ARM_PASSWORD'


def resolve_password(explicit=None, creds_path=None, environ=None):
    """Return the password, or raise SystemExit explaining how to provide one."""
    if explicit:
        return explicit

    environ = os.environ if environ is None else environ
    from_env = environ.get(ENV_VAR)
    if from_env:
        return from_env

    path = DEFAULT_CREDS if creds_path is None else creds_path
    if os.path.exists(path):
        try:
            with open(path, 'r', encoding='utf-8') as handle:
                payload = json.load(handle)
        except (OSError, ValueError) as error:
            raise SystemExit('cannot read %s: %s' % (path, error))
        password = (payload or {}).get('password')
        if password:
            return password
        raise SystemExit('%s has no "password" field' % path)

    raise SystemExit(
        'no password available.  Provide one by any of:\n'
        '  --password <value>\n'
        '  %s=<value> in the environment\n'
        '  %s containing {"password": "..."}\n'
        'The credential is deliberately not defaulted: this repository is public.'
        % (ENV_VAR, path))


def _self_test():
    import tempfile
    failures = []
    total = [0]

    def check(name, condition, detail=''):
        total[0] += 1
        if not condition:
            failures.append('%s %s' % (name, detail))
        print('%-52s %s %s' % (name, 'PASS' if condition else 'FAIL', detail))

    empty_dir = tempfile.mkdtemp()
    missing = os.path.join(empty_dir, 'nope.json')

    check('an explicit password wins',
          resolve_password('typed', creds_path=missing, environ={}) == 'typed')
    check('the environment is second',
          resolve_password(None, creds_path=missing,
                           environ={ENV_VAR: 'fromenv'}) == 'fromenv')
    check('the command line beats the environment',
          resolve_password('typed', creds_path=missing,
                           environ={ENV_VAR: 'fromenv'}) == 'typed')

    creds = os.path.join(empty_dir, 'creds.json')
    with open(creds, 'w', encoding='utf-8') as handle:
        json.dump({'password': 'fromfile'}, handle)
    check('the credential file is the last resort',
          resolve_password(None, creds_path=creds, environ={}) == 'fromfile')
    check('the environment beats the file',
          resolve_password(None, creds_path=creds,
                           environ={ENV_VAR: 'fromenv'}) == 'fromenv')

    try:
        resolve_password(None, creds_path=missing, environ={})
        check('missing everywhere raises', False, 'no exception')
    except SystemExit as error:
        check('missing everywhere raises and explains itself',
              ENV_VAR in str(error) and '--password' in str(error))

    with open(creds, 'w', encoding='utf-8') as handle:
        handle.write('{}')
    try:
        resolve_password(None, creds_path=creds, environ={})
        check('a file with no password field raises', False, 'no exception')
    except SystemExit:
        check('a file with no password field raises', True)

    import shutil
    import subprocess
    if shutil.which('git') is None:
        return
    # `git grep` exits 1 when nothing matches -- that is the PASS case here, so
    # the return code must not be allowed to raise.
    result = subprocess.run(
        ['git', 'grep', '-n', '-I', '-e', 'raspberrypi', '--', 'tools/'],
        cwd=os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        capture_output=True, text=True)
    # Only a --password argument counts.  `--host-key-alias raspberrypi` is an
    # SSH config alias, not a credential, and flagging it would train people to
    # ignore this check.
    offending = [line for line in result.stdout.splitlines()
                 if '--password' in line and 'default' in line]
    check('no literal password is still a default', not offending,
          offending[0][:120] if offending else '')

    print('')
    print('%d checks, %d failed' % (total[0], len(failures)))
    for item in failures:
        print('  FAILED: %s' % item)
    return 1 if failures else 0


if __name__ == '__main__':
    raise SystemExit(_self_test())
