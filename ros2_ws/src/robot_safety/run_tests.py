#!/usr/bin/env python3
"""Run the pure-Python watchdog tests without ROS 2 (see vla_controller's twin)."""

import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)


def main():
    verbosity = 2 if '-v' in sys.argv[1:] else 1
    suite = unittest.defaultTestLoader.discover(
        os.path.join(HERE, 'test'), pattern='test_*.py')
    result = unittest.TextTestRunner(verbosity=verbosity).run(suite)
    return 0 if result.wasSuccessful() else 1


if __name__ == '__main__':
    sys.exit(main())
