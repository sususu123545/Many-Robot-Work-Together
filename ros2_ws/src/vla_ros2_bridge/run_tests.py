#!/usr/bin/env python3
"""Run the pure-Python bridge tests on a workstation with no ROS 2 installed.

Mirrors ``vla_controller/run_tests.py``.  The interesting logic here is the
observation contract and the goal contract; both are plain dictionaries, so a
Windows workstation can assert them exactly before anything is deployed.

Usage:  python3 run_tests.py [-v]
"""

import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.dirname(HERE)          # .../ros2_ws/src, the package root

for path in (SRC, os.path.join(SRC, 'vla_controller'),
             os.path.join(SRC, 'armpi_ros2')):
    if path not in sys.path:
        sys.path.insert(0, path)


def main():
    verbosity = 2 if '-v' in sys.argv[1:] else 1
    suite = unittest.defaultTestLoader.discover(
        os.path.join(HERE, 'test'), pattern='test_*.py')
    result = unittest.TextTestRunner(verbosity=verbosity).run(suite)
    return 0 if result.wasSuccessful() else 1


if __name__ == '__main__':
    sys.exit(main())
