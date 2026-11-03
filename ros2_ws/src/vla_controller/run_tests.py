#!/usr/bin/env python3
"""Run the pure-Python tests on a workstation that has no ROS 2 installed.

``colcon test`` (or ``pytest``) is the supported path on the robot. This script
exists because the interesting logic in this package -- the safety filter, the
adapter, the chunk executor -- has to be verifiable before anything is deployed,
and the workstation used for development is Windows without ROS 2.

Usage:  python3 run_tests.py [-v]
"""

import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.dirname(HERE)

for path in (HERE, os.path.join(SRC, 'armpi_ros2')):
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
