"""Make the sibling packages importable without a colcon install.

The tests deliberately run against the checked-in sources, not an installed
workspace: the safety logic has to be verifiable on a workstation that has no
ROS 2 at all.
"""

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PACKAGE_ROOT = os.path.dirname(HERE)
SRC_ROOT = os.path.dirname(PACKAGE_ROOT)

for path in (PACKAGE_ROOT, os.path.join(SRC_ROOT, 'armpi_ros2')):
    if path not in sys.path:
        sys.path.insert(0, path)
