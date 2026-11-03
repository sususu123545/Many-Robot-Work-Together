"""Liveness supervision for the VLA control path.

:mod:`robot_safety.watchdog` is pure Python and testable without ROS 2;
:mod:`robot_safety.supervisor_node` is the ROS 2 shell that feeds it and
publishes the verdict.
"""

from .watchdog import SafetyLatch, Watchdog

__all__ = ['SafetyLatch', 'Watchdog']
