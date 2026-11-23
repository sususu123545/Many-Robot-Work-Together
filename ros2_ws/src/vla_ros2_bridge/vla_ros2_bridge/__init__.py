"""The head of the VLA loop: observation -> policy -> action chunk.

This package fills the missing half identified in ``docs/VLA架构落地-Round1.md``
section 6.  ``vla_controller`` already implements everything *after* a policy
has produced an action: safety filtering, IK adaptation and rolling chunk
execution.  What did not exist was anything that constructs an observation,
calls a policy, or sends a goal to ``/vla/execute_chunk``.  The ActionServer in
``vla_controller`` had no client at all, so a natural-language instruction could
not move the robot by one millimetre.

Package layout follows the same rule as ``vla_controller``:

* ``observation.py``, ``policy.py`` and ``bridge.py`` are pure Python.  They
  import neither ``rclpy`` nor any generated ROS 2 message, and are unit-tested
  on a Windows workstation with no ROS 2 installed (``run_tests.py``).
* ``observation_node.py`` and ``bridge_node.py`` are thin ROS 2 shells that turn
  those plain dictionaries into messages.  They contain no decision logic.

Honest scope: the remote-policy protocol in ``policy.py`` is a minimal
project-defined placeholder, NOT a LeRobot / SmolVLA standard.  No model
weights, no normalisation statistics and no recorded episodes exist in this
repository yet, so ``BridgePlanner`` refuses to de-normalise without statistics
rather than inventing them.
"""

from .bridge import (BridgeError, BridgePlanner, ChunkSpec)  # noqa: F401
from .observation import (ObservationBuilder, ObservationError,  # noqa: F401
                          SensorSnapshot)
from .policy import (POLICY_OUTPUT_NORMALIZED, POLICY_OUTPUT_PHYSICAL,  # noqa: F401
                     Policy, PolicyError, PolicyOutput, ScriptedPolicy)

__all__ = [
    'BridgeError', 'BridgePlanner', 'ChunkSpec',
    'ObservationBuilder', 'ObservationError', 'SensorSnapshot',
    'Policy', 'PolicyError', 'PolicyOutput', 'ScriptedPolicy',
    'POLICY_OUTPUT_NORMALIZED', 'POLICY_OUTPUT_PHYSICAL',
]
