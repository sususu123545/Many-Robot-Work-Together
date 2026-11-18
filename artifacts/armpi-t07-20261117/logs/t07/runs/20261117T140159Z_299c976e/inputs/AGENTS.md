# Project work orders

User requirement (2026-11-13): every work-order attempt must leave evidence for troubleshooting and retrospective review. During simulation on ycw, display the Gazebo GUI on ycw's logged-in desktop.

- Use unique run folders under `logs/tXX/runs/`; preserve failed attempts and earlier results.
- Save commands, configuration/source snapshots with hashes, runtime output, measurements, relevant images/video, acceptance decisions and unresolved items. Use `scripts/workorder_archive.py` where practical.
- Update the master work order and its linked progress record. Completion requires measured acceptance evidence; pending or failed checks stay explicit.
- Keep simulation in localhost-only ROS domains and isolated Gazebo partitions. Physical robot changes are separate from simulation authorization.
- Never archive credentials, private keys, tokens, or a full process environment. Select only required environment fields.
