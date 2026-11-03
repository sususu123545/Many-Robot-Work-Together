"""Small integration facade: RGB-D frame -> detections -> mission command."""
from __future__ import annotations

from dataclasses import asdict
import json
from pathlib import Path
from typing import Any, Mapping

from g1_mission_controller import MissionController
from g1_task_perception import RGBDColorDetector


class G1TaskStack:
    """Sensor-to-task adapter intended for Isaac, ROS 2, or MATRiX bridges.

    A host supplies a calibrated RGB-D frame, health flags, and asynchronous
    executor acknowledgements. This adapter never sends actuator commands.
    """

    def __init__(self, config: Mapping[str, Any]):
        self.controller = MissionController(config)
        perception_cfg = config.get("perception", {})
        self.detector = RGBDColorDetector.from_config(perception_cfg)
        self.frame_id = 0

    @classmethod
    def from_file(cls, path: str | Path) -> "G1TaskStack":
        with Path(path).open("r", encoding="utf-8") as stream:
            config = json.load(stream)
        if not isinstance(config, Mapping):
            raise ValueError("mission configuration root must be a JSON object")
        return cls(config)

    def update(self, frame: Mapping[str, Any], health: Mapping[str, Any] | None,
               action_result: Mapping[str, Any] | None = None) -> dict[str, Any]:
        self.frame_id += 1
        detections = self.detector.detect(frame, self.frame_id)
        command = self.controller.tick(float(frame.get("seconds", 0.0)), detections,
                                       health, action_result)
        return {
            "mission": command,
            "detections": [asdict(item) for item in detections],
        }
