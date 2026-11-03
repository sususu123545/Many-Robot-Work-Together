"""Fail-safe high-level controller for G1 tasks 2-3.

This module is simulator/ROS independent. It sequences target detections and
high-level executor acknowledgements; it does not synthesize joint commands or
claim that an unplug/grasp succeeded without an explicit verified result.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Any, Mapping, Sequence
import math


class MissionState(str, Enum):
    WAIT_CONFIG = "WAIT_CONFIG"
    FIND_WIRE = "FIND_WIRE"
    APPROACH_WIRE = "APPROACH_WIRE"
    REMOVE_WIRE = "REMOVE_WIRE"
    FIND_SUPPLY = "FIND_SUPPLY"
    APPROACH_SUPPLY = "APPROACH_SUPPLY"
    PICK_SUPPLY = "PICK_SUPPLY"
    PLACE_SUPPLY = "PLACE_SUPPLY"
    COMPLETE = "COMPLETE"
    SAFE_STOP = "SAFE_STOP"


@dataclass(frozen=True)
class MissionConfig:
    wire_sequence: tuple[str, ...]
    supply_order: tuple[str, ...]
    min_confidence: float = .72
    stable_frames: int = 3
    max_detection_age_s: float = .40
    action_timeout_s: float = 20.0
    max_action_retries: int = 1

    @classmethod
    def from_mapping(cls, raw: Mapping[str, Any]) -> "MissionConfig":
        wires = raw.get("wire_color_sequence")
        supplies = raw.get("medical_supply_order")
        if not isinstance(wires, list) or not wires or not all(isinstance(x, str) and x.strip() for x in wires):
            raise ValueError("wire_color_sequence must be a non-empty list of color labels")
        if not isinstance(supplies, list) or not supplies or not all(isinstance(x, str) and x.strip() for x in supplies):
            raise ValueError("medical_supply_order must be a non-empty list of item labels")
        perception = raw.get("perception", {})
        control = raw.get("control", {})
        if not isinstance(perception, Mapping) or not isinstance(control, Mapping):
            raise ValueError("perception and control must be objects")
        result = cls(
            tuple(x.strip().lower() for x in wires),
            tuple(x.strip().lower() for x in supplies),
            float(perception.get("min_confidence", .72)),
            int(perception.get("stable_frames", 3)),
            float(perception.get("max_detection_age_s", .40)),
            float(control.get("action_timeout_s", 20.0)),
            int(control.get("max_action_retries", 1)),
        )
        if not 0 < result.min_confidence <= 1:
            raise ValueError("perception.min_confidence must be in (0, 1]")
        if not 1 <= result.stable_frames <= 30:
            raise ValueError("perception.stable_frames must be from 1 to 30")
        if result.max_detection_age_s <= 0 or result.action_timeout_s <= 0 or result.max_action_retries < 0:
            raise ValueError("age/timeout must be positive and retries must be non-negative")
        return result


class MissionController:
    """Order-aware state machine that only emits auditable high-level actions."""

    ACCEPTED_KINDS = {
        "wire": {"wire_candidate", "color_target"},
        "supply": {"supply_candidate", "color_target"},
        "kit": {"kit_candidate", "color_target"},
    }

    def __init__(self, config: MissionConfig | Mapping[str, Any] | None):
        self.config_error: str | None = None
        if config is None:
            self.config = None
            self.config_error = "裁判给定的引线顺序和医疗用品清单尚未配置"
        elif isinstance(config, MissionConfig):
            self.config = config
        else:
            try:
                self.config = MissionConfig.from_mapping(config)
            except (TypeError, ValueError) as exc:
                self.config = None
                self.config_error = str(exc)
        self.state = MissionState.WAIT_CONFIG if self.config is None else MissionState.FIND_WIRE
        self.wire_index = 0
        self.supply_index = 0
        self.pending: dict[str, Any] | None = None
        self._next_command_id = 1
        self._retry_count = 0
        self._stable_label: str | None = None
        self._stable_position: tuple[float, float, float] | None = None
        self._stable_frames_seen: set[int] = set()
        self._stable_count = 0
        self.completed_wire_target_ids: set[str] = set()
        self.completed_supply_target_ids: set[str] = set()
        self.held_supply_target_id: str | None = None
        self.events: list[dict[str, Any]] = []
        self.last_reason = self.config_error or ""

    def reset_after_external_recovery(self) -> None:
        """Explicit operator reset; does not silently clear progress or faults."""
        if self.config is None:
            self.state = MissionState.WAIT_CONFIG
            return
        self.pending = None
        self._retry_count = 0
        self._clear_stability()
        self.state = MissionState.FIND_WIRE if self.wire_index < len(self.config.wire_sequence) else MissionState.FIND_SUPPLY
        self.last_reason = "operator_reset"

    def _transition(self, state: MissionState, now: float, reason: str) -> None:
        if state != self.state:
            self.events.append({"time_s": float(now), "from": self.state.value,
                                "to": state.value, "reason": reason})
            self.state = state
        self.last_reason = reason

    def _clear_stability(self) -> None:
        self._stable_label = None
        self._stable_position = None
        self._stable_frames_seen.clear()
        self._stable_count = 0

    @staticmethod
    def _value(detection: Any, key: str, default: Any = None) -> Any:
        return detection.get(key, default) if isinstance(detection, Mapping) else getattr(detection, key, default)

    def _stable_target(self, detections: Sequence[Any], label: str, role: str,
                       now: float) -> dict[str, Any] | None:
        assert self.config is not None
        eligible = []
        for d in detections:
            dlabel = str(self._value(d, "label", "")).lower()
            kind = str(self._value(d, "kind", ""))
            confidence = float(self._value(d, "confidence", 0.0))
            stamp = float(self._value(d, "timestamp", -math.inf))
            position = self._value(d, "position_base", None)
            target_id = self._value(d, "track_id", None)
            completed = (self.completed_wire_target_ids if role == "wire"
                         else self.completed_supply_target_ids if role == "supply" else set())
            if (dlabel != label or kind not in self.ACCEPTED_KINDS[role]
                    or (target_id is not None and str(target_id) in completed)
                    or confidence < self.config.min_confidence
                    or not isinstance(position, (list, tuple)) or len(position) != 3
                    or not all(math.isfinite(float(x)) for x in position)
                    or float(position[0]) <= .10
                    or not 0 <= now - stamp <= self.config.max_detection_age_s):
                continue
            eligible.append(d)
        if not eligible:
            self._clear_stability()
            return None

        # Nearest forward proposal is deterministic. Sequence enforcement is
        # separate: candidates of every other color/name are ignored.
        candidate = min(eligible, key=lambda d: float(self._value(d, "position_base")[0]))
        label_now = str(self._value(candidate, "label")).lower()
        position_now = tuple(float(x) for x in self._value(candidate, "position_base"))
        fid = int(self._value(candidate, "frame_id", round(float(self._value(candidate, "timestamp")) * 10)))
        consistent = (label_now == self._stable_label and self._stable_position is not None
                      and math.dist(position_now, self._stable_position) < .45)
        if not consistent:
            self._stable_label = label_now
            self._stable_position = position_now
            self._stable_frames_seen.clear()
            self._stable_count = 0
        if fid not in self._stable_frames_seen:
            self._stable_frames_seen.add(fid)
            self._stable_count += 1
            self._stable_position = position_now
        if self._stable_count < self.config.stable_frames:
            return None
        return {
            "label": label_now,
            "kind": str(self._value(candidate, "kind")),
            "confidence": float(self._value(candidate, "confidence")),
            "position_xyz_m": position_now,
            "target_id": (str(self._value(candidate, "track_id"))
                          if self._value(candidate, "track_id") is not None else None),
        }

    def _output(self, action: str, reason: str, **extra: Any) -> dict[str, Any]:
        response = {"state": self.state.value, "action": action, "reason": reason,
                    "wire_index": self.wire_index, "supply_index": self.supply_index}
        if self.pending is not None:
            response.update({k: v for k, v in self.pending.items() if k != "issued_at"})
        response.update(extra)
        return response

    def _issue(self, action: str, target: dict[str, Any] | None, now: float) -> dict[str, Any]:
        command_id = f"g1-task-{self._next_command_id:04d}"
        self._next_command_id += 1
        self.pending = {
            "command_id": command_id, "action": action,
            "target_label": target["label"] if target else None,
            "target_id": target.get("target_id") if target else None,
            "target_xyz_m": list(target["position_xyz_m"]) if target else None,
            "issued_at": float(now),
        }
        self._retry_count = 0
        self._clear_stability()
        return self._output(action, "await_executor_ack", target_confidence=target["confidence"] if target else None)

    def _safe_stop(self, now: float, reason: str) -> dict[str, Any]:
        self.pending = None
        self._clear_stability()
        self._transition(MissionState.SAFE_STOP, now, reason)
        return self._output("halt", reason)

    def _handle_ack(self, now: float, action_result: Mapping[str, Any] | None) -> dict[str, Any] | None:
        if self.pending is None:
            return None
        assert self.config is not None
        if now - self.pending["issued_at"] > self.config.action_timeout_s:
            # The executor may still be physically moving after a lost ack;
            # never issue a duplicate manipulation command on timeout.
            return self._safe_stop(now, "action_timeout")
        if not action_result or action_result.get("command_id") != self.pending["command_id"]:
            return self._output(self.pending["action"], "await_executor_ack")
        status = str(action_result.get("status", "running")).lower()
        if status == "running":
            return self._output(self.pending["action"], "executor_running")
        if status not in {"succeeded", "failed"}:
            return self._safe_stop(now, "invalid_executor_status")
        action = self.pending["action"]
        completed_target_id = self.pending.get("target_id")
        needs_verification = action in {"remove_wire", "pick_supply", "place_supply"}
        if needs_verification and action_result.get("verified") is not True and status == "succeeded":
            return self._safe_stop(now, "physical_result_unverified")
        if status == "failed":
            reason = "action_failed"
            if self._retry_count < self.config.max_action_retries:
                self._retry_count += 1
                self.pending["issued_at"] = float(now)
                self.pending["command_id"] = f"g1-task-{self._next_command_id:04d}"
                self._next_command_id += 1
                self.last_reason = reason + "_retry"
                return self._output(action, self.last_reason, retry=self._retry_count)
            return self._safe_stop(now, reason)

        self.pending = None
        self._retry_count = 0
        self._clear_stability()
        if action == "remove_wire" and completed_target_id is not None:
            self.completed_wire_target_ids.add(str(completed_target_id))
        elif action == "pick_supply" and completed_target_id is not None:
            self.held_supply_target_id = str(completed_target_id)
        elif action == "place_supply":
            if self.held_supply_target_id is not None:
                self.completed_supply_target_ids.add(self.held_supply_target_id)
            self.held_supply_target_id = None
        transition = {
            "approach_wire": MissionState.REMOVE_WIRE,
            "remove_wire": MissionState.FIND_WIRE if self.wire_index + 1 < len(self.config.wire_sequence) else MissionState.FIND_SUPPLY,
            "approach_supply": MissionState.PICK_SUPPLY,
            "pick_supply": MissionState.PLACE_SUPPLY,
            "place_supply": MissionState.FIND_SUPPLY if self.supply_index + 1 < len(self.config.supply_order) else MissionState.COMPLETE,
        }.get(action)
        if action == "remove_wire":
            self.wire_index += 1
        elif action == "place_supply":
            self.supply_index += 1
        if transition is None:
            return self._safe_stop(now, "unknown_pending_action")
        self._transition(transition, now, f"{action}_acknowledged")
        return None

    def tick(self, now: float, detections: Sequence[Any] = (),
             health: Mapping[str, Any] | None = None,
             action_result: Mapping[str, Any] | None = None) -> dict[str, Any]:
        """Return one bounded command for a single perception/control cycle.

        ``health`` must include healthy=True, sensor_age_s, fallen, and e_stop.
        Manipulation executor acknowledgements use status=running/succeeded/failed;
        removal and supply actions require verified=True before progress advances.
        """
        now = float(now)
        if self.config is None:
            return self._output("hold", self.config_error or "mission_not_configured")
        if health is None:
            return self._output("hold", "health_not_provided")
        if health.get("e_stop") is True:
            return self._safe_stop(now, "emergency_stop")
        if health.get("fallen") is True:
            return self._safe_stop(now, "fall_detected")
        if health.get("healthy") is not True:
            return self._safe_stop(now, "robot_health_not_ok")
        try:
            sensor_age = float(health["sensor_age_s"])
        except (KeyError, TypeError, ValueError):
            return self._safe_stop(now, "sensor_age_missing")
        if not math.isfinite(sensor_age) or sensor_age < 0 or sensor_age > self.config.max_detection_age_s:
            return self._safe_stop(now, "stale_sensor")
        if self.state == MissionState.SAFE_STOP:
            return self._output("halt", self.last_reason or "safe_stop_latched")
        if self.state == MissionState.COMPLETE:
            return self._output("halt", "mission_complete")

        ack_output = self._handle_ack(now, action_result)
        if ack_output is not None:
            return ack_output
        if self.state == MissionState.COMPLETE:
            return self._output("halt", "mission_complete")

        if self.state == MissionState.FIND_WIRE:
            if self.wire_index >= len(self.config.wire_sequence):
                self._transition(MissionState.FIND_SUPPLY, now, "wire_sequence_complete")
            else:
                label = self.config.wire_sequence[self.wire_index]
                target = self._stable_target(detections, label, "wire", now)
                if target is None:
                    return self._output("hold", f"searching_for_wire:{label}")
                self._transition(MissionState.APPROACH_WIRE, now, f"stable_wire:{label}")
                return self._issue("approach_wire", target, now)

        if self.state == MissionState.REMOVE_WIRE:
            label = self.config.wire_sequence[self.wire_index]
            target = self._stable_target(detections, label, "wire", now)
            if target is None:
                return self._output("hold", f"reacquiring_wire:{label}")
            return self._issue("remove_wire", target, now)

        if self.state == MissionState.FIND_SUPPLY:
            if self.supply_index >= len(self.config.supply_order):
                self._transition(MissionState.COMPLETE, now, "supply_order_complete")
                return self._output("halt", "mission_complete")
            label = self.config.supply_order[self.supply_index]
            target = self._stable_target(detections, label, "supply", now)
            if target is None:
                return self._output("hold", f"searching_for_supply:{label}")
            self._transition(MissionState.APPROACH_SUPPLY, now, f"stable_supply:{label}")
            return self._issue("approach_supply", target, now)

        if self.state == MissionState.PICK_SUPPLY:
            label = self.config.supply_order[self.supply_index]
            target = self._stable_target(detections, label, "supply", now)
            if target is None:
                return self._output("hold", f"reacquiring_supply:{label}")
            return self._issue("pick_supply", target, now)

        if self.state == MissionState.PLACE_SUPPLY:
            target = self._stable_target(detections, "kit", "kit", now)
            if target is None:
                return self._output("hold", "searching_for_kit")
            target = dict(target)
            target["label"] = "kit"
            return self._issue("place_supply", target, now)

        if self.state in {MissionState.APPROACH_WIRE, MissionState.APPROACH_SUPPLY}:
            return self._output("hold", "awaiting_action_result")
        return self._safe_stop(now, "unhandled_mission_state")
