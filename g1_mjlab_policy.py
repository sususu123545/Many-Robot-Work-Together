"""Isaac adapter for Unitree's packaged unitree_rl_mjlab G1 velocity ONNX.

This keeps the official deployment observation layout and action scaling. The
MJCF actuator order is mapped by joint name to Isaac's articulation order.
"""

from __future__ import annotations

from pathlib import Path
import sys

import numpy as np
import yaml


G1_MJLAB_JOINTS = (
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint",
)


class G1MjlabVelocityPolicy:
    """Run the official 98-observation, 29-action Unitree RL MJLab policy."""

    OBSERVATION_ORDER = (
        "base_ang_vel", "projected_gravity", "velocity_commands", "gait_phase",
        "joint_pos_rel", "joint_vel_rel", "last_action",
    )

    def __init__(
        self,
        model_path: Path,
        deploy_config_path: Path,
        joint_names: list[str],
        package_dir: Path | None = None,
    ):
        if package_dir is not None:
            sys.path.insert(0, str(package_dir))
        try:
            import onnxruntime as ort
        finally:
            if package_dir is not None:
                sys.path.remove(str(package_dir))

        self.config = yaml.safe_load(Path(deploy_config_path).read_text(encoding="utf-8"))
        self.session = ort.InferenceSession(str(model_path), providers=["CPUExecutionProvider"])
        self.input_name = self.session.get_inputs()[0].name
        self.output_name = self.session.get_outputs()[0].name
        self.input_shape = self.session.get_inputs()[0].shape
        self.joint_indices = np.asarray(
            [joint_names.index(name) for name in G1_MJLAB_JOINTS], dtype=np.int64
        )
        self.default_joint_pos = np.asarray(
            self.config["actions"]["JointPositionAction"]["offset"], dtype=np.float32
        )
        self.action_scale = np.asarray(
            self.config["actions"]["JointPositionAction"]["scale"], dtype=np.float32
        )
        self.last_action = np.zeros(29, dtype=np.float32)
        self.step_dt = float(self.config["step_dt"])
        self.gait_period = float(self.config["observations"]["gait_phase"]["params"]["period"])
        self.phase = 0.0
        self.debug_steps = 0
        self.command_ranges = self.config["commands"]["base_velocity"]["ranges"]

        if len(self.joint_indices) != 29 or len(set(self.joint_indices.tolist())) != 29:
            raise RuntimeError("MJLab policy must map 29 unique G1 joints")
        if self.default_joint_pos.shape != (29,) or self.action_scale.shape != (29,):
            raise RuntimeError("MJLab deployment config must define 29 joint offsets and scales")
        expected = self.input_shape[-1]
        if isinstance(expected, int) and expected != 98:
            raise RuntimeError(f"Expected official MJLab input [1, 98], found {self.input_shape}")
        print(
            f"G1 MJLab policy loaded: {model_path}; input={self.input_shape}; "
            f"output={self.session.get_outputs()[0].shape}; joints={len(self.joint_indices)}",
            flush=True,
        )

    @staticmethod
    def _to_np(tensor) -> np.ndarray:
        return tensor[0].detach().cpu().numpy().astype(np.float32, copy=False)

    def _term(self, name: str, sample: np.ndarray) -> np.ndarray:
        cfg = self.config["observations"][name]
        clip = cfg.get("clip")
        if clip is not None:
            sample = np.clip(sample, float(clip[0]), float(clip[1]))
        scale = np.asarray(cfg["scale"], dtype=np.float32)
        if scale.size != sample.size:
            raise RuntimeError(f"{name} scale has {scale.size} values for {sample.size} observations")
        return sample * scale

    def _command(self, command: tuple[float, float, float]) -> np.ndarray:
        r = self.command_ranges
        return np.asarray((
            np.clip(command[0], *r["lin_vel_x"]),
            np.clip(command[1], *r["lin_vel_y"]),
            np.clip(command[2], *r["ang_vel_z"]),
        ), dtype=np.float32)

    def act(self, robot, command: tuple[float, float, float]) -> np.ndarray:
        """Return target joint angles in Isaac articulation order."""
        jp = self._to_np(robot.data.joint_pos)[self.joint_indices]
        jv = self._to_np(robot.data.joint_vel)[self.joint_indices]
        ang = self._to_np(robot.data.root_ang_vel_b)
        gravity = self._to_np(robot.data.projected_gravity_b)
        cmd = self._command(command)

        self.phase = (self.phase + self.step_dt / self.gait_period) % 1.0
        phase_obs = np.asarray((
            np.sin(2.0 * np.pi * self.phase), np.cos(2.0 * np.pi * self.phase)
        ), dtype=np.float32)
        if float(np.linalg.norm(cmd)) < 0.1:
            phase_obs[:] = 0.0

        terms = {
            "base_ang_vel": ang,
            "projected_gravity": gravity,
            "velocity_commands": cmd,
            "gait_phase": phase_obs,
            "joint_pos_rel": jp - self.default_joint_pos,
            "joint_vel_rel": jv,
            "last_action": self.last_action.copy(),
        }
        obs = np.concatenate([
            self._term(name, terms[name]) for name in self.OBSERVATION_ORDER
        ]).astype(np.float32, copy=False)[None, :]
        if obs.shape != (1, 98):
            raise RuntimeError(f"MJLab policy expected [1, 98], got {obs.shape}")

        action = self.session.run([self.output_name], {self.input_name: obs})[0]
        action = np.asarray(action, dtype=np.float32).reshape(-1)
        if action.shape != (29,) or not np.isfinite(action).all():
            raise RuntimeError(
                f"Invalid MJLab policy action: shape={action.shape}, finite={np.isfinite(action).all()}"
            )
        self.last_action = action
        targets_policy_order = self.default_joint_pos + self.action_scale * action
        targets_articulation_order = np.zeros_like(targets_policy_order)
        targets_articulation_order[self.joint_indices] = targets_policy_order
        return targets_articulation_order
