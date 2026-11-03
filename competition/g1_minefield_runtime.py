"""Isaac adapter: RGB-D obstacle map + official G1 velocity policy.

This runtime only walks the G1 through the minefield. It detects red/yellow
objects for logging, but intentionally exposes no unplugging or manipulation
executor.
"""
from __future__ import annotations

import json
import math
import sys
import time
from pathlib import Path

import numpy as np
import torch

from g1_minefield_navigation import MinefieldNavigator
from g1_sensor_navigation import DepthObstacleMap, SensorOdometry
from g1_sim_runtime import load_policy, state_of


def run_minefield_policy(sim, robot, app, args, root: Path, rgbd):
    policy_source = args.g1_policy_source
    policy = load_policy(robot, root, policy_source)
    odom = SensorOdometry(
        root / "vendor/unitree_ros/robots/g1_description/g1_29dof_rev_1_0.urdf",
        start=(args.g1_x, 0.0, 0.0),
    )
    obstacle_map = DepthObstacleMap()
    navigator = MinefieldNavigator(
        goal_x=args.minefield_goal_x,
        lane_width=args.minefield_lane_width,
        speed=args.minefield_speed,
    )
    sys.path.insert(0, str(root / "task_logic"))
    from g1_task_perception import RGBDColorDetector
    mission_config = json.loads(
        (root / "task_logic/g1_mission_config.template.json").read_text(encoding="utf-8")
    )
    detector = RGBDColorDetector.from_config(mission_config.get("perception", {}))

    interval = round(0.02 / sim.cfg.dt)
    start_physics_time = float(sim.current_time)
    steps = 0
    command = (0.0, 0.0, 0.0)
    frame_id = 0
    latest_rgbd_s = -1.0
    seen_tracks: dict[str, str] = {}
    detections_log = []
    trace = []
    result = {
        "mode": "minefield_navigation_only",
        "status": "running",
        "policy_source": policy_source,
        "manipulation_enabled": False,
        "localization": "leg_encoders_imu_and_rgbd_depth_map",
        "navigation_inputs": ["RGB", "depth", "camera_intrinsics", "joint_encoders", "IMU"],
        "ground_truth_used_for_navigation": False,
        "simulator_safety_supervisor_uses_ground_truth": True,
        "start_position": state_of(robot)[0],
        "goal_x_m": float(args.minefield_goal_x),
        "minefield_lane_width_m": float(args.minefield_lane_width),
        "max_abs_roll_rad": 0.0,
        "max_abs_pitch_rad": 0.0,
        "min_base_height_m": float("inf"),
        "trace": trace,
        "events": [],
    }
    previous_phase = None
    terminal_since = None
    print("G1_MINEFIELD_RUNTIME_READY navigation=RGBD obstacle-map gait=official-Unitree-MJLab manipulation=disabled", flush=True)

    camera_interval = max(1, round(0.1 / sim.cfg.dt))
    max_steps = max(1, round(args.minefield_timeout / sim.cfg.dt))
    while app.is_running() and steps < max_steps:
        tick_started = time.perf_counter()
        seconds = steps * sim.cfg.dt
        if steps % interval == 0:
            joints = robot.data.joint_pos[0].detach().cpu().numpy()
            joint_velocities = robot.data.joint_vel[0].detach().cpu().numpy()
            imu_quat, gyro, gravity = rgbd.read_imu()
            qw, qx, qy, qz = imu_quat
            attitude_yaw = math.atan2(2 * (qw * qz + qx * qy),
                                      1 - 2 * (qy * qy + qz * qz))
            odom.predict(
                dict(zip(robot.joint_names, joints)), gyro, gravity, 0.02,
                dict(zip(robot.joint_names, joint_velocities)), attitude_yaw,
            )

            if steps > 0 and steps % camera_interval == 0:
                frame = rgbd.read(seconds, 0.1)
                latest_rgbd_s = seconds
                good = obstacle_map.update(frame, odom)
                frame_id += 1
                detections = detector.detect(frame, frame_id=frame_id)
                for detection in detections:
                    if detection.track_id is not None:
                        seen_tracks[detection.track_id] = detection.label
                if frame_id % 10 == 0:
                    row = {
                        "seconds": seconds,
                        "detections": [
                            {"label": d.label, "track_id": d.track_id,
                             "confidence": round(float(d.confidence), 3),
                             "position_base_m": [round(float(x), 3) for x in d.position_base]}
                            for d in detections
                        ],
                        "map_cells": int(len(obstacle_map.points)),
                        "map_update_ok": bool(good),
                    }
                    detections_log.append(row)
                    print("G1_MINEFIELD_RGBD " + json.dumps(row, ensure_ascii=False), flush=True)

            pos, roll, pitch, yaw = state_of(robot)
            result["min_base_height_m"] = min(result["min_base_height_m"], pos[2])
            result["max_abs_roll_rad"] = max(result["max_abs_roll_rad"], abs(roll))
            result["max_abs_pitch_rad"] = max(result["max_abs_pitch_rad"], abs(pitch))
            # This is a supervisor only; the planner receives sensor odometry and
            # the RGB-D map, never these simulator coordinates.
            fallen = pos[2] < 0.48 or abs(roll) > 0.85 or abs(pitch) > 0.85
            outside_lane = abs(odom.pose[1]) > args.minefield_lane_width / 2.0 - 0.15
            if fallen:
                result.update(status="stopped", reason="fall_or_tilt")
                command = (0.0, 0.0, 0.0)
            elif outside_lane:
                result.update(status="stopped", reason="lane_boundary_safety")
                command = (0.0, 0.0, 0.0)
            elif obstacle_map.frame_count < 4:
                if seconds >= 3.0:
                    result.update(status="stopped", reason="rgbd_map_not_ready")
                else:
                    navigator.phase = "WAIT_FOR_MAP"
                    navigator.last_reason = "collecting_valid_depth_frames"
                command = (0.0, 0.0, 0.0)
            else:
                nav = navigator.update(
                    pose=odom.pose,
                    obstacle_points=obstacle_map.points,
                    now_s=seconds,
                    sensor_stamp_s=obstacle_map.last_frame_seconds,
                    localization_stamp_s=obstacle_map.last_frame_seconds,
                    fallen=False,
                    dt_s=interval * sim.cfg.dt,
                )
                command = nav.velocity
                if nav.phase != previous_phase:
                    event = {"seconds": seconds, "phase": nav.phase,
                             "action": nav.action, "reason": nav.reason,
                             "command": list(command)}
                    result["events"].append(event)
                    print("G1_MINEFIELD_PHASE " + json.dumps(event), flush=True)
                    previous_phase = nav.phase
                if nav.phase == "COMPLETE":
                    result.update(status="minefield_section_complete", reason=nav.reason)
                elif nav.phase.endswith("STOP") or nav.phase in {"SAFE_STOP", "CLEARANCE_STOP", "LOCALIZATION_STOP"}:
                    result.update(status="stopped", reason=nav.reason)

            if result["status"] not in {"running"}:
                command = (0.0, 0.0, 0.0)
            target = policy.act(robot, command)
            robot.set_joint_position_target(torch.as_tensor(target, device=robot.device).unsqueeze(0))
            if steps % (interval * 5) == 0:
                trace.append({"seconds": seconds, "estimated_pose": odom.pose.tolist(),
                              "sim_pose_for_evaluation": pos, "roll": roll, "pitch": pitch,
                              "command": list(command), "phase": navigator.phase,
                              "map_cells": int(len(obstacle_map.points)),
                              "rgbd_frames": int(obstacle_map.frame_count),
                              "min_clearance_m": navigator.last_clearance_m,
                              "planned_path": navigator.path.tolist()})
            if result["status"] != "running":
                if terminal_since is None:
                    terminal_since = seconds
                elif seconds - terminal_since >= 1.0:
                    break

        robot.write_data_to_sim()
        sim.step(render=False)
        robot.update(sim.cfg.dt)
        steps += 1
        if steps % sim.cfg.render_interval == 0:
            sim.render()
        if args.minefield_realtime:
            time.sleep(max(0.0, sim.cfg.dt - (time.perf_counter() - tick_started)))

    if result["status"] == "running":
        result["status"] = "timeout" if app.is_running() else "window_closed"
    result.update(
        steps=steps,
        sim_seconds=steps * sim.cfg.dt,
        measured_physics_seconds=float(sim.current_time) - start_physics_time,
        finish_position=state_of(robot)[0],
        estimated_finish_pose=odom.pose.tolist(),
        obstacle_map_cells=int(len(obstacle_map.points)),
        obstacle_map_frames=int(obstacle_map.frame_count),
        detected_color_tracks=seen_tracks,
        detection_log=detections_log,
        navigation={"phase": navigator.phase, "replans": navigator.replans,
                    "blocked_plans": navigator.blocked_plans,
                    "stale_stops": navigator.stale_stops,
                    "min_clearance_m": navigator.last_clearance_m},
    )
    print("G1_MINEFIELD_RESULT " + json.dumps(
        {k: v for k, v in result.items() if k not in {"trace", "detection_log"}},
        ensure_ascii=False), flush=True)
    out = args.scene_out / "g1_minefield_navigation_result.json"
    out.write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")
    if args.preview and app.is_running():
        sim.pause()
        print(f"G1_MINEFIELD_INSPECT {out}; simulation paused for review", flush=True)
        while app.is_running():
            sim.render()
            time.sleep(1.0 / 30.0)
    return result, policy

