"""Build a separate Isaac Lab scene for G1's minefield and medical tasks.

The geometry follows the organizer's combined mine-reconnaissance and medical
supplies reference image. The organizer did not publish CAD dimensions, a
fixed wire-color sequence, or a supplies manifest, so those remain editable
task configuration rather than invented official values.
"""
from __future__ import annotations

import argparse
import json
import math
import random
import time
from pathlib import Path

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--preview", action="store_true", help="Keep a paused GUI preview open")
parser.add_argument("--frames", type=int, default=1, help="Physics frames for a headless validation")
parser.add_argument("--scene-out", type=Path, default=Path("/mnt/llm/g1_sim_ws/competition_sim/stages/codex_g1_minefield_medical_20261019"))
parser.add_argument("--g1-x", type=float, default=14.25, help="G1 start x, aligned with the forest exit")
parser.add_argument("--g1-y", type=float, default=5.0)
parser.add_argument("--rgbd-diagnostics", action="store_true",
                    help="Run onboard RGB-D target proposals in the paused preview; never actuates")
parser.add_argument("--g1-minefield-autonomy", action="store_true",
                    help="Walk through the minefield using RGB-D obstacle avoidance; no wire manipulation")
parser.add_argument("--g1-policy-source", choices=("unitree_rl_lab", "unitree_rl_mjlab"),
                    default="unitree_rl_mjlab")
parser.add_argument("--minefield-speed", type=float, default=0.22)
parser.add_argument("--minefield-goal-x", type=float, default=22.4)
parser.add_argument("--minefield-lane-width", type=float, default=4.7)
parser.add_argument("--minefield-timeout", type=float, default=90.0)
parser.add_argument("--minefield-realtime", action="store_true",
                    help="Pace the autonomous run near real time for visual inspection")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
if args.rgbd_diagnostics or args.g1_minefield_autonomy:
    # Isaac Lab disables render products unless camera support is enabled
    # before AppLauncher starts, even when the main GUI viewport is visible.
    args.enable_cameras = True
app = AppLauncher(args).app

import isaaclab.sim as sim_utils
import torch
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import Articulation, ArticulationCfg
from pxr import Gf, Usd, UsdGeom, Vt


ROOT = Path("/mnt/llm/g1_sim_ws/competition_sim")
G1_USD = Path("/mnt/llm/g1_sim_ws/unitree_model/G1/29dof/usd/g1_29dof_rev_1_0/g1_29dof_rev_1_0.usd")
Y = args.g1_y
START_X = 15.0
END_X = 28.5


def body_cfg(color, collision=True):
    return {
        "visual_material": sim_utils.PreviewSurfaceCfg(diffuse_color=color),
        "collision_props": sim_utils.CollisionPropertiesCfg(collision_enabled=collision),
        "rigid_props": sim_utils.RigidBodyPropertiesCfg(
            kinematic_enabled=True, disable_gravity=True, max_depenetration_velocity=1.0
        ),
    }


def box(path, size, pos, color, collision=True):
    cfg = sim_utils.CuboidCfg(size=tuple(size), **body_cfg(color, collision))
    cfg.func(path, cfg, translation=tuple(pos))


def cylinder(path, radius, height, axis, pos, color, collision=True):
    cfg = sim_utils.CylinderCfg(radius=radius, height=height, axis=axis,
                                **body_cfg(color, collision))
    cfg.func(path, cfg, translation=tuple(pos))


def cone(path, radius, height, pos, color, collision=False):
    cfg = sim_utils.ConeCfg(radius=radius, height=height, axis="Z",
                            **body_cfg(color, collision))
    cfg.func(path, cfg, translation=tuple(pos))


def curve(path, points, color, width=0.014):
    stage = __import__("omni.usd", fromlist=["get_context"]).get_context().get_stage()
    shape = UsdGeom.BasisCurves.Define(stage, path)
    shape.CreateTypeAttr("linear")
    shape.CreateCurveVertexCountsAttr([len(points)])
    shape.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*p) for p in points]))
    shape.CreateWidthsAttr([width])
    shape.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    shape.SetWidthsInterpolation(UsdGeom.Tokens.constant)


def fence(start_x, end_x, y):
    x = start_x
    index = 0
    while x <= end_x + 1e-6:
        height = (0.72, 0.82, 0.76, 0.88)[index % 4]
        color = (0.34, 0.22, 0.13) if index % 3 else (0.42, 0.28, 0.16)
        for side, label in ((-1, "Near"), (1, "Far")):
            py = y + side * 2.35
            cylinder(f"/World/Course/Boundary/{label}_Picket_{index:02d}",
                     0.045, height, "Z", (x, py, height / 2), color,
                     collision=(index % 3 == 0))
            cone(f"/World/Course/Boundary/{label}_PicketCap_{index:02d}",
                 0.046, 0.10, (x, py, height + 0.045), color)
        x += 0.30
        index += 1


def shrub(path, x, y, scale=1.0):
    for i, (dx, dy, radius) in enumerate(((-0.20, 0.0, 0.34), (0.16, 0.08, 0.38),
                                          (0.0, -0.16, 0.31)), start=1):
        cone(f"{path}_{i}", radius * scale, 0.52 * scale,
             (x + dx * scale, y + dy * scale, 0.26 * scale),
             (0.12, 0.29 + 0.02 * (i % 2), 0.15))


def mine_target(target_id, color_name, x, y):
    color = (0.88, 0.12, 0.08) if color_name == "red" else (0.96, 0.70, 0.06)
    prefix = f"/World/Course/Minefield/ActiveTarget_{target_id}_{color_name.title()}"
    # A visible socket, plug, and colored lead. The lead is visual-only; the
    # mission sequence and physical unplug/grasp evaluation are separate.
    box(f"{prefix}/Socket", (0.22, 0.16, 0.07), (x, y, 0.055), (0.16, 0.17, 0.18), True)
    box(f"{prefix}/Plug", (0.065, 0.065, 0.075), (x + 0.08, y, 0.13), color, True)
    cylinder(f"{prefix}/Cap", 0.035, 0.018, "Z", (x + 0.08, y, 0.177), (0.10, 0.10, 0.10), True)
    curve(f"{prefix}/Lead", [(x + 0.06, y + 0.02, 0.12),
                              (x - 0.04, y + 0.10, 0.08),
                              (x - 0.13, y + 0.06, 0.08)], color, 0.016)
    prim = __import__("omni.usd", fromlist=["get_context"]).get_context().get_stage().GetPrimAtPath(prefix)
    prim.SetCustomDataByKey("competition:target_kind", "active_wire")
    prim.SetCustomDataByKey("competition:wire_color", color_name)
    prim.SetCustomDataByKey("competition:sequence_index", -1)


def decoy(target_id, x, y):
    prefix = f"/World/Course/Minefield/Decoy_{target_id}"
    cylinder(f"{prefix}/Body", 0.09, 0.12, "Z", (x, y, 0.06), (0.26, 0.29, 0.30), True)
    cylinder(f"{prefix}/Cap", 0.105, 0.025, "Z", (x, y, 0.132), (0.14, 0.15, 0.16), True)
    prim = __import__("omni.usd", fromlist=["get_context"]).get_context().get_stage().GetPrimAtPath(prefix)
    prim.SetCustomDataByKey("competition:target_kind", "decoy")


def supply_station():
    x = 25.25
    box("/World/Course/Medical/TableTop", (1.35, 0.92, 0.10),
        (x, Y, 0.78), (0.88, 0.89, 0.86), True)
    for i, dx in enumerate((-0.56, 0.56)):
        for j, dy in enumerate((-0.37, 0.37)):
            box(f"/World/Course/Medical/TableLeg_{i}_{j}", (0.07, 0.07, 0.74),
                (x + dx, Y + dy, 0.37), (0.84, 0.85, 0.82), True)
    # Open-topped receiving kit: floor plus four low walls leaves a clear drop-in area.
    kit_x, kit_y = x + 0.34, Y + 0.08
    box("/World/Course/Medical/FirstAidKit_Floor", (0.64, 0.48, 0.04),
        (kit_x, kit_y, 0.94), (0.88, 0.10, 0.08), True)
    for i, (dx, dy, size) in enumerate(((0, -0.23, (0.64, 0.025, 0.18)),
                                         (0, 0.23, (0.64, 0.025, 0.18)),
                                         (-0.31, 0, (0.025, 0.44, 0.18)),
                                         (0.31, 0, (0.025, 0.44, 0.18)))):
        box(f"/World/Course/Medical/FirstAidKit_Wall_{i}", size,
            (kit_x + dx, kit_y + dy, 1.03), (0.88, 0.10, 0.08), True)
    # Illustrative items only; exact list/order was not published in the files.
    items = (("Bandage", (24.78, Y - 0.19, 0.91), (0.16, 0.14, 0.11), (0.91, 0.91, 0.85)),
             ("Medicine", (25.10, Y - 0.19, 0.91), (0.14, 0.14, 0.13), (0.84, 0.12, 0.12)),
             ("Saline", (25.42, Y - 0.19, 0.91), (0.11, 0.11, 0.17), (0.16, 0.64, 0.24)),
             ("Dressing", (25.74, Y - 0.19, 0.91), (0.17, 0.12, 0.10), (0.10, 0.28, 0.86)))
    for name, pos, size, color in items:
        box(f"/World/Course/Medical/Item_{name}", size, pos, color, True)
        prim = __import__("omni.usd", fromlist=["get_context"]).get_context().get_stage().GetPrimAtPath(f"/World/Course/Medical/Item_{name}")
        prim.SetCustomDataByKey("competition:target_kind", "medical_supply")
        prim.SetCustomDataByKey("competition:item_name", name.lower())


def spawn_course():
    # Global x coordinates align this independent scene to the forest exit and
    # the already built full-course task stations.
    box("/World/Course/Minefield/GrassBed", (13.7, 4.35, 0.04),
        ((START_X + END_X) / 2, Y, 0.018), (0.30, 0.40, 0.24), False)
    box("/World/Course/Minefield/SoilTaskPatch", (6.7, 3.95, 0.035),
        (18.75, Y, 0.039), (0.40, 0.31, 0.20), False)
    box("/World/Course/Medical/StationPatch", (2.7, 2.0, 0.03),
        (25.2, Y, 0.043), (0.34, 0.39, 0.26), False)
    for name, x, color in (("ForestExit", START_X, (0.10, 0.74, 0.18)),
                           ("MinefieldEnd", 22.7, (0.94, 0.66, 0.06)),
                           ("MedicalFinish", END_X, (0.10, 0.28, 0.90))):
        box(f"/World/Course/Markers/{name}", (0.08, 2.0, 0.012),
            (x, Y, 0.045), color, False)
    fence(START_X, END_X, Y)

    # Low shrubs and loose-rock surrogates frame the detection/pulling area.
    shrub("/World/Course/Minefield/Shrub_A", 16.8, Y + 1.72, 0.78)
    shrub("/World/Course/Minefield/Shrub_B", 20.0, Y - 1.70, 0.74)
    rng = random.Random(20261019)
    rock_spots = ((16.0, -1.45), (16.6, 1.35), (17.3, -1.40), (18.1, 1.50),
                  (18.8, -1.48), (19.5, 1.45), (20.3, -1.38), (21.1, 1.42))
    for i, (x, offset) in enumerate(rock_spots, start=1):
        sx = 0.22 + rng.random() * 0.14
        sy = 0.18 + rng.random() * 0.12
        sz = 0.08 + rng.random() * 0.05
        box(f"/World/Course/Minefield/Rock_{i:02d}", (sx, sy, sz),
            (x, Y + offset, sz / 2), (0.28, 0.29, 0.30), True)

    targets = (("R1", "red", 16.45, Y - 0.62), ("Y1", "yellow", 17.55, Y + 0.52),
               ("R2", "red", 18.65, Y + 0.94), ("Y2", "yellow", 19.55, Y - 0.80),
               ("R3", "red", 20.45, Y + 0.22))
    for target in targets:
        mine_target(*target)
    for target_id, x, offset in (("A", 16.95, 0.0), ("B", 18.0, -0.18),
                                 ("C", 19.92, 0.94), ("D", 20.85, -0.38)):
        decoy(target_id, x, Y + offset)

    supply_station()


def make_g1_cfg():
    spawn = sim_utils.UsdFileCfg(
        usd_path=str(G1_USD), activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False, retain_accelerations=False,
            linear_damping=0.0, angular_damping=0.0,
            max_linear_velocity=1000.0, max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True, solver_position_iteration_count=8,
            solver_velocity_iteration_count=4,
        ),
    )
    actuators = {
        "hip_pitch_yaw_waist_yaw": ImplicitActuatorCfg(
            joint_names_expr=[".*_hip_pitch_joint", ".*_hip_yaw_joint", "waist_yaw_joint"],
            effort_limit_sim=88.0, velocity_limit_sim=32.0,
            stiffness=40.17923863450712, damping=2.557889775413375,
            armature=0.01017752004132231,
        ),
        "hip_roll_knee": ImplicitActuatorCfg(
            joint_names_expr=[".*_hip_roll_joint", ".*_knee_joint"],
            effort_limit_sim=139.0, velocity_limit_sim=20.0,
            stiffness=99.09842777666111, damping=6.308801853496639,
            armature=0.025101924999999997,
        ),
        "ankles_waist": ImplicitActuatorCfg(
            joint_names_expr=[".*_ankle_.*_joint", "waist_roll_joint", "waist_pitch_joint"],
            effort_limit_sim=50.0, velocity_limit_sim=37.0,
            stiffness=28.50124619574858, damping=1.814445686848846,
            armature=0.00721945,
        ),
        "arms": ImplicitActuatorCfg(
            joint_names_expr=[".*_shoulder_.*_joint", ".*_elbow_joint", ".*_wrist_roll_joint"],
            effort_limit_sim=25.0, velocity_limit_sim=37.0,
            stiffness=14.25062309787429, damping=0.907222843292423,
            armature=0.003609725,
        ),
        "wrist_pitch_yaw": ImplicitActuatorCfg(
            joint_names_expr=[".*_wrist_(pitch|yaw)_joint"], effort_limit_sim=5.0,
            velocity_limit_sim=22.0, stiffness=16.77832748089279,
            damping=1.06814150219, armature=0.00425,
        ),
    }
    initial_joint_pos = {
        ".*_hip_pitch_joint": -0.10,
        ".*_knee_joint": 0.30,
        ".*_ankle_pitch_joint": -0.20,
        ".*_shoulder_pitch_joint": 0.35,
        "left_shoulder_roll_joint": 0.18,
        "right_shoulder_roll_joint": -0.18,
        ".*_elbow_joint": 0.87,
    }
    return ArticulationCfg(
        prim_path="/World/G1",
        spawn=spawn,
        init_state=ArticulationCfg.InitialStateCfg(pos=(args.g1_x, Y, 0.80),
                                                    joint_pos=initial_joint_pos),
        actuators=actuators,
        soft_joint_pos_limit_factor=0.9,
    )


def main():
    if args.g1_minefield_autonomy and args.rgbd_diagnostics:
        raise ValueError("Choose either read-only diagnostics or autonomous navigation")
    if args.g1_minefield_autonomy and not 0.20 <= args.minefield_speed <= 0.30:
        raise ValueError("official G1 walking speed must be in [0.20, 0.30] m/s")
    if args.minefield_timeout <= 0 or args.minefield_lane_width <= 1.0:
        raise ValueError("minefield timeout must be positive and lane width must exceed 1.0 m")
    sim = sim_utils.SimulationContext(
        sim_utils.SimulationCfg(dt=0.005, device=args.device,
                                render_interval=10 if (args.rgbd_diagnostics or args.g1_minefield_autonomy) else 4)
    )
    ground = sim_utils.GroundPlaneCfg(
        color=(0.26, 0.34, 0.20), size=(100.0, 100.0),
        physics_material=sim_utils.RigidBodyMaterialCfg(
            static_friction=1.0, dynamic_friction=0.8, restitution=0.0,
            friction_combine_mode="multiply", restitution_combine_mode="multiply",
        ),
    )
    ground.func("/World/Ground", ground)
    sim_utils.DomeLightCfg(intensity=1800.0, color=(0.78, 0.80, 0.84)).func(
        "/World/DomeLight", sim_utils.DomeLightCfg(intensity=1800.0, color=(0.78, 0.80, 0.84))
    )
    spawn_course()
    robot = Articulation(make_g1_cfg())
    rgbd = task_stack = None
    if args.rgbd_diagnostics or args.g1_minefield_autonomy:
        import sys
        sys.path.insert(0, str(ROOT))
        sys.path.insert(0, str(ROOT / "task_logic"))
        from g1_rgbd_sensor import RobotRGBD
        # Sensor prims must exist before sim.reset() so Isaac Lab initializes
        # their render products and annotators with the rest of the scene.
        rgbd = RobotRGBD()
        if args.rgbd_diagnostics:
            from g1_task_stack import G1TaskStack
            task_stack = G1TaskStack.from_file(ROOT / "task_logic" / "g1_mission_config.template.json")
            print("G1_TASK_RGBD_DIAGNOSTICS_READY config=empty actions=disabled", flush=True)
    sim.reset()
    robot.reset()
    robot.write_root_state_to_sim(robot.data.default_root_state.clone())
    robot.write_joint_state_to_sim(robot.data.default_joint_pos.clone(),
                                   robot.data.default_joint_vel.clone())
    robot.set_joint_position_target(robot.data.default_joint_pos)
    robot.write_data_to_sim()
    sim.step(render=False)
    robot.update(sim.cfg.dt)
    sim.set_camera_view(eye=(22.0, -8.5, 10.5), target=(22.0, Y, 0.65))

    args.scene_out.mkdir(parents=True, exist_ok=True)
    stage = __import__("omni.usd", fromlist=["get_context"]).get_context().get_stage()
    scene_path = args.scene_out / "g1_minefield_medical.usd"
    stage.GetRootLayer().Export(str(scene_path))
    check = Usd.Stage.Open(str(scene_path))
    assert check and check.GetPrimAtPath("/World/G1").IsValid(), "G1 reference missing from exported scene"
    names = [prim.GetName() for prim in check.Traverse()]
    counts = {
        "active_wire_targets": sum(name.startswith("ActiveTarget_") for name in names),
        "decoys": sum(name.startswith("Decoy_") for name in names),
        "medical_items": sum(name.startswith("Item_") for name in names),
    }
    assert counts == {"active_wire_targets": 5, "decoys": 4, "medical_items": 4}, counts
    spec = {
        "scene": "G1 minefield reconnaissance and traversal; medical props are a later stage",
        "scene_usd": str(scene_path),
        "robot": "official Unitree G1 29-DoF no-hand articulation",
        "coordinate_frame": {"x": "course-forward", "y": "cross-course", "z": "up"},
        "continuity": {"forest_exit_x_m": 15.0, "g1_start_x_m": args.g1_x,
                       "minefield_x_m": [15.0, 22.7], "medical_station_x_m": 25.25,
                       "scene_end_x_m": END_X, "lane_width_m": 4.7},
        "official_requirements": [
            "After the start command, identify and remove colored wires in the required order.",
            "The slide reference describes red/yellow targets and decoy targets.",
            "Then pick supplies in the required list order and place them into the designated first-aid kit.",
        ],
        "configurable_by_judge": {
            "wire_color_sequence": None,
            "medical_supply_order": None,
            "note": "The supplied official files do not specify these exact sequences or item lists.",
        },
        "geometry_status": "visual estimate from the official reference image; no surveyed CAD dimensions",
        "scene_scope": "independent stage; existing forest demonstration USD and source are untouched",
        "implemented": ["G1 start pose", "lane boundaries", "rocks and shrubs", "five red/yellow wire targets",
                        "four neutral decoys", "table", "open receiving kit", "four labeled supply proxies",
                        "standalone RGB-D color proposals", "live paused-preview camera diagnostic loop",
                        "ordered fail-safe task-state-machine scaffold",
                        "RGB-D mapped minefield navigation using official Unitree velocity policy"],
        "not_yet_implemented": ["real-scene color calibration",
                                "hand/manipulator action executor", "wire unplugging and supply placement"],
        "current_navigation_scope": "walk from forest exit through minefield to exit marker; avoid obstacles; log red/yellow candidates; manipulation disabled",
    }
    (args.scene_out / "g1_minefield_medical_spec.json").write_text(
        json.dumps(spec, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    print(f"SAVED_SCENE {scene_path}", flush=True)
    print(f"G1 joints={robot.num_joints} bodies={robot.num_bodies} start={args.g1_x:.2f},{Y:.2f}", flush=True)
    print(f"USD_VALIDATION {json.dumps(counts, sort_keys=True)}", flush=True)

    if args.g1_minefield_autonomy:
        from g1_minefield_runtime import run_minefield_policy
        run_minefield_policy(sim, robot, app, args, ROOT, rgbd)
    elif args.preview:
        frame_id = 0
        last_capture = 0.0
        steps = 0
        if args.rgbd_diagnostics:
            print("G1_TASK_RGBD_PREVIEW_ENTER physics=200Hz render=20Hz camera=10Hz", flush=True)
        else:
            sim.pause()
        while app.is_running():
            step_started = time.perf_counter()
            if rgbd is None:
                sim.render()
                time.sleep(1.0 / 30.0)
                continue

            # Camera products in Isaac Sim 4.5 need active physics/render ticks;
            # a paused Kit viewport alone leaves Camera.update waiting for a frame.
            if steps == 0:
                print("G1_TASK_FIRST_STEP_BEGIN", flush=True)
            robot.write_data_to_sim()
            if steps == 0:
                print("G1_TASK_FIRST_STEP_DATA_READY", flush=True)
            sim.step(render=False)
            if steps == 0:
                print("G1_TASK_FIRST_STEP_PHYSICS_READY", flush=True)
            robot.update(sim.cfg.dt)
            steps += 1
            if steps % sim.cfg.render_interval == 0:
                sim.render()
            seconds = steps * sim.cfg.dt
            if seconds - last_capture >= .10:
                frame_id += 1
                if frame_id == 1:
                    print("G1_TASK_FIRST_CAMERA_READ_BEGIN", flush=True)
                frame = rgbd.read(seconds, .10)
                result = task_stack.update(frame, {
                    "healthy": True, "sensor_age_s": 0.0,
                    "fallen": False, "e_stop": False,
                })
                if frame_id == 1:
                    print(f"G1_TASK_FIRST_CAMERA_READ_READY shape={frame['rgb'].shape}", flush=True)
                if frame_id % 10 == 0:
                    proposals = [{"label": d["label"], "track_id": d["track_id"],
                                  "confidence": round(d["confidence"], 2),
                                  "position_base_m": [round(x, 2) for x in d["position_base"]]}
                                 for d in result["detections"]]
                    print("G1_TASK_RGBD_FRAME " + json.dumps({
                        "frame": frame_id, "proposals": proposals,
                        "mission": result["mission"],
                    }, ensure_ascii=False), flush=True)
                last_capture = seconds
            time.sleep(max(0.0, sim.cfg.dt - (time.perf_counter() - step_started)))
    else:
        for _ in range(max(1, args.frames)):
            robot.write_data_to_sim()
            sim.step(render=False)
            robot.update(sim.cfg.dt)


if __name__ == "__main__":
    try:
        main()
    finally:
        if args.headless:
            # Kit 4.5 can hang during headless teardown after a successful export.
            import os
            import sys
            import threading
            exit_code = 1 if sys.exc_info()[0] else 0
            timer = threading.Timer(5.0, lambda: os._exit(exit_code))
            timer.daemon = True
            timer.start()
        app.close(wait_for_replicator=False)
