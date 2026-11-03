import sys, json, time
from pathlib import Path
sys.path.insert(0, "/mnt/llm/g1_sim_ws/competition_sim")
import run_competition_sim as base
import isaaclab.sim as sim_utils
import torch
import omni.usd
from isaaclab.assets import Articulation
from g1_sim_runtime import make_observer, run_policy, hold_balance

args = base.args
if args.robot != "g1" or not args.g1_forest:
    raise ValueError("Run with --robot g1 --g1-forest")

source_stage = Path("/mnt/llm/g1_sim_ws/competition_sim/stages/codex_g1_visible_20261019/g1_base_course.usd")
print(f"OPENING_FULL_SCENE {source_stage}", flush=True)
context = omni.usd.get_context()
context.open_stage(str(source_stage))
for i in range(900):
    base.simulation_app.update()
    stage = context.get_stage()
    if i > 20 and stage and stage.GetPrimAtPath("/World/Course/G1").IsValid():
        break
stage = context.get_stage()
if not stage or not stage.GetPrimAtPath("/World/Course/G1").IsValid():
    raise RuntimeError("Full forest course did not finish loading")
print(f"FULL_SCENE_READY after_updates={i}", flush=True)
if stage.GetPrimAtPath("/World/G1").IsValid():
    stage.RemovePrim("/World/G1")

sim = sim_utils.SimulationContext(
    sim_utils.SimulationCfg(dt=0.005, device=args.device or "cuda:0", render_interval=4)
)
robot = Articulation(base.make_g1_cfg())
sim.reset()
robot.reset()
material_props = robot.root_physx_view.get_material_properties()
material_props[..., 0] = 1.0
material_props[..., 1] = 0.8
material_props[..., 2] = 0.0
env_ids = torch.arange(robot.num_instances, device="cpu")
robot.root_physx_view.set_material_properties(material_props, env_ids)
robot.write_root_state_to_sim(robot.data.default_root_state.clone())
robot.write_joint_state_to_sim(robot.data.default_joint_pos.clone(), robot.data.default_joint_vel.clone())
robot.set_joint_position_target(robot.data.default_joint_pos)
robot.write_data_to_sim()
sim.step(render=False)
robot.update(sim.cfg.dt)
print(f"G1_READY joints={robot.num_joints} root={robot.data.root_pos_w[0].detach().cpu().tolist()} speed={args.g1_speed}", flush=True)
sim.set_camera_view(eye=(7.4, -7.0, 10.0), target=(7.4, base.G1_Y, 0.75))
args.scene_out.mkdir(parents=True, exist_ok=True)
stage.GetRootLayer().Export(str(args.scene_out / "g1_base_course.usd"))
observer = make_observer(args.capture_dir)
started = time.time()
result, policy = run_policy(sim, robot, base.simulation_app, args, base.ROOT, observer)
(args.scene_out / "g1_forest_result.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
report = {"robot_selection":"g1","scene_usd":str(args.scene_out / "g1_base_course.usd"),
          "policy_source":args.g1_policy_source,"command_mps":args.g1_speed,
          "elapsed_wall_s":time.time()-started,"g1_policy":result}
(args.scene_out / "g1_base_smoke.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
print(f"SIM_REPORT {args.scene_out / 'g1_base_smoke.json'}", flush=True)
if base.simulation_app.is_running():
    hold_balance(sim, robot, base.simulation_app, args, policy,
                 failed=result["status"] != "forest_section_complete")
