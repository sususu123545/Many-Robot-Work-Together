import json, shutil, sys
from pathlib import Path
RUN=Path(__file__).resolve().parent
ROOT=RUN.parents[3]
sys.path.insert(0,str(ROOT/'scripts'))
from workorder_archive import seal
d=json.loads((RUN/'t07_redball_grasp_plan.py.stdout.json').read_bytes())
a,b=d['arm'],d['base']
tool=d['current_tool_pose']['translation_m']
ball=d['live_tf_candidate_base_xyz']
height=tool[2]-(ball[2]-.019)
out={
 'run_id':RUN.name,'user_instruction':'Proceed directly with grasp; authorization accepted, guards retained.',
 'robot':'pi@192.0.2.1','motion_commands_sent':{'arm':0,'gripper':0,'base':0},
 'fresh_target_samples':d['camera_point_count'],'target_age_seconds':d['target_age_seconds'],
 'temperatures_C':{k:v['temperature'] for k,v in a['joints'].items()},
 'feedback_age_seconds':a['feedback_age_seconds'],
 'competing_publishers':d['competing_command_publishers'],
 'model_tool_translation_m':tool,'model_ball_translation_m':ball,
 'model_initial_tool_floor_clearance_m':height,'existing_minimum_clearance_m':.014,
 'physical_finger_floor_clearance_measured':False,
 'planner_error':'ValueError: Initial floor clearance too small for this attempt',
 'planner_returncode':1,'trajectory_generated':False,'motion_permissions_changed':False,
 'safety_observed':{'arm_estop':a['estop'],'base_estop':b['estop'],
  'arm_motion':a['enable_arm_motion'],'gripper_motion':a['enable_gripper_motion'],'wheels':b['wheel_command']},
 'acceptance':{'fresh_visual_target':'passed','fresh_feedback':'passed','candidate_pick_path':'failed_existing_initial_clearance_guard','grasp_lift_hold_release':'not_attempted'},
 'status':'preflight_failed_no_motion','grasp_success':False,
 'unresolved':['Physical lowest finger point, grasp center and floor are not independently measured.',
  'Prior floor contact and post-STOP posture drop remain unresolved.',
  'No lowered guard, old coordinate reuse or unguarded trajectory was used.'],
 'next_evidence':'Side view with ball, both fingers, floor and external millimeter reference before defining a physical approach.',
 'physical_success_training_export':False,
}
(RUN/'progress_result.json').write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
entry=f'''\n\n2026-11-17 用户要求直接抓球后新一轮预检 `{RUN.name}`：接受当前抓取授权，保留原安全检查。新相机完整红球10个点、末点年龄约0.051秒；六关节28–29℃，无竞争运动发布者。实际运行既有t07_prepare_pick_attempt.py，其prepare_pick_paths因初始模型工具点距估计地面{height*1000:.2f}毫米、小于原14毫米门槛而返回1（Initial floor clearance too small for this attempt），未生成轨迹。该值仅为模型估算，不代表夹指实测间隙。无任何机械臂、夹爪或底盘运动指令，无复位、许可或保护阈值修改；只读观察双端急停true、臂/夹爪许可false、轮指令全0。此前夹指触地及STOP后回落仍未解决，等待独立侧面几何测量后确定动作，不降低门槛继续下降。抓球未完成，T07不结单，成功训练导出关闭。证据logs/t07/runs/{RUN.name}/progress_result.json、camera.jpg、原始只读输出及commands内失败命令/源码/错误输出。\n'''
for relative in ('VLA_30天推进工单.md','docs/t07_progress.md','docs/t07_grasp_goal_progress.md'):
 with (ROOT/relative).open('a',encoding='utf-8') as f:f.write(entry)
 dest=RUN/'final_inputs'/relative
 dest.parent.mkdir(parents=True,exist_ok=True)
 shutil.copy2(ROOT/relative,dest)
seal(RUN,out['status'],'Existing candidate planner rejected initial clearance; zero motion; physical grasp pending independent clearance evidence.')
print(json.dumps({'status':out['status'],'model_clearance_mm':height*1000,'motion_commands':0,'sealed':True}))
