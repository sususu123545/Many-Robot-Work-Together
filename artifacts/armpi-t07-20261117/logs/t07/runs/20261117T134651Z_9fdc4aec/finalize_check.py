"""Summarize measured read-only results and preserve work-order evidence."""
import json, shutil, sys
from pathlib import Path
RUN = Path(__file__).resolve().parent
ROOT = RUN.parents[3]
sys.path.insert(0, str(ROOT / 'scripts'))
from workorder_archive import seal

query = json.loads((RUN / 't07_native_query.py.stdout.json').read_bytes())
geo = json.loads((RUN / 't07_redball_grasp_plan.py.stdout.json').read_bytes())
arm, base = geo['arm'], geo['base']
result = {
    'run_id': RUN.name, 'robot': 'pi@192.0.2.1',
    'operator_confirmation': 'User confirmed present, can stop, area clear, ball placed in this chat before inspection.',
    'motion_commands_sent': {'arm': 0, 'gripper': 0, 'chassis': 0},
    'control_services_called': 0,
    'latest_raw': {k: v['raw'] for k, v in arm['joints'].items()},
    'temperatures_C': {k: v['temperature'] for k, v in arm['joints'].items()},
    'feedback_age_seconds': arm['feedback_age_seconds'],
    'fault': arm['fault'], 'last_read_error': arm['current_read_error'],
    'safety_state_observed': {'arm_estop': arm['estop'], 'base_estop': base['estop'],
        'enable_arm_motion': arm['enable_arm_motion'], 'enable_gripper_motion': arm['enable_gripper_motion'],
        'wheel_command': base['wheel_command']},
    'competing_publishers': geo['competing_command_publishers'],
    'complete_target': geo.get('target_complete_sphere_verified'),
    'target_samples': geo.get('camera_point_count'), 'target_age_seconds': geo.get('target_age_seconds'),
    'camera_stamp': query.get('image_stamp'),
    'model_tool_translation_m': geo['current_tool_pose']['translation_m'],
    'model_ball_translation_m': geo.get('live_tf_candidate_base_xyz'),
    'model_tf_disagreement_m': geo.get('live_offline_distance_m'),
    'model_limitation': 'Internal TF agreement does not measure physical jaw center, lowest finger point, floor clearance or external localization accuracy.',
    'visual_observation': 'Complete red ball on tiled floor; fingers partially visible at bottom edge. Camera view does not establish side clearance or grasp alignment.',
    'acceptance': {'ssh': 'passed', 'fresh_feedback': 'passed', 'whole_ball_detected': 'passed',
        'physical_finger_floor_clearance': 'pending_external_measurement',
        'post_stop_holding': 'pending', 'grasp_lift_hold_release': 'not_attempted'},
    'status': 'awaiting_external_clearance_measurement', 'grasp_success': False,
    'next_required_evidence': 'Side view including red ball, both fingers, floor and an external millimeter ruler to establish physical lowest-point clearance and grasp-center height. No hand beneath arm or between fingers.',
    'training_success_export_enabled': False,
}
(RUN / 'progress_result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
entry = f'''\n\n2026-11-17 新SSH连接后抓球预检 `{RUN.name}`：用户重新确认在场可停止、范围清空且球已摆好。pi@192.0.2.1连接成功；六关节真实反馈年龄0.021–0.225秒、温度28–29℃，当前fault为null，无其他臂/夹爪/底盘运动发布者。完整红球检测9个有效点，末点年龄约0.119秒；相机图已保存。只读观察底盘/臂急停true、臂/夹爪许可false、轮指令全0。该轮未发任何运动、急停复位、扭矩或标定写入。相机仅见夹指局部，不能验证夹指最低点/真实夹持中心与地面的外部几何；历史触地和STOP后回落问题仍未解决，不能用同一模型的TF一致性代替实测或复用旧下降轨迹。等待包含红球、两夹指、地面及毫米尺的侧面测量证据后确定动作。抓球未完成，T07不结单，成功训练导出关闭。证据：`logs/t07/runs/{RUN.name}/progress_result.json`、`camera.jpg`及两份只读脚本原始输出/命令/源码/哈希。\n'''
for relative in ('VLA_30天推进工单.md', 'docs/t07_progress.md', 'docs/t07_grasp_goal_progress.md'):
    with (ROOT / relative).open('a', encoding='utf-8') as f:
        f.write(entry)
    dest = RUN / 'final_inputs' / relative
    dest.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(ROOT / relative, dest)
seal(RUN, result['status'], 'Read-only inspection recorded; zero motion; grasp pending physical finger/floor measurements.')
print(json.dumps({'run_id': RUN.name, 'status': result['status'], 'motion_commands': 0, 'sealed': True}))
