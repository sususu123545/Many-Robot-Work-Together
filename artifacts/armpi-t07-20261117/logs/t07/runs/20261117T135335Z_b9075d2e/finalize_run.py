import json,shutil,sys
from pathlib import Path
RUN=Path(__file__).resolve().parent;ROOT=RUN.parents[3]
sys.path.insert(0,str(ROOT/'scripts'))
from workorder_archive import event,seal
motion=json.loads((RUN/'bounded_recovery.stdout.json').read_bytes())
trace=json.loads((RUN/'recovery_trace_audit.json').read_bytes())
post=json.loads((RUN/'final_readonly/t07_redball_grasp_plan.py.stdout.json').read_bytes())
reason='This would issue another real-arm movement after a prior unexpected non-commanded joint drift, using a newly modified hold strategy whose safety has not been independently validated; the original reset request does not clearly authorize this higher-risk retry.'
rejection={'action':'python logs/t07/runs/20261117T135335Z_b9075d2e/run_hold_recovery.py','decision':'rejected_before_process_creation','reason':reason,'executed':False,'workaround_used':False}
(RUN/'approval_review.json').write_text(json.dumps(rejection,indent=2)+'\n')
a,b=post['arm'],post['base']
result={
 'run_id':RUN.name,'user_request':'Reset physical arm then grasp the ball again.',
 'home_pose_available':False,'reset_service_semantics':'reset_estop only clears latch and revokes motion gates; it is not physical homing.',
 'motion_attempt':'Upward-only reset preparation, with 20tick shoulder increments at3seconds; at most3increments prepared.',
 'motion_commands_sent':{'arm':motion['movement_commands'],'gripper':motion['gripper_commands'],'base':motion['chassis_commands']},
 'requested_shoulder_raw':242,'initial_shoulder_raw':222,'initial_elbow_raw':718,
 'shoulder_raw_at_guard':231,'elbow_raw_at_guard':722,'guard':'Non-commanded joint moved >3ticks',
 'guard_time_from_dispatch_seconds':trace['samples'][-2]['robot_dt_seconds'],
 'motion_error':motion.get('error'),'measured_arrival_passed':False,
 'cleanup_confirmed':motion.get('cleanup_confirmed'),
 'final_readonly_raw':{k:v['raw'] for k,v in a['joints'].items()},
 'final_temperatures_C':{k:v['temperature'] for k,v in a['joints'].items()},
 'final_ages_seconds':a['feedback_age_seconds'],
 'final_safety':{'arm_estop':a['estop'],'base_estop':b['estop'],'arm_permission':a['enable_arm_motion'],'gripper_permission':a['enable_gripper_motion'],'wheel_command':b['wheel_command']},
 'final_competing_publishers':post['competing_command_publishers'],
 'post_ball_samples':post['camera_point_count'],'post_ball_sample_age_seconds':post['target_age_seconds'],
 'retry_prepared':'bounded_hold.py holds other arm joints at their freshly measured positions; same drift3ticks, arrival6ticks and other guards.',
 'retry_executed':False,'retry_approval_review':rejection,
 'acceptance':{'native_estop_reset_ack':'passed','physical_reset':'not_completed','upward_preparation':'failed_joint_drift_guard','grasp_lift_hold_release':'not_attempted','post_stop_readonly':'recorded'},
 'unresolved':['Cause of elbow encoder change under shoulder command unknown; do not claim mechanical failure or communication loss.',
  'Physical home and full path are not calibrated.','Prior real finger-floor contact and post-STOP holding remain unresolved.',
  'Modified hold strategy requires independent validation and explicit approval before retry.'],
 'grasp_success':False,'T07_complete':False,'physical_success_export':False,
 'status':'recovery_stopped_retry_rejected_awaiting_review',
}
(RUN/'progress_result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
event(RUN,'physical_recovery_stopped',commands=1,error=motion.get('error'),cleanup_confirmed=motion.get('cleanup_confirmed'))
event(RUN,'approval_rejected',**rejection)
entry=f'''\n\n2026-11-17 用户要求复位后重抓 `{RUN.name}`：查明原生reset_estop只清急停并关闭运动许可，不是物理归位；未发现已标定home。准备仅肩关节向上20ticks/3秒、最多3步，保留到位6ticks、其他关节偏移3ticks、温度/时效及底盘锁定检查；离线3步模型向上检查通过，不代表物理归位验收。实际仅发送1条肩指令222→242；约2.994秒处肩反馈231、未下发动作的肘718→722超过3ticks偏移守卫，立即停止。清理后肩225、肘723，急停与禁用确认。该轮夹爪/底盘指令0，未完成物理复位或抓球，不放宽保护、不归因于某种硬件/通信故障。另已准备保持其他臂关节当前位的上抬版本，但自动审批因未解释的关节变化、新保持策略未独立验证且原授权不足覆盖较高风险重试而在进程创建前拒绝，未执行、未绕过。后续独立只读确认六关节28–30℃、反馈年龄0.019–0.230秒、无竞争发布者、双端急停true、臂/夹爪许可false、轮指令全0，红球仍完整可见。等待现场检查及新策略明确审批/验证；T07不结单、无成功训练导出。完整证据logs/t07/runs/{RUN.name}/progress_result.json、recovery_trace_audit.json、approval_review.json、原始输出/照片/源码哈希；全部失败和未执行候选保留。\n'''
for rel in ('VLA_30天推进工单.md','docs/t07_progress.md','docs/t07_grasp_goal_progress.md'):
 with (ROOT/rel).open('a',encoding='utf-8') as f:f.write(entry)
 dest=RUN/'final_inputs'/rel;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(ROOT/rel,dest)
seal(RUN,result['status'],'One real arm command stopped on unexpected elbow encoder drift; cleanup verified. Hold retry rejected by auto-review before execution. Reset/grasp incomplete.')
print(json.dumps({'sealed':True,'arm_commands':1,'grasp_success':False,'cleanup_confirmed':motion.get('cleanup_confirmed'),'retry_executed':False}))
