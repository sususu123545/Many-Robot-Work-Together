"""Offline closure of the interrupted November 17 attempt; no robot access."""
import json,shutil,sys
from pathlib import Path
RUN=Path(__file__).resolve().parent;ROOT=RUN.parents[3]
sys.path.insert(0,str(ROOT/'scripts'))
from workorder_archive import event,seal
if (RUN/'progress_result.json').exists():
 print('Already summarized; leave existing result unchanged.');raise SystemExit(0)
d=json.loads((RUN/'hold_bounded_hold.stdout.json').read_bytes())
p=json.loads((RUN/'hold_t07_redball_grasp_plan.stdout.json').read_bytes())
a,b=p['arm'],p['base'];dispatch=next(e for e in d['events'] if e['kind']=='dispatch')
initial=d['initial_raw'];ns=dispatch['stamp']['sec']*10**9+dispatch['stamp']['nanosec']
samples=[]
for e in d['events']:
 if e['kind']=='arm' and e['receive_ros_ns']>=ns:
  samples.append({'dt_seconds':(e['receive_ros_ns']-ns)*1e-9,'raw':{k:v['raw'] for k,v in e['data']['joints'].items()},'estop':e['data']['estop']})
result={'run_id':RUN.name,'offline_summary_date_local':'2026-11-18','observation_date_local':'2026-11-17',
 'current_live_state_verified':False,'source_identity':'Both program files match previous sealed manifest; normalized text hash and CRLF byte hash distinguished.',
 'user_authorization':'User requested execution after the hold-recovery proposal and prior rejection were disclosed.',
 'initial_local_attempt':'Cancelled at getpass before authentication after source-hash check used incompatible newline representations; no remote commands.',
 'motion_commands_sent':{'arm':d['movement_commands'],'gripper':d['gripper_commands'],'base':d['chassis_commands']},
 'dispatch':dispatch,'initial_raw':initial,'error':d.get('error'),'passed':d['passed'],'cleanup_confirmed':d['cleanup_confirmed'],
 'feedback_samples':samples,'post_raw':p.get('current_raw'),
 'post_safety_observed':{'arm_estop':a['estop'],'base_estop':b['estop'],'enable_arm_motion':a['enable_arm_motion'],'enable_gripper_motion':a['enable_gripper_motion'],'wheel_command':b['wheel_command']},
 'post_temperatures_C':{k:v['temperature'] for k,v in a['joints'].items()},
 'acceptance':{'native_estop_reset':'acknowledged','physical_home':'not_completed','upward_recovery':'failed_drift_guard','grasp_lift_hold_release':'not_attempted'},
 'unresolved':['Held elbow encoder changed from723 to727; cause not established.','No independently calibrated home trajectory.','Prior floor contact and post-stop holding behavior unresolved.'],
 'grasp_success':False,'physical_success_training_export':False,'status':'recovery_failed_stopped_archived'}
(RUN/'progress_result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
verification=RUN/'source_verification.json'
v=json.loads(verification.read_text(encoding='utf-8'));v['new_user_authorization']=result['user_authorization'];verification.write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
entry=f'''\n\n2026-11-18 补齐11月17日中断尝试 `{RUN.name}` 的离线归档（本次未连接真机、无新运动）：用户在保持策略风险说明后要求执行复位程序，审批获准后运行上一轮封存候选的逐字节相同副本。首次本地哈希检查混淆LF文本与CRLF文件字节，已在输入口令、建立SSH前取消并记录；随后依据封存manifest核验源码一致。实际仅1条臂指令：保持其他臂关节，肩225→245 ticks/3秒；保持目标的肘反馈723→727，超过原3ticks偏移守卫，立即中止。夹爪/底盘指令0。清理及后续只读记录确认双端急停true、臂/夹爪许可false、轮指令全0；最后只读肩227、肘727。该历史状态不代表11月18日实时状态。未完成物理归位、闭球或抬球，T07不结单、成功训练导出关闭；不据此判定具体硬件或通信故障，不放宽守卫重试。证据logs/t07/runs/{RUN.name}/progress_result.json、hold_bounded_hold.stdout.json、停止后检查输出、照片、命令/源码及哈希。\n'''
for rel in ('VLA_30天推进工单.md','docs/t07_progress.md','docs/t07_grasp_goal_progress.md'):
 with (ROOT/rel).open('a',encoding='utf-8') as f:f.write(entry)
 dest=RUN/'final_inputs'/rel;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(ROOT/rel,dest)
event(RUN,'interrupted_attempt_offline_closure',observation_date_local='2026-11-17',summary_date_local='2026-11-18',new_robot_commands=0)
seal(RUN,result['status'],'One arm command stopped on held-joint drift; original cleanup verified. Offline closure only; physical home and ball grasp incomplete.')
print(json.dumps({'sealed':True,'prior_arm_commands':d['movement_commands'],'new_robot_commands':0,'grasp_success':False}))
