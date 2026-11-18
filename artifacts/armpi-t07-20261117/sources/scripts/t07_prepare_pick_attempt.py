"""Produce one reviewable bounded pick program from a fresh complete target."""
import argparse,json,importlib.util,math,hashlib
from pathlib import Path
from t07_client_temperature_patch import attach_temperature_observer
ROOT=Path(__file__).resolve().parents[1]
def main():
 p=argparse.ArgumentParser();p.add_argument('--report',required=True);p.add_argument('--preview',required=True);a=p.parse_args();d=json.loads((ROOT/a.report).read_text(encoding='utf-8'))
 if d.get('error') or d.get('camera_point_count',0)<5 or d.get('target_complete_sphere_verified') is not True or not 0<=d.get('target_age_seconds',999)<=.5 or d.get('live_offline_distance_m',1)>.005:raise ValueError('Fresh complete target and matching measured geometry required')
 comp={'joint1':-3,'joint2':4,'joint3':-9,'joint4':7};geometry={'source_robot_ros_ns':d['source_robot_ros_ns'],'expected_raw':d['current_raw'],'ball':d['live_tf_candidate_base_xyz'],'compensation':comp,'source_report':a.report,'source_camera_point':d['latest_camera_point'],'static_scene_assumption':'One finite attempt on the last complete observed sphere; no base motion or manual scene change; no vision refresh claimed during final approach'}
 source=ROOT/'logs/t07/runs/20261116T092847Z_8844851d/candidate_sources/52_arm_kinematics.py';spec=importlib.util.spec_from_file_location('geo',source);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
 pure=(ROOT/'scripts/t07_pick_paths.py').read_text(encoding='utf-8');scope={'forward_tool_pose_raw':m.forward_tool_pose_raw,'inverse_kinematics_all':m.inverse_kinematics_all};exec(pure,scope);preview=scope['prepare_pick_paths'](d['current_raw'],geometry['ball'],comp)
 base=(ROOT/'scripts/t07_redball_pregrasp_stage.py').read_text(encoding='utf-8');prefix=base[:base.index('\nPLAN=')]
 prefix=prefix.replace("'t07_guarded_pregrasp_stage'","'t07_guarded_pick_'+__import__('uuid').uuid4().hex[:8]").replace('def gates(arm):','def gates(arm,gripper=False):').replace("Parameter('enable_gripper_motion',value=False)","Parameter('enable_gripper_motion',value=gripper)").replace('signal.alarm(60)','signal.alarm(90)')
 prefix=attach_temperature_observer(prefix)
 runtime=(ROOT/'scripts/t07_pick_runtime.txt').read_text(encoding='utf-8')
 runtime=runtime.replace("{k:v['temperature'] for k,v in a['joints'].items()}","{k:control_temperature(k,a)['value'] for k,v in a['joints'].items()}").replace("a['joints'][k]['temperature']","control_temperature(k,a)['value']").replace("a['joints']['r_joint']['temperature']","control_temperature('r_joint',a)['value']")
 text=prefix+'\n'+pure+'\nGEOMETRY='+repr(geometry)+'\n'+runtime;compile(text,'guarded_pick_attempt','exec')
 (ROOT/'scripts/t07_guarded_pick_attempt.py').write_bytes(text.encode());preview.update(geometry=geometry,source_sha256=hashlib.sha256(text.encode()).hexdigest(),kinematics_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),scope='Candidate only; online feedback and source age checks still required')
 (ROOT/a.preview).write_text(json.dumps(preview,indent=2)+'\n',encoding='utf-8');print(json.dumps({'prepared':True,'arm_waypoints':sum(map(len,preview['paths'].values())),'geometry_report':a.report,'source_sha256':preview['source_sha256']}))
if __name__=='__main__':main()
