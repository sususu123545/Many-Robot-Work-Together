import importlib.util,json,math,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def load(name,path):
 spec=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m
k=load('physical_geometry',ROOT/'logs/t07/runs/20261116T092847Z_8844851d/candidate_sources/52_arm_kinematics.py');p=load('pick_paths',ROOT/'scripts/t07_pick_paths.py');p.forward_tool_pose_raw=k.forward_tool_pose_raw;p.inverse_kinematics_all=k.inverse_kinematics_all
report=ROOT/'logs/t07/runs/20261117T050800Z_f8ef182f/native_houghplan_6cec88ea/stdout.log';d=json.loads(report.read_text());raw=d['current_raw'];ball=d['live_tf_candidate_base_xyz'];comp={'joint1':-3,'joint2':4,'joint3':-9,'joint4':7}
class Tests(unittest.TestCase):
 def test_recorded_scene_entire_pick_is_bounded(self):
  plan=p.prepare_pick_paths(raw,ball,comp);self.assertLessEqual(sum(map(len,plan['paths'].values())),24)
  for phase,path in plan['paths'].items():
   self.assertTrue(path)
   for row in path:self.assertGreaterEqual(row['desired_tcp'][2],plan['floor_m']+row['minimum_clearance'])
 def test_recovery_moves_up_from_low_standoff(self):
  plan=p.prepare_pick_paths(raw,ball,comp);self.assertGreater(plan['paths']['recover'][-1]['desired_tcp'][2],plan['initial_tcp'][2]+.023)
 def test_contact_and_lift_targets_remain_separate(self):
  plan=p.prepare_pick_paths(raw,ball,comp);contact=plan['paths']['contact'][-1]['desired_tcp'];lift=plan['paths']['lift'][-1]['desired_tcp'];self.assertLess(math.dist(contact,ball),.002);self.assertAlmostEqual(lift[2]-contact[2],.020,delta=.002)
 def test_impossible_target_rejected_before_any_interface(self):
  with self.assertRaises(ValueError):p.prepare_pick_paths(raw,[.5,.5,-.1],comp)
 def test_floor_penetration_rejected(self):
  rr=dict(raw);floor=p.tool(rr)[2]+.001
  with self.assertRaises(ValueError):p.make_path(rr,p.tool(rr),p.current_pitch(rr),comp,floor,.012)
 def test_unbounded_command_compensation_rejected(self):
  wrong=dict(comp);wrong['joint3']=-11
  with self.assertRaises(ValueError):p.prepare_pick_paths(raw,ball,wrong)
if __name__=='__main__':unittest.main()
