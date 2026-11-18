import ast,copy,importlib.util,types,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];spec=importlib.util.spec_from_file_location('observer',ROOT/'scripts/t07_temperature_observer.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
tree=ast.parse((ROOT/'scripts/t07_guarded_pick_attempt.py').read_text(encoding='utf-8'));guard=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='guard')
class Tests(unittest.TestCase):
 def setup_guard(self):
  now=100.;joints={k:{'raw':500,'temperature':30,'position_time':99.8,'temperature_time':98.4} for k in m.TEMPERATURE_SERVO_IDS}
  arm={'joints':joints,'feedback_real':True,'feedback_complete':True,'arm_feedback_real':True,'arm_feedback_complete':True,'dry_run':False,'fault':None};latest={'arm':arm,'base':{'estop':True,'wheel_command':[0,0,0,0]}}
  obs={k:{'source':'physical_temperature_read','joint':k,'servo_id':i,'temperature_C':31,'temperature_time':99.9,'retained':False} for k,i in m.TEMPERATURE_SERVO_IDS.items()}
  def select(name,a):return m.observed_temperature(name,a['joints'][name],obs.get(name),now)
  env={'time':types.SimpleNamespace(monotonic=lambda:now),'latest':latest,'received':{'arm':99.95,'base':99.95},'control_temperature':select}
  exec(compile(ast.Module(body=[guard],type_ignores=[]),'actual_client_guard','exec'),env);return env['guard'],arm,obs
 def test_fresh_stream_bridges_expired_batch_temperature_without_changing_state(self):
  g,a,obs=self.setup_guard();old=copy.deepcopy(a);self.assertIs(g(True,True),a);self.assertEqual(a,old)
 def test_stream_cannot_rescue_expired_encoder(self):
  g,a,obs=self.setup_guard();a['joints']['joint1']['position_time']=98.4
  with self.assertRaises(RuntimeError):g(True,True)
 def test_stream_does_not_bypass_invalid_native_complete_cycle(self):
  g,a,obs=self.setup_guard();a['feedback_complete']=False
  with self.assertRaises(RuntimeError):g(True,True)
 def test_expired_stream_still_stops(self):
  g,a,obs=self.setup_guard();obs['joint1']['temperature_time']=98.4
  with self.assertRaises(RuntimeError):g(True,True)
 def test_new_over_limit_temperature_not_hidden_by_older_cool_status(self):
  g,a,obs=self.setup_guard();obs['joint1']['temperature_C']=50
  with self.assertRaises(RuntimeError):g(True,True)
 def test_gripper_project40_limit_unchanged(self):
  g,a,obs=self.setup_guard();obs['r_joint']['temperature_C']=40
  with self.assertRaises(RuntimeError):g(True,True)
if __name__=='__main__':unittest.main()
