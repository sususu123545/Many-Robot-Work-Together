"""Exercise the actual staged reader, including timestamp expiry and hard faults."""
import ast,math,threading,types,unittest
from pathlib import Path
from unittest.mock import Mock
ROOT=Path(__file__).resolve().parents[1];source=ROOT/'logs/t07/runs/20261117T050800Z_f8ef182f/driver_patch/arm_node.py'
tree=ast.parse(source.read_text(encoding='utf-8'));method=next(n for c in tree.body if isinstance(c,ast.ClassDef) and c.name=='ArmNode' for n in c.body if isinstance(n,ast.FunctionDef) and n.name=='read_joint_feedback')
class TemperatureFault(OSError):pass
clock=types.SimpleNamespace(now=100.,monotonic=lambda:clock.now)
namespace={'time':clock,'JOINTS':{'joint1':(6,500,1,0,1000)},'TemperatureFault':TemperatureFault}
exec(compile(ast.Module(body=[method],type_ignores=[]),'staged_reader','exec'),namespace);reader=namespace['read_joint_feedback']
health_tree=ast.parse((ROOT/'vla_sim_adapter/reference/arm_health.py').read_text(encoding='utf-8'));complete_node=next(n for n in health_tree.body if isinstance(n,ast.FunctionDef) and n.name=='complete_feedback')
health_namespace={'math':math,'JOINTS':namespace['JOINTS']};exec(compile(ast.Module(body=[complete_node],type_ignores=[]),'existing_cycle_guard','exec'),health_namespace);complete=health_namespace['complete_feedback']
class Tests(unittest.TestCase):
 def setUp(self):clock.now=100.
 def owner(self,age=1.1,temp=30):
  previous={'raw':400,'temperature':temp,'temperature_time':clock.now-age}
  board=Mock();board.position.return_value=510;board.temperature.side_effect=TimeoutError('temperature cmd9 absent')
  return types.SimpleNamespace(board=board,state_lock=threading.RLock(),health=types.SimpleNamespace(samples={'joint1':previous}),temp_max=65)
 def test_real_prior_temperature_retains_original_time_and_new_encoder(self):
  o=self.owner();r=reader(o,'joint1');self.assertEqual(r['temperature_time'],98.9);self.assertEqual(r['position_time'],100);self.assertEqual(r['raw'],510);self.assertEqual(r['time'],98.9);self.assertTrue(r['temperature_retained']);self.assertIn('cmd9 absent',r['temperature_read_error'])
 def test_repeated_timeouts_never_renew_age(self):
  o=self.owner(age=.5);first=reader(o,'joint1');o.health.samples['joint1']=first;clock.now=100.9
  second=reader(o,'joint1');self.assertEqual(second['temperature_time'],99.5);o.health.samples['joint1']=second;clock.now=101.
  with self.assertRaises(TimeoutError):reader(o,'joint1')
 def test_no_initial_temperature_stops(self):
  o=self.owner();o.health.samples={}
  with self.assertRaises(TimeoutError):reader(o,'joint1')
 def test_expired_or_future_temperature_stops(self):
  for age in [1.5,1.6,-.1]:
   with self.subTest(age=age),self.assertRaises(TimeoutError):reader(self.owner(age=age),'joint1')
 def test_encoder_timeout_never_retained(self):
  o=self.owner();o.board.position.side_effect=TimeoutError('encoder absent')
  with self.assertRaisesRegex(TimeoutError,'encoder absent'):reader(o,'joint1')
  o.board.temperature.assert_not_called()
 def test_explicit_temperature_io_error_stops(self):
  o=self.owner();o.board.temperature.side_effect=OSError('servo reports fault')
  with self.assertRaises(OSError):reader(o,'joint1')
 def test_live_overtemperature_is_not_masked(self):
  o=self.owner();o.board.temperature.side_effect=None;o.board.temperature.return_value=65
  with self.assertRaises(TemperatureFault):reader(o,'joint1')
 def test_invalid_prior_value_cannot_be_retained(self):
  for temp in [65,-1,float('nan'),float('inf')]:
   with self.subTest(temp=temp),self.assertRaises(TimeoutError):reader(self.owner(temp=temp),'joint1')
 def test_success_uses_new_real_temperature_time(self):
  o=self.owner();o.board.temperature.side_effect=None;o.board.temperature.return_value=34;r=reader(o,'joint1')
  self.assertEqual(r['temperature'],34);self.assertEqual(r['temperature_time'],100);self.assertFalse(r['temperature_retained']);self.assertIsNone(r['temperature_read_error'])
 def test_invalid_new_encoder_still_stops(self):
  o=self.owner();o.board.position.return_value=1001
  with self.assertRaises(OSError):reader(o,'joint1')
 def test_original_cycle_guard_rechecks_age_at_cycle_end(self):
  o=self.owner(age=1.1);r=reader(o,'joint1');self.assertTrue(complete({'joint1':r},100.3));self.assertFalse(complete({'joint1':r},100.5))
 def test_original_cycle_guard_never_treats_encoder_time_as_temperature_time(self):
  r=reader(self.owner(age=1.4),'joint1');self.assertEqual(r['position_time'],100.);self.assertFalse(complete({'joint1':r},100.2))
if __name__=='__main__':unittest.main()
