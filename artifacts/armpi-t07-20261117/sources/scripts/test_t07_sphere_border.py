"""Reject the actual clipped red-ball contour even when its blob score is good."""
import ast
from pathlib import Path
import unittest
source=Path(__file__).with_name('t07_redball_grasp_plan.py').read_text(encoding='utf-8')
fn=next(n for n in ast.parse(source).body if isinstance(n,ast.FunctionDef) and n.name=='complete_sphere')
scope={};exec(compile(ast.Module(body=[fn],type_ignores=[]),'<actual production pure guard>','exec'),scope)
guard=scope['complete_sphere']
class BorderTests(unittest.TestCase):
 def test_actual_clipped_contour_rejected(self):
  self.assertFalse(guard({'bbox':[273,0,103,83],'center_x':324.5,'center_y':28.24,'radius_px':56.12}))
 def test_actual_complete_contour_accepted(self):
  self.assertTrue(guard({'bbox':[273,57,98,112],'center_x':322.2,'center_y':112.5,'radius_px':56.15}))
 def test_fitted_circle_must_also_fit(self):
  self.assertFalse(guard({'bbox':[5,5,20,20],'center_x':15,'center_y':15,'radius_px':20}))
 def test_missing_detection_is_not_target(self):self.assertFalse(guard(None))
if __name__=='__main__':unittest.main()
