import ast
import importlib.util
import json
from pathlib import Path
import subprocess
import tempfile
import time
from unittest import TestCase, main, mock

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('bootstrap', ROOT / 'car_setup/ros2/arm_bootstrap.py')
BOOT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BOOT)


class BootTests(TestCase):
    def test_disabled_never_takes_hardware(self):
        with tempfile.TemporaryDirectory() as d, mock.patch.object(BOOT.subprocess, 'run') as run:
            root = Path(d)
            (root / 'arm_startup_pose.json').write_text(json.dumps({'startup': {'enabled':False}}))
            BOOT.prepare_boot_arm('image', root, root/'runtime')
            run.assert_not_called()

    def test_only_rrc_once_per_boot(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            (root/'arm_startup_pose.json').write_text(json.dumps({'startup': {'enabled':True,'target_raw':{'joint3':907},'near_limit_recovery':{'enabled':True}}}))
            reply = 'RESULT_JSON=' + json.dumps({'servo':4,'in_range':True,'messages':2})
            with mock.patch.object(BOOT.subprocess, 'run', return_value=subprocess.CompletedProcess([],0,reply,'')) as run:
                BOOT.prepare_boot_arm('image',root,root/'runtime')
                BOOT.prepare_boot_arm('image',root,root/'runtime')
                self.assertEqual(run.call_count,1)
                args = run.call_args.args[0]
                self.assertEqual(args.count('--device'),1)
                self.assertIn('/dev/rrc:/dev/rrc',args)
                self.assertIn('none',args)
                self.assertNotIn('--privileged',args)

    def test_failed_receipt_prevents_automatic_retry(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d)
            (root/'arm_startup_pose.json').write_text(json.dumps({'startup':{'enabled':True,'target_raw':{'joint3':907},'near_limit_recovery':{'enabled':True}}}))
            runtime=root/'runtime';runtime.mkdir()
            (runtime/'arm-bootstrap.json').write_text('{"ok": false}')
            with mock.patch.object(BOOT.subprocess,'run') as run:
                with self.assertRaises(RuntimeError):BOOT.prepare_boot_arm('image',root,runtime)
                run.assert_not_called()

    def test_current_feedback_guard(self):
        source=(ROOT/'car_setup/ros2/arm_startup_pose_ros.py').read_text()
        tree=ast.parse(source)
        definitions=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name in ('fresh_feedback','reject_fault')]
        names=['joint1','joint2','joint3','joint4','joint5','r_joint']
        state={'received_at':time.monotonic()}
        scope={'time':time,'state':state,'ALL':names,'LIMITS':{n:(0,1000) for n in names}}
        exec(compile(ast.Module(body=definitions,type_ignores=[]),source,'exec'),scope)
        view={'feedback_complete':True,'feedback_real':True,'feedback_age_seconds':{n:.2 for n in names},'joints':{n:{'raw':500,'temperature':25} for n in names}}
        self.assertTrue(scope['fresh_feedback'](view))
        view['feedback_age_seconds']['joint3']=125
        self.assertFalse(scope['fresh_feedback'](view))
        for fault in ({'stop_reasons':['manual_estop']},{'current_read_error':'Invalid physical feedback: joint3'}):
            with self.assertRaises(SystemExit):scope['reject_fault'](fault)

    def test_not_ready_cannot_match_ready(self):
        source=(ROOT/'car_setup/ros2/arm_startup_pose.py').read_text()
        self.assertIn("'READY' in out.splitlines()",source)
        self.assertNotIn('READY','NOT_READY'.splitlines())


if __name__=='__main__':main()
