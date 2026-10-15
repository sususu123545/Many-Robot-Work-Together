"""Run the actual stdin pose script against fake ROS; never touch hardware."""
import contextlib
import io
import importlib.util
import json
import os
from pathlib import Path
import signal
import sys
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT/'car_setup/ros2/arm_startup_pose_ros.py').read_text(encoding='utf-8')


class Harness:
    def __init__(self, revoke_only=False, failure=None):
        self.now=100.0
        self.revoke_only=revoke_only
        self.failure=failure
        self.arm=True if revoke_only else False
        self.gripper=True if revoke_only else False
        self.calls=[];self.events=[];self.handlers={};self.scope={};self.subscription=None
        self.estop=True if failure=='feedback' else False
        self.node=NS(create_subscription=self.subscribe,create_client=self.client,
                     create_publisher=self.publisher,destroy_node=lambda:self.events.append('destroy'),
                     get_clock=lambda:NS(now=lambda:NS(to_msg=lambda:NS())))

    def clock(self):
        self.now+=.05
        return self.now

    def subscribe(self,typ,topic,callback,depth):
        self.subscription=callback

    def status(self):
        names=['joint1','joint2','joint3','joint4','joint5','r_joint']
        return {'feedback_complete':True,'feedback_real':True,
                'feedback_age_seconds':{n:0.1 for n in names},
                'joints':{n:{'raw':480 if n=='joint1' and self.failure=='during_travel'
                            else (500 if n!='r_joint' else 700),'temperature':25} for n in names},
                'reset_eligible':True,'poll_pending':False,'stop_reasons':[],
                'current_read_error':'read failed' if self.failure=='feedback' else None,
                'enable_arm_motion':self.arm,'enable_gripper_motion':self.gripper,
                'estop':self.estop,'fault_count':0,'health_state':'READY'}

    def spin(self,*args,**kwargs):
        import json
        self.clock()
        if self.failure!='status_missing' or not any(c[0]=='close' for c in self.calls):
            self.subscription(NS(data=json.dumps(self.status())))

    def client(self,typ,name):
        def call(request):
            if name.endswith('reset_estop'):
                self.calls.append(('reset',None));self.estop=False
                result=NS(success=True,message='ok')
            else:
                parameters={p.name:p.value.bool_value for p in request.parameters}
                closing=not any(parameters.values())
                self.calls.append(('close' if closing else 'open',parameters))
                self.events.append('close' if closing else 'open')
                if not closing or self.failure not in ('refused','ack_missing'):
                    self.arm=parameters['enable_arm_motion'];self.gripper=parameters['enable_gripper_motion']
                result=NS(result=NS(successful=not(closing and self.failure=='refused'),reason='test refusal'))
                if closing and self.failure=='ack_missing':
                    return NS(done=lambda:False,result=lambda:None)
            return NS(done=lambda:True,result=lambda:result)
        return NS(srv_name=name,call_async=call,
                  wait_for_service=lambda **kwargs:not(self.revoke_only and self.failure=='service_missing'))

    def publisher(self,*args):
        self.events.append('publisher')
        if self.failure=='publisher':raise RuntimeError('publisher failed after grant')
        if self.failure=='sigterm':self.handlers[signal.SIGTERM](signal.SIGTERM,None)
        if self.failure=='keyboard':raise KeyboardInterrupt()
        def publish(msg):
            self.events.append('POSITION_COMMAND')
            if self.failure=='during_travel':raise RuntimeError('command path failed')
        return NS(get_subscription_count=lambda:1,publish=publish)

    def run(self):
        def module(name,**attrs):
            value=ModuleType(name);value.__dict__.update(attrs);return value
        class Message:
            def __init__(self,**kwargs):
                self.header=NS(stamp=None);self.__dict__.update(kwargs)
        class Request:pass
        ros=module('rclpy',init=lambda:None,shutdown=lambda:self.events.append('shutdown'),
                   spin_once=self.spin,spin_until_future_complete=lambda *a,**kw:self.clock())
        modules={'rclpy':ros,'rclpy.node':module('rclpy.node',Node=lambda name:self.node),
                 'std_msgs.msg':module('std_msgs.msg',String=Message),
                 'std_srvs.srv':module('std_srvs.srv',Trigger=NS(Request=Request)),
                 'rcl_interfaces.srv':module('rcl_interfaces.srv',SetParametersAtomically=NS(Request=Request)),
                 'rcl_interfaces.msg':module('rcl_interfaces.msg',Parameter=Message,ParameterValue=Message,
                                            ParameterType=NS(PARAMETER_BOOL=1)),
                 'trajectory_msgs.msg':module('trajectory_msgs.msg',JointTrajectory=Message,JointTrajectoryPoint=Message),
                 'builtin_interfaces.msg':module('builtin_interfaces.msg',Duration=Message)}
        env={'JOINTS':'joint1,joint2,joint3,joint4,joint5',
             **{'POSE_joint'+str(n):'500' for n in range(1,6)},
             'ARMPI_POSE_REVOKE_ONLY':'1' if self.revoke_only else '0'}
        self.output=io.StringIO()
        with patch.dict(sys.modules,modules),patch.dict(os.environ,env,clear=True),\
             patch('time.monotonic',self.clock),\
             patch('signal.signal',lambda sig,handler:self.handlers.update({sig:handler})),\
             contextlib.redirect_stdout(self.output):
            try:
                exec(compile(SOURCE,'arm_startup_pose_ros.py','exec'),self.scope)
            except BaseException as error:return error
        raise AssertionError('script should always exit explicitly')


class PermissionCleanupTests(unittest.TestCase):
    def assert_closed(self,harness,command_expected=False):
        self.assertFalse(harness.arm);self.assertFalse(harness.gripper)
        self.assertLess(harness.events.index('close'),harness.events.index('destroy'))
        self.assertIn('"confirmed": true',harness.output.getvalue())
        self.assertEqual('POSITION_COMMAND' in harness.events,command_expected)

    def test_success_reclaims_both_gates_before_shutdown(self):
        h=Harness();error=h.run()
        self.assertIsInstance(error,SystemExit);self.assertEqual(error.code,0)
        self.assert_closed(h)

    def test_failure_after_authorization_reclaims_gates(self):
        h=Harness(failure='publisher');error=h.run()
        self.assertIsInstance(error,RuntimeError);self.assert_closed(h)

    def test_failure_during_command_path_reclaims_gates(self):
        h=Harness(failure='during_travel');error=h.run()
        self.assertIsInstance(error,RuntimeError);self.assert_closed(h,command_expected=True)

    def test_bad_feedback_still_allows_false_only_cleanup(self):
        h=Harness(failure='feedback');error=h.run()
        self.assertIsInstance(error,SystemExit);self.assertNotEqual(error.code,0)
        self.assert_closed(h);self.assertTrue(h.estop)
        self.assertFalse(any(c[0] in ('open','reset') for c in h.calls))

    def test_sigterm_reclaims_gates(self):
        h=Harness(failure='sigterm');error=h.run()
        self.assertEqual(error.code,128+signal.SIGTERM);self.assert_closed(h)

    def test_keyboard_interrupt_reclaims_gates(self):
        h=Harness(failure='keyboard');error=h.run()
        self.assertIsInstance(error,KeyboardInterrupt);self.assert_closed(h)

    def test_revoke_only_never_resets_or_constructs_a_publisher(self):
        h=Harness(revoke_only=True);error=h.run()
        self.assertEqual(error.code,0);self.assert_closed(h)
        self.assertNotIn('publisher',h.events)
        self.assertTrue(all(c[0]=='close' for c in h.calls))

    def test_no_success_on_missing_service_ack_status_or_refusal(self):
        for failure in ('service_missing','ack_missing','status_missing','refused'):
            with self.subTest(failure=failure):
                h=Harness(revoke_only=True,failure=failure);error=h.run()
                self.assertIsInstance(error,SystemExit);self.assertNotEqual(error.code,0)
                self.assertIn('"confirmed": false',h.output.getvalue())
                self.assertNotIn('publisher',h.events)


class PoseResultTests(unittest.TestCase):
    def run_wrapper(self,code,output):
        spec=importlib.util.spec_from_file_location('pose_wrapper',ROOT/'car_setup/ros2/arm_startup_pose.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        cfg=NS(exists=lambda:True,read_text=lambda **kw:json.dumps(
            {'startup':{'enabled':True,'target_raw':{'joint3':907}}}))
        script=NS(exists=lambda:True,read_text=lambda **kw:'pass')
        from unittest.mock import Mock
        saved=Mock()
        with patch.object(module,'CONFIG',cfg),patch.object(module,'ROS_SCRIPT',script),\
             patch.object(module,'wait_core',return_value='test_container'),\
             patch.object(module,'run_in_container',return_value=(code,output,'')),\
             patch.object(module,'write_result',saved),patch.object(module,'log'):
            result=module.main()
        return result,saved.call_args.args[0]

    def test_reached_pose_without_cleanup_is_not_success(self):
        code,summary=self.run_wrapper(0,'RESULT_JSON={"reached":true}')
        self.assertEqual(code,6);self.assertFalse(summary['ok'])

    def test_success_requires_confirmed_cleanup(self):
        code,summary=self.run_wrapper(0,'RESULT_JSON={"reached":true}\nCLEANUP_JSON={"confirmed":true}')
        self.assertEqual(code,0);self.assertTrue(summary['ok']);self.assertTrue(summary['cleanup']['confirmed'])

    def test_cleanup_cannot_hide_pose_failure(self):
        code,summary=self.run_wrapper(3,'RESULT_JSON={"reached":false}\nCLEANUP_JSON={"confirmed":true}')
        self.assertEqual(code,3);self.assertFalse(summary['ok'])


if __name__=='__main__':unittest.main()
