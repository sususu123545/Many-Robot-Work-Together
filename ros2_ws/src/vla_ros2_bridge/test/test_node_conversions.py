"""Exercise the ROS 2 shell's conversions without a ROS 2 installation.

The generated message classes and rclpy are stubbed with keyword-only stand-ins.
This does NOT prove the package builds or that the real generated classes accept
these fields -- only ``colcon build`` on the robot can do that.  What it does
prove is that the shell uses the message fields it claims to use, that the
timestamp maths preserves nanoseconds, and that no positional constructor
slips back in (rosidl Python messages are keyword-only).
"""

import math
import sys
import types
import unittest


class KwMsg:
    """Stand-in that behaves like a rosidl message: keyword-only, attributes set."""

    def __init__(self, **kwargs):
        for key, value in kwargs.items():
            setattr(self, key, value)


class NodeStub:
    pass


class CompressedImageStub(KwMsg):
    """Generated CompressedImage has default-valued fields; the stub must too."""

    def __init__(self, **kwargs):
        kwargs.setdefault('format', '')
        kwargs.setdefault('data', b'')
        super().__init__(**kwargs)


ORIGINAL = {}


def _module(name, **attributes):
    module = types.ModuleType(name)
    for key, value in attributes.items():
        setattr(module, key, value)
    ORIGINAL.setdefault(name, sys.modules.get(name))
    sys.modules[name] = module
    return module


def setUpModule():
    _module('builtin_interfaces')
    _module('builtin_interfaces.msg', Time=KwMsg, Duration=KwMsg)
    _module('geometry_msgs')
    _module('geometry_msgs.msg', Point=KwMsg, Quaternion=KwMsg, Twist=KwMsg)
    _module('nav_msgs')
    _module('nav_msgs.msg', Odometry=KwMsg)
    _module('sensor_msgs')
    _module('sensor_msgs.msg', CompressedImage=CompressedImageStub,
            JointState=KwMsg)
    _module('std_msgs')
    _module('std_msgs.msg', Header=KwMsg, String=KwMsg, Bool=KwMsg)
    _module('trajectory_msgs')
    _module('trajectory_msgs.msg', JointTrajectory=KwMsg, JointTrajectoryPoint=KwMsg)
    _module('rclpy')
    _module('rclpy.node', Node=NodeStub)
    _module('rclpy.qos', qos_profile_sensor_data=object())
    _module('rclpy.action', ActionClient=NodeStub, ActionServer=NodeStub,
            CancelResponse=NodeStub, GoalResponse=NodeStub)
    _module('rclpy.callback_groups', ReentrantCallbackGroup=NodeStub)
    _module('rclpy.executors', MultiThreadedExecutor=NodeStub)
    _module('vla_interfaces')
    _module('vla_interfaces.msg', VlaObservation=KwMsg, VlaAction=KwMsg,
            VlaActionChunk=KwMsg, VlaSafetyVerdict=KwMsg)

    class ExecuteActionChunk:
        Goal = KwMsg

    _module('vla_interfaces.action', ExecuteActionChunk=ExecuteActionChunk)


def tearDownModule():
    for name, previous in ORIGINAL.items():
        if previous is None:
            sys.modules.pop(name, None)
        else:
            sys.modules[name] = previous


class TestTimeConversions(unittest.TestCase):

    def test_nanoseconds_round_trip(self):
        from vla_ros2_bridge.observation_node import seconds, time_from_seconds
        from vla_ros2_bridge.bridge_node import duration_from_seconds

        for value in (100.0, 100.4, 1794403002.205834146):
            rebuilt = seconds(time_from_seconds(value))
            self.assertAlmostEqual(rebuilt, value, places=6)
            duration = duration_from_seconds(value)
            self.assertEqual(duration.sec, int(value))

    def test_time_uses_keywords(self):
        from vla_ros2_bridge.observation_node import time_from_seconds

        value = time_from_seconds(12.25)
        self.assertEqual(value.sec, 12)
        self.assertEqual(value.nanosec, 250000000)


class TestJointConversion(unittest.TestCase):

    def test_observer_and_executor_decode_driver_gripper_units(self):
        from vla_ros2_bridge.observation_node import ticks_from_joint_state
        from vla_controller.chunk_executor_node import _ticks_from_joint_state

        message = KwMsg(name=['joint3', 'r_joint'],
                        position=[(500 - 925) * math.radians(240) / 1000,
                                  (128 - 700) * math.radians(180) / 1000],
                        header=KwMsg(stamp=KwMsg(sec=100, nanosec=250000000)))
        for decode in (ticks_from_joint_state, _ticks_from_joint_state):
            raw, stamp = decode(message)
            self.assertEqual(raw, {'joint3': 925, 'r_joint': 128})
            self.assertEqual(stamp, 100.25)

    def test_radians_to_ticks_and_back(self):
        from vla_ros2_bridge.observation_node import ticks_from_joint_state
        from vla_controller import spec

        names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'r_joint']
        ticks = [495, 644, 696, 166, 499, 339]
        message = KwMsg(name=names,
                        position=[spec.tick_to_radians(n, t)
                                  for n, t in zip(names, ticks)],
                        header=KwMsg(stamp=KwMsg(sec=100, nanosec=0)))
        raw, stamp = ticks_from_joint_state(message)
        self.assertEqual(stamp, 100.0)
        self.assertEqual(raw, dict(zip(names, ticks)))

    def test_out_of_range_encoder_is_refused_not_clamped(self):
        from vla_ros2_bridge.observation_node import ticks_from_joint_state

        message = KwMsg(name=['joint3'], position=[999.0],
                        header=KwMsg(stamp=KwMsg(sec=1, nanosec=0)))
        self.assertEqual(ticks_from_joint_state(message), ({}, 0.0))

    def test_nonfinite_is_refused(self):
        from vla_ros2_bridge.observation_node import ticks_from_joint_state

        message = KwMsg(name=['joint2'], position=[float('nan')],
                        header=KwMsg(stamp=KwMsg(sec=1, nanosec=0)))
        self.assertEqual(ticks_from_joint_state(message), ({}, 0.0))


class TestMessageBuilding(unittest.TestCase):

    def test_observation_message_uses_keyword_construction(self):
        from vla_ros2_bridge.observation import ObservationBuilder, SensorSnapshot
        from vla_ros2_bridge.observation_node import ObservationNode

        def pose_provider(_ticks):
            return {'position': (0.1, 0.2, 0.3),
                    'quaternion': (0.0, 0.0, 0.0, 1.0)}

        snapshot = SensorSnapshot(
            joint_ticks={'joint1': 500, 'joint2': 500, 'joint3': 500,
                         'joint4': 500, 'joint5': 500, 'r_joint': 300},
            joint_stamp=100.0, wrist_image=b'\xff\xd8x', wrist_image_format='jpeg',
            wrist_stamp=100.01, instruction='pick up the red ball')
        observation = ObservationBuilder(pose_provider=pose_provider).build(snapshot)
        # to_message does not use instance state, so an unbound call is enough
        # to exercise the field binding.
        message = ObservationNode.to_message(None, observation)
        self.assertEqual(message.instruction, 'pick up the red ball')
        self.assertAlmostEqual(message.tcp_position_m.x, 0.1)
        self.assertAlmostEqual(message.tcp_position_m.z, 0.3)
        self.assertAlmostEqual(message.tcp_orientation.w, 1.0)
        self.assertEqual(message.wrist_image.data, b'\xff\xd8x')
        self.assertEqual(message.front_image.data, b'')
        self.assertEqual(len(message.joint_names), 6)

    def test_observation_round_trip_through_bridge_dict(self):
        from vla_ros2_bridge.bridge_node import observation_to_dict

        message = KwMsg(
            header=KwMsg(stamp=KwMsg(sec=100, nanosec=500000000),
                         frame_id='arm_base'),
            instruction='pick up the red ball', robot_config_id='armpi_pro_v1',
            tcp_position_m=KwMsg(x=0.1, y=0.2, z=0.3),
            tcp_orientation=KwMsg(x=0.0, y=0.0, z=0.0, w=1.0),
            joint_names=['joint1'], joint_positions_rad=[0.0],
            gripper_opening=0.5,
            wrist_image_stamp=KwMsg(sec=100, nanosec=0),
            wrist_image=KwMsg(format='jpeg', data=b'img'),
            front_image_stamp=KwMsg(sec=0, nanosec=0),
            front_image=KwMsg(format='', data=b''))
        data = observation_to_dict(message)
        self.assertAlmostEqual(data['header']['stamp'], 100.5)
        self.assertEqual(data['tcp_position_m'], (0.1, 0.2, 0.3))
        self.assertEqual(data['wrist_image']['data'], b'img')
        self.assertEqual(data['front_image']['data'], b'')


if __name__ == '__main__':
    unittest.main()
