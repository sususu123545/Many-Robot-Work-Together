"""Tests for the policy contract and the deterministic stand-in."""

import base64
import math
import unittest

from vla_controller import spec
from vla_ros2_bridge.policy import (POLICY_OUTPUT_NORMALIZED,
                                    POLICY_OUTPUT_PHYSICAL, PolicyError,
                                    PolicyOutput, RemotePolicy, ScriptedPolicy,
                                    decode_policy_response)

OBS = {'tcp_position_m': (0.0, 0.10, 0.12),
       'gripper_opening': 0.6,
       'instruction': 'pick up the red ball',
       'wrist_image': {'format': 'jpeg', 'data': b'\xff\xd8raw'}}


class TestPolicyOutput(unittest.TestCase):

    def test_valid_output(self):
        out = PolicyOutput([(0, 0.01, 0, 0, 0, 0, 0.5)] * 8, 'p0', 0.35, 1.0,
                           POLICY_OUTPUT_PHYSICAL)
        self.assertEqual(len(out), 8)
        self.assertEqual(out.output_kind, POLICY_OUTPUT_PHYSICAL)

    def test_bad_dimension(self):
        with self.assertRaises(PolicyError):
            PolicyOutput([(0, 0.01, 0)] * 4, 'p0', 0.35, 1.0)

    def test_nonfinite(self):
        with self.assertRaises(PolicyError):
            PolicyOutput([(0, float('nan'), 0, 0, 0, 0, 0.5)] * 4,
                         'p0', 0.35, 1.0)

    def test_empty(self):
        with self.assertRaises(PolicyError):
            PolicyOutput([], 'p0', 0.35, 1.0)

    def test_unknown_kind(self):
        with self.assertRaises(PolicyError):
            PolicyOutput([(0,) * 7], 'p0', 0.35, 1.0, 'mystery')

    def test_empty_policy_id(self):
        with self.assertRaises(PolicyError):
            PolicyOutput([(0,) * 7], '', 0.35, 1.0)


class TestScriptedPolicy(unittest.TestCase):

    def test_shape_and_determinism(self):
        policy = ScriptedPolicy(goal_position=(0.0, 0.20, 0.12), step_m=0.010,
                                horizon=16)
        first = policy.predict(OBS)
        second = policy.predict(OBS)
        self.assertEqual(first.actions, second.actions)
        self.assertEqual(len(first), 16)
        self.assertEqual(len(first.actions[0]), spec.ACTION_DIM)
        self.assertEqual(first.output_kind, POLICY_OUTPUT_PHYSICAL)
        self.assertEqual(first.actions[0][6], 0.6)  # gripper held, not a delta

    def test_step_is_bounded_and_points_at_the_goal(self):
        out = ScriptedPolicy(step_m=0.010, horizon=4).predict(OBS)
        action = out.actions[0]
        self.assertAlmostEqual(action[1], 0.010, places=9)
        self.assertAlmostEqual(math.sqrt(sum(v * v for v in action[:3])),
                               0.010, places=9)
        self.assertEqual(action[0], 0.0)
        self.assertEqual(action[2], 0.0)

    def test_last_step_is_not_oversized(self):
        out = ScriptedPolicy(goal_position=(0.0, 0.103, 0.12),
                             step_m=0.010, horizon=4).predict(OBS)
        self.assertAlmostEqual(out.actions[0][1], 0.003, places=9)

    def test_at_goal_is_hold_still(self):
        out = ScriptedPolicy(goal_position=OBS['tcp_position_m'],
                             horizon=4).predict(OBS)
        self.assertEqual(out.actions[0][:6], (0.0,) * 6)
        self.assertEqual(out.actions[0][6], 0.6)

    def test_unrealizable_components_are_never_emitted(self):
        out = ScriptedPolicy(horizon=4).predict(OBS)
        for action in out.actions:
            self.assertEqual(action[4], 0.0)  # dry
            self.assertEqual(action[5], 0.0)  # drz

    def test_bad_observation(self):
        with self.assertRaises(PolicyError):
            ScriptedPolicy().predict({'tcp_position_m': (0, 0, 0)})
        with self.assertRaises(PolicyError):
            ScriptedPolicy().predict(dict(OBS, gripper_opening=1.4))


class TestRemotePolicy(unittest.TestCase):

    def test_decode_valid_response(self):
        payload = {'policy_id': 'smolvla_test', 'control_dt': 0.35,
                   'validity_s': 1.0, 'actions': [[0.0] * 7] * 8}
        out = decode_policy_response(payload)
        self.assertEqual(out.policy_id, 'smolvla_test')
        self.assertEqual(out.output_kind, POLICY_OUTPUT_NORMALIZED)
        self.assertEqual(len(out), 8)

    def test_decode_rejects_missing_actions(self):
        with self.assertRaises(PolicyError):
            decode_policy_response({'policy_id': 'p'})

    def test_to_wire_base64_encodes_images(self):
        wire = RemotePolicy.to_wire(OBS)
        self.assertEqual(wire['wrist_image']['data'],
                         base64.b64encode(b'\xff\xd8raw').decode('ascii'))
        self.assertEqual(wire['instruction'], OBS['instruction'])

    def test_transport_is_injectable_and_response_is_decoded(self):
        seen = {}

        def fake_transport(endpoint, body, timeout_s):
            seen.update(endpoint=endpoint, body=body, timeout_s=timeout_s)
            return {'policy_id': 'remote_v1', 'control_dt': 0.35,
                    'validity_s': 1.0, 'actions': [[0.0] * 7] * 4}

        policy = RemotePolicy('http://gpu-box:8080/vla', transport=fake_transport)
        out = policy.predict(OBS)
        self.assertEqual(seen['endpoint'], 'http://gpu-box:8080/vla')
        self.assertEqual(out.policy_id, 'remote_v1')
        self.assertEqual(out.output_kind, POLICY_OUTPUT_NORMALIZED)

    def test_transport_failure_is_a_policy_error(self):
        def broken(*_args):
            raise OSError('connection refused')

        with self.assertRaises(PolicyError):
            RemotePolicy('http://gpu-box:8080/vla', transport=broken).predict(OBS)

    def test_missing_policy_id_is_rejected(self):
        def no_id(*_args):
            return {'actions': [[0.0] * 7] * 4}

        with self.assertRaises(PolicyError):
            RemotePolicy('http://x', transport=no_id).predict(OBS)


if __name__ == '__main__':
    unittest.main()
