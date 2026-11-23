"""Policy side of the head: a protocol, a deterministic stand-in, a remote hook.

There is no trained model in this repository.  Rather than leave the head of the
loop empty until a checkpoint exists, this module fixes the *contract* a policy
must satisfy, so the bridge and its tests can be written now and a real server
can be dropped in later without touching the executor.

Two output kinds are distinguished explicitly:

``normalized``
    What a real network emits.  De-normalising requires the statistics of the
    dataset that model was trained on.  ``BridgePlanner`` refuses to run without
    them -- see ``normalization.NormalizationError`` and Round1 section 7.
``physical``
    Already in metres / radians / [0,1] gripper.  Only the deterministic
    stand-in uses this, for offline and on-robot closed-loop bring-up.

``RemotePolicy`` speaks a small project-defined JSON protocol.  It is NOT a
LeRobot / SmolVLA standard and has never been exercised against a real VLA
server; it exists so the service shape is pinned down instead of blocking all
other work on a decision about somebody else's API.
"""

import math

from vla_controller import spec

POLICY_OUTPUT_NORMALIZED = 'normalized'
POLICY_OUTPUT_PHYSICAL = 'physical'
_OUTPUT_KINDS = (POLICY_OUTPUT_NORMALIZED, POLICY_OUTPUT_PHYSICAL)


class PolicyError(ValueError):
    """A policy returned something the bridge must not execute."""


class PolicyOutput:
    """The complete answer to one observation."""

    __slots__ = ('actions', 'policy_id', 'control_dt', 'validity_s', 'output_kind')

    def __init__(self, actions, policy_id, control_dt, validity_s,
                 output_kind=POLICY_OUTPUT_NORMALIZED):
        if output_kind not in _OUTPUT_KINDS:
            raise PolicyError('Unknown policy output kind: %r' % (output_kind,))
        if not isinstance(policy_id, str) or not policy_id:
            raise PolicyError('policy_id must be a non-empty string')
        normalised = []
        for index, action in enumerate(actions):
            try:
                values = tuple(float(value) for value in action)
            except (TypeError, ValueError):
                raise PolicyError('action %d is not a sequence of numbers' % index)
            if len(values) != spec.ACTION_DIM:
                raise PolicyError('action %d has %d components, expected %d'
                                  % (index, len(values), spec.ACTION_DIM))
            if not all(math.isfinite(value) for value in values):
                raise PolicyError('action %d contains non-finite values' % index)
            normalised.append(values)
        if not normalised:
            raise PolicyError('policy returned an empty action sequence')
        if not math.isfinite(control_dt) or control_dt <= 0:
            raise PolicyError('control_dt must be finite and positive')
        if not math.isfinite(validity_s) or validity_s <= 0:
            raise PolicyError('validity_s must be finite and positive')
        self.actions = tuple(normalised)
        self.policy_id = str(policy_id)
        self.control_dt = float(control_dt)
        self.validity_s = float(validity_s)
        self.output_kind = str(output_kind)

    def __len__(self):
        return len(self.actions)

    def __repr__(self):
        return ('PolicyOutput(policy=%r, actions=%d, kind=%s, dt=%.2f)'
                % (self.policy_id, len(self.actions), self.output_kind,
                   self.control_dt))


class Policy:
    """Interface every policy implements.  ``observation`` is a plain mapping."""

    policy_id = 'abstract'

    def predict(self, observation):
        raise NotImplementedError


class ScriptedPolicy(Policy):
    """Deterministic stand-in that walks the TCP toward a fixed point.

    This is deliberately NOT a grasping policy and NOT a learned behaviour.  It
    exists to exercise the whole head: observation -> chunk -> ActionServer ->
    safety filter -> IK -> ``/arm_controller/command`` -> re-observe.  If this
    loop closes on the robot, whatever replaces it only has to produce actions.

    Each action is a small step from the *current measured* TCP, because the
    executor re-solves from fresh joint feedback on every tick.  Producing
    cumulative targets here would double the motion on the second action.
    """

    def __init__(self, goal_position=(0.0, 0.20, 0.12), step_m=0.010,
                 horizon=16, control_dt=0.35, validity_s=1.0,
                 policy_id='scripted_tcp_walk_v0'):
        if len(goal_position) != 3 or not all(math.isfinite(float(v))
                                              for v in goal_position):
            raise ValueError('goal_position must be three finite numbers')
        if not math.isfinite(step_m) or step_m <= 0:
            raise ValueError('step_m must be finite and positive')
        if not isinstance(horizon, int) or horizon < 1:
            raise ValueError('horizon must be a positive integer')
        self.goal = tuple(float(value) for value in goal_position)
        self.step_m = float(step_m)
        self.horizon = int(horizon)
        self.control_dt = float(control_dt)
        self.validity_s = float(validity_s)
        self.policy_id = str(policy_id)

    def predict(self, observation):
        try:
            current = tuple(float(value)
                            for value in observation['tcp_position_m'])
            gripper = float(observation['gripper_opening'])
        except (KeyError, TypeError, ValueError) as error:
            raise PolicyError('observation lacks a usable TCP pose: %s' % error)
        if len(current) != 3 or not all(math.isfinite(v) for v in current):
            raise PolicyError('TCP position is not three finite numbers')
        if not math.isfinite(gripper) or not 0.0 <= gripper <= 1.0:
            raise PolicyError('gripper opening is outside [0, 1]')

        delta = [self.goal[i] - current[i] for i in range(3)]
        distance = math.sqrt(sum(value * value for value in delta))
        step = [0.0, 0.0, 0.0] if distance < 1e-9 else \
            [value / distance * min(self.step_m, distance) for value in delta]

        # dx, dy, dz, drx, dry, drz, gripper.  dry/drz are physically impossible
        # on this arm; emitting them here would be caught downstream, but a
        # stand-in that produces invalid actions is a bad stand-in.
        action = (step[0], step[1], step[2], 0.0, 0.0, 0.0, gripper)
        return PolicyOutput([action] * self.horizon, self.policy_id,
                            self.control_dt, self.validity_s,
                            POLICY_OUTPUT_PHYSICAL)


def decode_policy_response(payload):
    """Validate the JSON a remote policy server is expected to return.

    Kept as a free function so the contract can be unit-tested without a
    network, a server or any ROS 2 installation.  The scene camera is absent on
    this platform, so a server asking for ``front_image`` is rejected rather
    than being sent an empty frame it would silently learn from.
    """
    if not isinstance(payload, dict):
        raise PolicyError('policy response must be a JSON object')
    if 'actions' not in payload:
        raise PolicyError('policy response has no actions')
    policy_id = str(payload.get('policy_id') or '')
    control_dt = float(payload.get('control_dt', 0.35))
    validity_s = float(payload.get('validity_s', 1.0))
    return PolicyOutput(payload['actions'], policy_id, control_dt, validity_s,
                        POLICY_OUTPUT_NORMALIZED)


class RemotePolicy(Policy):
    """HTTP policy client for a project-defined minimal JSON protocol.

    The request body is the observation dictionary (compressed images are
    base64-encoded by ``to_wire()``); the response is decoded by
    ``decode_policy_response``.  ``transport`` is injectable: pass a callable
    ``(endpoint, body, timeout_s) -> dict`` to test or to swap in gRPC later.

    This has NEVER been run against a real VLA server.  It is a placeholder
    shape, documented as such in the package docstring.
    """

    def __init__(self, endpoint, timeout_s=2.0, transport=None,
                 policy_id='remote_unspecified'):
        if not isinstance(endpoint, str) or not endpoint:
            raise ValueError('endpoint must be a non-empty string')
        self.endpoint = endpoint
        self.timeout_s = float(timeout_s)
        self.transport = transport or self._http
        self.policy_id = str(policy_id)

    @staticmethod
    def to_wire(observation):
        """Observation -> JSON-safe dict (image bytes become base64 strings)."""
        import base64
        import copy

        wire = copy.deepcopy(dict(observation))
        for key in ('wrist_image', 'front_image'):
            image = wire.get(key)
            if isinstance(image, dict) and isinstance(image.get('data'), bytes):
                image['data'] = base64.b64encode(image['data']).decode('ascii')
        return wire

    def _http(self, endpoint, body, timeout_s):
        import json
        import urllib.request

        request = urllib.request.Request(
            endpoint, data=json.dumps(body).encode('utf-8'),
            headers={'Content-Type': 'application/json'}, method='POST')
        with urllib.request.urlopen(request, timeout=timeout_s) as response:
            return json.loads(response.read().decode('utf-8'))

    def predict(self, observation):
        body = self.to_wire(observation)
        try:
            payload = self.transport(self.endpoint, body, self.timeout_s)
        except Exception as error:
            raise PolicyError('policy server call failed: %s' % error)
        result = decode_policy_response(payload)
        if not result.policy_id:
            raise PolicyError('remote policy returned no policy_id')
        return result
