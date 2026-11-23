#!/usr/bin/env python3
"""Run the VLA control-path logic on a workstation, with no ROS 2 and no robot.

Two things happen here:

1. every pure-Python suite is executed (vla_controller, robot_safety,
   robot_agent, vla_ros2_bridge);
2. a scripted trace pushes scenarios through the same filter and adapter the
   node uses, then closes the loop from observation through a policy to an
   actual ``/vla/execute_chunk`` goal shape.

The trace is not a substitute for a test -- a test asserts, this only reports.
It exists so that a reviewer can see the numbers instead of the claims, and so
that a regression in behaviour (not just in a bound) is visible at a glance.

Usage:  python3 tools/vla_offline_selftest.py [--no-tests]
"""

import os
import subprocess
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(REPO, 'ros2_ws', 'src')

for path in (os.path.join(SRC, 'vla_controller'), os.path.join(SRC, 'armpi_ros2'),
             os.path.join(SRC, 'robot_safety'), os.path.join(SRC, 'robot_agent'),
             os.path.join(SRC, 'vla_ros2_bridge')):
    if path not in sys.path:
        sys.path.insert(0, path)

from vla_controller import spec                                       # noqa: E402
from vla_controller.adapter import AdapterError, TcpDeltaAdapter      # noqa: E402
from vla_controller.chassis import ChassisGuard                       # noqa: E402
from vla_controller.normalization import NormalizationError, NormalizationStats  # noqa: E402
from vla_controller.safety import Context, Limits, SafetyFilter       # noqa: E402
from vla_ros2_bridge.bridge import BridgeError, BridgePlanner         # noqa: E402
from vla_ros2_bridge.observation import ObservationBuilder, SensorSnapshot  # noqa: E402
from vla_ros2_bridge.policy import (POLICY_OUTPUT_NORMALIZED,          # noqa: E402
                                    POLICY_OUTPUT_PHYSICAL,
                                    PolicyOutput, ScriptedPolicy)

#: A real measured pose: the arm unfolded in the air on 2026-11-01.
RAW = {'joint1': 495, 'joint2': 644, 'joint3': 696, 'joint4': 166,
       'joint5': 499, 'r_joint': 339}
NOW = 100.0


def run_suites():
    results = []
    for package in ('vla_controller', 'robot_safety', 'robot_agent',
                    'vla_ros2_bridge'):
        root = os.path.join(SRC, package)
        completed = subprocess.run(
            [sys.executable, os.path.join(root, 'run_tests.py')],
            cwd=root, capture_output=True, text=True)
        tail = (completed.stdout or completed.stderr).strip().splitlines()
        results.append((package, completed.returncode, tail[-1] if tail else ''))
    return results


def describe_verdict(verdict):
    if not verdict.accepted:
        return 'REJECT %-18s %s' % (verdict.code, verdict.reason[:60])
    limited = '' if verdict.scale > 0.995 else ' scaled to %.2f' % verdict.scale
    dropped = '' if not verdict.dropped else ' dropped %s' % ','.join(verdict.dropped)
    return 'PASS   %-18s%s%s' % ('ok', limited, dropped)


def trace():
    limits = Limits()
    filter_ = SafetyFilter(limits)
    adapter = TcpDeltaAdapter()
    guard = ChassisGuard()
    pose = adapter.pose_from_raw(RAW)

    # The base has been standing still for a while: feed the evidence the node
    # would have collected from /odom.
    stamp = 0.0
    while stamp <= NOW:
        guard.observe_odom(0.0, 0.0, 0.0, stamp)
        stamp = round(stamp + 0.1, 6)
    parked, base_detail = guard.parked(NOW)

    print('Pose from measured feedback: %r' % (pose,))
    print('Base parked: %s (%s)' % (parked, base_detail))
    print('Implied TCP speed limit: %.4f m/s (%d mm per %.2f s)'
          % (limits.max_speed_mps, round(limits.max_step_m * 1000), limits.control_dt))
    print('Non-realizable components on this arm: %s'
          % ', '.join(spec.NON_REALIZABLE_COMPONENTS))
    print()

    def context(**overrides):
        values = dict(now=NOW, observation_stamp=NOW - 0.1, chunk_stamp=NOW - 0.2,
                      chunk_validity_s=1.0, tcp_position=pose.position,
                      gripper_opening=spec.gripper_tick_to_opening(RAW['r_joint']),
                      arm_ready=True, base_parked=parked, base_detail=base_detail)
        values.update(overrides)
        return Context(**values)

    scenarios = [
        ('10 mm forward', (0, 0.010, 0, 0, 0, 0, 0.4), context()),
        ('10 mm forward, 5 deg pitch', (0, 0.010, 0, 0.0873, 0, 0, 0.4), context()),
        ('30 mm forward (over the step limit)', (0, 0.030, 0, 0, 0, 0, 0.4), context()),
        ('120 mm forward (policy timing broken)', (0, 0.120, 0, 0, 0, 0, 0.4), context()),
        ('roll command this arm cannot do', (0, 0.005, 0, 0, 0.05, 0, 0.4), context()),
        ('chunk 2 s old', (0, 0.010, 0, 0, 0, 0, 0.4), context(chunk_stamp=NOW - 2.0)),
        ('base moving', (0, 0.010, 0, 0, 0, 0, 0.4),
         context(base_parked=False, base_detail='base motion detected on /odom')),
    ]

    header = ('%-38s %-46s %s' % ('scenario', 'filter', 'geometry'))
    print(header)
    print('-' * len(header))
    for label, action, ctx in scenarios:
        verdict = filter_.check(action, ctx)
        if verdict.accepted:
            try:
                solution = adapter.solve(RAW, verdict.action)
                geometry = ('scale %.2f %-4s residual %.2f mm -> %s'
                            % (solution.scale, solution.branch,
                               solution.residual_m * 1000.0, solution.targets))
            except AdapterError as error:
                geometry = 'adapter rejects: %s' % str(error)[:70]
        else:
            geometry = '-'
        print('%-38s %-46s %s' % (label, describe_verdict(verdict), geometry))
    print()

    print('Normalisation statistics are refused rather than divided by:')
    good_scale = (0.01, 0.01, 0.01, 0.05, 0.05, 0.05, 0.5)
    checks = (
        ('statistics from another action space',
         'armpi_pro_tcp_delta_v0', good_scale),
        ('a channel with no spread in the dataset',
         spec.ACTION_SPACE_ID,
         (0.01, 0.01, 0.01, 0.05, 0.05, 0.05, 0.0)),
    )
    for label, space, scale in checks:
        try:
            stats = NormalizationStats(space, 'mean_std', (0,) * 7, scale)
            stats.denormalize([0] * 7, spec.ACTION_SPACE_ID)
            print('  %-44s -> accepted (unexpected)' % label)
        except NormalizationError as error:
            print('  %-44s -> %s' % (label, str(error)[:70]))


def head_trace():
    """Close the loop: observation -> policy -> chunk -> servo targets.

    Until 2026-11-12 the ActionServer on /vla/execute_chunk had no client, so
    this half of the system could not be exercised at all.  The numbers below
    are produced by the very same modules the ROS 2 nodes would call; only the
    generated message classes are absent on this workstation.
    """
    from vla_ros2_bridge.observation import quaternion_from_matrix

    stamp = 100.0
    now = 100.4
    adapter = TcpDeltaAdapter()
    pose = adapter.pose_from_raw(RAW)
    snapshot = SensorSnapshot(
        joint_ticks=dict(RAW), joint_stamp=stamp,
        wrist_image=b'\xff\xd8offline-jpeg-placeholder',
        wrist_image_format='jpeg', wrist_stamp=stamp + 0.02,
        instruction='pick up the red ball')

    # Use the REAL forward kinematics plus a real matrix->quaternion conversion,
    # rather than a fake pose provider, so the printed TCP is measurable.
    def pose_provider(ticks):
        return {'position': pose.position,
                'quaternion': quaternion_from_matrix(pose.rotation)}

    observation = ObservationBuilder(pose_provider=pose_provider).build(snapshot)
    print('Head of the loop (no ROS 2 on this workstation)')
    print('-' * 60)
    print('  observation  instruction=%r' % observation['instruction'])
    print('               tcp=(%.4f, %.4f, %.4f) m, quaternion=%s'
          % (*observation['tcp_position_m'],
             tuple(round(v, 4) for v in observation['tcp_orientation'])))
    print('               joints=%d, gripper_opening=%.3f, wrist_image=%d B'
          % (len(observation['joint_names']), observation['gripper_opening'],
             len(observation['wrist_image']['data'])))

    policy = ScriptedPolicy(goal_position=(0.0, 0.20, 0.12), step_m=0.010,
                            horizon=16, control_dt=0.35, validity_s=1.0)
    output = policy.predict(observation)
    planner = BridgePlanner(limits=Limits(), execute_steps=4)
    goal = planner.plan(observation, output, now)
    chunk = goal.chunk
    print('  policy       %s -> %d actions, kind=%s'
          % (output.policy_id, len(output), output.output_kind))
    print('  chunk        frame=%s space=%s dt=%.2f s validity=%.2f s mask=%s'
          % (chunk['action_frame'], chunk['action_space_id'],
             chunk['control_dt'], chunk['validity'],
             ''.join(str(int(v)) for v in chunk['realizable_mask'])))
    first = chunk['actions'][0]
    print('  first action %s'
          % ' '.join('%s=%+.4f' % (key, value) for key, value in first.items()))

    # Feed that action through the EXECUTOR's filter and adapter, so the trace
    # ends at real servo targets rather than at a plausible-looking dictionary.
    context = Context(now=now, observation_stamp=stamp, chunk_stamp=now,
                      chunk_validity_s=chunk['validity'],
                      tcp_position=pose.position,
                      gripper_opening=observation['gripper_opening'],
                      arm_ready=True, base_parked=True, base_detail='')
    verdict = SafetyFilter(Limits()).check(
        tuple(first[name] for name in spec.ACTION_COMPONENTS), context)
    print('  filter       %s' % describe_verdict(verdict))
    if verdict.accepted:
        try:
            solution = adapter.solve(RAW, verdict.action)
            print('  servo targets %s (residual %.2f mm)'
                  % (solution.targets, solution.residual_m * 1000.0))
        except AdapterError as error:
            print('  servo targets adapter rejects: %s' % error)
    print()

    print('  The bridge refuses to submit:')
    cases = (
        ('an observation 2 s old',
         lambda: planner.plan(observation, output, stamp + 2.0)),
        ('a normalised policy with no statistics',
         lambda: planner.plan(observation,
                              PolicyOutput([(0.0,) * 7] * 8, 'remote', 0.35, 1.0,
                                           POLICY_OUTPUT_NORMALIZED), now)),
        ('a policy running at the wrong control_dt',
         lambda: planner.plan(observation,
                              PolicyOutput([(0.0,) * 7] * 8, 'remote', 0.5, 1.0,
                                           POLICY_OUTPUT_PHYSICAL),
                              now)),
        ('an action commanding an unrealisable rotation',
         lambda: planner.plan(observation,
                              PolicyOutput([(0, 0.01, 0, 0, 0.05, 0, 0.5)] * 8,
                                           'p', 0.35, 1.0,
                                           POLICY_OUTPUT_PHYSICAL), now)),
    )
    for label, call in cases:
        try:
            call()
            print('    %-46s -> ACCEPTED (unexpected)' % label)
        except BridgeError as error:
            print('    %-46s -> %s' % (label, error.code))
    print()


def main():
    failed = False
    if '--no-tests' not in sys.argv[1:]:
        print('Offline test suites')
        print('-' * 60)
        for package, code, summary in run_suites():
            print('  %-16s %-6s %s' % (package, 'OK' if code == 0 else 'FAIL', summary))
            failed = failed or code != 0
        print()
    trace()
    head_trace()
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
