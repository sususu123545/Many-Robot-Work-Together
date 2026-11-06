#!/usr/bin/env python3
"""One-shot grasp: locate the ball, position the arm, close the jaws, lift.

The sequence that worked on the real car (2026-11-03 19:45, user-confirmed
"夹起来了"), with the two defects that later broke it removed:

  1. secure the ARM gate   (poll for the node's health window, then reset once)
  2. locate the ball       (pixel -> camera -> tool -> arm_base, hand-eye + FK)
  3. position the CHASSIS  so the ball sits 0.20..0.31 m out -- ONCE, measuring
     again after every creep.  It is frozen for the rest of the run: moving it
     translates arm_base and invalidates the ball coordinates, which is exactly
     what made an earlier version chase a stale target (53 mm FK residual, jaws
     closing in mid-air).
  4. for each dz, high to low:  walk the joints there one at a time (the vendor
     node ignores multi-joint trajectories and the dead-band eats small steps),
     take the GRIPPER gate, open in graded steps, then close until the feedback
     stalls.  A stall at or below 420 means the ball is between the jaws.
  5. secure the ARM gate again and lift; the jaws must NOT spring back to ~460.

dz (tool point above the ball centre) is the one number that decides success,
and it equals L, the drop of the fingertip below the vendor IK tool point.
0.030 put the tips on the floor, 0.069 missed by 7 cm.  Until L is measured the
levels are swept from the safe end -- too high only misses, too low can stall a
servo against the floor, which browned out the whole servo bus on 2026-11-03.

Exit code 0 = grasped, 2 = refused/failed safely, 1 = unexpected.
"""

import argparse
import json
import math
import os
import subprocess
import sys
import time
import urllib.request
from io import BytesIO

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, 'perception'))
sys.path.insert(0, os.path.join(REPO, 'ros2_ws', 'src', 'armpi_ros2', 'armpi_ros2'))

from PIL import Image                                       # noqa: E402
import car_approach as ca                                   # noqa: E402
from arm_kinematics import forward_tool_pose_raw            # noqa: E402
from arm_kinematics import inverse_kinematics_all           # noqa: E402
from arm_kinematics import forward_kinematics               # noqa: E402
from ball_in_arm_base import (camera_point, load_calibration,  # noqa: E402
                              mat_mul, mat_vec, CALIBRATION)
from gripper_jog import (status, read_r, reset_estop, set_gates,  # noqa: E402
                         read_stamp)
from rosbridge_client import Rosbridge                      # noqa: E402
import detectors                                            # noqa: E402
from ground_localizer import locate as locate_on_plane      # noqa: E402

SNAPSHOT = ('http://127.0.0.1:19099/snapshot'
            '?topic=/rear_camera/image_raw&type=ros_compressed')
TICK_RAD = math.radians(240.0) / 1000.0
SIGNS = {'joint1': 1.0, 'joint2': 1.0, 'joint3': -1.0, 'joint4': 1.0}
SAFE = {'joint1': (0, 1000), 'joint2': (150, 860),
        'joint3': (40, 950), 'joint4': (0, 980)}

# Mechanical range the vendor node publishes, copied verbatim from arm_jog.py so
# the in-process jog refuses exactly what the subprocess refused.  SAFE above is
# narrower on purpose: that one is the *grasp workspace*, not the joint range,
# and conflating the two would either block legal moves or allow illegal ones.
JOINT_LIMITS = {'joint1': (0, 1000), 'joint2': (125, 875),
                'joint3': (0, 1000), 'joint4': (0, 1000)}

# The arm controller silently drops a trajectory that moves one joint further
# than this in a single message.  Measured 2026-11-05: a 442-tick command simply
# never happened.  arm_jog enforces the same cap; it is the only thing standing
# between a bad IK solution and a 400-tick lunge.
MAX_SINGLE_STEP = 100

# Swept 2026-11-04 with the ball on the floor: the arm solves IK up to y=0.325 m
# (joint2 down at 166) and has no solution past 0.350 m.  The comfortable band is
# 0.25-0.30 m, where joint2 stays between 238 and 346.
REACH_MAX_Y = 0.34

# Wrist, elbow, shoulder, base.  Distal joints move while the arm is still
# upright, so they fight the least gravity.
JOINT_ORDER = {'joint4': 0, 'joint3': 1, 'joint2': 2, 'joint1': 3}

# Never let a joint-space path put the tool point closer than this to the
# supporting surface.  Calibrated against the one grasp that worked: the tool
# point sat 7.7 mm off the floor there and the jaws still cleared it by about a
# centimetre, because the fingertips hang slightly ABOVE the tool reference
# point.  8 mm therefore admits every legitimate grasp while still refusing the
# 5 mm approach that preceded a shoulder stall and a latched servo bus.
# 15 mm (was 7): 2026-11-06 the jaw scraped a tabletop -- ball localisation
# error is +-5 mm, so 7 mm of margin left the tip ~0 mm from the surface
# when the ball sat near furniture.
SAFE_FLOOR_MARGIN = 0.015

# Long descents are split into steps of at most this much vertical travel.
# 30 mm was too coarse for the last hop: the run on 2026-11-05 walked down 79/49/19
# with errors of 9/6/4 mm, then the final step to -10 mm overshot by 7.6 mm and put
# the tool point 1 mm UNDER the floor, tripping the guard after the fact.  The
# overshoot scales with step size, so shorten the steps instead of loosening the
# guard.
DESCENT_STEP_M = 0.018

# A 35 mm ball fits between the jaws at r_joint 298.  The open command is a
# request, not a promise: the same jaw reached 135 when fresh and stalled at 274
# after a few cycles, and both are wide enough.
OPEN_ADEQUATE = 295

# Smallest command that still moves a servo. Measured: 60 ticks delivers ~55,
# while 7-8 ticks delivers nothing at all.
MIN_JOG_TICKS = 14

# Largest gripper step the controller accepts in one message. Beyond this it
# answers "Step too large; use incremental targets" and drops the command.
GRIPPER_STEP = 40

# 10 mm forward.  Operator, after two clean grasps: "抓夹在抓的时候，再往前0.5cm更好",
# then "如果5mm不行就1cm更保险" -- and 10 mm is the right call, because 5 mm falls
# inside the servo dead-band (8-10 ticks ~ 8 mm) and might silently not happen.
# 10 mm is still far from the danger zone: jaws open 90 mm, ball is 35 mm across, so
# the ball can sit up to ~27 mm off centre before it touches a finger.  The rejected
# 40 mm offset was on the wrong side of that line.
TOOL_OFFSET = (0.0, 0.010, 0.0)

# Where the supporting surface sits in the arm_base frame, and which detector to
# trust.  Both are overridable from the command line.
VISION = {'backend': 'color', 'model': None, 'label': None, 'plane': -0.094}

# Tool point height above the ball centre.  First entry is the best estimate;
# the rest are fallbacks.
#
# dz decides whether the jaws close on the ball or in the air.  Two independent
# estimates of it now agree:
#   * touching the floor on 2026-11-03 stalled the shoulder with tool_z = -0.050,
#     which means dz ~ 0.044 (soft -- that stall may have been the bus dying, and
#     the attempt browned out the servo board, so it is not repeatable);
#   * the vendor URDF's own finger meshes put the pad mid point 45.7 mm below the
#     tool frame (sim/gripper_geometry.py), which is exactly the dz that places a
#     ball between the pads.
#
# The jaws are long (the finger bodies span nearly 100 mm below the tool point),
# so the vertical tolerance is generous -- roughly dz 0.01..0.09 all put the ball
# somewhere between the pads.  A miss is therefore more likely HORIZONTAL, which
# is why step [3/5] freezes the chassis and re-measures after every creep.
# Lowest first, because only the low end actually grips.  On 2026-11-05 a run
# with dz=+6 mm landed the tool point 5 mm ABOVE the ball centre, the jaws closed
# over the top of it and stopped at 446 (empty-jaw is 452) -- the operator's
# summary was exactly "在高度上还不够低".  The successful grab the day before had
# the tool point 8.5 mm BELOW the centre.  Sweeping high-to-low wasted the useful
# heights: each miss nudged the ball 40+ mm away, and by the time the low level
# came up it had no IK solution at all.
DZ_LEVELS = (-0.010, -0.006, 0.000)
EMPTY_CLOSE = 430     # jaws reach ~445 with nothing between them
BALL_CLOSE = 420      # anything at or below this means the ball is in the jaws


def log(message):
    print(message, flush=True)


def secure_gate(bridge, arm, gripper, timeout=120):
    """Grab the gate inside the node's health window (the timer cycles to zero)."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        state = status(bridge)
        if state.get('reset_eligible') and (state.get('healthy_seconds') or 0) > 11:
            if reset_estop(bridge):
                return set_gates(bridge, arm=arm, gripper=gripper)
        time.sleep(2)
    return False


def read_ticks_and_stamp(bridge):
    """One /joint_states message yields both the joint ticks and a usable stamp.

    The arm node rejects any command whose header stamp is not the robot clock,
    so a move needs a stamp as well as the current position.  Taking both from
    the same message halves the round trips compared with read_stamp() followed
    by read_ticks(), and each round trip is a 0.3 s drain plus the wait.
    """
    bridge.drain(0.3)
    message = bridge.wait_for_topic('/joint_states', 8.0, newest=True)
    if message is None:
        return None, None
    ticks = {}
    for name, position in zip(message['name'], message['position']):
        if name in SIGNS:
            ticks[name] = int(round(500.0 + SIGNS[name] * float(position) / TICK_RAD))
    header = (message.get('header') or {}).get('stamp') or {}
    stamp = ({'sec': int(header['sec']), 'nanosec': int(header.get('nanosec', 0))}
             if 'sec' in header else None)
    return ticks, stamp


def read_ticks(bridge):
    return read_ticks_and_stamp(bridge)[0]


def grab_frame():
    with urllib.request.urlopen(SNAPSHOT, timeout=15) as response:
        return Image.open(BytesIO(response.read())).convert('RGB')


def locate_ball():
    """Find the ball, but only accept a target the arm could actually reach.

    实测教训：柜子上有红色花纹，偶尔被当成球，算出来 1.29 m 远——抓取流程
    拿这种假目标只会白跑。按像素半径卡在 18..140 px（约 0.08..0.6 m），
    比"能不能看见"更贴近抓取的真实前提。
    """
    image = grab_frame()
    blob, reason = ca.locate_ball(image)
    if blob is not None and not 18.0 <= blob['radius_px'] <= 140.0:
        reason = 'candidate too small/large (r=%.0f px, not a graspable ball)' % blob['radius_px']
        blob = None
    if blob is None:
        from car_scan import connected_components
        best = None
        for component in connected_components(ca.redness_mask(image)):
            radius = (component['area'] / 3.14159) ** 0.5
            if 18 <= radius <= 140 and component['fill'] >= 0.5:
                if best is None or radius > best['radius_px']:
                    best = {'center_x': component['center_x'],
                            'center_y': component['center_y'],
                            'radius_px': radius}
        if best is None:
            return None, reason
        blob = best
    return blob, ''


def ball_in_base(ticks):
    """Ball centre in arm_base, from whichever detector is configured.

    The estimator casts the LOWEST pixel of the silhouette onto the floor plane
    and reads the object's height from the pixel height at that distance.  Two
    consequences matter for grasping:

      * no size prior, so a 60 mm ball, a bottle or a box all work unchanged --
        the old R/sin(alpha) form assumed a 35 mm sphere and quietly rescaled
        everything else;
      * the contact point is where the object actually touches the table, which
        is the coordinate the descent should aim at.

    Depth accuracy checked against the old estimator on 2026-11-04: the implied
    sphere radius came out 17.2 mm against a true 17.5 mm, i.e. 1.7 %, with no
    size assumption in the maths.
    """
    image = grab_frame()
    detector = detectors.build(VISION['backend'], model=VISION['model'])
    candidates = detector.detect(image)
    if VISION['label']:
        candidates = [d for d in candidates if d.label == VISION['label']]

    # Pick by SHAPE, not by area.  On 2026-11-05 the scene held a red ball at
    # (317,440) r=25 next to a red plastic basket at (257,108) r=87, and "largest
    # blob" chose the basket -- which then reported an object 236 mm tall floating
    # 118 mm above the floor, i.e. arithmetic nonsense from a wrong target.  The
    # ball is the only thing that is simultaneously round (w/h near 1), solid
    # (high fill), plausibly sized, and nearest to the camera (largest row).
    def round_enough(det):
        if not det.height:
            return False
        radius = (det.width * det.height / math.pi) ** 0.5
        return (0.85 <= det.width / det.height <= 1.20
                and det.confidence >= 0.62
                and 14.0 <= radius <= 70.0)

    shaped = [det for det in candidates if round_enough(det)]
    detection = max(shaped, key=lambda det: det.center_y) if shaped else None
    if detection is None:
        # Nothing round: fall back so the caller still sees a reason, but the
        # height sanity check downstream will reject nonsense.
        detection = detectors.pick_best(candidates)
    if detection is None:
        return None, 'the %s detector saw nothing' % VISION['backend']

    result, error = locate_on_plane(detection, ticks, VISION['plane'])
    if result is None:
        return None, error
    if not 5.0 <= result['height_mm'] <= 250.0:
        return None, ('implausible object height %.0f mm, refusing to aim at it'
                      % result['height_mm'])

    tool = forward_tool_pose_raw(ticks)
    return {'pixel': result['pixel'],
            'camera_distance_m': result['centre_distance_m'],
            'contact': result['contact'],
            'object_height_mm': result['height_mm'],
            'ball': result['centre'],
            'tool': [round(v, 4) for v in tool['translation_m']]}, ''


def ball_in_base_legacy(ticks):
    """Old estimator: distance = R / sin(angular radius).

    Exact only for a 35 mm sphere on the optical axis.  At the 27 degrees
    off-axis this camera usually works at, the disc is not a circle and the
    formula drifts.  Kept so a regression can be bisected from the command line.
    """
    blob, reason = locate_ball()
    if blob is None:
        return None, reason
    rotation_ct, translation_ct, _ = load_calibration(CALIBRATION)
    point_cam, distance = camera_point(blob['center_x'], blob['center_y'],
                                       blob['radius_px'])
    tool = forward_tool_pose_raw(ticks)
    point_tool = [sum(rotation_ct[i][k] * point_cam[k] for k in range(3))
                  + translation_ct[i] for i in range(3)]
    point_base = [sum(tool['rotation_matrix'][i][k] * point_tool[k] for k in range(3))
                  + tool['translation_m'][i] for i in range(3)]
    return {'pixel': [round(blob['center_x'], 1), round(blob['center_y'], 1),
                      round(blob['radius_px'], 1)],
            'camera_distance_m': round(distance, 4),
            'ball': [round(v, 4) for v in point_base],
            'tool': [round(v, 4) for v in tool['translation_m']]}, ''


def locate_with_backoff(bridge, ticks, label='locate'):
    """定位球；看不到就后退几厘米再试。

    换姿态（尤其切到前伸的预备位）会改变相机视角，0.25 m 的球会掉出画面下缘。
    盲目重试没用，后退一点让球重新进画面才是正解（2026-11-03 实测）。
    """
    estimator = VISION.get('estimator') or ball_in_base
    for attempt in range(3):
        located, reason = estimator(ticks)
        if located:
            return located, ''
        log('    %s: ball not visible (%s) -> backing off %.0f cm'
            % (label, reason, 6 + attempt * 0))
        drive(bridge, -0.05, 0.06 / 0.05)
        time.sleep(0.8)
        ticks = read_ticks(bridge) or ticks
    return None, 'still not visible after backing off'



def solve(ball, current, dz):
    """Pick the reachable pose that puts the TOOL POINT at ball + dz.

    ``dz`` is passed in, never searched.  It used to be "anything >= 35 mm",
    chosen by minimum joint travel, which is exactly how the same code produced
    both failures on 2026-11-03:  dz=0.030 put the fingertips on the floor (jaws
    then could not open, r_joint stuck at 367) and dz=0.069 closed the jaws 7 cm
    above the ball ("抓夹太高了").  A single scalar decided between success and
    nonsense, and nothing pinned it down.

    Grasping a ball of radius R resting on the floor needs the fingertips at the
    ball centre, i.e. ``dz = L`` (L = how far the tips hang below the tool
    point).  Until L is measured, callers sweep a few dz levels from HIGH to LOW:
    starting high only misses, starting low risks pressing the floor.
    """
    target = (ball[0] + TOOL_OFFSET[0],
              ball[1] + TOOL_OFFSET[1],
              ball[2] + dz + TOOL_OFFSET[2])
    rows = []
    # The wrist pitch is swept in 1-degree steps. It used to be 2, which is
    # eight times coarser than the 0.24-degree a single servo tick can express,
    # so the solver kept discarding poses that were a tenth of a degree away
    # from a hit and returning a neighbour that landed 8-12 mm off. With a 35 mm
    # ball and jaws that push rather than grab when they are off-centre, that
    # margin is the difference between holding and squeezing the ball away.
    for pitch in range(-150, -108):
        try:
            solutions = inverse_kinematics_all(target[0], target[1], target[2], pitch)
        except ValueError:
            continue
        for solution in solutions:
            servo = solution['servo_targets']
            j4, j3, j2, j1 = servo[3], servo[4], servo[5], servo[6]
            if not all(SAFE[name][0] <= value <= SAFE[name][1]
                       for name, value in (('joint1', j1), ('joint2', j2),
                                           ('joint3', j3), ('joint4', j4))):
                continue
            fx, fy, fz = forward_kinematics(solution['theta3'], solution['theta4'],
                                            solution['theta5'], solution['theta6'])
            error = math.dist((fx, fy, fz), target)
            travel = sum(abs(value - current[name]) for name, value in
                         (('joint1', j1), ('joint2', j2), ('joint3', j3),
                          ('joint4', j4)))
            rows.append({'travel': travel, 'error_mm': error * 1000,
                         'dz': dz, 'pitch': pitch,
                         'target': [round(v, 4) for v in target],
                         'joints': {'joint1': j1, 'joint2': j2,
                                    'joint3': j3, 'joint4': j4}})
    rows.sort(key=lambda row: (row['error_mm'] > 2.0, row['travel']))
    return rows[0] if rows else None


# ---------------------------------------------------------------------------
# Joint commands
#
# Every micro-move used to be a ``python3 arm_jog.py`` subprocess: a fresh
# interpreter, a fresh rosbridge connection, three status reads, a latch reset,
# an authorisation -- and only then one 2.2 s servo move.  A grasp needs 40-80 of
# them, so most of the wall-clock time went into launching processes rather than
# into moving joints.  The operator described the symptom from outside: "抓夹会有
# 很久的微动，大概1分钟，然后把抓夹放到正确的抓取位置".
#
# The publisher is now the process that already owns the connection.  Everything
# that made arm_jog safe is kept:
#
#   * the 0..1000 / 125..875 mechanical limit check;
#   * the 100-tick single-step cap the controller enforces *silently*;
#   * a fresh robot-clock stamp on every message (a zero stamp is rejected and
#     the only symptom is a servo that will not move while status reads READY);
#   * reading the joint back and refusing to call an unmoved joint a success --
#     "commanded, accepted, not moved" has happened on this arm before;
#   * re-securing the gate when a move reports exactly zero progress, which is
#     what the subprocess used to do unconditionally before every step.
#
# A caller that has not bound a connection still gets the old subprocess path,
# so nothing that imports this module can be broken by the change.
# ---------------------------------------------------------------------------

_BOUND = {'bridge': None}


def bind_bridge(bridge):
    """Let jog() publish on a connection the caller already owns."""
    _BOUND['bridge'] = bridge
    return bridge


def _reseat_gate(bridge):
    """Clear the boot latch and open the ARM gate from inside this process.

    Returns True only if it actually re-secured something.  A gate that was
    already open means the stall is mechanical (dead-band, contact, a servo that
    will not turn), and retrying the identical command would just waste 3 s.
    """
    try:
        state = status(bridge)
    except SystemExit as error:
        log('    gate check failed: %s' % error)
        return False
    if not state.get('estop') and state.get('enable_arm_motion'):
        return False
    log('    gate was closed (estop=%s arm_motion=%s) -- re-securing'
        % (state.get('estop'), state.get('enable_arm_motion')))
    if not reset_estop(bridge, quiet=True):
        return False
    return set_gates(bridge, arm=True, gripper=False, quiet=True)


def jog_ex(joint, delta, duration=1.5, bridge=None, retry=True):
    """Move one joint by ``delta`` ticks.  Returns ``(moved, ticks_after)``.

    ``ticks_after`` is the reading taken at the end of the move, so a caller
    that needs the new pose does not have to read /joint_states again -- that
    read was costing another 0.4 s on every hop of a descent.
    """
    bridge = bridge or _BOUND['bridge']
    if bridge is None:
        return _jog_subprocess(joint, delta, duration), None
    if joint not in SIGNS:
        log('    unknown joint %r' % joint)
        return False, None

    delta = int(delta)
    ticks, stamp = read_ticks_and_stamp(bridge)
    if ticks is None or joint not in ticks:
        log('    %s: no fresh joint feedback' % joint)
        return False, None
    before = ticks[joint]
    if delta == 0:
        return True, ticks

    target = before + delta
    low, high = JOINT_LIMITS[joint]
    if not low <= target <= high:
        log('    %s %+d refused: %d is outside %d..%d'
            % (joint, delta, target, low, high))
        return False, ticks
    if abs(delta) > MAX_SINGLE_STEP:
        log('    %s refused: a %d-tick single step (the controller caps at %d)'
            % (joint, abs(delta), MAX_SINGLE_STEP))
        return False, ticks
    if stamp is None:
        log('    %s: no robot-clock stamp -- the node would drop this command'
            % joint)
        return False, ticks

    bridge.publish('/arm_controller/command', {
        'header': {'stamp': stamp, 'frame_id': ''},
        'joint_names': [joint],
        'points': [{'positions': [SIGNS[joint] * (target - 500.0) * TICK_RAD],
                    'velocities': [], 'accelerations': [], 'effort': [],
                    'time_from_start': {'sec': int(duration),
                                        'nanosec': int(round((duration % 1) * 1e9))}}],
    }, 'trajectory_msgs/msg/JointTrajectory')

    time.sleep(duration + 0.8)
    after_ticks, _ = read_ticks_and_stamp(bridge)
    if after_ticks is None:
        log('    %s: no feedback after the move' % joint)
        return False, None
    after = after_ticks[joint]
    moved = abs(after - before)
    wanted = abs(target - before)
    ok = moved >= max(2, wanted * 0.6)
    log('    %s %+d %s  (target %d, moved %d of %d, error %d)'
        % (joint, delta, 'ok' if ok else 'NO MOVEMENT',
           target, moved, wanted, abs(after - target)))
    if not ok and moved == 0 and retry and _reseat_gate(bridge):
        return jog_ex(joint, delta, duration, bridge, retry=False)
    return ok, after_ticks


def jog(joint, delta, duration=1.5, bridge=None):
    """Backwards-compatible form: True only if the joint actually moved."""
    return jog_ex(joint, delta, duration, bridge)[0]


def _jog_subprocess(joint, delta, duration):
    """Fallback for callers that never bound a connection."""
    result = subprocess.run(
        [sys.executable, os.path.join(HERE, 'arm_jog.py'), '--joint', joint,
         '--delta-ticks', str(int(delta)), '--duration', str(duration)],
        capture_output=True, text=True, timeout=150)
    moved = 'moved as commanded' in result.stdout
    log('    %s %+d -> %s' % (joint, delta, 'ok' if moved else 'no movement'))
    return moved


def move_all(bridge, targets, duration=2.4):
    """Send several joint targets in ONE trajectory message.

    JointTrajectory carries parallel lists, so all four joints can be commanded
    together.  move_to used to jog them one at a time -- one message, one
    interpreter launch and one fresh rosbridge connection PER JOINT, and a grasp
    needs several iterations per hop.  That is well over a hundred process
    launches in a single grasp, and it dwarfed the servo travel it was driving.
    """
    names, positions = [], []
    for joint in ('joint1', 'joint2', 'joint3', 'joint4'):
        if joint in targets:
            names.append(joint)
            positions.append(SIGNS[joint] * (targets[joint] - 500.0) * TICK_RAD)
    if not names:
        return False
    stamp = read_stamp(bridge)
    if stamp is None:
        return False
    message = {
        'header': {'stamp': stamp, 'frame_id': ''},
        'joint_names': names,
        'points': [{'positions': positions,
                    'velocities': [], 'accelerations': [], 'effort': [],
                    'time_from_start': {'sec': int(duration),
                                        'nanosec': int(round((duration % 1) * 1e9))}}],
    }
    bridge.publish('/arm_controller/command', message,
                   'trajectory_msgs/msg/JointTrajectory')
    return True


def move_to(target, current, bridge, tolerance=10, attempts=8, max_step=90):
    """Walk each joint to its target, retrying while the dead-band eats steps.

    Three attempts used to be the limit and a long move could not finish inside
    it -- 3 x 60 ticks caps the travel at 180 ticks, so a pose needing more
    quietly ended short and the caller then measured a 53 mm FK residual and
    blamed the geometry (2026-11-03).

    Joints are visited WRIST FIRST, not root first.  Lowering the shoulder first
    lays the arm out horizontally, which is exactly when the elbow and wrist
    carry the most torque: on 2026-11-04 joint2 reached 257 and then joint3 and
    joint4 each sat through eight "no movement" rounds, creeping only 31 and 49
    ticks because the servo dead-band swallowed every 60-tick command.  Moving
    them while the arm is still upright gives them far less load to fight.

    Returns (ticks, ok).  ``ok`` is False when the tool point came within
    SAFE_FLOOR_MARGIN of the supporting surface, which has already happened once:
    with the shoulder down and the elbow still up, the arm formed an L whose tool
    point sat 8.6 mm BELOW the floor and pressed on it (user: "现在压到地板了").
    Joint-space interpolation guarantees nothing about the path the tool takes,
    so the only defence is to look at the tool height after every step.
    """
    ticks = dict(current)
    floor = VISION['plane']
    # Per-joint sequential jogging.  A multi-joint version was tried and reverted
    # on 2026-11-05: sending all four targets in one JointTrajectory is 4x fewer
    # messages, but the Cartesian result is not controllable, because each joint
    # gets its own allowance and the tool point moves by their SUM.  A 90-tick
    # command dropped the tool point 14 cm when the hop asked for 4.5 cm, and the
    # guard tripped on the floor.  Shortening the allowance to 35 made it safe but
    # no faster than this.  Measured: sequential 346 s OK, multi-joint 182 s FAIL,
    # multi-joint-35 360 s FAIL.  Keep the version that works.
    for joint in sorted(target, key=lambda name: JOINT_ORDER.get(name, 9)):
        value = target[joint]
        for attempt in range(attempts):
            delta = value - ticks[joint]
            if abs(delta) <= tolerance:
                break
            step = max(-max_step, min(max_step, delta))
            # Anything under about a dozen ticks vanishes into the servo
            # dead-band: joint3 sat through ten consecutive "-7 -> no movement"
            # rounds during refinement and never budged.  Inflate to a step that
            # will actually move; the loop re-reads feedback and corrects after.
            if 0 < abs(step) < MIN_JOG_TICKS:
                step = MIN_JOG_TICKS if step > 0 else -MIN_JOG_TICKS
            # jog_ex hands back the reading it took after the move, so the loop
            # does not spend another 0.4 s re-reading /joint_states.  That read
            # used to happen on every hop of every descent.
            _, fresh = jog_ex(joint, step, duration=2.2, bridge=bridge)
            if fresh is None:
                break
            ticks.update(fresh)
            tool_z = forward_tool_pose_raw(ticks)['translation_m'][2]
            if tool_z < floor + SAFE_FLOOR_MARGIN:
                log('    ABORT: tool point is only %.0f mm above the surface'
                    % ((tool_z - floor) * 1000))
                return ticks, False
    return ticks, True


def open_wide(bridge, floor_tick=169):
    """Open the jaw in verified increments.

    The node rejects any single step beyond about 60 ticks with "Step too large;
    use incremental targets" -- and in this call the rejection is invisible,
    because read_r simply hands back the old position.  That made a perfectly
    healthy gripper look mechanically stuck at 397 while every command was being
    dropped on the floor (2026-11-04).  Send small steps and confirm each one
    actually moved the jaw.
    """
    position = read_r(bridge) or 440
    for attempt in range(24):
        if position <= floor_tick + 8:
            break
        step = max(-GRIPPER_STEP, min(GRIPPER_STEP, floor_tick - position))
        gripper_send(bridge, position + step, duration=2.0)
        time.sleep(2.4)
        fresh = read_r(bridge)
        if fresh is None:
            continue
        if fresh == position and attempt >= 2:
            log('    open stalled at %d -- jaw will not go wider' % fresh)
            break
        if fresh != position:
            log('    open -> %d' % fresh)
        position = fresh
    return position


def close_until_stall(bridge, limit=470):
    """Close in big steps until the feedback stops advancing (= something is there)."""
    position = read_r(bridge) or 169
    stalled = 0
    trace = []
    while position < limit:
        target = min(position + 60, limit)
        gripper_send(bridge, target, duration=2.0)
        time.sleep(3.0)
        fresh = read_r(bridge)
        if fresh is None:
            break
        trace.append(fresh)
        log('    close %d -> %d' % (target, fresh))
        if fresh == position:
            stalled += 1
            if stalled >= 2:
                break
        else:
            stalled = 0
        position = fresh
    return position, trace


def gripper_gate(bridge):
    return secure_gate(bridge, arm=False, gripper=True)


def arm_gate(bridge):
    return secure_gate(bridge, arm=True, gripper=False)


def gripper_send(bridge, tick, duration=2.0):
    bridge.drain(0.3)
    message = bridge.wait_for_topic('/joint_states', 6.0)
    stamp = message.get('header', {}).get('stamp') if message else {'sec': 0, 'nanosec': 0}
    bridge.publish('/gripper_controller/command', {
        'header': {'stamp': stamp, 'frame_id': ''},
        'joint_names': ['r_joint'],
        'points': [{'positions': [((tick - 700) * math.pi / 1000.0)],
                    'velocities': [], 'accelerations': [], 'effort': [],
                    'time_from_start': {'sec': int(duration), 'nanosec': 0}}],
    }, 'trajectory_msgs/msg/JointTrajectory')


def drive(bridge, linear, seconds):
    twist = {'linear': {'x': float(linear), 'y': 0.0, 'z': 0.0},
             'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}}
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        bridge.publish('/cmd_vel', twist, 'geometry_msgs/msg/Twist')
        time.sleep(0.05)
    for _ in range(3):
        bridge.publish('/cmd_vel', {'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                                    'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}},
                       'geometry_msgs/msg/Twist')
        time.sleep(0.05)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--open-tick', type=int, default=169, help='wide-open r_joint')
    parser.add_argument('--close-limit', type=int, default=470)
    parser.add_argument('--lift', type=int, default=90, help='total joint2 lift ticks')
    parser.add_argument('--no-lift', action='store_true')
    parser.add_argument('--dz', default=None,
                        help='comma separated dz levels in metres, high to low')
    parser.add_argument('--vision', default='ground', choices=('ground', 'legacy'),
                        help='ground: ray-meets-plane, no size prior.  '
                             'legacy: the old 35 mm-sphere assumption')
    parser.add_argument('--backend', default='color', choices=('color', 'onnx'),
                        help='which detector feeds the ground estimator')
    parser.add_argument('--model', default='/opt/vision/models/yolo11n.onnx')
    parser.add_argument('--label', default=None,
                        help='restrict the onnx detector to one class name')
    parser.add_argument('--plane', type=float, default=-0.094,
                        help='height of the supporting surface in arm_base (m)')
    parser.add_argument('--approach-chassis', action='store_true',
                        help='let the car creep toward a distant ball.  Off by '
                             'default: measured 2026-11-04, the outstretched arm '
                             'pushes the ball away instead of getting closer')
    args = parser.parse_args()
    levels = ([float(part) for part in args.dz.split(',')] if args.dz
              else list(DZ_LEVELS))

    VISION['backend'] = args.backend
    VISION['model'] = args.model
    VISION['label'] = args.label
    VISION['plane'] = args.plane
    VISION['estimator'] = ball_in_base if args.vision == 'ground' else ball_in_base_legacy
    log('vision: %s estimator, %s detector, plane z=%.3f m'
        % (args.vision, args.backend, args.plane))

    report = {'ok': False, 'stage': 'start', 'detail': ''}
    bridge = bind_bridge(Rosbridge('127.0.0.1', 19091, timeout=8))
    try:
        log('[1/5] securing the arm gate')
        if not arm_gate(bridge):
            report.update(stage='arm_gate', detail='cannot secure arm gate')
            return finish(report, 2)
        time.sleep(1)

        ticks = read_ticks(bridge)
        if ticks is None:
            report.update(stage='feedback', detail='no joint feedback')
            return finish(report, 2)
        report['joints_start'] = ticks

        log('[2/5] locating the ball')
        located, reason = locate_with_backoff(bridge, ticks, 'locate')
        if located is None:
            report.update(stage='locate', detail='ball not visible: %s' % reason)
            return finish(report, 2)
        log('    ball in arm_base: %s  (pixel %s)'
            % (located['ball'], located['pixel']))

        # The chassis positions ONCE, here, and is frozen afterwards.  Moving it
        # translates arm_base, so a ball measured before the move no longer sits
        # at the same arm_base coordinates -- that stale target is what produced
        # FK residuals of 53 mm and jaws closing in mid-air on 2026-11-03.
        #
        # 2026-11-04: creeping closer is only worth it when the arm genuinely
        # cannot reach.  Attempting it unconditionally made things worse -- each
        # 60 mm command moved the car about 12 mm (mecanum slip), the ball sat at
        # pixel v=470 of 480 so approaching pushed it out of frame, the re-locate
        # failed, locate_with_backoff backed off, and the two loops cancelled out.
        # y drifted 0.336 -> 0.359 m and all three IK levels became unreachable,
        # although dz=0.054 had been solvable at the starting position.
        log('[3/5] checking whether the current pose already reaches the ball')
        reachable = [dz for dz in levels
                     if solve(located['ball'], ticks, dz) is not None]
        ball_y = located['ball'][1]
        if reachable:
            log('    yes, IK exists at dz=%s -- chassis stays put'
                % ', '.join('%.0f mm' % (dz * 1000) for dz in reachable))
        elif ball_y > REACH_MAX_Y:
            # Measured 2026-11-04: commanding the chassis forward 100 mm made the
            # ball vanish, and backing off 150 mm revealed it 77 mm FARTHER away
            # than before -- the outstretched arm had pushed it.  Closing the gap
            # by driving is therefore not an option, and beyond REACH_MAX_Y the
            # arm cannot get there at all.  Say so instead of thrashing.
            report.update(stage='reach',
                          detail=('ball at y=%.3f m is past the arm reach of '
                                  '%.2f m; move it closer by hand'
                                  % (ball_y, REACH_MAX_Y)))
            return finish(report, 2)
        elif not args.approach_chassis:
            report.update(stage='reach',
                          detail=('no IK at y=%.3f m and chassis approach is '
                                  'disabled (it pushes the ball away)' % ball_y))
            return finish(report, 2)
        else:
            log('    no IK from here, so the chassis has to close the gap')
            for attempt in range(5):
                y = located['ball'][1]
                if 0.20 <= y <= 0.31:
                    log('    ball y=%.3f m -- in range, chassis frozen from here' % y)
                    break
                gap = max(-0.06, min(0.06, y - 0.26))
                log('    ball y=%.3f m -> %s %.2f m'
                    % (y, 'forward' if gap > 0 else 'back', abs(gap)))
                drive(bridge, 0.07 if gap > 0 else -0.07, abs(gap) / 0.07)
                time.sleep(0.8)
                ticks = read_ticks(bridge) or ticks
                located, reason = locate_with_backoff(bridge, ticks, 're-locate')
                if located is None:
                    report.update(stage='locate',
                                  detail='lost the ball while positioning: %s' % reason)
                    return finish(report, 2)
            else:
                log('    ball y=%.3f m is still out of range, trying anyway'
                    % located['ball'][1])
        report['ball'] = located['ball']
        report['pixel'] = located['pixel']

        # Order the dz levels by distance.  Measured 2026-11-05: a ball at 0.245 m
        # gripped at dz=-10 mm, one at 0.22 m gripped at 0 mm.  Trying the wrong
        # level first costs an entire descent+close+lift cycle -- 3 to 5 minutes --
        # which is the single biggest waste in a run, bigger than any hop size.
        ball_y = located['ball'][1]
        if ball_y >= 0.238:
            preferred = (-0.010, -0.006, 0.000)
        elif ball_y >= 0.205:
            preferred = (0.000, -0.006, -0.010)
        else:
            preferred = (0.000, -0.010, -0.006)
        ordered = [dz for dz in preferred if dz in levels]
        ordered += [dz for dz in levels if dz not in ordered]
        levels = ordered
        log('[4/5] grasp at y=%.3f m, trying dz=%s'
            % (ball_y, ' '.join('%+.0f' % (dz * 1000) for dz in levels)))

        grasped = None
        for index, dz in enumerate(levels):
            log('  attempt %d/%d: tool point %+d mm above the ball centre'
                % (index + 1, len(levels), round(dz * 1000)))
            fresh = read_ticks(bridge) or ticks
            # Re-locate before every attempt after the first.  A failed close
            # pushes the ball, and aiming at a stale coordinate is how a retry
            # turns into a guaranteed miss.  The camera sits ~0.2 m up after the
            # lift, so the ball is back in view at this point.
            # Re-locate before every attempt after the first, but NEVER reverse.
            # `locate_with_backoff` backs the car up 6 cm whenever the camera
            # cannot see the ball -- and "cannot see the ball" is the NORMAL state
            # once the jaws are down around it, because the fingers occlude it.
            # On 2026-11-05 the operator watched exactly this: the jaws reached
            # the ball, the car then reversed and dragged it away.  His words:
            # "抓夹只差闭合就抓到小球了，为什么小车要后退".  From here on the chassis
            # is frozen, full stop.
            if index > 0:
                again, reason = None, 'no joint feedback'
                probe = read_ticks(bridge)
                if probe is not None:
                    again, reason = ball_in_base(probe)
                    fresh = probe
                if again is not None:
                    drift = math.dist(again['ball'], located['ball'])
                    located = again
                    log('     ball re-measured, moved %.0f mm since the last attempt'
                        % (drift * 1000))
                    report.setdefault('relocations', []).append(
                        {'dz': dz, 'drift_mm': round(drift * 1000, 1),
                         'ball': again['ball']})
                else:
                    log('     ball not visible (%s) -- keeping the last fix, NOT '
                        'backing off: the jaws are what is hiding it' % reason)
            solution = solve(located['ball'], fresh, dz)
            if solution is None:
                log('     no IK at dz=%.3f -> skipping this level' % dz)
                continue
            if not arm_gate(bridge):
                report.update(stage='arm_gate', detail='cannot secure arm gate')
                return finish(report, 2)
            time.sleep(1)
            # Refine in closed loop.  One pass leaves 10-21 mm of error because
            # the servos have an 8-10 tick dead-band and a 60-tick command only
            # delivers about 55: every joint finishes a few ticks short and the
            # errors add up along the arm.  Re-solving from the MEASURED joints
            # each time walks the tool point in instead of hoping one pass lands.
            # Swing j1 into line FIRST, on its own, before any descent.
            # A ball sitting 50 mm to the side needs j1 to travel ~48 ticks, and
            # mixing that yaw into the descent path overruns badly: on 2026-11-05
            # the very first hop overshot 6 mm and put the tool point 1.3 mm under
            # the floor (the guard caught it, health stayed READY).  Yaw first,
            # then descend straight down.
            wanted = solve(located['ball'], ticks, dz)
            if wanted is not None:
                yaw_delta = wanted['joints']['joint1'] - ticks['joint1']
                if abs(yaw_delta) >= MIN_JOG_TICKS:
                    log('     aligning j1 by %+d ticks before the descent' % yaw_delta)
                    while abs(wanted['joints']['joint1'] - ticks['joint1']) >= MIN_JOG_TICKS:
                        step = wanted['joints']['joint1'] - ticks['joint1']
                        _, fresh_j1 = jog_ex('joint1', max(-55, min(55, step)),
                                             duration=1.6, bridge=bridge)
                        if fresh_j1 is None:
                            break
                        ticks = fresh_j1
                    log('     j1 now %d (want %d)'
                        % (ticks['joint1'], wanted['joints']['joint1']))

            # Walk down in ~18 mm hops before refining.  One 425-tick shoulder
            # drop overshot 14 mm on 2026-11-05 and drove the fingertips into the
            # table, which stalled the shoulder and latched the servo bus into
            # READ_ERROR.  Short hops keep every move small enough for the servo
            # dead-band to absorb, so the guard on the final height is never the
            # thing standing between us and a grasp.
            height = forward_tool_pose_raw(ticks)['translation_m'][2] - located['ball'][2]
            walk = []
            # Two-speed descent only.  A "coarse drop" (one 90-tick jump down to
            # 60 mm above the ball) was tried and reverted on 2026-11-05: it saved
            # hops but removed the buffer before the final approach, so the next
            # level overshot 8 mm and drove the tool point through the floor,
            # wasting a whole attempt -- 519 s versus 256 s for this version.
            # Speed ideas that trade away the approach buffer have all failed.
            while height - dz > 0.045:
                height -= 0.045
                walk.append(height)
            while height - dz > DESCENT_STEP_M:
                height -= DESCENT_STEP_M
                walk.append(height)
            walk.append(dz)
            log('     descent plan: %d hops %s'
                % (len(walk), ' '.join('%+.0f' % (d * 1000) for d in walk)))

            # OPEN THE JAW BEFORE DESCENDING.  A closed gripper is a wedge: the
            # two fingers are pressed together, so coming down on the ball simply
            # shoves it aside instead of straddling it.  open_wide() used to run
            # AFTER move_to, which meant the entire descent happened with the
            # jaws shut -- and every failed attempt on 2026-11-05 pushed the ball
            # 47-53 mm away.  The operator spotted it: "要把抓夹打开呀，闭合就很容易
            # 把小球推走".
            if not gripper_gate(bridge):
                report.update(stage='gripper_gate',
                              detail='cannot open the jaw before the descent')
                return finish(report, 2)
            pre_open = open_wide(bridge, args.open_tick)
            log('     jaw opened to %d before descending' % pre_open)
            if not arm_gate(bridge):
                report.update(stage='arm_gate',
                              detail='cannot re-authorise the arm after opening')
                return finish(report, 2)
            time.sleep(1)

            miss = None
            blocked_by_floor = False
            for hop, step_dz in enumerate(walk):
                solution = solve(located['ball'], ticks, step_dz)
                if solution is None:
                    log('     hop %d: no IK at dz=%.0f mm' % (hop + 1, step_dz * 1000))
                    break
                # Big steps far from the floor, small ones near it: the guard has
                # only 15 mm of room and a coarse command overshoots.
                ticks, path_ok = move_to(solution['joints'], ticks, bridge,
                                         tolerance=3, attempts=12,
                                         max_step=60 if step_dz > 0.045 else 40)
                if not path_ok:
                    log('     hop %d: aborted near the surface' % (hop + 1))
                    blocked_by_floor = True
                    break
                tool = forward_tool_pose_raw(ticks)['translation_m']
                miss = math.dist(tool, solution['target'])
                log('     hop %d: dz=%+.0f mm -> %.0f mm off'
                    % (hop + 1, step_dz * 1000, miss * 1000))

            # A floor abort means the tool point is ALREADY inside
            # SAFE_FLOOR_MARGIN.  Refining from there commands yet another
            # descent into the same wall.  Measured 2026-11-06: attempt 1
            # aborted at 11 mm above the surface, the refinement pass then
            # aborted again at 5 mm, and the level was thrown away after ~50 s
            # of motion that could never have helped.  Go straight to the next
            # dz level instead -- it starts from a pose the guard has not
            # rejected.
            if blocked_by_floor:
                log('     the surface stopped this descent; not refining into it')
            # Skip the refinement pass when the hops already landed well.  Each
            # move_to here costs 20-30 s, and on a good run the last hop is within
            # a couple of millimetres anyway (measured: 1-4 mm), so the loop would
            # spend half a minute to gain nothing.
            elif miss is not None and miss <= 0.008:
                log('     within %.0f mm after the hops, skipping refinement'
                    % (miss * 1000))
            else:
                for refine in range(2):
                    solution = solve(located['ball'], ticks, dz)
                    if solution is None:
                        break
                    ticks, path_ok = move_to(solution['joints'], ticks, bridge,
                                             tolerance=3, attempts=10, max_step=30)
                    if not path_ok:
                        log('     refine %d: aborted near the surface' % (refine + 1))
                        blocked_by_floor = True
                        break
                    tool = forward_tool_pose_raw(ticks)['translation_m']
                    miss = math.dist(tool, solution['target'])
                    log('     refine %d: %.0f mm off' % (refine + 1, miss * 1000))
                    if miss <= 0.008:
                        break
            tool = forward_tool_pose_raw(ticks)['translation_m']
            report.setdefault('attempts', []).append(
                {'dz': dz, 'joints': ticks,
                 'tool': [round(v, 4) for v in tool],
                 'miss_mm': None if miss is None else round(miss * 1000, 1),
                 'aborted_by_floor_guard': blocked_by_floor})
            if miss is None:
                log('     the tool point never reached this level -> skipping it')
                continue
            if miss > 0.016:
                log('     still %.0f mm off after refinement -> skipping this level'
                    % (miss * 1000))
                continue

            if not gripper_gate(bridge):
                report.update(stage='gripper_gate', detail='cannot secure gripper gate')
                return finish(report, 2)
            time.sleep(1)
            # The jaw is already open, opened before the descent so the fingers
            # could straddle the ball rather than bulldoze it.  Just read where
            # it ended up; re-opening here would only add a second, slower pass.
            position = read_r(bridge) or pre_open
            report['r_open'] = position
            # The jaw does not always reach the requested open tick.  Measured
            # range across one session: 135 when fresh, 274 after several open
            # and close cycles.  A 35 mm ball still fits at 298, so refusing at
            # >260 threw away the single best-aligned attempt of the run
            # (dz=0.038, 9.9 mm off) for no physical reason.
            if position > OPEN_ADEQUATE:
                report.update(stage='gripper_open',
                              detail='jaw did not open (r_joint=%s)' % position)
                return finish(report, 2)

            closed, trace = close_until_stall(bridge, args.close_limit)
            report.setdefault('close_attempts', []).append(
                {'dz': dz, 'trace': trace, 'r': closed})
            log('     jaws stopped at %d -- %s'
                % (closed, 'ball between them' if closed <= BALL_CLOSE
                   else 'nothing between them'))
            if closed <= BALL_CLOSE:
                grasped = {'dz': dz, 'r_closed': closed}
                break
            log('     raising and trying the next height')

        if grasped is None:
            report.update(stage='close',
                          detail='jaws never stalled on the ball at dz=%s mm'
                          % [round(v * 1000) for v in levels])
            return finish(report, 2)
        report['dz_used'] = grasped['dz']
        report['r_closed'] = grasped['r_closed']

        log('[5/5] lifting and checking the jaws hold')
        if not arm_gate(bridge):
            report.update(stage='arm_gate_lift', detail='cannot secure arm gate for lift')
            return finish(report, 2)
        time.sleep(1)
        if not args.no_lift:
            remaining = args.lift
            while remaining > 0:
                step = min(45, remaining)
                jog('joint2', step, duration=1.8)
                remaining -= step
                time.sleep(0.8)
        # Read the jaw angle with retries, and NEVER default it to 0.
        # `read_r(bridge) or 0` turned a failed read into 0, and 0 is <=
        # EMPTY_CLOSE, so a missing reading was reported as "the ball is still
        # between them" -- a false positive on the single check that decides
        # whether the grasp worked.  Observed 2026-11-05: the run printed
        # "jaws held at 0" while the ball was genuinely held at 324.
        held = None
        for attempt in range(5):
            held = read_r(bridge)
            if held is not None:
                break
            time.sleep(0.6)
        if held is None:
            report.update(stage='lift',
                          detail='could not read the jaw angle after lifting; '
                                 'refusing to call this a success')
            return finish(report, 2)
        report['r_after_lift'] = held
        # An empty jaw springs back to ~460 once the gate moves to the arm
        report['ok'] = held <= EMPTY_CLOSE
        report['stage'] = 'done' if report['ok'] else 'lift'
        report['detail'] = ('jaws held at %d -> the ball is still between them' % held
                            if report['ok'] else
                            'jaws returned to %d (the ball slipped)' % held)
        return finish(report, 0 if report['ok'] else 2)
    finally:
        bridge.close()


def finish(report, code):
    print('RESULT ' + json.dumps(report, ensure_ascii=False), flush=True)
    return code


if __name__ == '__main__':
    sys.exit(main())
