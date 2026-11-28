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
import servo_health                                          # noqa: E402
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

# What the servo can actually resolve.  A joint asked for a target closer than
# this simply never gets there, so any convergence test finer than 12 ticks is
# unsatisfiable by construction.
TICK_DEADBAND = 12

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
# summary was exactly "在高度上还不够低".  Sweeping high-to-low wastes the useful
# heights: each miss nudges the ball 40+ mm away, and by the time the low level
# comes up it has no IK solution at all.
# Deepest first.  A successful grasp must put the ball against the MIDDLE of the
# finger pads, and r_closed is the tell: historical solid grips stopped at
# 334 / 345 / 350 / 390, while the 2026-11-11 attempt that only caught the ball
# with the LOWER edge of the fingers closed all the way to 402 (empty is ~445).
# That attempt used dz=+20 mm, i.e. the tool point 20 mm above the ball centre,
# which holds the pads too high; the operator saw the consequence directly
# ("这次抓取小球很极限是抓夹的下边部分抓到小球，接触面小很容易掉") and the ball
# then fell out.  So aim lower first and let the sweep climb only if it must.
#
# Back to (0.008, 0.016) on 2026-11-16 evening.  +20/+28 looked safer on paper
# (more clearance from the guard line) but it loses the grasp: the descent
# cannot close the last ~15 mm because joint3 stalls 9 ticks short of target and
# 9 ticks is inside the 12-tick dead-band, so move_tool_to reports "never
# reached this level" and the level is skipped with the pads ABOVE the ball.
# The close then runs to 452 (pads touching, nothing between them).
# dz=+8 is the level that has actually worked: 2026-11-12 measured miss 6.2 mm
# and r_closed 390 with the pads on the ball.
# Deepest first: the funnel ejects balls contacted on their upper hemisphere
# -- 2026-11-24 run10 missed laterally by only 9 mm yet the taper slid the
# ball out of the V (pads closed to 450, empty), while run9's descent happened
# to land the tool at ball-centre height and held.  Contact depth, not lateral
# aim alone, decides wedge vs eject.  The operator asked for a deeper bite
# still (both wins settled at 383-386, barely past EDGE_GRIP 395), so 0.000
# -- tool point at the located ball centre -- is tried first.  Its margin over
# the floor-guard line is only 0.3-2.5 mm (the located centre varies), so it
# aborts often; that is acceptable because an aborted level falls through to
# 0.004 at the cost of ~1.5 min.  Anything below 0.000 aims through the guard
# line and can never land.
DZ_LEVELS = (0.000, 0.004, 0.008, 0.016)
EDGE_GRIP = 395       # r_closed above this means only the ball's edge is held
# Every level must be ABOVE the ball centre, and positive means "tool point
# higher than the centre".  The old (-0.010, -0.006, 0.000) trio was impossible
# on this floor: measured 2026-11-10, the plane is z=-0.094, SAFE_FLOOR_MARGIN
# keeps the tool point at or above z=-0.079, and a 30 mm ball's centre sits at
# z=-0.0805 -- BELOW the guard line.  So dz<=0 asked the arm to descend into a
# zone the guard forbids; all three levels aborted with "tool point is only
# 4 mm above the surface", the tool ended ~12 mm under the ball centre, and the
# jaws closed beneath the ball three rounds in a row.
EMPTY_CLOSE = 430     # jaws reach ~445 with nothing between them
# 350, not 420: on the raised-box setup the bite depth DECIDES whether the grip
# survives the lift.  Measured 2026-11-25 over seven episodes on a 74 mm box:
# closes at 322-334 (lateral miss 6-7 mm) held; closes at 371, 400 and 406
# (miss 10-15 mm) all shed the ball during the lift.  Accepting 420 let the
# script lift a grip it should have rejected, wasting the episode, when the
# +/-15 mm nudge sweep -- which exists to fix exactly that lateral miss -- would
# have been tried instead.  A close above this now counts as "not held" and the
# sweep continues.
BALL_CLOSE = 350
MIN_CLOSE_TRAVEL = 60   # ...but only if the jaws actually travelled this far

# Refuse to close when the tool point sits this far above the ball centre --
# the jaws would stall on air and the attempt would be silently wasted.
# 2026-11-24: after a floor-abort lift the close ran 79 mm above the ball.
MIDAIR_CLOSE_LIMIT_M = 0.045


def close_blocked_midair(tool_z, ball_z, limit=MIDAIR_CLOSE_LIMIT_M):
    """True when closing now would grab nothing but air."""
    return (tool_z - ball_z) > limit


def jaws_hold(closed, opened):
    """True only when the jaws really closed onto something.

    ``closed <= BALL_CLOSE`` on its own is not evidence.  On 2026-11-11 the jaw
    sat at 169 -- its own OPEN value -- because the gripper gate had been dropped
    before the close command, so nothing moved; 169 <= 420 was then reported as
    "ball between the jaws" while the photograph showed the ball still on the
    floor.  A real grasp both stops early AND travels a long way from open
    (measured successful closes: 334-390 starting from 169-298).
    """
    if closed is None or opened is None:
        return False
    return closed <= BALL_CLOSE and (closed - opened) >= MIN_CLOSE_TRAVEL

# --- closing on the ball, not on the estimate ------------------------------
# On 2026-11-10 the arm reported "within 6 mm" of the ball, the jaws then closed
# empty at 448, and the camera showed the ball sitting 25-30 mm BEYOND the
# fingertips, with the fingers passing in front of it.  The ground estimator's
# forward range reads short by about that much at this height.  Rather than bake
# a fudge constant into solve(), walk the tool in until the jaws actually stall:
# an empty close reaches ~445, a held 35 mm ball stops them at 300-420, and that
# stall is physical ground truth that no estimate can argue with.
CLOSE_CREEPS = 3      # kept for compatibility; the sweep below drives the retries
CREEP_MM = 0.012      # 12 mm per crawl: small enough to never bulldoze the ball
LATERAL_MM = 15       # ~1.5 cm sideways per nudge -- the offset the operator saw
# joint1 at ~0.20 m reach turns 0.24 deg per tick ~= 0.84 mm of tool travel, so
# 18 ticks is the ~15 mm of sideways movement needed here.
LATERAL_TICKS = 18


def log(message):
    print(message, flush=True)


def secure_gate(bridge, arm, gripper, timeout=120):
    """Grab the gate inside the node's health window (the timer cycles to zero).

    Refuses outright when a servo is already at its critical temperature.  On
    2026-11-06 a jammed wrist took servo 5 to 65 C and the protection tripped
    seven times while the pipeline kept authorising motion; the temperature was
    readable the whole time.  A recorded ``last_fault`` is only logged, not
    refused on: it survives an estop reset and would block a healthy arm
    forever.
    """
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        state = status(bridge)
        health = servo_health.read_health(state)
        hot = {sid: value for sid, value in health['temps'].items()
               if value >= servo_health.CRITICAL_TEMP_C}
        if hot:
            log('    REFUSING to authorise motion: servo %s'
                % ', '.join('%d at %d C' % (sid, value)
                            for sid, value in sorted(hot.items())))
            return False
        if health['last_fault']:
            log('    note: last fault was [%s] %s'
                % (health['last_fault'].get('kind'),
                   health['last_fault'].get('message')))
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


#: The ball is ~10 cm from the wrist camera when the jaws hold it, so it reads
#: as a big near blob.  Measured 2026-11-24/25: held = 76.9 px radius, dropped
#: (box 25 cm below) = 11.1 px.  45 px sits between them with margin.
HELD_BALL_RADIUS_PX = 45.0


def ball_in_jaw():
    """Camera check for "is the ball actually in the jaws?", for after a lift.

    The jaw reading cannot answer this.  It is a position-controlled servo that
    holds its last commanded target, so it reports ~the closed value whether or
    not the ball is still there -- measured 2026-11-25 episode 3: the run
    reported "jaws held at 391" (a false success) while the camera showed an
    empty gripper and the operator watched the ball drop.

    Returns (held, note); held is None when the camera cannot be read, so a
    camera fault never turns a real grasp into a failure.
    """
    try:
        image = grab_frame()
    except Exception as error:                      # noqa: BLE001 - report, don't die
        return None, 'camera unavailable: %s' % error
    from car_scan import connected_components
    components = [c for c in connected_components(ca.redness_mask(image))
                  if c['area'] >= 400]
    if not components:
        return False, 'no red blob in the camera'
    best = max(components, key=lambda c: c['area'])
    radius = (best['area'] / 3.14159) ** 0.5
    # Size alone is not the test.  A held ball sits ~10 cm from the lens, so it
    # is BIGGER than the frame -- it gets clipped by the bottom edge and only an
    # arc is left, whose area-derived radius comes out SMALL (measured
    # 2026-11-25 episode 4: a genuinely held ball read r=42 px, under the 45 px
    # threshold, and the run was wrongly reported as a shed grip).  A ball that
    # fell off is fully inside the frame -- it has room to be a complete circle.
    # So: touching the bottom edge while being wide means it is right at the
    # jaws; a complete small circle lower down means it landed somewhere.
    bottom_y = best['center_y'] + best.get('height', 0) / 2.0
    touches_bottom = bottom_y >= image.size[1] - 6
    wide = best.get('width', 0) >= 90
    held = (touches_bottom and wide) or radius >= HELD_BALL_RADIUS_PX
    return held, ('largest red blob r=%.0f px, w=%d, bottom=%.0f (frame %d) at '
                  '(%d,%d)' % (radius, best.get('width', 0), bottom_y,
                               image.size[1], best['center_x'], best['center_y']))


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
    # Distance is the check the height cannot make.  A red object far across the
    # room still reports a sensible HEIGHT, because the ground-ray math divides
    # the apparent size by the very distance that is wrong.  Measured 2026-11-11:
    # a 12.5 px blob high in the frame came back as a ball 1.727 m away, the
    # grasp was refused with "past the arm reach", and a round was lost.  The
    # ball we are chasing is always on the floor within the 0.34 m reach.
    planar = math.hypot(result['centre'][0], result['centre'][1])
    if planar > 0.60:
        return None, ('red object %.2f m away is outside any graspable range, '
                      'so it is not the ball' % planar)

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
    """Locate the ball.  NO reversing -- that cure is worse than the disease.

    The original idea (2026-11-03) was sound for a stale camera pose: back up a
    little so a ball that fell below the frame comes back into view.  In practice
    it destroys the setup it is trying to fix.  "Cannot see the ball" is the
    NORMAL state during a grasp -- the open jaws sit right over it and occlude it
    -- so the tool reversed away from a perfectly good position and then
    complained the ball was out of reach.

    Measured 2026-11-11 (evening): two consecutive back-offs of 6 cm each moved
    the ball from 0.23 m to 0.453 m, past the 0.34 m arm reach, and the round was
    thrown away with "move it closer by hand".  Distance is approach_ball's job;
    this function's job is only to say whether the ball is visible right now.
    """
    estimator = VISION.get('estimator') or ball_in_base
    located, reason = estimator(ticks)
    if located:
        return located, ''
    # Detection is intermittent on a dim scene (R median ~145): measured
    # 2026-11-28, 1 frame in 3 missed a ball that two other frames found, and a
    # single miss aborted the whole episode at this stage.  Re-read the frame --
    # nothing moves, so this is exactly the "is the ball visible right now"
    # question asked again, not a workaround for a bad pose.
    for attempt in range(1, 4):
        time.sleep(0.5)
        located, reason = estimator(ticks)
        if located:
            log('    %s: ball found on retry %d' % (label, attempt))
            return located, ''
    log('    %s: ball not visible after 4 frames (%s) -- not reversing, that '
        'only pushes it away' % (label, reason))
    return None, reason



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


def move_to(target, current, bridge, tolerance=TICK_DEADBAND, attempts=8,
            max_step=90):
    """Walk each joint to its target, retrying while the dead-band eats steps.

    ``tolerance`` must never be finer than the servo dead-band.  It was 3 for
    every descent, and the dead-band is ~12 ticks, so the loop could never
    satisfy its own exit condition: it ran all 12 attempts on every joint, at
    ~2.4 s per attempt.  Measured 2026-11-11: one descent hop spent about two
    minutes printing "joint3/4 -14 NO MOVEMENT (moved 7 of 14)" -- the joints
    WERE moving, just never to within 3 ticks of a target they cannot resolve.
    Asking for the dead-band is both honest and four times faster.

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
            # Re-evaluate the cap BEFORE EVERY JOINT STEP, not once per hop.
            # descent_step_cap() at the hop level was not enough: move_to() then
            # walked several 60-tick shoulder commands with the same allowance,
            # and the second of them put the tool through the floor guard
            # (2026-11-12, accept_r4.log).  A raised pose leaves 125 mm of room,
            # so the hop-level cap allowed 60; after one step only ~60 mm is
            # left and the next 60-tick step is already too much.
            live_cap = descent_step_cap(ticks, max_step)
            step = max(-live_cap, min(live_cap, step))
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


def descent_step_cap(ticks, base):
    """Shrink a descent step as the tool approaches the floor guard.

    A joint step is not a Cartesian step.  When the arm is stretched toward a
    ball at 0.26 m, one 60-tick shoulder command can drop the tool point tens of
    millimetres, while the target sits only ~12 mm above the guard line.  The
    guard then trips halfway through a hop and the whole height level is thrown
    away (2026-11-12, accept_r3.log: hop 2 aborted, empty close at 448).

    Use at most half of the remaining vertical room, assuming a conservative
    1 mm of tool travel per tick.  High above the floor this leaves ``base``
    untouched, so the fast part of the descent is unchanged; near the ball it
    converges to a single dead-band step.
    """
    tool_z = forward_tool_pose_raw(ticks)['translation_m'][2]
    room_m = tool_z - (VISION['plane'] + SAFE_FLOOR_MARGIN)
    # 0.20, not 0.35.  Even the tighter coefficient let a single hop cross the
    # guard on 2026-11-23 (tmp/run1.log): hop 1 asked the tool to descend 95 mm
    # but its IK target moved joint3 by 96 ticks and joint2 by 190, and joint3
    # travels roughly 1 mm of tool height PER TICK -- so the hop spent ~170 mm
    # of vertical room and hit the guard at 8 mm above the floor.  Budgeting a
    # fifth of the remaining room per step keeps one joint's overshoot from
    # consuming the whole hop's allowance.
    cap = max(MIN_JOG_TICKS, int(max(0.0, room_m) * 1000 * 0.20))
    return min(base, cap)


def move_tool_to(point, ticks, bridge, max_step=25, rounds=3, tol=0.016,
                 floor=None):
    """Drive the TOOL POINT to `point`, closing the loop on forward kinematics.

    move_to() aims at the IK solution's JOINT angles, and every servo then
    settles up to a dead-band (12 ticks ~ 2.9 deg) away from them, which at a
    0.25 m reach is about 12 mm of tool travel.  So move_to can report success
    while the fingers are a centimetre from where they were sent: measured
    2026-11-11, a commanded z of -0.0758 landed at -0.0660 and the jaws closed
    on nothing, three times in a row.

    Two things make this loop work where a plain re-solve does not:
      * it measures the achieved point with kinematics and corrects the
        residual, exactly the way the chassis turns close their loop on odom;
      * it COMMANDS THE CORRECTION WITH OVERSHOOT, because a correction smaller
        than one dead-band would be swallowed by the same dead-band that caused
        it.  The overshoot is clamped against the floor guard.
    """
    floor = VISION['plane'] if floor is None else floor
    ceiling = floor + SAFE_FLOOR_MARGIN
    for _ in range(rounds):
        tool = forward_tool_pose_raw(ticks)['translation_m']
        residual = [point[i] - tool[i] for i in range(3)]
        distance = math.sqrt(sum(r * r for r in residual))
        if distance <= tol:
            return ticks, True
        # NO overshoot.  The 1.6x correction was added to beat the 12-tick
        # servo dead-band, but move_to now converges at tolerance=12 with a
        # closed-loop re-measurement, and hops 1-3 of a real descent landed
        # within 1-2 mm without it (2026-11-12, tmp/accept_r1.log).  On the last
        # hop the overshoot pushed the tool 13 mm BELOW the aim, tripped the
        # floor guard and turned a good approach into an empty close at r=444.
        # Commanding exactly the target is both more accurate and safer here.
        command = [tool[i] + residual[i] for i in range(3)]
        # Never command deeper than the target itself; the guard is a backstop,
        # not an aiming point.
        command[2] = max(command[2], min(point[2], ceiling + 0.001))
        ball = (command[0] - TOOL_OFFSET[0],
                command[1] - TOOL_OFFSET[1],
                command[2] - TOOL_OFFSET[2])
        solution = solve(ball, ticks, 0.0)
        if solution is None:
            return ticks, False
        ticks, path_ok = move_to(solution['joints'], ticks, bridge,
                                 tolerance=TICK_DEADBAND, attempts=10,
                                 max_step=max_step)
        if not path_ok:
            return ticks, False
    tool = forward_tool_pose_raw(ticks)['translation_m']
    return ticks, math.dist(tool, point) <= 0.012


def open_wide(bridge, floor_tick=115):
    """Open the jaw in verified increments.

    The node rejects any single step beyond about 60 ticks with "Step too large;
    use incremental targets" -- and in this call the rejection is invisible,
    because read_r simply hands back the old position.  That made a perfectly
    healthy gripper look mechanically stuck at 397 while every command was being
    dropped on the floor (2026-11-04).  Send small steps and confirm each one
    actually moved the jaw.
    """
    position = read_r(bridge) or 440
    stalled = 0
    regates = 0
    for attempt in range(24):
        if position <= floor_tick + 8:
            break
        step = max(-GRIPPER_STEP, min(GRIPPER_STEP, floor_tick - position))
        gripper_send(bridge, position + step, duration=2.0)
        time.sleep(2.4)
        fresh = read_r(bridge)
        if fresh is None:
            continue
        if abs(fresh - position) <= 2 and attempt >= 2:
            # Same +/-1 tick wobble as close_until_stall: strict equality let a
            # jaw sitting at 397 look like it was "moving" on every other read.
            # was never commanded.  Re-secure once before believing a stall.
            if regates < 2:
                regates += 1
                log('    jaw did not move -- re-securing the gripper gate')
                if gripper_gate(bridge):
                    continue
            # Three identical reads, not one.  A single unchanged reading is not
            # proof of a stall: measured 2026-11-11 evening, the jaw reported
            # "open stalled at 397" and then opened to 114 when asked again a
            # moment later -- the servo was simply slower than the read.  Giving
            # up early left every descent with only ~40 mm of gap around a 35 mm
            # ball, which is exactly why the fingers kept touching it on the way
            # down.
            stalled += 1
            time.sleep(1.2)
            if stalled >= 3:
                log('    open stalled at %d -- jaw will not go wider' % fresh)
                break
            continue
        stalled = 0
        if fresh != position:
            log('    open -> %d' % fresh)
        position = fresh
    return position


def close_until_stall(bridge, limit=470):
    """Close in big steps until the feedback stops advancing (= something is there)."""
    position = read_r(bridge) or 169
    stalled = 0
    regates = 0
    trace = []
    steps = 0
    while position < limit:
        # Belt and braces: even if the stall detector is fooled again, the close
        # must end.  15 steps is well above the ~6 a 128 -> 470 sweep needs.
        steps += 1
        if steps > 15:
            log('    close loop hit its step cap at %d -- stopping' % position)
            break
        target = min(position + 60, limit)
        gripper_send(bridge, target, duration=2.0)
        time.sleep(3.0)
        fresh = read_r(bridge)
        if fresh is None:
            break
        trace.append(fresh)
        log('    close %d -> %d' % (target, fresh))
        # A +/-1 tick wobble is normal servo noise, NOT motion.  Strict equality
        # was fooled by the jaw alternating 131/132 on 2026-11-12: the loop never
        # accumulated a stall, ran 55 reads (3 s each = 165 s of dead waiting),
        # and only stopped when three 131s happened to line up.  Compare with a
        # tolerance and bail out quickly on a jaw that is genuinely stuck.
        #
        # The tolerance is 12, not 2.  Measured 2026-11-25 on a ball held at a
        # deep bite: the pads pressed into the foam and the jaw oscillated
        # 334 <-> 341 -- a 7-tick wobble under load, well outside +/-2 -- so
        # every step looked like progress and the loop ran 40 close commands
        # (200 s of sustained stalling, gripper servo 43 C) before being killed
        # by hand.  A real step advances ~55 of the 60 asked, so 12 separates
        # motion from wobble with a wide margin.
        if abs(fresh - position) <= 12:
            # A jaw that never leaves its open value was not commanded at all.
            # The motion gate has a lifetime and it expires on its own mid-run --
            # measured 2026-11-11: the status stream showed estate=False
            # arm_motion=False while the descent was still going, so every
            # subsequent joint and jaw command was silently dropped and the whole
            # round failed with "jaws never stalled" while the ball sat between
            # the fingers.  The arm path already re-secures and retries; the
            # gripper path did not.  Do it here before believing the stall.
            if regates < 2:
                regates += 1
                log('    jaw did not move -- re-securing the gripper gate')
                if gripper_gate(bridge):
                    continue
            # Same reasoning as open_wide: three identical reads, not two.  The
            # jaw is sometimes slower than the read, and a premature stall here
            # reports a healthy gripper as "never moved" (2026-11-11).
            stalled += 1
            if stalled >= 3:
                break
        else:
            stalled = 0
        position = fresh
    return position, trace


def gripper_gate(bridge):
    # Keyword arguments, not positional.  secure_gate's signature is
    # (bridge, arm, gripper); on 2026-11-11 a refactor passed the two booleans
    # by position and swapped them, so every "arm_gate" actually opened the
    # gripper and vice versa.  The symptom was commands silently dropped while
    # status still read READY -- which was misdiagnosed as a servo bus fault and
    # cost several unnecessary power cycles.  Never write these as bare booleans.
    return gate_with_retry(
        lambda b: secure_gate(b, arm=False, gripper=True), bridge)


def arm_gate(bridge):
    return gate_with_retry(
        lambda b: secure_gate(b, arm=True, gripper=False), bridge)


def gate_with_retry(function, bridge, *arguments, tries=3, wait=4.0):
    """Take a motion gate, tolerating a momentary drop in arm feedback.

    The servo bus occasionally misses a reply, and reset_estop then answers
    "Need 10 seconds of fresh valid five-joint arm feedback" -- which is not a
    permanent fault, it clears on its own a few seconds later.  Treating that as
    fatal aborted entire grasp rounds: measured 2026-11-11 evening, a transient
    drop stopped the lateral sweep after its first miss, so the round that would
    have been rescued by the second nudge was thrown away instead.
    """
    last_error = ''
    for attempt in range(tries):
        if attempt:
            log('    gate retry %d/%d after a feedback drop (%s)'
                % (attempt + 1, tries, last_error[:80]))
            time.sleep(wait)
        try:
            if function(bridge, *arguments):
                return True
            last_error = 'gate refused'
        except (Exception, SystemExit) as error:
            # status()/reset_estop() raise on a transient feedback drop.  The
            # original version of this helper had no except at all, so the very
            # exception it was written to tolerate escaped and the retries
            # never ran (found by tools/test_gate_semantics.py on 2026-11-12).
            # SystemExit is included because gripper_jog.status uses it for a
            # failed status read; here it means "try again", not "quit".
            last_error = '%s: %s' % (type(error).__name__, error)
    return False


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
    # 115, not 169: the jaw's mechanical stop is near 100 and it will happily
    # reach ~107, but the old default asked for 169 and open_wide stopped as
    # soon as it was within 8 ticks -- so every descent happened with a gap of
    # only ~40 mm around a 35 mm ball.  Measured 2026-11-11, the operator saw
    # the consequence: "抓夹碰到小球之后，小球滚到抓夹中间，然后抓到" -- the
    # fingers were touching the ball on the way down and relying on it rolling
    # into place.  15 ticks of margin is kept off the stop.
    parser.add_argument('--open-tick', type=int, default=115,
                        help='wide-open r_joint (stop is ~100)')
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
        # Baseline the fault counter: a fault recorded BEFORE this run is
        # history (it survives an estop reset), but one that appears during the
        # run means something is binding right now and the arm should stop.
        monitor = servo_health.Monitor(status(bridge))
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
        z_bias = 0.0        # measured (achieved - target) height error so far
        for index, dz in enumerate(levels):
            # The IK solution lands lower than asked on this arm, and the answer
            # is measured, not assumed: on 2026-11-11 targets of -0.056 / -0.062 /
            # -0.068 came out at -0.087 / -0.093 / -0.090, i.e. 22-31 mm too low,
            # which put the fingers under the ball and the floor guard then
            # aborted every attempt.  So aim higher by whatever the previous
            # level missed by, and let the sweep converge instead of repeating
            # the same miss three times.
            # Never aim below the ball centre.  The floor guard line is z=-0.079
            # and a ball on this floor has its centre at about -0.079, so a
            # negative dz would ask the tool to descend into a forbidden zone.
            # A stale +45 mm bias once produced aim_dz=-10 mm and the level was
            # lost before it started.  Floor at 6 mm keeps DZ_LEVELS[0]=8 usable.
            aim_dz = max(0.006, min(0.045, dz - z_bias))
            log('  attempt %d/%d: tool point %+d mm above the ball centre%s'
                % (index + 1, len(levels), round(aim_dz * 1000),
                   '' if abs(z_bias) < 0.002
                   else ' (aiming %+.0f mm to cancel a %+.0f mm miss)'
                   % (-z_bias * 1000, z_bias * 1000)))
            # Everything downstream of here (the hop plan, solve(), the report)
            # uses this corrected height; enumerate() rebinds dz next round.
            dz = aim_dz
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
            # How much legal descent is left BEFORE the step is chosen.  This
            # used to be decided from the BALL's height, which is wrong: a ball
            # sitting 15 mm above the floor made every hop of the whole descent
            # crawl at 12 ticks, so a 4-hop descent spent minutes.  What matters
            # is where the TOOL is right now -- high up it can step 60 ticks,
            # and only the last 50 mm has to be gentle.
            def floor_room(pose):
                return (forward_tool_pose_raw(pose)['translation_m'][2]
                        - VISION['plane'] - SAFE_FLOOR_MARGIN)

            near_floor = floor_room(ticks) < 0.050
            for hop, step_dz in enumerate(walk):
                solution = solve(located['ball'], ticks, step_dz)
                if solution is None:
                    log('     hop %d: no IK at dz=%.0f mm' % (hop + 1, step_dz * 1000))
                    break
                # Big steps far from the floor, small ones near it: the guard has
                # only 15 mm of room and a coarse command overshoots.
                #
                # Measured 2026-11-10, three dz levels in a row: the descent
                # aborted with "tool point is only 4 mm above the surface", the
                # tool ended at z=-0.090 when the ball centre is at -0.0783 and
                # the guard line at -0.079, i.e. ~12 mm TOO LOW -- and the jaws
                # then closed underneath the ball (trace ran to 447, empty).
                # The ball centre sits only 15.7 mm above the floor, so at this
                # height there is no room for a 40-tick last step.
                room = floor_room(ticks)
                near_floor = room < 0.050
                # 20/16 instead of 60/40: a joint step is not a Cartesian step,
                # and joint3 converts ~1 mm of tool height per tick.  2026-11-23
                # a 40-49 tick step spent the entire hop's vertical budget in one
                # command and the level was lost at 8 mm above the floor.
                max_step = descent_step_cap(
                    ticks, 12 if near_floor else (20 if step_dz > 0.045 else 16))
                # Close the loop on the TOOL POINT, not on the joint angles: a
                # joint-space move stops within one dead-band of its target and
                # that is ~12 mm at the fingers, which is why every grasp landed
                # about a centimetre off (2026-11-11).
                ticks, path_ok = move_tool_to(solution['target'], ticks, bridge,
                                              max_step=max_step)
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
            # ...but only when the VERTICAL error is small too.  `miss` is the
            # 3D distance to the aim, and a 7 mm total with 2 mm lateral and
            # 6 mm of height passes "landed well" while the jaws then close on
            # the ball's upper hemisphere: measured 2026-11-28, a lateral-offset
            # grasp closed at 390 (equatorial bites measure 322-350) purely
            # because the descent stopped ~6 mm high.  The tool height is what
            # decides the bite, so refine whenever it is off by more than 3 mm.
            elif (miss is not None and miss <= 0.008
                  and abs(tool[2] - solution['target'][2]) <= 0.003):
                log('     within %.0f mm after the hops (z %+.0f mm), skipping '
                    'refinement'
                    % (miss * 1000, (tool[2] - solution['target'][2]) * 1000))
            elif miss is not None and miss <= 0.008:
                log('     within %.0f mm but %+.0f mm high/low -- refining the '
                    'height, it decides where the jaws close'
                    % (miss * 1000, (tool[2] - solution['target'][2]) * 1000))
            else:
                for refine in range(2):
                    solution = solve(located['ball'], ticks, dz)
                    if solution is None:
                        break
                    ticks, path_ok = move_tool_to(solution['target'], ticks,
                                                  bridge,
                                                  max_step=descent_step_cap(
                                                      ticks,
                                                      12 if near_floor else 16))
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
            # What the IK actually delivered, versus what it was told.  Fed into
            # the next level's aim so a systematic miss is corrected instead of
            # repeated.  Clamped: one bad hop must not throw the aim far away.
            wanted_z = located['ball'][2] + dz + TOOL_OFFSET[2]
            if blocked_by_floor:
                # The tool we just measured is AFTER the "lift clear" recovery,
                # not where the descent actually stopped.  Feeding that inflated
                # height into the next level's aim produced +45 mm of bogus bias
                # on 2026-11-12.  A guard abort carries no height information.
                z_bias = 0.0
                log('     height calibration skipped: the floor guard aborted '
                    'this level, so its final tool height is not a measurement')
            else:
                z_bias = max(-0.045, min(0.045, tool[2] - wanted_z))
            report.setdefault('attempts', []).append(
                {'dz': dz, 'joints': ticks,
                 'tool': [round(v, 4) for v in tool],
                 'wanted': [round(located['ball'][0] + TOOL_OFFSET[0], 4),
                            round(located['ball'][1] + TOOL_OFFSET[1], 4),
                            round(wanted_z, 4)],
                 'z_bias_mm': round(z_bias * 1000, 1),
                 'miss_mm': None if miss is None else round(miss * 1000, 1),
                 'aborted_by_floor_guard': blocked_by_floor})
            log('     landed %+.0f mm in z against the aim (bias now %+.0f mm)'
                % ((tool[2] - wanted_z) * 1000, z_bias * 1000))
            if blocked_by_floor:
                # The guard stopped the arm hard against the surface.  Starting
                # the next level from down there just shoves the ball again --
                # measured 2026-11-11, the ball moved 63 mm between two attempts.
                log('     lifting clear of the surface before the next level')
                if arm_gate(bridge):
                    for _ in range(8):
                        now = read_ticks(bridge) or ticks
                        if (forward_tool_pose_raw(now)['translation_m'][2]
                                > VISION['plane'] + 0.080):
                            ticks = now
                            break
                        _, lifted = jog_ex('joint2', 40, duration=2.0,
                                           bridge=bridge)
                        if lifted:
                            ticks.update(lifted)
                        time.sleep(0.4)
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

            # Search around the ball with the jaws themselves as the sensor.
            # The operator watched the 2026-11-10 attempts and reported the ball
            # sitting 1-2 cm to the RIGHT of the fingers while the camera put it
            # 23 px from the centre of the image -- i.e. the optical axis is not
            # on the gripper's centre line.  A fixed offset cannot be trusted
            # either, because arm_base's x flips meaning as joint1 turns (the
            # same ball read x=-0.60 at joint1=645 and x=+0.18 at joint1=318).
            # So sweep instead: one close, one nudge each way, then a little
            # forward.  Each close either stalls on the ball (r <= 420) or does
            # not, and that is the only judge that cannot be argued with.
            #
            # EVERY sideways move happens ABOVE the ball, never at its height.
            # The first version nudged j1 while the jaws were down beside the
            # ball, which swept the open fingers straight through it: the
            # operator watched exactly that on 2026-11-11 ("在左右调整的时候，抓夹
            # 还碰到小球了"), it is why the ball drifted 63 mm between two
            # attempts, and it silently moved the thing we were aiming at.
            # So: rise clear, shift, descend vertically, close.  Horizontal
            # motion within 45 mm of the ball is forbidden from here on.
            NUDGES = (('as is', 0, 0.0),
                      ('%d mm to one side' % LATERAL_MM, LATERAL_TICKS, 0.0),
                      ('%d mm to the other side' % LATERAL_MM,
                       -2 * LATERAL_TICKS, 0.0))
            clear_z = max(located['ball'][2] + 0.060,
                          VISION['plane'] + 0.075)
            closed = None
            trace = []
            for index, (label, j1_delta, forward) in enumerate(NUDGES):
                if index:
                    log('     %s -- rising clear of the ball first' % label)
                    if not arm_gate(bridge):
                        log('     cannot re-authorise the arm -- stopping')
                        break
                    perch = (located['ball'][0], located['ball'][1] - 0.010,
                             clear_z)
                    ticks, rose = move_tool_to(perch, ticks, bridge, max_step=40)
                    if not rose:
                        log('     could not rise clear -- stopping the sweep')
                        break
                    if j1_delta:
                        moved, after = jog_ex('joint1', j1_delta, bridge=bridge)
                        if after:
                            ticks = after
                        time.sleep(0.8)
                    aim = (located['ball'][0], located['ball'][1] + forward,
                           located['ball'][2])
                    solution = solve(aim, ticks, dz)
                    if solution is None:
                        log('     no IK for that nudge -- stopping')
                        break
                    if not gripper_gate(bridge):
                        log('     cannot secure the gripper gate -- stopping')
                        break
                    open_wide(bridge, args.open_tick)
                    position = read_r(bridge) or position
                    if not arm_gate(bridge):
                        log('     cannot re-authorise the arm -- stopping')
                        break
                    ticks, came_down = move_tool_to(
                        solution['target'], ticks, bridge,
                        max_step=descent_step_cap(ticks, 25))
                    if not came_down:
                        log('     the surface stopped the descent -- stopping')
                        break
                    time.sleep(0.5)
                    # arm_gate() above disarmed the gripper; without this the
                    # close is dropped and the jaw reads its own open value.
                    if not gripper_gate(bridge):
                        log('     cannot re-arm the gripper -- stopping the sweep')
                        break
                # The mid-air guard guards EVERY close, not only nudges.
                # Measured 2026-11-24: a floor-abort lift raised the tool to
                # ~93 mm above the plane and the first close then ran there,
                # ~79 mm above the ball centre -- the jaws stalled at 452 on
                # nothing while the operator watched them close well above the
                # ball.  The guard existed only inside the nudge branch.
                if close_blocked_midair(
                        forward_tool_pose_raw(ticks)['translation_m'][2],
                        located['ball'][2]):
                    log('     jaws are still %.0f mm above the ball -- not '
                        'closing in mid-air' % (height * 1000))
                    continue
                closed, trace = close_until_stall(bridge, args.close_limit)
                report.setdefault('close_attempts', []).append(
                    {'dz': dz, 'nudge': label, 'opened': position,
                     'trace': trace, 'r': closed})
                held = jaws_hold(closed, position)
                log('     jaws stopped at %d (opened at %d) -- %s'
                    % (closed, position,
                       'ball between them' if held
                       else ('jaw never moved' if closed == position
                             else 'nothing between them')))
                if held:
                    break
            if jaws_hold(closed, position):
                grasped = {'dz': dz, 'r_closed': closed}
                if closed > EDGE_GRIP:
                    # Says out loud what the number means, because a marginal
                    # grip is how a "successful" grasp ends up on the floor a
                    # second later (2026-11-11: closed at 402, good enough to
                    # pass, ball dropped anyway).
                    log('     only the EDGE of the ball is between the jaws '
                        '(closed %d) -- a deeper dz would grip better' % closed)
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
        jaw_says_held = held <= EMPTY_CLOSE
        # ...but the jaw reading cannot prove the ball is there (see ball_in_jaw).
        # Ask the camera too, and let the camera overrule a jaw false positive.
        seen, note = ball_in_jaw()
        report['camera_after_lift'] = {'held': seen, 'note': note}
        log('    camera after lift: %s (%s)'
            % ({True: 'ball in the jaws', False: 'no ball',
                None: 'unknown'}[seen], note))
        if seen is False and jaw_says_held:
            report['ok'] = False
            report['stage'] = 'lift'
            report['detail'] = ('jaw read %d but the camera sees no ball -- the '
                                'grip shed it (measured 2026-11-25)' % held)
            return finish(report, 2)
        report['ok'] = jaw_says_held
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
