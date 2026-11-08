#!/usr/bin/env python3
"""One-command ball retrieval.

Pipeline (per round, up to --max-rounds):

    1. SEARCH   search_around (4 directions on the first round, then 1)
                -> found?  extract ball position from its stdout
    2. APPROACH approach_ball  (aim + creep, exits 0 when inside grasp band)
    3. GRASP    grasp_sequence (ok:true means jaws held through the lift)
    4. NOT FOUND? pick the emptiest lidar sector, turn, walk --walk-m, retry

The sibling tools run as SUBPROCESSES: each manages its own rosbridge
connection, estop and gates, which keeps the commander simple and every
stage's log intact.  The lidar read and the chassis motion (turn_to / drive)
run here in-process -- publishing must never gap 0.35 s (MotionLease), and a
subprocess boundary is exactly such a gap.

Every round appends to artifacts/ball_history.json and, from 2026-11-06, that
history is READ back: `ball_memory` turns the last sighting plus the odometry
driven since into a single heading worth checking first.  A hit turns a
four-quadrant sweep (four returns to the boot pose and up to four rotations, each
gated on lidar clearance) into a one-quadrant sweep.  A miss falls through to the
full sweep, so the memory can only save time, never cost coverage.
"""

import argparse
import math
import re
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from rosbridge_client import Rosbridge          # noqa: E402
import ball_memory                              # noqa: E402
import grasp_sequence as gs                     # noqa: E402
import approach_ball as ab                      # noqa: E402
from scan_clean import clean_scan, emptiest_sector, rotation_clearance_agreed  # noqa: E402

HISTORY = Path(__file__).resolve().parent.parent / 'artifacts' / 'ball_history.json'

# search_around prints "FOUND at bearing 90 deg (relative to the start heading)"
# and "ball [x, y, z]  -> 0.412 m ahead, 33.9 mm tall".  The bearing is relative
# to the heading at the START of that sweep, which is the pose we snapshot before
# launching it -- so the two together give an odom-frame sighting.
BEARING_RE = re.compile(r'FOUND at bearing\s+(-?\d+(?:\.\d+)?)\s+deg')
RANGE_RE = re.compile(r'->\s*([\d.]+)\s*m ahead')

# A map-frame pose is only worth recording if the particle filter has actually
# collapsed.  See read_map_pose() for why: amcl happily publishes the mean of a
# 1.2 m-wide cloud, and that number is worse than nothing because it looks
# authoritative.  Limits are ~2x the converged figures measured 2026-11-07.
MAP_POSE_MAX_SIGMA_XY = 0.35      # metres
MAP_POSE_MAX_SIGMA_YAW = 25.0     # degrees


def log(msg):
    print(msg, flush=True)


def run_tool(args, label):
    """Run a sibling CLI tool, stream its tail, return (ok, output)."""
    log('=== %s: %s ===' % (label, ' '.join(args)))
    result = subprocess.run(
        [sys.executable] + args, capture_output=True, text=True, timeout=900)
    for line in result.stdout.strip().splitlines()[-10:]:
        log('  | ' + line)
    return result.returncode == 0, result.stdout


def append_history(entry):
    """Append one legacy event record next to the memory entries.

    ``ball_memory`` writes ``kind`` records; the round-by-round log below writes
    ``event`` records.  Both live in one file so a human reading it sees the
    whole story in order, and ``ball_memory.predict`` simply ignores records it
    does not recognise.
    """
    hist = ball_memory.load(str(HISTORY))
    hist.append(entry)
    ball_memory.save(str(HISTORY), hist, limit=400)


def commit_memory(entries):
    """Write our kind records WITHOUT erasing the legacy round log.

    Bug found on the first real ball_memory run, 2026-11-07.  main() loads
    `history` ONCE at startup, but append_history() re-reads the file on every
    event, so the two writers drift apart.  Each `ball_memory.save(HISTORY,
    history)` then wrote a stale snapshot and silently deleted the round events
    appended since.  The finished run had 31 records where 34 were expected:
    three 'search' / 'approach' / 'grasp' events had vanished.  The sighting and
    outcome survived only because they happened to be written last.

    Merge against the CURRENT disk contents instead: keep every non-kind record
    (the human-readable log) and take the kind records from our in-memory list.
    """
    on_disk = ball_memory.load(str(HISTORY))
    legacy = [item for item in on_disk if not item.get('kind')]
    mine = [item for item in entries if item.get('kind')]
    ball_memory.save(str(HISTORY), legacy + mine)


def read_pose(bridge):
    """Car pose in the odom frame: metres, yaw in degrees CCW.

    Drain first.  Without it `wait_for_topic` hands back a socket-buffered
    sample, and this project has already measured four consecutive turns as
    0.0 deg because of exactly that (2026-11-05).
    """
    bridge.drain(0.4)
    m = bridge.wait_for_topic('/odom', timeout=3)
    if m is None:
        return None
    p = m['pose']['pose']['position']
    o = m['pose']['pose']['orientation']
    x, y, z, w = o['x'], o['y'], o['z'], o['w']
    yaw = math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
    return {'x': p['x'], 'y': p['y'], 'yaw': yaw}


def read_map_pose():
    """Car pose in the MAP frame, from nav2_amcl's /amcl_pose.

    Uses its OWN rosbridge connection.  Subscribing to /odom and /amcl_pose on
    one connection cannibalises messages on this car (measured 2026-11-06: a
    shared connection delivered only 29% of /cmd_vel).  Returns None when amcl
    is not up, so callers can fall back to odom without guessing.

    ALSO returns None when amcl has not converged.  A particle filter with a
    broad cloud still publishes its mean, and that mean is worthless -- writing
    it into `ball_memory` would create a cross-boot record that points the wrong
    way with full confidence, which is worse than having no record at all.
    nav2_amcl publishes exactly this trap at startup: with a global seed and no
    motion, sigma_xy sits around 1.2 m and sigma_yaw around 160 deg.
    Thresholds are ~2x the converged figures measured on 2026-11-07
    (sigma_xy 0.26 m after a 120 deg turn).
    """
    bridge = Rosbridge('127.0.0.1', 19091, timeout=10)
    try:
        bridge.subscribe('/amcl_pose',
                         'geometry_msgs/msg/PoseWithCovarianceStamped')
        bridge.drain(0.5)
        # 6 s, not 2 s: nav2_amcl only publishes when the filter actually updates,
        # and updates are gated by update_min_d / update_min_a.  A stationary car
        # therefore goes quiet -- measured 2026-11-08, /amcl_pose had a publisher
        # but emitted nothing until the car turned.  Waiting only 2 s used to give
        # up between two updates even while the car was searching.
        m = bridge.wait_for_topic('/amcl_pose', timeout=6)
    finally:
        bridge.close()
    if m is None:
        return None
    covariance = m['pose'].get('covariance') or []
    if len(covariance) >= 36:
        sigma_xy = math.sqrt(max(0.0, covariance[0]))
        sigma_yaw = math.degrees(math.sqrt(max(0.0, covariance[35])))
        if sigma_xy > MAP_POSE_MAX_SIGMA_XY or sigma_yaw > MAP_POSE_MAX_SIGMA_YAW:
            print('map pose rejected: amcl has not converged '
                  '(sigma_xy=%.2f m, sigma_yaw=%.0f deg; limits %.2f m / %.0f deg)'
                  % (sigma_xy, sigma_yaw, MAP_POSE_MAX_SIGMA_XY,
                     MAP_POSE_MAX_SIGMA_YAW))
            print('  the car needs a slow turn before /amcl_pose can be trusted')
            return None
    p = m['pose']['pose']['position']
    o = m['pose']['pose']['orientation']
    yaw = math.degrees(math.atan2(2 * (o['w'] * o['z'] + o['x'] * o['y']),
                                  1 - 2 * (o['y'] ** 2 + o['z'] ** 2)))
    return {'x': p['x'], 'y': p['y'], 'yaw': yaw}


def sighting_from(output, pose_before, frame='odom'):
    """Turn search_around's stdout into a memory entry, or None.

    The bearing recorded is 0 -- the ball is taken to be dead ahead of the
    heading where it was spotted -- because that is the only claim the data
    supports.  search_around finds the ball somewhere inside a +/-200 tick j1
    sweep (+-48 deg), and it reports the ball in ``arm_base``, a frame with no
    transform to ``base_link`` in this stack.  Pretending to a precise bearing
    would be inventing a number; +-48 deg is still plenty to pick which of the
    four quadrants to check first, which is all this memory is for.
    """
    bearing = BEARING_RE.search(output)
    if bearing is None or pose_before is None:
        return None
    distance = RANGE_RE.search(output)
    return {'pose': {'x': pose_before['x'], 'y': pose_before['y'],
                     'yaw': pose_before['yaw'] + float(bearing.group(1))},
            'bearing': 0.0,
            'frame': frame,
            'range': float(distance.group(1)) if distance else 0.6}


def run_search(bridge, quadrants, label, frame='odom'):
    """Run search_around for N quadrants; return (found, output, pose_before).

    In map mode a missing /amcl_pose returns pose_before=None instead of
    falling back to an odom pose: stamping an odom pose as `frame='map'` would
    create a confidently wrong cross-boot memory.  sighting_from() then skips
    the memory write entirely, and the search itself is unaffected.
    """
    if frame == 'map':
        pose_before = read_map_pose()
        if pose_before is None:
            log('  /amcl_pose unavailable -- sighting will not be remembered')
    else:
        pose_before = read_pose(bridge)
    ok, out = run_tool(
        ['tools/search_around.py', '--quadrants', str(quadrants), '--ensure-gate'],
        label)
    return ('FOUND' in out), out, pose_before


def read_scan(bridge):
    bridge.subscribe('/scan', 'sensor_msgs/msg/LaserScan')
    bridge.drain(0.5)
    m = bridge.wait_for_topic('/scan', timeout=4)
    if m is None:
        return []
    return clean_scan(m['ranges'], m['angle_increment'])


def walk_sector(bridge, sector_deg, distance_m):
    """Turn toward sector_deg (0 = ahead, 90 = left) then walk in segments.

    Lidar angles are counterclockwise from the nose; turn_to's positive is
    also counterclockwise, so no sign gymnastics -- just normalise >180.

    Returns True when the walk happened, False when the rotation gate refused.
    The gate was missing here until 2026-11-06: search_around checked clearance
    before spinning but this path did not, so the one relocation that ever ran
    turned blind.  It is the same three-scan, two-vote check, for the same
    reason -- the robot's own wheels return intermittently and a single frame
    used to decide whether the car was allowed to move.
    """
    turn = sector_deg if sector_deg <= 180 else sector_deg - 360.0
    if abs(turn) > 8:
        scans = [read_scan(bridge) for _ in range(3)]
        clear, offenders = rotation_clearance_agreed(scans)
        if not clear:
            nearest = min(offenders, key=lambda p: p[1])
            log('  rotation BLOCKED: nearest %.2f m at %.0f deg -- not turning'
                % (nearest[1], nearest[0]))
            return False
        log('  turning %+0.0f deg toward the emptiest sector (clearance agreed '
            'over %d scans)' % (turn, len([s for s in scans if s])))
        hist = ab.turn_to(bridge, turn)
        for h in hist[-2:]:
            log('    %s' % h)
        time.sleep(0.8)
    segments = max(1, int(round(distance_m / 0.25)))
    for seg in range(segments):
        log('  walking segment %d/%d (%.0f mm)' % (seg + 1, segments, 250))
        gs.drive(bridge, 0.08, 0.25 / 0.08)
        time.sleep(0.8)
    return True


def read_odom(bridge):
    """Odometry position, or None.  Drain first -- see read_pose."""
    bridge.drain(0.4)
    message = bridge.wait_for_topic('/odom', timeout=3)
    if message is None:
        return None
    position = message['pose']['pose']['position']
    return (position['x'], position['y'])


def _rear_nearest(bridge, arc=(115.0, 245.0)):
    """(angle, distance) of the nearest real return behind the car, or (None, None).

    The arc is the whole rear half, not the 150-210 tail.  Measured 2026-11-06
    after a collision: the chair leg sat at 137-149 deg while the rear check
    looked at 150-210 and reported 0.358 m clear -- 86 mm and 12 deg away from
    the truth.
    """
    points = read_scan(bridge)
    behind = [(a, r) for a, r in points if arc[0] <= a <= arc[1]]
    if not behind:
        return None, None
    return min(behind, key=lambda pair: pair[1])


def rear_clearance(bridge, arc=(115.0, 245.0)):
    """Nearest real thing behind the car, metres, or None if the scan is silent."""
    return _rear_nearest(bridge, arc)[1]


def _distance_at(bridge, angle, half_width_deg=10.0):
    """Nearest return within a narrow slice around ``angle``, or None."""
    points = read_scan(bridge)
    lo, hi = angle - half_width_deg, angle + half_width_deg
    ranges = [r for a, r in points if lo <= a <= hi]
    return min(ranges) if ranges else None


BACK_OFF_M = 0.20

# Never reverse closer than this to anything, whatever the caller asked for.
# The chassis has no force or current sensing, so contact is not observable
# electrically -- distance is the only thing standing between the car and a
# chair leg.
REVERSE_SAFETY_M = 0.30

# Short bursts, re-checked every time.  The old version measured the rear ONCE
# before the loop and then reversed blind for up to four bursts; the chair was
# hit on burst two and the car kept going.
REVERSE_STEP_M = 0.10

# A burst that delivers less than this fraction of what was commanded is not
# slip, it is something in the way.  Slip alone was measured at 48-101%.
REVERSE_STALL_RATIO = 0.30

# ... but stalling is NOT what happened on 2026-11-06.  The operator reports the
# car PUSHED the chair along, which means the wheels turned, the encoders
# counted, and the delivery ratio looked healthy while furniture was being
# dragged.  Travel-based stall detection cannot see that.
#
# The signature that does work: a static obstacle falls behind as the car
# reverses, so the nearest rear return must grow by roughly the distance driven.
# An obstacle that stays put in the same frame is moving WITH the car.
PUSH_TOLERANCE_M = 0.05


def reverse_measured(bridge, distance_m=BACK_OFF_M, max_bursts=6):
    """Back up by a MEASURED distance, re-checking the rear before every burst.

    Three independent stop conditions, because each of the first two missed the
    real incident on 2026-11-06:

    * geometry -- refuse to start, or to continue, when anything is within
      REVERSE_SAFETY_M behind the car;
    * pushing -- if the obstacle behind does not fall away by roughly the
      distance driven, it is being dragged (the reported failure: the car
      pushed a chair instead of stalling);
    * stalling -- if a burst delivers far less travel than commanded, the car is
      held fast and further bursts would only strain the motors.

    Returns True if the car actually moved at least 60% of the request without
    being flagged for pushing.
    """
    start = read_odom(bridge)
    if start is None:
        log('  cannot reverse: no odometry')
        return False

    travelled = 0.0
    stalls = 0
    for burst in range(max_bursts):
        target_angle, rear_before = _rear_nearest(bridge)
        if rear_before is None:
            log('  reverse aborted: no lidar return behind the car')
            break
        room = rear_before - REVERSE_SAFETY_M
        if room <= 0.02:
            log('  reverse aborted: %.2f m behind is inside the %.2f m safety '
                'floor' % (rear_before, REVERSE_SAFETY_M))
            break
        step = min(REVERSE_STEP_M, room, distance_m - travelled)
        if step < 0.03:
            break

        before = read_odom(bridge)
        gs.drive(bridge, -0.08, step / 0.08)
        time.sleep(0.8)
        now = read_odom(bridge)
        if before is None or now is None:
            break
        moved = math.hypot(now[0] - before[0], now[1] - before[1])
        travelled += moved

        # Follow the SAME obstacle, not whatever happens to be nearest now.
        # Comparing the whole-arc minimum would fire spuriously whenever a
        # closer object entered the arc from the side while we backed away.
        rear_after = _distance_at(bridge, target_angle)
        if rear_after is not None:
            expected = rear_before + moved
            if rear_after < expected - PUSH_TOLERANCE_M:
                log('  reverse aborted: the thing behind did not fall away '
                    '(%.0f deg: %.2f -> %.2f m after moving %.2f, expected '
                    'about %.2f) -- the car is not actually getting anywhere'
                    % (target_angle, rear_before, rear_after, moved, expected))
                return False

        if moved < step * REVERSE_STALL_RATIO:
            stalls += 1
            log('  burst %d moved %.3f m of %.3f commanded -- something is in '
                'the way (%d/2)' % (burst + 1, moved, step, stalls))
            if stalls >= 2:
                log('  reverse aborted: the car is not moving as commanded')
                break
        else:
            stalls = 0
        if travelled >= distance_m * 0.9:
            break

    log('  reversed %.2f m in total (asked %.2f)' % (travelled, distance_m))
    return travelled >= distance_m * 0.6


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--max-rounds', type=int, default=6)
    parser.add_argument('--walk-m', type=float, default=0.5)
    parser.add_argument('--target', type=float, default=0.24)
    parser.add_argument('--map-frame', action='store_true',
                        help='use nav2_amcl /amcl_pose so ball memory survives '
                             'a reboot (requires a saved map + amcl running)')
    args = parser.parse_args()

    bridge = Rosbridge('127.0.0.1', 19091, timeout=15)
    started = time.monotonic()
    history = ball_memory.load(str(HISTORY))

    # Off by default: without amcl the only honest frame is odom, whose pose
    # resets to zero on every boot.  With --map-frame and a working amcl the
    # same code path stores a map-anchored sighting that survives a power cycle.
    frame = 'odom'
    if args.map_frame:
        if read_map_pose() is not None:
            frame = 'map'
        else:
            log('memory: --map-frame requested but /amcl_pose is unavailable '
                '-- falling back to odom for this run')
    log('memory: using %s frame' % frame)

    # Ask the memory before searching.  One remembered heading costs a single
    # rotation to test and, when it is right, saves three rotations plus three
    # returns to the boot pose.  When it is wrong the full sweep still runs, so
    # coverage is never reduced -- only the order changes.
    guess = None
    pose0 = read_map_pose() if frame == 'map' else read_pose(bridge)
    if pose0 is None:
        log('memory: cannot read pose -- searching without it')
    else:
        guess = ball_memory.preferred_heading(history, pose0, frame=frame)
        if guess:
            log('memory: last seen %+.0f deg, %.2f m out, %.0f s ago '
                '(drove %.2f m since, confidence %.2f)'
                % (guess['bearing_deg'], guess['range_m'], guess['age_s'],
                   guess['travelled_m'], guess['confidence']))
        else:
            log('memory: nothing recent enough to act on -- full sweep')

    for round_index in range(1, args.max_rounds + 1):
        found, out, pose_before = False, '', None

        if guess and round_index == 1:
            turn = guess['bearing_deg']
            turn = turn if turn <= 180 else turn - 360
            log('=== ROUND 1: memory heading %+.0f deg ===' % turn)
            if abs(turn) > 8:
                for line in ab.turn_to(bridge, turn)[-2:]:
                    log('  ' + line)
                time.sleep(0.8)
            found, out, pose_before = run_search(bridge, 1, 'ROUND 1: memory sweep', frame)
            append_history({'t': time.strftime('%H:%M:%S'), 'round': 1,
                            'event': 'search-memory', 'bearing': round(turn, 1),
                            'found': found})
            if not found:
                log('  memory was wrong -- falling back to the full sweep')

        if not found:
            quadrants = '4' if round_index == 1 else '1'
            found, out, pose_before = run_search(
                bridge, quadrants, 'ROUND %d: search' % round_index, frame)
            append_history({'t': time.strftime('%H:%M:%S'), 'round': round_index,
                            'event': 'search', 'found': found})

        if found:
            sighting = sighting_from(out, pose_before, frame)
            if sighting is not None:
                ball_memory.record_sighting(
                    history, sighting['pose'], sighting['bearing'],
                    sighting['range'], source='search_around',
                    frame=sighting['frame'])
                commit_memory(history)
                log('  remembered: %.2f m ahead at heading %+.0f deg'
                    % (sighting['range'], sighting['pose']['yaw']))

        if not found:
            log('  ball not visible -- relocating')
            pts = read_scan(bridge)
            if not pts:
                log('  no lidar data either -- waiting 2 s and retrying')
                time.sleep(2)
                continue
            # Relocation ladder (2026-11-06).  The old code had exactly one rung
            # -- "no corridor in the front half -> wait 3 s" -- and that rung is
            # what artifacts/ball_history.json recorded six times.  A car whose
            # nose is against furniture has no way out under that rule, and the
            # retry re-reads the same blocked scan.
            #
            #   1. turn toward the best direction in the front half and walk
            #   2. if the front half has nothing, consider the whole circle
            #   3. if the turn is refused, reverse a measured distance and retry
            #   4. only then wait
            sector, clear_m = emptiest_sector(pts, front_half=True)
            half = 'front'
            if sector is None:
                sector, clear_m = emptiest_sector(pts, front_half=False)
                half = 'any'
                if sector is not None:
                    log('  front half blocked; best direction overall is %.0f deg '
                        '(%.2f m)' % (sector, clear_m or 0))
            if sector is not None:
                append_history({'t': time.strftime('%H:%M:%S'), 'round': round_index,
                                'event': 'relocate', 'sector': sector,
                                'half': half, 'clear_m': round(clear_m, 2)})
                if walk_sector(bridge, sector, args.walk_m):
                    continue
                log('  the turn was refused -- trying to reverse instead')

            if reverse_measured(bridge, BACK_OFF_M):
                append_history({'t': time.strftime('%H:%M:%S'), 'round': round_index,
                                'event': 'back-off', 'metres': BACK_OFF_M})
                continue
            log('  cannot turn and cannot reverse -- waiting 3 s')
            append_history({'t': time.strftime('%H:%M:%S'), 'round': round_index,
                            'event': 'no-corridor'})
            time.sleep(3)
            continue

        # Ball found -- approach.
        ok, out = run_tool(
            ['tools/approach_ball.py', '--target', str(args.target),
             '--margin', '0.03', '--max-steps', '24'],
            'ROUND %d: approach' % round_index)
        append_history({'t': time.strftime('%H:%M:%S'), 'round': round_index,
                        'event': 'approach', 'ok': ok})
        if not ok:
            log('  approach failed -- searching again')
            continue

        # In the grasp band -- grasp.
        ok, out = run_tool(['tools/grasp_sequence.py', '--vision', 'ground'],
                           'ROUND %d: grasp' % round_index)
        success = '"ok": true' in out
        append_history({'t': time.strftime('%H:%M:%S'), 'round': round_index,
                        'event': 'grasp', 'ok': success})
        # A successful grasp means the ball is in the jaws: every remembered
        # sighting is spent, and `predict` refuses to resurrect them.  A failed
        # one leaves the memory intact -- the ball is still out there.
        ball_memory.record_outcome(history, success,
                                   note='round %d' % round_index)
        commit_memory(history)
        if success:
            log('=== BALL GRASPED in %.1f min over %d round(s) ==='
                % ((time.monotonic() - started) / 60, round_index))
            return 0
        log('  grasp failed -- searching again')

    log('=== ball not retrieved after %d rounds (%.1f min) ==='
        % (args.max_rounds, (time.monotonic() - started) / 60))
    return 1


if __name__ == '__main__':
    sys.exit(main())
