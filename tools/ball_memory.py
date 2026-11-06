#!/usr/bin/env python3
"""Ball memory: remember where the ball was, in a frame that survives driving.

Why a separate module
---------------------
The car has no map and no SLAM yet (P2).  What it does have is encoder
odometry, which is bad but not useless for one specific question: "given where
I have driven since I last saw the ball, which way should I look first?"

That question does NOT need accuracy.  An answer that is right to +-45 degrees
turns a four-quadrant sweep into a one-quadrant sweep.  An answer that is 30 cm
off is still fine, because nothing here is ever used to command a grasp -- the
grasp always re-measures the ball with the camera.

What is stored
--------------
A sighting is ``(car pose, bearing, range)``, not a 3D point.  The pose is the
odom-frame ``(x, y, yaw)``; the bearing is degrees counter-clockwise from the
car's nose, the same convention the lidar sectors and ``approach_ball.turn_to``
already use; the range is metres.

Storing a bearing instead of a point is deliberate.  The camera reports the ball
in ``arm_base``, and ``arm_base`` has no transform to ``base_link`` in this
stack, so converting a camera fix into an odom point would be guesswork dressed
up as arithmetic.  A bearing inferred from the sweep heading plus a forward
range keeps every number traceable to something that was actually measured.

Accuracy, stated honestly
-------------------------
Encoder odometry drifts badly on mecanum wheels: measured 2026-11-04, commanding
0.100 m produced 0.062 m (38% short), and rotation efficiency has swung between
31% and 127% depending on battery state.  ``turn_to`` closes its own loop on
odometry, so yaw is decent; translation is not.  Therefore a prediction older
than ``STALE_AFTER_S``, or one that requires trusting more than
``MAX_TRUSTED_TRAVEL_M`` of driving, is discounted rather than believed.

Run ``python3 tools/ball_memory.py`` for the offline self-test (no ROS needed).
"""

import json
import math
import os
import time

# A memory older than this is not worth acting on: the ball may have been moved,
# and the odometry drift it has accumulated dominates any real displacement.
STALE_AFTER_S = 1800.0

# Beyond this much driving since the sighting, the accumulated odometry error
# (measured 38% linear) is larger than the area a search quadrant covers, so the
# memory cannot narrow anything down.
MAX_TRUSTED_TRAVEL_M = 1.5

# How much a predicted bearing is trusted.  Used for ordering, never for aiming.
CONFIDENCE_FRESH = 0.9

# Weight of the travelled-distance penalty.  At half the trust radius the
# confidence loses this fraction.
DRIFT_PENALTY = 0.5

HISTORY_LIMIT = 400


def _now():
    return time.time()


def load(path):
    """Read the history list, tolerating a missing or corrupt file."""
    try:
        with open(path, 'r') as handle:
            data = json.load(handle)
    except (OSError, ValueError):
        return []
    if not isinstance(data, list):
        return []
    return [item for item in data if isinstance(item, dict)]


def save(path, entries, limit=HISTORY_LIMIT):
    folder = os.path.dirname(os.path.abspath(path))
    if folder and not os.path.isdir(folder):
        os.makedirs(folder)
    with open(path, 'w') as handle:
        json.dump(entries[-limit:], handle, indent=1)


def record_sighting(entries, pose, bearing_deg, range_m, when=None, source='search'):
    """Append one sighting.  ``pose`` is ``{'x','y','yaw'}`` in odom metres/degrees."""
    entries.append({
        't': _now() if when is None else float(when),
        'kind': 'sighting',
        'x': round(float(pose['x']), 4),
        'y': round(float(pose['y']), 4),
        'yaw': round(float(pose['yaw']), 2),
        'bearing': round(float(bearing_deg), 1),
        'range': round(float(range_m), 3),
        'source': source,
    })
    return entries[-1]


def record_outcome(entries, ok, when=None, note=''):
    """Append the result of a grasp attempt.

    ``ok=True`` means the ball is in the jaws.  Every sighting before it is then
    spent: predicting where to look for a ball the robot is already holding is
    the one way this module could actively mislead, so `predict` refuses.
    """
    entries.append({
        't': _now() if when is None else float(when),
        'kind': 'outcome',
        'ok': bool(ok),
        'note': str(note)[:80],
    })
    return entries[-1]


def _last(entries, kind):
    for entry in reversed(entries):
        if entry.get('kind') == kind:
            return entry
    return None


def _live_sightings(entries):
    """Sightings that are still worth considering.

    Two rules: anything before the last successful grasp is dead (the ball was
    picked up), and old sightings decay out of the window.
    """
    last_ok = None
    for index, entry in enumerate(entries):
        if entry.get('kind') == 'outcome' and entry.get('ok'):
            last_ok = index
    # Start just after the successful grasp, or at the very beginning when there
    # has never been one.  Writing this as `entries[(last_ok or 0) + 1:]` looks
    # equivalent and silently drops the first sighting -- the self-test caught
    # exactly that on the first run.
    start = 0 if last_ok is None else last_ok + 1
    return [entry for entry in entries[start:] if entry.get('kind') == 'sighting']


def _to_car_frame(entry, pose_now):
    """Where the remembered ball sits relative to the car's CURRENT nose.

    Returns (forward_m, left_m).  The car frame here is the odom convention:
    x forward, y left, yaw counter-clockwise.  Both the sighting and the current
    pose are in the odom frame, so this is a rigid transform with no scale term.
    """
    yaw_then = math.radians(entry['yaw'])
    bearing = math.radians(entry['bearing'])
    distance = entry['range']
    # the ball's position in the odom frame, as reconstructed from the sighting
    ball_x = entry['x'] + distance * math.cos(yaw_then + bearing)
    ball_y = entry['y'] + distance * math.sin(yaw_then + bearing)
    # ... then expressed in the frame the car occupies right now
    dx = ball_x - pose_now['x']
    dy = ball_y - pose_now['y']
    yaw_now = math.radians(pose_now['yaw'])
    forward = dx * math.cos(yaw_now) + dy * math.sin(yaw_now)
    left = -dx * math.sin(yaw_now) + dy * math.cos(yaw_now)
    return forward, left


def travelled_since(entry, pose_now):
    """Straight-line distance the car has driven since that sighting, metres."""
    return math.hypot(pose_now['x'] - entry['x'], pose_now['y'] - entry['y'])


def predict(entries, pose_now, now=None, stale_after_s=STALE_AFTER_S):
    """Rank the remembered sightings as headings to check, best first.

    Returns a list of dicts::

        {'bearing_deg': 34.0,      # CCW from the nose, ready for turn_to
         'range_m': 0.7,
         'age_s': 240.0,
         'travelled_m': 0.4,
         'confidence': 0.75,
         'quadrant': 0}            # 0/90/180/270, matching the search sectors

    The list is empty when nothing is remembered, when the ball was already
    picked up, or when every sighting is too old or too far away to trust.
    """
    now = _now() if now is None else now
    ranked = []
    for entry in _live_sightings(entries):
        age = now - entry['t']
        if age < 0 or age > stale_after_s:
            continue
        travelled = travelled_since(entry, pose_now)
        if travelled > MAX_TRUSTED_TRAVEL_M:
            continue
        forward, left = _to_car_frame(entry, pose_now)
        distance = math.hypot(forward, left)
        if distance < 0.05:
            continue
        bearing = math.degrees(math.atan2(left, forward))
        # Multiplicative, not additive.  An additive blend was tried first and
        # was useless: a sighting 29 minutes into a 30-minute window still scored
        # 0.57 because "the car has not driven since" handed it full marks, so
        # the whole confidence range was 0.57..0.90 and the staleness window did
        # nothing.  Age is the dominant term here -- only a human moves the ball.
        # The square root keeps a half-hour-old memory at 0.64 instead of 0.45.
        age_factor = max(0.0, 1.0 - age / stale_after_s) ** 0.5
        drift_factor = 1.0 - DRIFT_PENALTY * min(1.0, travelled / MAX_TRUSTED_TRAVEL_M)
        confidence = CONFIDENCE_FRESH * age_factor * drift_factor
        ranked.append({'bearing_deg': round(bearing, 1),
                       'range_m': round(distance, 3),
                       'age_s': round(age, 1),
                       'travelled_m': round(travelled, 3),
                       'confidence': round(confidence, 2),
                       'quadrant': quadrant_of(bearing)})
    ranked.sort(key=lambda item: (-item['confidence'], item['age_s']))
    return ranked


def quadrant_of(bearing_deg):
    """Snap a bearing to the 4-quadrant sweep the search actually performs.

    Quadrant centres are 0/90/180/270, i.e. the headings `search_around` spins
    to.  Bearing -170 and +170 both land in 180: they are 20 degrees apart in
    reality and there is no point pretending otherwise.
    """
    wrapped = (float(bearing_deg) + 180.0) % 360.0 - 180.0
    if -45.0 <= wrapped < 45.0:
        return 0
    if 45.0 <= wrapped < 135.0:
        return 90
    if wrapped >= 135.0 or wrapped < -135.0:
        return 180
    return 270


def preferred_heading(entries, pose_now, min_confidence=0.5, now=None):
    """The single heading worth checking first, or None.

    Only the best candidate is returned, and only if it clears
    ``min_confidence``.  A weak guess is worse than no guess: it costs a rotation
    and an arm tuck to test, and the fallback four-quadrant sweep would have
    found the ball anyway.
    """
    ranked = predict(entries, pose_now, now=now)
    if not ranked or ranked[0]['confidence'] < min_confidence:
        return None
    return ranked[0]


# ---------------------------------------------------------------------------
# self-test: python3 tools/ball_memory.py   (no ROS, no robot)
# ---------------------------------------------------------------------------

def _self_test():
    failures = []
    total = [0]

    def check(name, condition, detail=''):
        total[0] += 1
        if not condition:
            failures.append('%s %s' % (name, detail))
        print('%-52s %s %s' % (name, 'PASS' if condition else 'FAIL', detail))

    T0 = 1_000_000.0
    pose_here = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}

    # A ball seen 0.8 m straight ahead, from exactly where we are standing.
    entries = []
    record_sighting(entries, pose_here, 0.0, 0.8, when=T0)
    ranked = predict(entries, pose_here, now=T0 + 10)
    check('straight-ahead sighting predicts 0 deg',
          ranked and abs(ranked[0]['bearing_deg']) < 0.5,
          str(ranked[0]['bearing_deg'] if ranked else None))
    check('range is carried through', ranked and abs(ranked[0]['range_m'] - 0.8) < 1e-6)

    # Same sighting, but the car has since turned 90 deg CCW: the ball is now
    # 90 deg to the RIGHT, i.e. bearing -90.
    turned = {'x': 0.0, 'y': 0.0, 'yaw': 90.0}
    ranked = predict(entries, turned, now=T0 + 10)
    check('after a 90 deg CCW turn the ball reads -90 deg',
          ranked and abs(ranked[0]['bearing_deg'] + 90.0) < 0.5,
          str(ranked[0]['bearing_deg'] if ranked else None))

    # ... and with the car 0.5 m forward of where it was, the ball is closer.
    advanced = {'x': 0.5, 'y': 0.0, 'yaw': 0.0}
    ranked = predict(entries, advanced, now=T0 + 10)
    check('driving 0.5 m closer shortens the range',
          ranked and abs(ranked[0]['range_m'] - 0.3) < 1e-6,
          str(ranked[0]['range_m'] if ranked else None))

    # A ball seen 45 deg to the left, and the car then turned 45 deg left:
    # dead ahead.
    entries = []
    record_sighting(entries, pose_here, 45.0, 0.6, when=T0)
    ranked = predict(entries, {'x': 0.0, 'y': 0.0, 'yaw': 45.0}, now=T0 + 5)
    check('turning toward a sighting brings it to 0 deg',
          ranked and abs(ranked[0]['bearing_deg']) < 0.5,
          str(ranked[0]['bearing_deg'] if ranked else None))

    # Stale memory is ignored.
    entries = []
    record_sighting(entries, pose_here, 0.0, 0.8, when=T0)
    check('a sighting older than the window is dropped',
          predict(entries, pose_here, now=T0 + STALE_AFTER_S + 1) == [])

    # Too much driving in between: the odometry error eats the answer.
    far = {'x': 3.0, 'y': 0.0, 'yaw': 0.0}
    check('driving further than the trust radius drops the memory',
          predict(entries, far, now=T0 + 30) == [])

    # A successful grasp invalidates every sighting before it.
    entries = []
    record_sighting(entries, pose_here, 0.0, 0.8, when=T0)
    record_outcome(entries, True, when=T0 + 60)
    check('a successful grasp clears the memory',
          predict(entries, pose_here, now=T0 + 120) == [])

    # A FAILED grasp does not clear it -- the ball is still out there.
    entries = []
    record_sighting(entries, pose_here, 0.0, 0.8, when=T0)
    record_outcome(entries, False, when=T0 + 60)
    check('a failed grasp keeps the memory alive',
          len(predict(entries, pose_here, now=T0 + 120)) == 1)

    # A newer sighting outranks an older one.
    entries = []
    record_sighting(entries, pose_here, 170.0, 0.9, when=T0)
    record_sighting(entries, pose_here, 10.0, 0.7, when=T0 + 600)
    ranked = predict(entries, pose_here, now=T0 + 610)
    check('the fresher sighting is ranked first',
          abs(ranked[0]['bearing_deg'] - 10.0) < 0.5, str(ranked[0]['bearing_deg']))

    # Quadrant snapping, including the wrap-around.
    cases = [(0, 0), (44, 0), (46, 90), (90, 90), (134, 90), (136, 180),
             (-170, 180), (170, 180), (-90, 270), (-46, 270)]
    wrong = [(b, q, quadrant_of(b)) for b, q in cases if quadrant_of(b) != q]
    check('quadrant snapping handles the wrap-around', not wrong, str(wrong))

    # Weak guesses must not be acted on.
    entries = []
    record_sighting(entries, pose_here, 0.0, 0.8, when=T0)
    weak = preferred_heading(entries, pose_here, now=T0 + STALE_AFTER_S - 5)
    check('a nearly-stale sighting is not used as a heading',
          weak is None or weak['confidence'] < 0.5, str(weak))

    # Round-trip through disk.
    import tempfile
    path = os.path.join(tempfile.mkdtemp(), 'history.json')
    entries = []
    record_sighting(entries, pose_here, 12.0, 0.55, when=T0)
    save(path, entries)
    check('save/load round-trips', load(path) == entries)
    with open(path, 'w') as handle:
        handle.write('{not json')
    check('a corrupt history file loads as empty', load(path) == [])
    check('a missing history file loads as empty', load(path + '.nope') == [])

    print('')
    print('%d checks, %d failed' % (total[0], len(failures)))
    for item in failures:
        print('  FAILED: %s' % item)
    return 1 if failures else 0


if __name__ == '__main__':
    raise SystemExit(_self_test())
