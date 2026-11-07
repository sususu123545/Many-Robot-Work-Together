#!/usr/bin/env python3
"""Measure how far the fingertips hang below the tool point, from an image.

Why this is needed
------------------
``dz`` is chosen so that the tool point sits some distance above the ball centre,
on the assumption that the fingertips then land level with that centre.  The
constant relating the two -- how far the tips hang below the tool point -- came
from the STL mesh and reads 45.7 mm.  On the real robot the tips visibly stop
about 10 mm above the ball centre, so the mesh value is too large and every
grasp closes in mid-air.

What this does
--------------
One picture is enough, because both ends of the measurement are visible:

  * the ball's horizontal extent is never clipped (only its bottom is), so its
    width gives the pixel scale directly -- 35 mm across the full diameter;
  * the ball's top edge plus that radius gives its centre row;
  * the gripper fingers are the only near-black objects in the frame, and their
    uppermost row is where the tips are.

    L = dz - (ball_centre_row - fingertip_row) * mm_per_pixel

Run it while the arm is hovering over a ball with the jaws open -- that is the
pose the grasp starts from.

Usage
    python3 tools/perception/gripper_reach.py                    # live camera
    python3 tools/perception/gripper_reach.py --image shot.png   # a saved frame
    python3 tools/perception/gripper_reach.py --dz 0.046 --ball-diameter 35
"""

import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
REPO = os.path.dirname(TOOLS)
sys.path.insert(0, TOOLS)
sys.path.insert(0, HERE)

SNAPSHOT = ('http://127.0.0.1:19099/snapshot'
            '?topic=/rear_camera/image_raw&type=ros_compressed')


def grab(url=SNAPSHOT, timeout=15):
    import urllib.request
    from io import BytesIO
    from PIL import Image
    with urllib.request.urlopen(url, timeout=timeout) as response:
        return Image.open(BytesIO(response.read())).convert('RGB')


def find_tips(image, dark_threshold=75, search_from=0.75, min_run=6):
    """Uppermost dark rows on the left and right halves.

    The fingers are the only large near-black things the camera sees: the table
    is pale, the ball is red.  Two guards matter here, both learned from the
    first attempt at this measurement:

      * only the bottom slice is searched -- there are faint dark scratches on
        this tabletop that put a "fingertip" high up in the middle of the frame;
      * a row must contribute at least ``min_run`` dark pixels, which a scratch
        never does but a finger always will.
    """
    width, height = image.size
    grey = image.convert('L')
    pixels = grey.load()
    start_row = int(height * search_from)
    left_top, right_top = None, None
    for y in range(start_row, height):
        left_run, right_run = 0, 0
        for x in range(width):
            if pixels[x, y] < dark_threshold:
                if x < width // 2:
                    left_run += 1
                else:
                    right_run += 1
        if left_top is None and left_run >= min_run:
            left_top = y
        if right_top is None and right_run >= min_run:
            right_top = y
        if left_top is not None and right_top is not None:
            break
    return left_top, right_top


def live_dz(port=19091):
    """How far the tool point currently sits above the ball centre, in metres.

    Read straight off the arm instead of being typed in: the whole point of the
    calibration is that the number comes from what the machine actually did.
    """
    from rosbridge_client import Rosbridge
    import grasp_sequence as gs
    bridge = Rosbridge('127.0.0.1', port, timeout=10)
    try:
        ticks = gs.read_ticks(bridge)
    finally:
        bridge.close()
    if ticks is None:
        print('no joint feedback')
        return None
    located, reason = gs.ball_in_base(ticks)
    if located is None:
        print('cannot locate the ball from this pose: %s' % reason)
        return None
    tool_z = gs.forward_tool_pose_raw(ticks)['translation_m'][2]
    return tool_z - located['ball'][2]


def find_ball(image):
    """Red blob, required to look round and to be fully inside the frame."""
    import detectors
    found = detectors.ColorDetector(min_radius=12.0, max_radius=200.0,
                                    min_fill=0.6, aspect_range=(0.75, 1.35))
    detections = found.detect(image)
    if not detections:
        return None
    return max(detections, key=lambda d: d.width * d.height)


def measure(image, dz, ball_diameter_mm):
    """Return a dict with everything the calibration needs, or an error string."""
    ball = find_ball(image)
    if ball is None:
        return None, 'no round red blob in the frame'

    width, height = image.size
    if ball.bottom_v > height - 2:
        return None, ('the ball bottom is cut off by the frame -- move the arm '
                      'back until the whole ball fits, then measure again')
    if ball.top_v < 2:
        return None, 'the ball touches the top edge -- move the camera closer'

    ratio = ball.height / ball.width
    if not 0.75 <= ratio <= 1.35:
        return None, ('the red silhouette is %.0fx%.0f px, not round -- the '
                      'colour mask is catching reflections' % (ball.width, ball.height))

    left_top, right_top = find_tips(image)
    if left_top is None and right_top is None:
        return None, 'no dark pixels found -- are the fingers in view?'

    # A circle measured two ways: averaging the extents halves the error from
    # anti-aliased edges.
    width_px = (ball.width + ball.height) / 2.0
    mm_per_pixel = ball_diameter_mm / width_px
    centre_row = (ball.top_v + ball.bottom_v) / 2.0

    tips = [value for value in (left_top, right_top) if value is not None]
    tip_row = min(tips)
    gap_px = centre_row - tip_row
    gap_mm = gap_px * mm_per_pixel

    return {'ball_top_row': round(ball.top_v, 1),
            'ball_bottom_row': round(ball.bottom_v, 1),
            'ball_width_px': round(ball.width, 1),
            'ball_height_px': round(ball.height, 1),
            'ball_centre_row': round(centre_row, 1),
            'left_tip_row': left_top,
            'right_tip_row': right_top,
            'tip_row': tip_row,
            'gap_px': round(gap_px, 1),
            'mm_per_pixel': round(mm_per_pixel, 4),
            'tips_above_centre_mm': round(gap_mm, 1),
            'L_mm': round((dz * 1000) - gap_mm, 1)}, ''


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--image', help='analyse a saved frame instead of the camera')
    parser.add_argument('--dz', type=float, default=None,
                        help='how far the tool point is above the ball centre (m). '
                             'Omitted: read it from the live arm, which is what '
                             'you want when calibrating on the robot')
    parser.add_argument('--ball-diameter', type=float, default=35.0,
                        help='true diameter of the ball on the table (mm)')
    parser.add_argument('--url', default=SNAPSHOT)
    args = parser.parse_args()

    if args.image:
        from PIL import Image
        image = Image.open(args.image).convert('RGB')
        if args.dz is None:
            raise SystemExit('--image needs --dz (no arm to read it from)')
        dz = args.dz
    else:
        image = grab(args.url)
        dz = args.dz if args.dz is not None else live_dz()
        if dz is None:
            return 2
    print('image %dx%d' % image.size)
    print('tool point %.1f mm above the ball centre' % (dz * 1000))

    result, error = measure(image, dz, args.ball_diameter)
    if result is None:
        print('cannot measure: %s' % error)
        return 2

    print('ball rows         : top %.0f  bottom %.0f  (centre %.0f)'
          % (result['ball_top_row'], result['ball_bottom_row'],
             result['ball_centre_row']))
    print('ball size         : %.0f x %.0f px  -> %.4f mm/px  (true %.0f mm)'
          % (result['ball_width_px'], result['ball_height_px'],
             result['mm_per_pixel'], args.ball_diameter))
    print('fingertip rows    : left %s  right %s  -> using %d'
          % (result['left_tip_row'], result['right_tip_row'], result['tip_row']))
    print()
    print('tips sit %.1f mm above the ball centre (negative = tips below)'
          % result['tips_above_centre_mm'])
    print('with the tool point %.0f mm above centre,' % (dz * 1000))
    print('  L (tool point -> fingertips) = %.1f mm' % result['L_mm'])
    print()
    print('CAUTION: this pixel ratio assumes both sit at the same depth, and they')
    print('do not -- the tips are higher and therefore nearer the camera. Treat a')
    print('value measured at large dz as an upper bound only; measure near the')
    print('grasp pose, where the depths nearly match, or use the grab sweep.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
