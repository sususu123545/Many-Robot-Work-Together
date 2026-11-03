#!/usr/bin/env python3
"""Look for the red ball by turning the base, with perception running off-board.

STATUS 2026-11-03: the sweep motion is proven on the real robot; the classifier
below is NOT. It was measured against six real frames from this room and it still
returns mahogany furniture as a candidate (chair backs reach saturation 241, the
ball 235). Treat a "found" from this file as a candidate to look at, never as
proof. The real detector (armpi_ros2.red_ball_detector, OpenCV contours) is the
authority; it cannot run here because the board's sshd will not start processes.

Why perception is here and not on the robot
-------------------------------------------
On 2026-11-03 the board's sshd accepted logins but never confirmed an ``exec``,
so nothing new could be started on it, and the red-ball detector was not among
the processes already running. Port forwarding to rosbridge still worked, so the
camera stream and ``/cmd_vel`` remained reachable. Rather than wait for a repair
that a reboot did not deliver, the detector is reimplemented here with Pillow --
same colour thresholds, same blob gates -- and drives the existing chassis.

What it does
------------
Step and stare: turn a fixed angle, stop, let the base settle, look, repeat. The
reason is in ``robot_agent/search.py``: the real detector only localises after
several still frames, and this one applies the same rule, so it also cannot work
while the base is turning.

Safety, given that a real robot moves here
------------------------------------------
* speed stays below the chassis limit (0.20 rad/s);
* every publish interval is 50 ms, so the chassis lease (0.35 s) acts as a
  deadman -- if this script dies or the link stalls, the wheels stop by themselves;
* a camera stall longer than ``--frame-timeout`` aborts the sweep;
* the base is stopped explicitly on every exit path.

Usage:
    python3 tools/car_scan.py --dry-run          # detection only, no motion
    python3 tools/car_scan.py                    # sweep 360 deg
    python3 tools/car_scan.py --max-deg 180
"""

import argparse
import base64
import io
import json
import math
import os
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(REPO, 'ros2_ws', 'src', 'robot_agent'))

from PIL import Image, ImageChops  # noqa: E402

from robot_agent.search import ScanPlanner, ScanState  # noqa: E402
from rosbridge_client import Rosbridge, RosbridgeError  # noqa: E402

# Colour gates copied from armpi_ros2.red_ball_detector.find_red_sphere.
HUE_LOW_MAX = 8          # 12 deg of 360, expressed in Pillow's 0..255 hue
HUE_HIGH_MIN = 119       # 168 deg
HUE_HIGH_MAX = 128       # 180 deg
SAT_MIN = 75
VAL_MIN = 45

MIN_BLOB_PX = 100        # a 35 mm ball at 0.8 m is about 160 px with occlusion
MAX_BLOB_PX = 4000       # a 35 mm ball is ~3000 px even at 0.3 m; bigger is furniture
MAX_FILL = 1.0
MIN_FILL = 0.25          # a disc fills about 0.79 of its box; occlusion lowers it a lot
# Mahogany furniture is dark red-brown and survives every shape gate. Measured on
# six real frames from this room: the ball averages saturation 235, the wood
# 110..131. Saturation separates them; geometry does not.
MIN_MEAN_SAT = 180
MIN_ASPECT, MAX_ASPECT = 0.50, 2.00


def red_mask(image_rgb):
    """Binary L-mode mask of red pixels, built with LUTs so it stays fast."""
    hue, sat, val = image_rgb.convert('HSV').split()
    low = hue.point(lambda h: 255 if h <= HUE_LOW_MAX else 0)
    high = hue.point(lambda h: 255 if HUE_HIGH_MIN <= h <= HUE_HIGH_MAX else 0)
    saturated = sat.point(lambda s: 255 if s >= SAT_MIN else 0)
    bright = val.point(lambda v: 255 if v >= VAL_MIN else 0)
    return ImageChops.multiply(
        ImageChops.multiply(ImageChops.lighter(low, high), saturated), bright)


def connected_components(mask, scale=2):
    """Red components as (area, bbox, fill) in full-resolution coordinates.

    This is the part the first version of this file got wrong: it took one
    bounding box for the whole mask, so a ball on the floor and a mahogany table
    leg merged into a single box whose fill ratio was near zero, and every real
    detection was thrown away. The on-board OpenCV detector evaluates contours
    individually; this reproduces that.

    The mask is halved before labelling so that the Python-level flood fill stays
    cheap. A 35 mm ball at 1.5 m is still about 30 half-resolution pixels.
    """
    width, height = mask.size
    small = mask.resize((max(1, width // scale), max(1, height // scale)),
                        Image.NEAREST)
    pixels = small.load()
    small_w, small_h = small.size
    seen = set()
    components = []
    for y in range(small_h):
        for x in range(small_w):
            if pixels[x, y] <= 127 or (x, y) in seen:
                continue
            queue = [(x, y)]
            seen.add((x, y))
            points = []
            while queue:
                cx, cy = queue.pop()
                points.append((cx, cy))
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        nx, ny = cx + dx, cy + dy
                        if (0 <= nx < small_w and 0 <= ny < small_h
                                and (nx, ny) not in seen
                                and pixels[nx, ny] > 127):
                            seen.add((nx, ny))
                            queue.append((nx, ny))
            if not points:
                continue
            xs = [p[0] for p in points]
            ys = [p[1] for p in points]
            box = (min(xs) * scale, min(ys) * scale,
                   (max(xs) + 1) * scale, (max(ys) + 1) * scale)
            box_w = box[2] - box[0]
            box_h = box[3] - box[1]
            area = len(points) * scale * scale
            fill = len(points) / float(max(1, box_w // scale) * max(1, box_h // scale))
            components.append({'area': area, 'bbox': box, 'fill': fill,
                               'width': box_w, 'height': box_h,
                               'center_x': (box[0] + box[2]) / 2.0,
                               'center_y': (box[1] + box[3]) / 2.0,
                               'radius_px': max(box_w, box_h) / 2.0})
    return components


def mean_saturation(image_rgb, mask, box, stride=2):
    """Average saturation of the masked pixels inside ``box``.

    Sampling every ``stride`` pixels keeps this cheap; the value only has to
    separate "saturated red" from "dark brown wood", not be exact.
    """
    saturation = image_rgb.convert('HSV').split()[1]
    total = 0.0
    count = 0
    for y in range(box[1], box[3], stride):
        for x in range(box[0], box[2], stride):
            if mask.getpixel((x, y)) > 127:
                total += saturation.getpixel((x, y))
                count += 1
    return (total / count) if count else 0.0


def find_red_ball(image_rgb, min_px=MIN_BLOB_PX, max_px=MAX_BLOB_PX):
    """Pick the most ball-like red component, or explain why none qualifies.

    Every component is judged on its own, then the best candidate wins. Judging
    only the largest component would always return the furniture.
    """
    mask = red_mask(image_rgb)
    components = connected_components(mask)
    if not components:
        return None, 'no_red_component'
    scored = []
    rejected = {}
    for component in components:
        if component['area'] < min_px:
            rejected['too_small'] = rejected.get('too_small', 0) + 1
            continue
        if component['area'] > max_px:
            rejected['too_large'] = rejected.get('too_large', 0) + 1
            continue
        aspect = component['width'] / float(max(1, component['height']))
        if not MIN_ASPECT <= aspect <= MAX_ASPECT:
            rejected['aspect'] = rejected.get('aspect', 0) + 1
            continue
        if component['fill'] < MIN_FILL:
            rejected['fill'] = rejected.get('fill', 0) + 1
            continue
        saturation = mean_saturation(image_rgb, mask, component['bbox'])
        if saturation < MIN_MEAN_SAT:
            rejected['dull'] = rejected.get('dull', 0) + 1
            continue
        component['aspect'] = round(aspect, 3)
        component['mean_saturation'] = round(saturation, 1)
        scored.append(component)
    if not scored:
        return None, 'no_ball_shape (%s)' % (rejected or 'none')
    best = max(scored, key=lambda c: c['fill'])
    return best, None


class FrameReader(threading.Thread):
    """Keeps the newest decoded frame and a stability-filtered verdict."""

    def __init__(self, bridge, topic, stability_frames=4, center_px=12.0,
                 radius_fraction=0.30):
        super().__init__(daemon=True)
        self.bridge = bridge
        self.topic = topic
        self.stability_frames = stability_frames
        self.center_px = center_px
        self.radius_fraction = radius_fraction
        self.lock = threading.Lock()
        self.latest = None          # (monotonic, detection, reason, image)
        self.history = []
        self.frames = 0
        self.stop = False
        bridge.subscribe(topic, 'sensor_msgs/msg/CompressedImage', throttle_rate=200)

    def run(self):
        while not self.stop:
            try:
                message = self.bridge.wait_for_topic(self.topic, 3.0)
            except RosbridgeError:
                return
            if message is None:
                continue
            try:
                raw = base64.b64decode(message['data'])
                image = Image.open(io.BytesIO(raw))
                image.load()
                image = image.convert('RGB')
            except Exception as error:      # noqa: BLE001 - a bad frame is not fatal
                with self.lock:
                    self.latest = (time.monotonic(), None, 'decode: %s' % error, None)
                continue
            detection, reason = find_red_ball(image)
            self.frames += 1
            if detection is None:
                self.history = []
            else:
                self.history.append(detection)
                self.history = self.history[-self.stability_frames:]
            stable = (detection is not None
                      and len(self.history) >= self.stability_frames
                      and self._consistent())
            with self.lock:
                self.latest = (time.monotonic(),
                               detection if stable else None,
                               'stable' if stable else (reason or 'not_stable'),
                               image)

    def _consistent(self):
        xs = [d['center_x'] for d in self.history]
        ys = [d['center_y'] for d in self.history]
        rs = [d['radius_px'] for d in self.history]
        spread = max(max(xs) - min(xs), max(ys) - min(ys))
        if spread > self.center_px:
            return False
        mean_radius = sum(rs) / len(rs)
        return (max(rs) - min(rs)) <= mean_radius * self.radius_fraction

    def snapshot(self):
        with self.lock:
            return self.latest


class Chassis:
    """Publishes /cmd_vel at a fixed rate; stops if this process dies."""

    def __init__(self, bridge, topic='/cmd_vel'):
        self.bridge = bridge
        self.topic = topic
        bridge.advertise(topic, 'geometry_msgs/msg/Twist')

    def drive(self, angular, linear=0.0):
        self.bridge.publish(self.topic, {
            'linear': {'x': float(linear), 'y': 0.0, 'z': 0.0},
            'angular': {'x': 0.0, 'y': 0.0, 'z': float(angular)}})

    def stop(self, repeats=5, interval=0.05):
        for _ in range(repeats):
            self.drive(0.0)
            time.sleep(interval)


def chassis_motion_enabled(bridge):
    """Read the chassis' own verdict instead of trusting a stale assumption."""
    bridge.subscribe('/chassis_control/status', 'std_msgs/msg/String')
    message = bridge.wait_for_topic('/chassis_control/status', 6.0)
    if not message:
        return None, 'no /chassis_control/status'
    try:
        status = json.loads(message['data'])
    except ValueError:
        return None, 'status is not JSON'
    if status.get('dry_run'):
        return False, 'chassis is in dry_run'
    if status.get('estop'):
        return False, 'chassis motion is latched'
    if status.get('hardware_fault'):
        return False, 'chassis hardware fault'
    if not status.get('enable_motion'):
        return False, 'chassis enable_motion is false'
    return True, 'enable_motion true, no fault'


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--port', type=int, default=19091)
    parser.add_argument('--topic', default='/rear_camera/image_raw/compressed')
    parser.add_argument('--speed', type=float, default=0.18)
    parser.add_argument('--step-deg', type=float, default=45.0)
    parser.add_argument('--max-deg', type=float, default=360.0)
    parser.add_argument('--settle', type=float, default=0.8)
    parser.add_argument('--observe', type=float, default=2.0)
    parser.add_argument('--confirm', type=float, default=0.4)
    parser.add_argument('--duration', type=float, default=180.0)
    parser.add_argument('--frame-timeout', type=float, default=4.0)
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args(argv)

    if args.speed > 0.20:
        raise SystemExit('refusing: chassis caps angular velocity at 0.20 rad/s')

    control = Rosbridge('127.0.0.1', args.port, timeout=6)
    vision = Rosbridge('127.0.0.1', args.port, timeout=6)

    enabled, detail = chassis_motion_enabled(control)
    print('chassis: %s (%s)' % (enabled, detail), flush=True)

    reader = FrameReader(vision, args.topic)
    reader.start()

    print('waiting for the first frame...', flush=True)
    deadline = time.monotonic() + 15
    while time.monotonic() < deadline and reader.snapshot() is None:
        time.sleep(0.1)
    if reader.snapshot() is None:
        print('no frames arriving from %s' % args.topic, flush=True)
        return 1
    print('frames arriving', flush=True)

    if args.dry_run:
        for index in range(6):
            time.sleep(0.4)
            stamp, detection, reason, image = reader.snapshot() or (0, None, 'none', None)
            print('  [%d] %-28s %s' % (index, reason,
                                       '' if not detection else
                                       'center=(%.0f,%.0f) r=%.1f px area=%d'
                                       % (detection['center_x'], detection['center_y'],
                                          detection['radius_px'], detection['area_px'])),
                  flush=True)
            if image is not None and detection is not None:
                image.save(os.path.join(REPO, 'snapshots', 'red_candidate.jpg'))
                print('      saved snapshots/red_candidate.jpg', flush=True)
        reader.stop = True
        reader.bridge.close()
        control.close()
        return 0

    if not enabled:
        print('refusing to move: %s' % detail, flush=True)
        return 1

    chassis = Chassis(control)
    planner = ScanPlanner(
        step_rad=math.radians(args.step_deg),
        max_sweep_rad=math.radians(args.max_deg),
        spin_speed=args.speed, settle_s=args.settle, observe_s=args.observe,
        confirm_s=args.confirm, max_duration_s=args.duration)

    found = None
    planner.start(time.monotonic())
    print('sweeping %.0f deg in %.0f deg steps at %.2f rad/s'
          % (args.max_deg, args.step_deg, args.speed), flush=True)
    last_state = None
    frame_stall_since = None
    try:
        while planner.running:
            now = time.monotonic()
            stamp, detection, reason, image = reader.snapshot() or (0, None, 'none', None)
            if now - stamp > args.frame_timeout:
                if frame_stall_since is None:
                    frame_stall_since = now
                if now - frame_stall_since > 3.0:
                    chassis.stop()
                    print('ABORT: no frames for %.1f s' % (now - stamp), flush=True)
                    break
            else:
                frame_stall_since = None

            command = planner.tick(now, detection is not None)
            if command.state != last_state:
                last_state = command.state
                print('  %-8s turned=%5.1f deg  seen=%s'
                      % (command.state, math.degrees(command.travelled_rad),
                         'yes' if detection else 'no'), flush=True)
            chassis.drive(command.wz if command.kind == 'spin' else 0.0)
            if command.kind == 'found':
                found = detection
                if image is not None:
                    image.save(os.path.join(REPO, 'snapshots', 'ball_found.jpg'))
                break
            time.sleep(0.05)
    finally:
        chassis.stop()
        chassis.bridge.unadvertise('/cmd_vel')
        reader.stop = True

    summary = planner.snapshot()
    print('\nresult: state=%s turned=%.1f deg steps=%d'
          % (summary['state'], summary['travelled_deg'], summary['steps_done']),
          flush=True)
    if found:
        print('candidate at (%.0f, %.0f) px, radius %.1f px, area %d px, '
              'saturation %.0f -- VERIFY THE SAVED FRAME, the shape gates alone '
              'are not proof'
              % (found['center_x'], found['center_y'], found['radius_px'],
                 found['area'], found.get('mean_saturation', 0.0)), flush=True)
        print('snapshot: snapshots/ball_found.jpg', flush=True)
        code = 0
    else:
        print('not found in %.0f deg' % summary['travelled_deg'], flush=True)
        code = 2
    vision.close()
    control.close()
    return code


if __name__ == '__main__':
    sys.exit(main())
