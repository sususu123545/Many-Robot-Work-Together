#!/usr/bin/env python3
"""Watch the camera continuously, and optionally turn while watching.

The step-and-stare sweep in ``car_scan.py`` stops for every look. That is the
right shape for the *on-board* detector, which needs five still frames to
localise a target, but it is wasteful when the classification happens here:
one look per 45 degrees is 8 chances per revolution, while a 5 Hz stream at
0.15 rad/s gives about 170.

Blur is not the obstacle people expect it to be. At 0.15 rad/s with a ~30 ms
exposure the smear is 0.0045 rad, which at fx=541 is about 2.4 px -- smaller
than the ball. So a single moving frame is still usable.

False positives are the real obstacle: this room has mahogany furniture whose
average saturation (241) rivals the ball's (235), and a single frame cannot tell
them apart. The answer here is not a cleverer single-frame rule but *temporal
agreement* -- a real object stays red for several consecutive frames while the
base turns -- followed by an immediate stop and a strict re-check at standstill.

Usage:
    python3 tools/car_watch.py                    # monitor only, no motion
    python3 tools/car_watch.py --spin 0.15        # turn and watch
    python3 tools/car_watch.py --spin 0.12 --hits 3
"""

import argparse
import base64
import io
import math
import os
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)

from PIL import Image, ImageDraw  # noqa: E402

from car_scan import find_red_ball, red_mask  # noqa: E402
from rosbridge_client import Rosbridge, RosbridgeError  # noqa: E402


class LiveView(threading.Thread):
    """Decodes frames as fast as rosbridge delivers them and keeps the newest."""

    def __init__(self, bridge, topic, throttle_ms=150):
        super().__init__(daemon=True)
        self.bridge = bridge
        self.topic = topic
        self.stop_flag = False
        self.lock = threading.Lock()
        self.frame = None
        self.frame_at = 0.0
        self.detection = None
        self.reason = 'starting'
        self.frames = 0
        self.decode_errors = 0
        bridge.subscribe(topic, 'sensor_msgs/msg/CompressedImage',
                         throttle_rate=throttle_ms)

    def run(self):
        while not self.stop_flag:
            try:
                message = self.bridge.wait_for_topic(self.topic, 2.0)
            except RosbridgeError:
                return
            if message is None:
                continue
            try:
                image = Image.open(io.BytesIO(base64.b64decode(message['data'])))
                image.load()
                image = image.convert('RGB')
            except Exception:                       # noqa: BLE001 - skip bad frame
                self.decode_errors += 1
                continue
            detection, reason = find_red_ball(image)
            with self.lock:
                self.frame = image
                self.frame_at = time.monotonic()
                self.detection = detection
                self.reason = reason
                self.frames += 1

    def snapshot(self):
        with self.lock:
            return self.frame, self.frame_at, self.detection, self.reason

    def hit(self):
        with self.lock:
            return self.detection


class Driver:
    def __init__(self, bridge, topic='/cmd_vel'):
        self.bridge = bridge
        bridge.advertise(topic, 'geometry_msgs/msg/Twist')
        self.topic = topic

    def spin(self, wz):
        self.bridge.publish(self.topic, {
            'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'angular': {'x': 0.0, 'y': 0.0, 'z': float(wz)}})

    def halt(self, repeats=6):
        for _ in range(repeats):
            self.bridge.publish(self.topic, {
                'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}})
            time.sleep(0.04)


def save_hit(image, detection, path):
    """Save the frame with the candidate outlined, so a human can judge it."""
    annotated = image.copy()
    draw = ImageDraw.Draw(annotated)
    box = detection['bbox']
    draw.rectangle(box, outline=(0, 255, 0), width=3)
    draw.text((box[0], max(0, box[1] - 14)),
              'area=%d S=%.0f' % (detection['area'],
                                  detection.get('mean_saturation', 0)), fill=(0, 255, 0))
    annotated.save(path, quality=92)
    return annotated


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--port', type=int, default=19091)
    parser.add_argument('--topic', default='/rear_camera/image_raw/compressed')
    parser.add_argument('--throttle-ms', type=int, default=150)
    parser.add_argument('--spin', type=float, default=0.0,
                        help='angular velocity in rad/s; 0 monitors without moving')
    parser.add_argument('--seconds', type=float, default=0.0,
                        help='stop after this long; 0 runs until interrupted')
    parser.add_argument('--hits', type=int, default=3,
                        help='consecutive frames a candidate must survive')
    parser.add_argument('--resume', action='store_true',
                        help='keep turning after saving a candidate')
    parser.add_argument('--save-dir', default=os.path.join(REPO, 'snapshots', 'live'))
    args = parser.parse_args(argv)

    if args.spin > 0.20:
        raise SystemExit('refusing: chassis caps angular velocity at 0.20 rad/s')
    os.makedirs(args.save_dir, exist_ok=True)

    control = Rosbridge('127.0.0.1', args.port, timeout=6)
    vision = Rosbridge('127.0.0.1', args.port, timeout=6)
    view = LiveView(vision, args.topic, args.throttle_ms)
    view.start()
    driver = Driver(control) if args.spin else None

    deadline = time.monotonic() + 15
    while time.monotonic() < deadline and view.frames == 0:
        time.sleep(0.1)
    if view.frames == 0:
        print('no frames from %s' % args.topic, flush=True)
        return 1
    print('streaming at %.0f ms throttle (%.1f Hz target)'
          % (args.throttle_ms, 1000.0 / max(1, args.throttle_ms)), flush=True)
    if driver:
        print('turning at %.2f rad/s (%.0f deg/s)'
              % (args.spin, math.degrees(args.spin)), flush=True)
    print('Ctrl-C to stop\n', flush=True)

    started = time.monotonic()
    last_publish = 0.0
    hits = 0
    turn = 0.0
    saved = 0
    last_report = 0.0
    try:
        while True:
            now = time.monotonic()
            if args.seconds and now - started > args.seconds:
                break
            # Publish at 20 Hz, not as fast as the loop runs. An unthrottled
            # publish loop saturates the SSH tunnel and starves the image
            # stream -- measured: frames stopped arriving after 4 s.
            if driver and now - last_publish >= 0.05:
                driver.spin(args.spin)
                last_publish = now
            image, frame_at, detection, reason = view.snapshot()
            if now - frame_at > 2.0:
                print('  ! frames stalled %.1f s' % (now - frame_at), flush=True)
                if driver:
                    driver.halt()
                    break
            if detection:
                hits += 1
            else:
                hits = 0

            if now - last_report >= 0.5:
                last_report = now
                rate = view.frames / max(0.1, now - started)
                red = mask_pixels(image) if image is not None else 0
                print('  %5.1fs  frames=%4d (%.1f Hz)  red=%4d px  %s%s'
                      % (now - started, view.frames, rate, red,
                         'CANDIDATE' if detection else reason[:44],
                         '' if driver else ''),
                      flush=True)

            time.sleep(0.01)

            if detection and hits >= args.hits:
                if driver:
                    driver.halt()
                turn = (now - started) * args.spin
                saved += 1
                path = os.path.join(args.save_dir,
                                    'hit_%02d_%.0fdeg.jpg' % (saved, math.degrees(turn)))
                save_hit(image, detection, path)
                print('\n  >>> candidate after %.0f deg: area=%d r=%.0f px '
                      'S=%.0f at (%.0f,%.0f)'
                      % (math.degrees(turn), detection['area'], detection['radius_px'],
                         detection.get('mean_saturation', 0),
                         detection['center_x'], detection['center_y']), flush=True)
                print('  >>> saved %s' % os.path.relpath(path, REPO), flush=True)
                if not args.resume:
                    print('  >>> stopped. look at the saved frame; the shape gates '
                          'alone are not proof.', flush=True)
                    break
                time.sleep(1.5)
                hits = 0
                started = now      # keep turning from here
    except KeyboardInterrupt:
        print('\ninterrupted', flush=True)
    finally:
        if driver:
            driver.halt()
            control.unadvertise('/cmd_vel')
        view.stop_flag = True
        time.sleep(0.2)
        vision.close()
        control.close()

    print('\nelapsed %.1fs, %d frames, %.0f deg swept, %d candidate(s) saved'
          % (time.monotonic() - started + (0 if turn else 0), view.frames,
             math.degrees((time.monotonic() - started) * args.spin), saved),
          flush=True)
    return 0


def mask_pixels(image):
    """Cheap red-pixel count for the live readout; not a decision input."""
    if image is None:
        return 0
    return red_mask(image).histogram()[255]


if __name__ == '__main__':
    sys.exit(main())
