#!/usr/bin/env python3
"""Collect labelled training images from the live camera.

Why this exists
---------------
A YOLO model needs a few hundred images of the actual scene: this camera, this
lens, this lighting, these objects.  Public datasets do not help, because the
fisheye distortion and the downward view are ours alone.  Clicking out boxes by
hand for several hundred frames is the slowest part of the whole pipeline, so
this tool does the boring half:

  * grabs frames from the running camera at a chosen interval,
  * runs whatever detector is available and writes a YOLO-format label file,
  * stores the joint ticks and pose alongside, so any frame can be replayed at
    the exact arm configuration it was taken in.

The labels are a starting point, not the truth.  A red-threshold box is a good
guess for the ball and useless for a bottle, so the workflow is: auto-label what
the colour detector can see, hand-correct in any annotation tool, then train.

Output layout (YOLO convention)
    datasets/<name>/images/frame_000123.png
    datasets/<name>/labels/frame_000123.txt      # cls cx cy w h, normalised
    datasets/<name>/meta.jsonl                   # ticks, detector, timestamp

Usage
    python3 tools/perception/collect_dataset.py --name ball --count 60 --interval 1.0
    python3 tools/perception/collect_dataset.py --name ball --count 200 --auto-label none
    python3 tools/perception/collect_dataset.py --name ball --from-dir shots/
"""

import argparse
import json
import os
import sys
import time
import urllib.request
from io import BytesIO

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
REPO = os.path.dirname(TOOLS)
sys.path.insert(0, TOOLS)
sys.path.insert(0, HERE)

SNAPSHOT = ('http://127.0.0.1:19099/snapshot'
            '?topic=/rear_camera/image_raw&type=ros_compressed')
SIGNS = {'joint1': 1.0, 'joint2': 1.0, 'joint3': -1.0, 'joint4': 1.0, 'joint5': 1.0}
TICK_RAD = 0.00418879  # radians per tick, 240 deg over 1000 ticks


def grab(url=SNAPSHOT, timeout=15):
    """One frame from the camera relay as a PIL image."""
    from PIL import Image
    with urllib.request.urlopen(url, timeout=timeout) as response:
        return Image.open(BytesIO(response.read())).convert('RGB')


def read_ticks(port=19091, timeout=8.0):
    """Live joint ticks, or None.  Kept optional: labelling works without ROS."""
    try:
        from rosbridge_client import Rosbridge
    except ImportError:
        return None
    bridge = None
    try:
        bridge = Rosbridge('127.0.0.1', port, timeout=10)
        bridge.drain(0.3)
        message = bridge.wait_for_topic('/joint_states', timeout, newest=True)
        if message is None:
            return None
        return {name: int(round(500.0 + SIGNS[name] * float(position) / TICK_RAD))
                for name, position in zip(message['name'], message['position'])
                if name in SIGNS}
    except Exception:
        return None
    finally:
        if bridge is not None:
            bridge.close()


def to_yolo(detection, width, height, class_id):
    """Pixel box -> 'cls cx cy w h', all normalised to 0..1."""
    cx = detection.center_x / width
    cy = detection.center_y / height
    box_w = detection.width / width
    box_h = detection.height / height
    clamp = lambda v: min(1.0, max(0.0, v))                       # noqa: E731
    return '%d %.6f %.6f %.6f %.6f' % (class_id, clamp(cx), clamp(cy),
                                       clamp(box_w), clamp(box_h))


def prepare(root):
    images = os.path.join(root, 'images')
    labels = os.path.join(root, 'labels')
    for path in (images, labels):
        if not os.path.isdir(path):
            os.makedirs(path)
    return images, labels


def next_index(images_dir):
    """Continue numbering instead of overwriting an earlier session."""
    highest = -1
    for name in os.listdir(images_dir):
        if name.startswith('frame_') and name.endswith('.png'):
            try:
                highest = max(highest, int(name[6:-4]))
            except ValueError:
                continue
    return highest + 1


def save_pair(image, detection, images_dir, labels_dir, index, class_id,
              suffix=''):
    name = 'frame_%06d%s' % (index, suffix)
    image_path = os.path.join(images_dir, name + '.png')
    image.save(image_path)
    label_path = os.path.join(labels_dir, name + '.txt')
    if detection is None:
        open(label_path, 'w').close()          # empty file = background image
        return image_path, label_path, None
    line = to_yolo(detection, image.size[0], image.size[1], class_id)
    with open(label_path, 'w', encoding='ascii') as handle:
        handle.write(line + '\n')
    return image_path, label_path, line


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--name', default='ball', help='dataset name / folder under datasets/')
    parser.add_argument('--root', default=os.path.join(REPO, 'datasets'))
    parser.add_argument('--count', type=int, default=30)
    parser.add_argument('--interval', type=float, default=1.0,
                        help='seconds between frames when capturing live')
    parser.add_argument('--class-id', type=int, default=0)
    parser.add_argument('--auto-label', default='color', choices=('color', 'onnx', 'none'),
                        help='pre-fill labels with this detector; none leaves them empty')
    parser.add_argument('--model', default='/opt/vision/models/yolo11n.onnx')
    parser.add_argument('--from-dir', help='label existing images instead of capturing')
    parser.add_argument('--url', default=SNAPSHOT)
    parser.add_argument('--port', type=int, default=19091)
    args = parser.parse_args()

    root = os.path.join(args.root, args.name)
    images_dir, labels_dir = prepare(root)
    meta_path = os.path.join(root, 'meta.jsonl')

    detector = None
    module = None
    if args.auto_label != 'none':
        import detectors as module
        detector = module.build(args.auto_label, model=args.model)

    written = 0
    index = next_index(images_dir)

    if args.from_dir:
        from PIL import Image
        names = sorted(n for n in os.listdir(args.from_dir)
                       if n.lower().endswith(('.png', '.jpg', '.jpeg')))
        for name in names[:args.count]:
            image = Image.open(os.path.join(args.from_dir, name)).convert('RGB')
            found = module.pick_best(detector.detect(image)) if detector else None
            save_pair(image, found, images_dir, labels_dir, index,
                      args.class_id, suffix='')
            written += 1
            index += 1
        print('labelled %d images from %s' % (written, args.from_dir))
        return 0

    for step in range(args.count):
        try:
            image = grab(args.url)
        except Exception as error:                  # noqa: BLE001
            print('frame %d: camera unavailable (%s)' % (step, error))
            break
        found = module.pick_best(detector.detect(image)) if detector else None
        image_path, _, label_line = save_pair(
            image, found, images_dir, labels_dir, index, args.class_id)
        ticks = read_ticks(args.port, timeout=3.0)
        with open(meta_path, 'a', encoding='utf-8') as handle:
            handle.write(json.dumps({'image': os.path.basename(image_path),
                                     'label': label_line,
                                     'ticks': ticks,
                                     'detector': args.auto_label,
                                     'time': time.strftime('%Y-%m-%dT%H:%M:%S')}) + '\n')
        written += 1
        index += 1
        status = 'no object' if found is None else '%s %.2f' % (found.label, found.confidence)
        print('[%3d/%3d] %s  %s' % (step + 1, args.count, os.path.basename(image_path), status))
        if step + 1 < args.count:
            time.sleep(args.interval)

    print()
    print('saved %d frames to %s' % (written, root))
    print('images: %s' % images_dir)
    print('labels: %s' % labels_dir)
    if args.auto_label == 'none':
        print('labels are empty by design: annotate before training')
    else:
        print('auto labels are a first guess -- correct them before training')
    return 0


if __name__ == '__main__':
    sys.exit(main())
