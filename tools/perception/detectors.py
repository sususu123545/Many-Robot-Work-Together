#!/usr/bin/env python3
"""Object detection back-ends behind one interface.

Why this file exists
--------------------
``ground_localizer.locate()`` only ever needs five numbers from a detector:
where the object sits horizontally, how far down it reaches, and how big it
looks.  It does not care whether those numbers came from a colour threshold or
from YOLO.  So the detector is written as a plug-in and the geometry code stays
untouched when the detector is swapped.

The five numbers, and why each one is needed
--------------------------------------------
    center_x   - the ray that finds the contact point on the plane
    bottom_v   - the LOWEST pixel of the silhouette; this is the contact pixel.
                 For a colour blob it is center_y + radius; for a YOLO box it is
                 y + height/2.  Kept as a field so callers never guess.
    top_v      - highest pixel, used to get the pixel height
    width, height - box or bounding box of the blob, in pixels

Two back-ends
-------------
``ColorDetector``  the existing redness threshold + connected components.  Free,
                   instant, and already proven on this ball.  Works on anything
                   strongly coloured against a dull background.
``OnnxDetector``   any YOLO-family ONNX exported by ultralytics.  Costs a model
                   file and CPU time, but finds objects by shape and can tell a
                   bottle from a ball.  Falls back with a clear message when the
                   model file is absent, so nothing crashes on a machine that
                   has no weights yet.

Usage
-----
    python3 tools/perception/detectors.py --backend color
    python3 tools/perception/detectors.py --backend onnx --model /opt/vision/yolo11n.onnx
"""

import argparse
import os
import sys
import time

try:
    import numpy as np
except ImportError:          # colour backend runs fine without numpy
    np = None

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
REPO = os.path.dirname(TOOLS)
sys.path.insert(0, TOOLS)
sys.path.insert(0, os.path.join(REPO, 'ros2_ws', 'src', 'armpi_ros2', 'armpi_ros2'))

# Class names in ultralytics order. A COCO-trained export says nothing about
# itself beyond "84 numbers per box", so having this table here is what turns
# column 47 into the word "banana". Class ids follow the model exactly.
COCO80 = (
    'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train',
    'truck', 'boat', 'traffic light', 'fire hydrant', 'stop sign',
    'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow',
    'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag',
    'tie', 'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball', 'kite',
    'baseball bat', 'baseball glove', 'skateboard', 'surfboard',
    'tennis racket', 'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon',
    'bowl', 'banana', 'apple', 'sandwich', 'orange', 'broccoli', 'carrot',
    'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch', 'potted plant',
    'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse', 'remote',
    'keyboard', 'cell phone', 'microwave', 'oven', 'toaster', 'sink',
    'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear',
    'hair drier', 'toothbrush',
)


class Detection(object):
    """One object, in pixels.  The only currency the geometry layer accepts."""

    __slots__ = ('label', 'confidence', 'center_x', 'center_y',
                 'bottom_v', 'top_v', 'width', 'height', 'extra')

    def __init__(self, label, confidence, center_x, center_y,
                 bottom_v, top_v, width, height, extra=None):
        self.label = label
        self.confidence = confidence
        self.center_x = float(center_x)
        self.center_y = float(center_y)
        self.bottom_v = float(bottom_v)
        self.top_v = float(top_v)
        self.width = float(width)
        self.height = float(height)
        self.extra = extra or {}

    @property
    def radius_px(self):
        """Equivalent circle radius, matching the older blob convention."""
        return (self.width + self.height) / 4.0

    def as_blob(self):
        """Dict shaped like the old blob, for code that has not migrated."""
        return {'center_x': self.center_x, 'center_y': self.center_y,
                'radius_px': self.radius_px,
                'width': self.width, 'height': self.height}

    def __repr__(self):
        return ('Detection(%s %.2f at %.0f,%.0f box %.0fx%.0f)'
                % (self.label, self.confidence, self.center_x, self.center_y,
                   self.width, self.height))


class ColorDetector(object):
    """Redness threshold.  No model file, no dependencies beyond the repo."""

    name = 'color'

    def __init__(self, min_radius=10.0, max_radius=160.0,
                 min_fill=0.5, aspect_range=(0.5, 2.0), min_confidence=0.0):
        self.min_radius = min_radius
        self.max_radius = max_radius
        self.min_fill = min_fill
        self.aspect_range = aspect_range
        self.min_confidence = min_confidence

    def detect(self, image):
        """image: PIL.Image or numpy RGB array.  Returns [Detection]."""
        import car_approach as ca
        from car_scan import connected_components

        mask = ca.redness_mask(image)
        found = []
        for component in connected_components(mask):
            radius = (component['area'] / 3.14159) ** 0.5
            if not self.min_radius <= radius <= self.max_radius:
                continue
            if component['fill'] < self.min_fill:
                continue
            aspect = component['width'] / float(max(1, component['height']))
            low, high = self.aspect_range
            if not low <= aspect <= high:
                continue
            center_y = component['center_y']
            found.append(Detection(
                label='ball',
                confidence=min(1.0, component['fill']),
                center_x=component['center_x'],
                center_y=center_y,
                bottom_v=center_y + radius,
                top_v=center_y - radius,
                width=component['width'],
                height=component['height'],
                extra={'area': component['area']}))
        found.sort(key=lambda d: -d.confidence)
        return [d for d in found if d.confidence >= self.min_confidence]


class OnnxDetector(object):
    """YOLO-family ONNX from ultralytics.  Letterbox in, boxes out.

    Supports both layout conventions ultralytics emits:
        [1, 4+nc, N]   (default export, needs a transpose)
        [1, N, 4+nc]   (already transposed)
    """

    name = 'onnx'

    def __init__(self, model_path, input_size=320, confidence=0.35,
                 iou_threshold=0.45, labels=None, max_detections=20,
                 intra_threads=0):
        self.model_path = model_path
        self.input_size = int(input_size)
        self.confidence = confidence
        self.iou_threshold = iou_threshold
        self.labels = list(labels) if labels else []
        self.max_detections = max_detections
        self.intra_threads = intra_threads
        self._session = None
        self._input_name = None
        self._input_shape = None

    def available(self):
        return os.path.exists(self.model_path)

    def load(self):
        if self._session is not None:
            return self
        if np is None:
            raise RuntimeError('the onnx backend needs numpy: pip install numpy')
        if not self.available():
            raise RuntimeError('model file not found: %s' % self.model_path)
        import onnxruntime as ort
        options = ort.SessionOptions()
        # The Pi has four cores and the arm node shares them; keep the default
        # thread count small so inference does not starve the servo loop.
        options.intra_op_num_threads = self.intra_threads or 2
        options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        self._session = ort.InferenceSession(self.model_path, options,
                                             providers=['CPUExecutionProvider'])
        model_input = self._session.get_inputs()[0]
        self._input_name = model_input.name
        self._input_shape = model_input.shape
        # A fixed-shape export rejects any other size, and the error onnxruntime
        # raises for that is a wall of text. Adopt the model's own size instead.
        if isinstance(self._input_shape, (list, tuple)) and len(self._input_shape) == 4:
            height = self._input_shape[2]
            if isinstance(height, int) and height > 0:
                if height != self.input_size:
                    print('note: model input is %dx%d, overriding --size %d'
                          % (height, self._input_shape[3], self.input_size))
                self.input_size = height
        return self

    def _letterbox(self, image):
        """Resize keeping aspect, pad to a square, return array and transform."""
        from PIL import Image
        if not isinstance(image, Image.Image):
            image = Image.fromarray(np.asarray(image))
        width, height = image.size
        size = self.input_size
        scale = min(size / float(width), size / float(height))
        new_w, new_h = int(round(width * scale)), int(round(height * scale))
        pad_x, pad_y = (size - new_w) // 2, (size - new_h) // 2
        canvas = Image.new('RGB', (size, size), (114, 114, 114))
        canvas.paste(image.resize((new_w, new_h), Image.BILINEAR), (pad_x, pad_y))
        array = np.asarray(canvas, dtype=np.float32) / 255.0
        array = np.transpose(array, (2, 0, 1))[None, ...]
        return np.ascontiguousarray(array), scale, pad_x, pad_y

    def detect(self, image):
        self.load()
        tensor, scale, pad_x, pad_y = self._letterbox(image)
        outputs = self._session.run(None, {self._input_name: tensor})
        raw = np.asarray(outputs[0])
        if raw.ndim == 3:
            raw = raw[0]
        # ultralytics default: (4+nc, N); transposed exports: (N, 4+nc)
        if raw.shape[0] < raw.shape[1]:
            raw = raw.T
        if raw.shape[1] < 5:
            return []

        boxes = raw[:, :4]
        scores = raw[:, 4:]
        if scores.shape[1] == 0:
            return []
        labels = self.labels
        if not labels and scores.shape[1] == len(COCO80):
            labels = COCO80
        class_ids = scores.argmax(axis=1)
        confidences = scores[np.arange(scores.shape[0]), class_ids]
        keep = confidences >= self.confidence
        if not np.any(keep):
            return []
        boxes, confidences, class_ids = boxes[keep], confidences[keep], class_ids[keep]

        # cx, cy, w, h in letterboxed pixels -> original pixels
        cx = (boxes[:, 0] - pad_x) / scale
        cy = (boxes[:, 1] - pad_y) / scale
        bw = boxes[:, 2] / scale
        bh = boxes[:, 3] / scale

        indices = _nms(cx, cy, bw, bh, confidences, self.iou_threshold)

        found = []
        for index in indices[:self.max_detections]:
            class_id = int(class_ids[index])
            label = labels[class_id] if class_id < len(labels) else str(class_id)
            height = float(bh[index])
            found.append(Detection(
                label=label,
                confidence=float(confidences[index]),
                center_x=float(cx[index]),
                center_y=float(cy[index]),
                bottom_v=float(cy[index] + height / 2.0),
                top_v=float(cy[index] - height / 2.0),
                width=float(bw[index]),
                height=height,
                extra={'class_id': class_id}))
        return found


def _nms(cx, cy, bw, bh, scores, iou_threshold):
    """Greedy non-maximum suppression on centre/size boxes.  Returns indices."""
    x1, y1 = cx - bw / 2.0, cy - bh / 2.0
    x2, y2 = cx + bw / 2.0, cy + bh / 2.0
    areas = np.maximum(bw, 0) * np.maximum(bh, 0)
    order = scores.argsort()[::-1]
    keep = []
    while order.size > 0:
        i = order[0]
        keep.append(int(i))
        if order.size == 1:
            break
        rest = order[1:]
        xx1 = np.maximum(x1[i], x1[rest])
        yy1 = np.maximum(y1[i], y1[rest])
        xx2 = np.minimum(x2[i], x2[rest])
        yy2 = np.minimum(y2[i], y2[rest])
        inter = np.maximum(0.0, xx2 - xx1) * np.maximum(0.0, yy2 - yy1)
        union = areas[i] + areas[rest] - inter
        iou = np.where(union > 0, inter / np.maximum(union, 1e-9), 0.0)
        order = rest[iou <= iou_threshold]
    return keep


def pick_best(detections, label=None, prefer='largest'):
    """Pick one detection: filter by label, then largest or most confident."""
    candidates = [d for d in detections if label is None or d.label == label]
    if not candidates:
        return None
    if prefer == 'confidence':
        return max(candidates, key=lambda d: d.confidence)
    return max(candidates, key=lambda d: d.width * d.height)


def build(spec='color', model=None, **kwargs):
    """Factory used by the higher-level tools."""
    if spec == 'color':
        return ColorDetector(**kwargs)
    if spec == 'onnx':
        if not model:
            raise SystemExit('onnx backend needs --model <path.onnx>')
        return OnnxDetector(model, **kwargs)
    raise SystemExit('unknown detector backend: %s' % spec)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--backend', default='color', choices=('color', 'onnx'))
    parser.add_argument('--model', default='/opt/vision/models/yolo11n.onnx')
    parser.add_argument('--image', help='use a local file instead of the live camera')
    parser.add_argument('--url', default='http://127.0.0.1:19099/snapshot'
                                        '?topic=/rear_camera/image_raw&type=ros_compressed')
    parser.add_argument('--size', type=int, default=320)
    parser.add_argument('--repeat', type=int, default=1,
                        help='run the detector N times and report the mean latency')
    args = parser.parse_args()

    from PIL import Image
    if args.image:
        image = Image.open(args.image).convert('RGB')
    else:
        import urllib.request
        from io import BytesIO
        with urllib.request.urlopen(args.url, timeout=15) as response:
            image = Image.open(BytesIO(response.read())).convert('RGB')
    print('image %dx%d' % image.size)

    detector = build(args.backend, model=args.model,
                     **({'input_size': args.size} if args.backend == 'onnx' else {}))
    if args.backend == 'onnx' and not detector.available():
        print('model file missing: %s' % args.model)
        print('export it with: yolo export model=yolo11n.pt format=onnx imgsz=%d' % args.size)
        return 2

    started = time.time()
    detections = detector.detect(image)
    if args.repeat > 1:
        # one warm-up first, then the timed runs
        detector.detect(image)
        started = time.time()
        for _ in range(args.repeat):
            detector.detect(image)
        elapsed = (time.time() - started) / args.repeat
        print('mean latency %.1f ms  (%.1f fps)'
              % (elapsed * 1000, 1.0 / max(elapsed, 1e-6)))
    else:
        print('latency %.1f ms' % ((time.time() - started) * 1000))

    if not detections:
        print('nothing detected')
        return 1
    for detection in detections:
        print('  %r' % detection)
    return 0


if __name__ == '__main__':
    sys.exit(main())
