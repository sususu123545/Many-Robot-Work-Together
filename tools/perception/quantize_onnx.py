#!/usr/bin/env python3
"""Shrink a YOLO ONNX for the Pi: fp32 -> int8 dynamic quantization.

Two reasons, both measured on this project:

  * the link to the car moves roughly 100-300 KiB/s, so 12.8 MB is minutes and
    3.5 MB is seconds. Transfer time is a real constraint, not a nicety;
  * on a Cortex-A72 without any vectorised int8 kernels for fp32, dynamic
    quantization typically halves inference time.

Dynamic quantization needs no calibration set: it converts the weights and
computes activation scales on the fly. Accuracy loss on detection is small
(often 1-2 mAP), and this model is a bootstrap anyway -- the final one will be
trained on our own data and exported straight to int8.

Usage
    python tools/perception/quantize_onnx.py models/yolov8n_coco.onnx
    python tools/perception/quantize_onnx.py in.onnx --output out.onnx --check image.png
"""

import argparse
import os
import sys


def quantize(source, output, per_channel=True, reduce_range=False):
    """Dynamic int8 quantization. Returns (input_bytes, output_bytes)."""
    try:
        from onnxruntime.quantization import QuantType, quantize_dynamic
    except ImportError:
        raise SystemExit('needs onnxruntime and onnx: pip install onnxruntime onnx')
    quantize_dynamic(source, output,
                     weight_type=QuantType.QInt8,
                     per_channel=per_channel,
                     reduce_range=reduce_range)
    return os.path.getsize(source), os.path.getsize(output)


def compare(original, quantized, image_path, size=640):
    """Run both models on one image and report agreement and speed."""
    import time
    import numpy as np
    import onnxruntime as ort
    from PIL import Image

    image = Image.open(image_path).convert('RGB')
    tensor = np.asarray(image.resize((size, size)), dtype=np.float32) / 255.0
    tensor = np.ascontiguousarray(np.transpose(tensor, (2, 0, 1))[None, ...])

    results = {}
    for name, path in (('fp32', original), ('int8', quantized)):
        session = ort.InferenceSession(path, providers=['CPUExecutionProvider'])
        input_name = session.get_inputs()[0].name
        session.run(None, {input_name: tensor})            # warm up
        started = time.time()
        runs = 5
        for _ in range(runs):
            output = session.run(None, {input_name: tensor})[0]
        elapsed = (time.time() - started) / runs
        results[name] = (output, elapsed)
        print('  %-4s %7.1f ms/run   shape %s'
              % (name, elapsed * 1000, np.asarray(output).shape))

    small, large = results['int8'][1], results['fp32'][1]
    print('  speedup %.2fx' % (large / max(small, 1e-9)))
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('source')
    parser.add_argument('--output', help='default: <source>_int8.onnx')
    parser.add_argument('--no-per-channel', action='store_true',
                        help='per-tensor weights; smaller, slightly less accurate')
    parser.add_argument('--check', help='image to run both models on for a timing check')
    parser.add_argument('--size', type=int, default=640)
    args = parser.parse_args()

    output = args.output or os.path.splitext(args.source)[0] + '_int8.onnx'
    before, after = quantize(args.source, output,
                             per_channel=not args.no_per_channel)
    print('%s -> %s' % (args.source, output))
    print('  %.1f MB -> %.1f MB  (%.0f%% of original, %.1fx smaller)'
          % (before / 1e6, after / 1e6, 100.0 * after / before, before / float(after)))

    if args.check:
        if not os.path.exists(args.check):
            print('check image not found: %s' % args.check)
            return 0
        print('timing both models on %s' % args.check)
        compare(args.source, output, args.check, args.size)
    return 0


if __name__ == '__main__':
    sys.exit(main())
