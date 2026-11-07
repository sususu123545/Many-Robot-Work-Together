#!/usr/bin/env python3
"""Read-only probe: why does slam_toolbox's message_filter drop every scan?

History
-------
v1 : every scan stamp is +0.042..+0.050 s newer than the newest
     ``odom->base_link`` transform.  Guessed "tf2 must extrapolate".
v2 : asked ``lookup_transform('base_link', 'base_laser', scan_stamp)`` and got
     92/92 OK.  **Wrong question** -- those frames share a *static* transform,
     so tf2 never walks ``odom->base_link``.  slam_toolbox works in the odom
     frame; the chain that matters is ``odom <- base_link <- base_laser``.
v3 : asked the right chain.  At the scan's own stamp: 0 OK / 92 FAIL, while
     ``latest`` was OK.  But v3 also tried a blocking lookup *inside the scan
     callback*, which on a single-threaded executor stops TF from arriving --
     so "waiting does not help" was an artefact of the probe, not a fact.

v4 removes that artefact.  The scan callback only records
``(stamp, now)``; the main loop, outside any callback, re-runs the lookup once
the stamp is old enough that the odom TF *must* have caught up:

  * if the delayed lookup succeeds -> the data is fine, MessageFilter would also
    have resolved it, and the drop has to be a queue/config problem in
    slam_toolbox
  * if it still fails -> the scan stamp really is unserviceable and the fix must
    be on the timestamp side

Prints the real exception text, which v3 threw away.

Touches nothing and publishes nothing.  Run inside the core container.
"""
import collections
import sys
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener

WINDOW_S = 14.0
DELAY_S = 0.30          # how long a stamp must age before we test it
MAX_REPORTS = 6


class Probe(Node):
    def __init__(self):
        super().__init__('tf_scan_probe')
        self.buffer = Buffer(cache_time=Duration(seconds=60.0))
        self.listener = TransformListener(self.buffer, self)
        self.tf_stamps = []
        self.static_frames = []
        self.pending = collections.deque()
        self.scans = 0
        self.lags = []
        self.instant_ok = 0
        self.instant_fail = 0
        self.instant_names = {}
        self.delayed_ok = 0
        self.delayed_fail = 0
        self.delayed_names = {}
        self.examples = []

        latched = QoSProfile(depth=100)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.create_subscription(TFMessage, '/tf', self.on_tf, 200)
        self.create_subscription(TFMessage, '/tf_static', self.on_static, latched)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 50)

    # -- subscriptions -----------------------------------------------------
    def on_tf(self, message):
        for transform in message.transforms:
            if (transform.header.frame_id == 'odom'
                    and transform.child_frame_id == 'base_link'):
                self.tf_stamps.append(transform.header.stamp.sec
                                      + transform.header.stamp.nanosec * 1e-9)

    def on_static(self, message):
        for transform in message.transforms:
            pair = '%s->%s' % (transform.header.frame_id,
                               transform.child_frame_id)
            if pair not in self.static_frames:
                self.static_frames.append(pair)
                print('TF_STATIC %s' % pair, flush=True)

    def on_scan(self, message):
        self.scans += 1
        header = message.header.stamp
        self.pending.append((Time(seconds=header.sec,
                                  nanoseconds=header.nanosec),
                             header.sec + header.nanosec * 1e-9,
                             time.monotonic()))

    # -- main-loop work ----------------------------------------------------
    def drain_pending(self):
        now = time.monotonic()
        while self.pending and now - self.pending[0][2] >= DELAY_S:
            when, stamp, _ = self.pending.popleft()
            self.instant_check(when, stamp)
            self.delayed_check(when, stamp)

    def instant_check(self, when, stamp):
        ok = self.buffer.can_transform('odom', 'base_laser', when,
                                       timeout=Duration(seconds=0.0))
        if ok:
            self.instant_ok += 1
        else:
            self.instant_fail += 1

    def delayed_check(self, when, stamp):
        try:
            self.buffer.lookup_transform('odom', 'base_laser', when,
                                         timeout=Duration(seconds=0.0))
            self.delayed_ok += 1
        except Exception as error:                          # noqa: BLE001
            self.delayed_fail += 1
            name = type(error).__name__
            self.delayed_names[name] = self.delayed_names.get(name, 0) + 1
            if len(self.examples) < MAX_REPORTS:
                self.examples.append('stamp=%.3f %s: %s'
                                     % (stamp, name, str(error).strip()))


def stats(values, fmt='%+.3f'):
    if not values:
        return 'n=0'
    ordered = sorted(values)
    return ('n=%d min=' + fmt + ' med=' + fmt + ' max=' + fmt) % (
        len(ordered), ordered[0], ordered[len(ordered) // 2], ordered[-1])


def main():
    rclpy.init()
    node = Probe()
    started = time.monotonic()
    while rclpy.ok() and time.monotonic() - started < WINDOW_S:
        rclpy.spin_once(node, timeout_sec=0.02)
        node.drain_pending()

    elapsed = time.monotonic() - started
    print('---')
    print('window                 : %.1f s' % elapsed)
    print('scans                  : %d (%.1f Hz)'
          % (node.scans, node.scans / elapsed))
    print('odom->base_link TF     : %d (%.1f Hz)'
          % (len(node.tf_stamps), len(node.tf_stamps) / elapsed))
    print('static frames          : %s' % (node.static_frames or 'NONE'))
    print('instant @stamp         : OK=%d FAIL=%d'
          % (node.instant_ok, node.instant_fail))
    print('delayed %.2fs @stamp   : OK=%d FAIL=%d %s'
          % (DELAY_S, node.delayed_ok, node.delayed_fail,
             node.delayed_names or ''))
    for line in node.examples:
        print('  %s' % line)
    try:
        node.buffer.lookup_transform('odom', 'base_laser', Time(),
                                     timeout=Duration(seconds=1.0))
        print('latest                 : OK')
    except Exception as error:                              # noqa: BLE001
        print('latest                 : %s' % type(error).__name__)
    print('undrained              : %d' % len(node.pending))
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
