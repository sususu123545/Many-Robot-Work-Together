#!/usr/bin/env python3
"""Offline tests for cross-boot (map-frame) ball memory. No ROS, no car.

The whole point of the C route: a sighting anchored to a saved map must still
point at the ball after a reboot AND after the car has been physically moved,
while an odom sighting must keep its short, this-run-only lifetime.

Run: python tools/test_ball_memory_cross_boot.py
"""
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, 'perception'))

import ball_memory as bm                                        # noqa: E402

T0 = 1_000_000.0
DAY = 24 * 3600.0


def sighting(frame='map', t=T0, pose=(0.0, 0.0, 0.0), bearing=0.0,
             range_m=0.6):
    entries = []
    bm.record_sighting(entries, {'x': pose[0], 'y': pose[1], 'yaw': pose[2]},
                       bearing, range_m, when=t, frame=frame)
    return entries


class TestCrossBoot(unittest.TestCase):

    def test_map_memory_survives_days_while_odom_expires(self):
        for frame in ('odom', 'map'):
            entries = sighting(frame=frame)
            pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
            fresh = bm.predict(entries, pose, now=T0 + 10, frame=frame)
            later = bm.predict(entries, pose, now=T0 + 3 * DAY, frame=frame)
            if frame == 'odom':
                self.assertEqual(len(fresh), 1)
                self.assertEqual(later, [], 'odom memory must not survive hours')
            else:
                self.assertEqual(len(fresh), 1)
                self.assertEqual(len(later), 1,
                                 'map memory must survive a multi-day gap')
                self.assertGreater(later[0]['confidence'], 0.5)

    def test_map_memory_still_points_at_the_ball_after_car_is_moved(self):
        """The reboot scenario: odom resets, amcl reports a NEW map pose."""
        # Ball seen dead ahead at 0.6 m from map origin, i.e. at (0.6, 0).
        entries = sighting(frame='map', t=T0)
        # After a reboot the operator put the car 1 m to the right, same heading.
        # A pure odom hint would now be meaningless; a map hint must still work.
        pose_after_reboot = {'x': 0.0, 'y': -1.0, 'yaw': 0.0}
        ranked = bm.predict(entries, pose_after_reboot, now=T0 + 10,
                            frame='map')
        self.assertEqual(len(ranked), 1)
        # dx=0.6, dy=1.0 -> atan2(1.0, 0.6) = 59 degrees to the left.
        self.assertAlmostEqual(ranked[0]['bearing_deg'], 59.0, delta=1.0)
        self.assertEqual(ranked[0]['quadrant'], 90)
        self.assertEqual(ranked[0]['frame'], 'map')

    def test_successful_grasp_does_not_burn_map_prior(self):
        entries = sighting(frame='map', t=T0)
        before = bm.predict(entries, {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
                            now=T0 + 10, frame='map')[0]['confidence']
        bm.record_outcome(entries, True, when=T0 + 20, note='round 1')
        after = bm.predict(entries, {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
                           now=T0 + 30, frame='map')
        self.assertEqual(len(after), 1,
                         'a grasp must not permanently erase the cross-boot prior')
        self.assertLess(after[0]['confidence'], before,
                        'but a spent sighting should be trusted less')
        self.assertGreater(after[0]['confidence'], 0.5,
                           'and still clear min_confidence so it is usable')

    def test_successful_grasp_still_clears_odom_sightings(self):
        entries = sighting(frame='odom', t=T0)
        bm.record_outcome(entries, True, when=T0 + 20)
        self.assertEqual(
            bm.predict(entries, {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
                       now=T0 + 30, frame='odom'), [],
            'odom behaviour must be unchanged: ball is in the jaws now')

    def test_frames_never_mix(self):
        """An odom pose must not accidentally consume a map sighting."""
        map_entries = sighting(frame='map', t=T0)
        odom_entries = sighting(frame='odom', t=T0)
        pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self.assertEqual(bm.predict(map_entries, pose, now=T0 + 10,
                                    frame='odom'), [])
        self.assertEqual(bm.predict(odom_entries, pose, now=T0 + 10,
                                    frame='map'), [])
        mixed = map_entries + odom_entries
        self.assertEqual(len(bm.predict(mixed, pose, now=T0 + 10,
                                        frame='map')), 1)
        self.assertEqual(len(bm.predict(mixed, pose, now=T0 + 10,
                                        frame='odom')), 1)

    def test_legacy_records_without_frame_stay_odom(self):
        """Old ball_history.json entries have no 'frame' key."""
        legacy = [{'t': T0, 'kind': 'sighting', 'x': 0.0, 'y': 0.0,
                   'yaw': 0.0, 'bearing': 0.0, 'range': 0.6}]
        pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self.assertEqual(len(bm.predict(legacy, pose, now=T0 + 10)), 1)
        self.assertEqual(bm.predict(legacy, pose, now=T0 + 3 * DAY), [])
        self.assertEqual(bm.predict(legacy, pose, now=T0 + 10, frame='map'), [])


if __name__ == '__main__':
    unittest.main(verbosity=2)
