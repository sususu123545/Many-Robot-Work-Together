from __future__ import annotations

import unittest

import numpy as np

from g1_minefield_navigation import MinefieldNavigator


class MinefieldNavigatorTests(unittest.TestCase):
    def test_rgbd_obstacle_barrier_produces_a_clear_slalom_path(self):
        nav = MinefieldNavigator(goal_x=4.0)
        obstacle_wall = np.array([[1.4, y] for y in np.arange(-0.45, 0.46, 0.09)])
        self.assertTrue(nav.plan((0.0, 0.0, 0.0), obstacle_wall))
        self.assertGreater(len(nav.path), 4)
        d = np.linalg.norm(nav.path[:, None, :] - obstacle_wall[None, :, :], axis=2)
        self.assertTrue(np.all(np.min(d, axis=1) >= nav.obstacle_clearance - 1e-6))
        self.assertGreater(float(np.max(np.abs(nav.path[:, 1]))), 0.45)

    def test_full_width_obstacle_wall_fails_closed(self):
        nav = MinefieldNavigator(goal_x=4.0)
        barrier = np.array([[1.4, y] for y in np.arange(-2.25, 2.26, 0.06)])
        self.assertFalse(nav.plan((0.0, 0.0, 0.0), barrier))
        self.assertEqual(nav.blocked_plans, 1)
        self.assertEqual(len(nav.path), 0)

    def test_stale_rgbd_and_fall_stop_without_any_manipulation_action(self):
        nav = MinefieldNavigator()
        stale = nav.update(pose=(14.25, 0.0, 0.0), obstacle_points=np.empty((0, 2)),
                           now_s=3.0, sensor_stamp_s=2.0, localization_stamp_s=3.0)
        self.assertEqual((stale.action, stale.phase, stale.reason),
                         ("halt", "SENSOR_STOP", "rgbd_stale"))
        recovered = nav.update(pose=(14.25, 0.0, 0.0), obstacle_points=np.empty((0, 2)),
                               now_s=3.1, sensor_stamp_s=3.1, localization_stamp_s=3.1)
        self.assertEqual(recovered.action, "walk")
        fallen = nav.update(pose=(14.25, 0.0, 0.0), obstacle_points=np.empty((0, 2)),
                            now_s=3.2, sensor_stamp_s=3.2, localization_stamp_s=3.2,
                            fallen=True)
        self.assertEqual((fallen.action, fallen.phase), ("halt", "SAFE_STOP"))
        self.assertEqual(set((stale.action, recovered.action, fallen.action)), {"halt", "walk"})

    def test_sensor_warmup_and_exit_completion(self):
        nav = MinefieldNavigator(goal_x=14.5)
        warmup = nav.update(pose=(14.25, 0.0, 0.0), obstacle_points=np.empty((0, 2)),
                            now_s=1.0, sensor_stamp_s=1.0, localization_stamp_s=1.0)
        self.assertEqual((warmup.phase, warmup.action), ("SENSOR_WARMUP", "halt"))
        done = nav.update(pose=(14.5, 0.0, 0.0), obstacle_points=np.empty((0, 2)),
                          now_s=3.0, sensor_stamp_s=3.0, localization_stamp_s=3.0)
        self.assertEqual((done.phase, done.reason), ("COMPLETE", "minefield_exit_reached"))

    def test_obstacle_already_passed_does_not_trigger_forward_clearance_stop(self):
        nav = MinefieldNavigator(goal_x=8.0)
        result = nav.update(pose=(5.0, 0.0, 0.0),
                            obstacle_points=np.array([[4.6, 0.0]]),
                            now_s=3.0, sensor_stamp_s=3.0, localization_stamp_s=3.0,
                            warmup_s=0.0)
        self.assertEqual(result.action, "walk")
        self.assertNotEqual(result.phase, "CLEARANCE_STOP")

    def test_front_obstacle_inside_clearance_still_stops(self):
        nav = MinefieldNavigator(goal_x=8.0)
        result = nav.update(pose=(5.0, 0.0, 0.0),
                            obstacle_points=np.array([[5.3, 0.0]]),
                            now_s=3.0, sensor_stamp_s=3.0, localization_stamp_s=3.0,
                            warmup_s=0.0)
        self.assertEqual((result.action, result.phase), ("halt", "CLEARANCE_STOP"))
        self.assertAlmostEqual(result.clearance_m, 0.3)

    def test_path_tracking_uses_official_policy_lateral_velocity(self):
        nav = MinefieldNavigator(goal_x=4.0)
        nav.path = np.array([[0.0, 0.0], [0.65, 0.4], [1.3, 0.8]])
        nav.last_plan_s = 3.0
        result = nav.update(pose=(0.0, 0.0, 0.0), obstacle_points=np.empty((0, 2)),
                            now_s=3.1, sensor_stamp_s=3.1, localization_stamp_s=3.1,
                            warmup_s=0.0, dt_s=0.1)
        self.assertEqual(result.action, "walk")
        self.assertGreater(result.lateral_mps, 0.0)


if __name__ == "__main__":
    unittest.main()
