"""Exercise obstacle decisions without ROS, sensors or motor publishers."""
import importlib.util
import math
import sys
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock, patch


SOURCE = Path(__file__).resolve().parents[1] / 'car_setup' / 'wall_follow.py'
sys.path.insert(0, str(SOURCE.parent))
ROS_MODULES = ('rospy', 'tf2_ros', 'sensor_msgs', 'sensor_msgs.msg',
               'nav_msgs', 'nav_msgs.msg', 'std_msgs', 'std_msgs.msg',
               'tf', 'tf.transformations', 'chassis_control', 'chassis_control.msg')
with patch.dict('sys.modules', {name: MagicMock() for name in ROS_MODULES}):
    spec = importlib.util.spec_from_file_location('wall_follow_under_test', SOURCE)
    wall = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(wall)


class ObstacleDecisions(unittest.TestCase):
    def setUp(self):
        wall.P.travel_direction = 1
        # Skip __init__: it connects ROS subscribers and publishers on a real car.
        self.car = wall.WallFollower.__new__(wall.WallFollower)
        self.car.state = 'FOLLOW'
        self.car.t0 = time.time()
        self.car.estop_hits = self.car.obstacle_hits = 0
        self.car.obstacle_since = self.car.obstacle_clear_since = None
        self.car.obstacle_kind = 'front'
        self.car.turn_sign = 1
        self.car.last_pose = None
        self.car.last_pose_check = None
        self.car.start_pose = None
        self.car.path_len = 0.0
        self.car.stuck_count = 0
        self.car.lost_since = None
        self.car.area_anchor = None
        self.car.area_since = time.time()
        self.car.escape_sign = 0
        self.car.escape_t0 = 0.0
        self.car.escape_move_pose = None
        self.car.escape_move_t0 = 0.0
        self.car.escape_pose_src = 'odom'
        self.car.pose_src = 'odom'
        self.car.resume_state = 'FOLLOW'
        self.car.frontier_goal = None
        self.car.frontier_blacklist = []
        self.car.goal_reached_count = 0
        self.car.lock = threading.Lock()
        self.car.map_grid = None
        self.car.map_stamp = 0.0
        self.car.pose = lambda: None
        self.commands = []
        self.car.stop_motors = lambda: self.commands.append(('stop',))
        self.car.send_translation = lambda x, y: self.commands.append(('move', x, y))
        self.car.send_rotate = lambda a: self.commands.append(('turn', a))
        self.sec = dict(front=0.60, side=0.55, front_side=0.60,
                        counts={'front': 10},
                        left=0.60, right=0.55, front_left=0.60,
                        front_right=0.55, raw_estop=0.60,
                        raw_min={'side': 0.55, 'left': 0.60, 'right': 0.55,
                                 'front_left': 0.60, 'front_right': 0.55})
        self.car.sectors = lambda: self.sec

    def test_sixty_cm_no_longer_pauses_forward_travel(self):
        self.car.step()
        self.assertEqual(self.car.state, 'FOLLOW')
        self.assertEqual(self.commands[-1][0], 'move')
        self.assertGreater(self.commands[-1][2], 0)

    def test_thirty_nine_cm_stops_then_waits(self):
        self.sec['front'] = 0.39
        self.car.step()
        self.assertEqual(self.commands, [('stop',)])
        self.car.step()
        self.assertEqual(self.car.state, 'WAIT_OBSTACLE')
        self.assertTrue(all(c[0] == 'stop' for c in self.commands))

    def test_sudden_near_obstacle_cannot_skip_wait_and_turn(self):
        self.sec['front'] = 0.39
        self.car.step()
        self.car.step()
        self.assertEqual(self.car.state, 'WAIT_OBSTACLE')
        self.assertTrue(all(c[0] == 'stop' for c in self.commands))

    def test_only_qualified_turn_side_can_win_tie_break(self):
        self.sec.update(left=0.41, front_left=0.41, right=0.39, front_right=0.39)
        with patch.object(wall.P, 'wall_side', 1):
            self.assertEqual(self.car.choose_turn_sign(self.sec), 1)
        self.sec.update(left=0.39, front_left=0.39, right=0.41, front_right=0.41)
        with patch.object(wall.P, 'wall_side', -1):
            self.assertEqual(self.car.choose_turn_sign(self.sec), -1)

    def test_blocked_both_sides_keeps_waiting(self):
        self.car.state = 'WAIT_OBSTACLE'
        self.car.obstacle_since = time.time() - 5
        self.sec.update(front=0.39, left=0.39, right=0.39)
        self.car.step()
        self.assertEqual(self.car.state, 'WAIT_OBSTACLE')
        self.assertEqual(self.commands[-1], ('stop',))

    def test_half_metre_open_side_permits_turn_after_wait(self):
        self.car.state = 'WAIT_OBSTACLE'
        self.car.obstacle_since = time.time() - 5
        self.sec.update(front=0.39, left=0.41, right=0.39)
        self.car.step()
        self.assertEqual(self.car.state, 'TURN')
        self.assertEqual(self.car.turn_sign, 1)
        self.assertEqual(self.commands[-1], ('stop',))

    def test_recovery_needs_distance_and_stable_clear_time(self):
        self.car.state = 'WAIT_OBSTACLE'
        self.car.obstacle_since = time.time()
        self.car.obstacle_clear_since = time.time() - 2
        self.sec['front'] = 0.49
        self.car.step()
        self.assertEqual(self.car.state, 'WAIT_OBSTACLE')
        self.assertIsNone(self.car.obstacle_clear_since)
        self.sec['front'] = 0.51
        self.car.step()
        self.assertEqual(self.car.state, 'WAIT_OBSTACLE')
        self.car.obstacle_clear_since = time.time() - 2
        self.car.step()
        self.assertEqual(self.car.state, 'FOLLOW')

    def test_turning_still_stops_at_32cm(self):
        self.car.state = 'TURN'
        self.sec['left'] = 0.31
        self.car.step()
        self.assertEqual(self.car.state, 'WAIT_OBSTACLE')
        self.assertEqual(self.commands, [('stop',)])

    def test_turn_does_not_finish_while_raw_front_is_still_blocked(self):
        self.car.state = 'TURN'
        self.car.turn_sign = 1
        self.sec.update(front=0.61, raw_estop=0.28,
                        left=0.80, front_left=0.80,
                        raw_min={'side': 0.80, 'left': 0.80,
                                 'front_left': 0.80,
                                 'right': 0.80, 'front_right': 0.80})
        self.car.step()
        self.assertEqual(self.car.state, 'TURN')
        self.assertEqual(self.commands[-1][0], 'turn')

    def test_raw_front_hard_stop_overrides_filtered_clearance(self):
        self.sec.update(front=0.90, raw_estop=0.29)
        self.car.step()
        self.assertEqual(self.car.state, 'FOLLOW')
        self.assertEqual(self.commands, [('stop',)])
        self.car.step()
        self.assertEqual(self.car.state, 'WAIT_OBSTACLE')

    def test_persistent_right_front_obstacle_turns_left(self):
        self.sec.update(front=0.90, raw_estop=0.29,
                        left=0.60, front_left=0.60,
                        right=0.60, front_right=0.29,
                        raw_min={'side': 0.60, 'left': 0.60,
                                 'right': 0.60, 'front_left': 0.60,
                                 'front_right': 0.29})
        self.car.step()
        self.car.step()
        self.assertEqual(self.car.state, 'WAIT_OBSTACLE')
        self.car.obstacle_since = time.time() - 3.0
        self.car.step()
        self.assertEqual(self.car.state, 'TURN')
        self.assertEqual(self.car.turn_sign, 1)

    def test_turning_stops_when_near_point_is_on_turn_path(self):
        self.car.state = 'TURN'
        self.car.turn_sign = 1
        self.sec.update(front=0.90, front_left=0.29,
                        raw_min={'side': 0.60, 'left': 0.29,
                                 'right': 0.60, 'front_left': 0.29,
                                 'front_right': 0.60})
        self.car.step()
        self.assertEqual(self.car.state, 'WAIT_OBSTACLE')
        self.assertEqual(self.commands, [('stop',)])

    def test_scan_loss_still_stops(self):
        self.car.sectors = lambda: None
        self.car.step()
        self.assertEqual(self.commands, [('stop',)])

    def test_reverse_sectors_rotate_all_safety_fields(self):
        defs = self.car.sector_definitions(-1, 1)
        def contains(key, degrees):
            angle = wall.norm_deg(degrees)
            return any(lo <= angle <= hi for lo, hi in defs[key])
        self.assertTrue(contains('front', 179))
        self.assertTrue(contains('estop', -170))
        self.assertTrue(contains('front_right', 135))
        self.assertTrue(contains('side', 90))
        self.assertFalse(contains('front', 0))

    def test_reverse_uses_rear_obstacle_for_hard_stop(self):
        wall.P.travel_direction = -1
        self.sec['raw_estop'] = 0.29
        self.car.step()
        self.assertEqual(self.commands, [('stop',)])

    def test_reverse_cruise_commands_negative_y(self):
        wall.P.travel_direction = -1
        self.car.step()
        self.assertEqual(self.commands[-1][0], 'move')
        self.assertLess(self.commands[-1][2], 0)

    def test_reverse_turn_changes_physical_rotation_sign(self):
        wall.P.travel_direction = -1
        self.car.state = 'TURN'
        self.car.turn_sign = 1
        self.sec['front'] = 0.42
        self.car.step()
        self.assertEqual(self.commands[-1], ('turn', -wall.P.turn_rate))

    def test_reverse_without_rear_scan_stops(self):
        wall.P.travel_direction = -1
        self.sec['counts'] = {'front': 0}
        self.car.step()
        self.assertEqual(self.commands, [('stop',)])

    def test_real_sector_extraction_checks_rear_not_nose(self):
        wall.P.travel_direction = -1
        self.car.lock = threading.Lock()
        self.car.laser_yaw_offset = lambda: 0.0
        self.car.scan = SimpleNamespace(
            angle_min=-math.pi, angle_increment=math.pi / 180.0,
            ranges=[2.0] * 360)
        self.car.raw_ranges = [2.0] * 360
        self.car.filtered_ranges = [2.0] * 360
        self.car.raw_ranges[359] = self.car.filtered_ranges[359] = 0.28
        self.car.raw_ranges[180] = self.car.filtered_ranges[180] = 0.15
        self.car.scan_stamp = time.time()
        sec = wall.WallFollower.sectors(self.car)
        self.assertAlmostEqual(sec['raw_estop'], 0.28)
        self.assertEqual(sec['counts']['front'] > 0, True)
        self.assertAlmostEqual(sec['raw_rear'], 0.15)

    def test_reverse_recovery_does_not_turn_into_blocked_path(self):
        wall.P.travel_direction = -1
        self.car.state = 'RECOVER'
        self.car.recover_t0 = time.time() - 2.5
        self.sec['front_right'] = 0.25
        with patch.object(wall.P, 'wall_side', -1):
            self.car.step()
        self.assertEqual(self.car.state, 'WAIT_OBSTACLE')
        self.assertTrue(all(cmd[0] == 'stop' for cmd in self.commands))

    def test_effective_motion_resets_accumulated_stuck_count(self):
        self.car.stuck_count = 2
        self.car.last_pose = (0.0, 0.0, 0.0)
        self.car.last_pose_check = ((0.0, 0.0, 0.0),
                                    time.time() - wall.P.stuck_secs - 1.0)
        self.car.pose = lambda: (wall.P.stuck_reset_dist + 0.01, 0.0, 0.0)
        self.car.step()
        self.assertEqual(self.car.state, 'FOLLOW')
        self.assertEqual(self.car.stuck_count, 0)

    def test_open_travel_direction_does_not_spin_for_side_wall_error(self):
        wall.P.travel_direction = -1
        self.sec['front'] = 0.90
        self.sec['side'] = 0.32
        self.car.step()
        self.assertEqual(self.car.state, 'FOLLOW')
        self.assertEqual(self.commands[-1][0], 'move')
        self.assertLess(self.commands[-1][2], 0)

    def test_area_dwell_starts_escape_toward_open_direction(self):
        wall.P.travel_direction = -1
        self.car.area_anchor = (0.0, 0.0, 0.0)
        self.car.area_since = time.time() - wall.P.area_dwell_secs - 1.0
        self.car.pose = lambda: (0.10, 0.05, 0.0)
        self.sec['front'] = 0.90
        self.car.step()
        self.assertEqual(self.car.state, 'ESCAPE')
        self.car.escape_t0 = time.time() - wall.P.area_escape_turn_secs - 0.1
        self.car.step()
        self.assertEqual(self.commands[-1][0], 'move')
        self.assertLess(self.commands[-1][2], 0)

    def test_area_dwell_does_not_override_hard_stop(self):
        self.car.area_anchor = (0.0, 0.0, 0.0)
        self.car.area_since = time.time() - wall.P.area_dwell_secs - 1.0
        self.car.pose = lambda: (0.0, 0.0, 0.0)
        self.sec.update(front=0.90, raw_estop=0.29)
        self.car.step()
        self.assertEqual(self.car.state, 'FOLLOW')
        self.assertEqual(self.commands, [('stop',)])

    def test_escape_requires_measured_progress_not_elapsed_duration(self):
        self.car.state = 'ESCAPE'
        self.car.escape_t0 = time.time() - 12.0
        self.car.escape_move_t0 = time.time() - 11.0
        self.car.escape_move_pose = (0.0, 0.0, 0.0)
        self.car.pose = lambda: (0.10, 0.0, 0.0)
        with patch.object(wall.P, 'area_escape_timeout', 35.0):
            self.car.step_area_escape(self.sec)
        self.assertEqual(self.car.state, 'ESCAPE')
        self.assertEqual(self.commands[-1][0], 'move')

    def test_escape_timeout_without_progress_pauses(self):
        self.car.state = 'ESCAPE'
        self.car.escape_t0 = time.time() - 36.0
        self.car.escape_move_t0 = time.time() - 36.0
        self.car.escape_move_pose = (0.0, 0.0, 0.0)
        self.car.pose = lambda: (0.10, 0.0, 0.0)
        self.car.step_area_escape(self.sec)
        self.assertEqual(self.car.state, 'PAUSED_RECOVERY')
        self.assertEqual(self.commands[-1], ('stop',))

    def test_obstacle_clear_does_not_claim_escape_complete(self):
        self.car.state = 'WAIT_OBSTACLE'
        self.car.resume_state = 'ESCAPE'
        self.car.obstacle_since = time.time() - 3.0
        self.car.obstacle_clear_since = time.time() - 2.0
        self.sec.update(front=0.70, side=0.70, raw_estop=0.70,
                        raw_min={'side': 0.70})
        self.car.step_obstacle_wait(self.sec)
        self.assertEqual(self.car.state, 'ESCAPE')
        self.assertIn('确认脱离', self.car.msg)

    def test_frontier_goal_commands_reverse_only_after_heading_aligned(self):
        wall.P.travel_direction = -1
        self.car.map_grid = SimpleNamespace(
            header=SimpleNamespace(frame_id='map'))
        self.car.map_stamp = time.time()
        self.car.map_pose = lambda frame='map': (0.0, 0.0, 0.0)
        self.car.frontier_goal = {'x': -1.0, 'y': 0.0, 'gain_cells': 20}
        self.car.step_frontier(self.sec)
        self.assertEqual(self.commands[-1][0], 'move')
        self.assertLess(self.commands[-1][2], 0.0)

    def test_frontier_goal_stops_when_turn_path_is_blocked(self):
        self.car.map_grid = SimpleNamespace(
            header=SimpleNamespace(frame_id='map'))
        self.car.map_stamp = time.time()
        self.car.map_pose = lambda frame='map': (0.0, 0.0, 0.0)
        self.car.frontier_goal = {'x': 0.0, 'y': 1.0, 'gain_cells': 20}
        self.sec.update(left=0.20, front_left=0.20,
                        raw_min={'side': 0.55, 'left': 0.20,
                                 'front_left': 0.20, 'right': 0.55,
                                 'front_right': 0.55})
        self.car.step_frontier(self.sec)
        self.assertEqual(self.car.state, 'WAIT_OBSTACLE')
        self.assertEqual(self.commands[-1], ('stop',))

    def test_frontier_no_progress_triggers_bounded_recovery(self):
        self.car.state = 'EXPLORE'
        self.car.frontier_goal = {'x': 1.0, 'y': 0.0,
                                  'frontier_x': 1.2, 'frontier_y': 0.0,
                                  'gain_cells': 10}
        self.car.last_pose = (0.0, 0.0, 0.0)
        self.car.pose = lambda: (0.0, 0.0, 0.0)
        self.car.last_pose_check = ((0.0, 0.0, 0.0),
                                    time.time() - wall.P.stuck_secs - 1.0)
        self.car.step()
        self.assertEqual(self.car.state, 'RECOVER')
        self.assertIsNone(self.car.frontier_goal)
        self.assertTrue(self.car.frontier_blacklist)


if __name__ == '__main__':
    unittest.main()
