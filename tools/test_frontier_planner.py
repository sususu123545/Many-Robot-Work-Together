"""Unit tests for conservative OccupancyGrid frontier selection."""
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

SOURCE = Path(__file__).resolve().parents[1] / 'car_setup'
sys.path.insert(0, str(SOURCE))
from frontier_planner import select_frontier_goal


def make_grid(width=100, height=100, resolution=0.05):
    origin = SimpleNamespace(
        position=SimpleNamespace(x=-2.5, y=-2.5),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))
    info = SimpleNamespace(width=width, height=height,
                           resolution=resolution, origin=origin)
    return SimpleNamespace(info=info, data=[0] * (width * height))


class FrontierPlannerTests(unittest.TestCase):
    def setUp(self):
        self.grid = make_grid()
        # Unknown area east of the robot; all other map cells are known free.
        for y in range(35, 66):
            for x in range(72, 91):
                self.grid.data[y * 100 + x] = -1

    def test_selects_known_free_approach_to_unknown_area(self):
        goal = select_frontier_goal(
            self.grid, (0.0, 0.0, 0.0), clearance_radius=0.10,
            min_range=0.40, max_range=2.0)
        self.assertIsNotNone(goal)
        self.assertGreater(goal['x'], 0.45)
        self.assertLess(abs(goal['y']), 0.40)
        self.assertGreater(goal['gain_cells'], 0)

    def test_does_not_choose_frontier_behind_occupied_wall(self):
        for y in range(100):
            self.grid.data[y * 100 + 62] = 100
        goal = select_frontier_goal(
            self.grid, (0.0, 0.0, 0.0), clearance_radius=0.10,
            min_range=0.40, max_range=2.0)
        self.assertIsNone(goal)

    def test_blacklisted_failed_goal_is_skipped(self):
        first = select_frontier_goal(
            self.grid, (0.0, 0.0, 0.0), clearance_radius=0.10,
            min_range=0.40, max_range=2.0)
        self.assertIsNotNone(first)
        second = select_frontier_goal(
            self.grid, (0.0, 0.0, 0.0), clearance_radius=0.10,
            min_range=0.40, max_range=2.0,
            blacklist=[(first['frontier_x'], first['frontier_y'])])
        self.assertIsNone(second)

    def test_reverse_mode_prefers_goal_behind_the_robot_when_gain_matches(self):
        for y in range(35, 66):
            for x in range(72, 91):
                self.grid.data[y * 100 + x] = 0
                self.grid.data[y * 100 + (99 - x)] = -1
        goal = select_frontier_goal(
            self.grid, (0.0, 0.0, 0.0), travel_direction=-1,
            clearance_radius=0.10, min_range=0.40, max_range=2.0)
        self.assertIsNotNone(goal)
        self.assertLess(goal['x'], -0.45)


if __name__ == '__main__':
    unittest.main()
