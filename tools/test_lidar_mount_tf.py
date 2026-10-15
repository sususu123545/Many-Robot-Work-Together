"""Verify measured translation defaults without importing ROS launch modules."""
import ast
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]


class LidarMountTests(unittest.TestCase):
    def test_diff_preserves_missing_eof_newline(self):
        from apply_lidar_translation import exact_patch
        patch = exact_patch('a\nb\nend', 'a\nc\nend')
        self.assertTrue(patch.endswith('\n'))
        self.assertIn(' end\n\\ No newline at end of file\n', patch)
        self.assertIn('-b\n+c\n', patch)

    def test_ros1_translation_and_unchanged_orientation(self):
        node = ET.parse(ROOT / 'car_setup/ld06.launch').getroot().find("node[@name='base_to_laser']")
        args = node.attrib['args'].split()
        self.assertEqual(args[6:8], ['base_link', 'base_laser'])
        self.assertEqual([float(x) for x in args[:6]], [-.01, .01, .18, 0, 0, 0])
        self.assertEqual(len(args), 9)
        self.assertEqual(node.get('required', 'false'), 'false')
        self.assertEqual(node.get('respawn', 'false'), 'false')

    def test_ros2_translation_matches_ros1(self):
        tree = ast.parse((ROOT / 'ros2_ws/src/armpi_ros2/launch/base.launch.py').read_text(encoding='utf-8'))
        candidates = [n for n in ast.walk(tree) if isinstance(n, ast.Dict)
                      and any(isinstance(k, ast.Constant) and k.value == 'lidar_x' for k in n.keys)]
        self.assertEqual(len(candidates), 1)
        cfg = ast.literal_eval(candidates[0])
        self.assertEqual([float(cfg[k]) for k in ('lidar_x', 'lidar_y', 'lidar_z', 'lidar_yaw')], [-.01, .01, .18, 0])

    def test_driver_cropping_stays_disabled(self):
        root = ET.parse(ROOT / 'car_setup/ld06.launch').getroot()
        crop = root.find("node[@name='LD06']/param[@name='enable_angle_crop_func']")
        self.assertEqual(crop.get('value'), 'false')


if __name__ == '__main__':
    unittest.main()
