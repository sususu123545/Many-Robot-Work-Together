"""Check configured TF math; not a substitute for physical axis tests."""
import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


class MountTransformTests(unittest.TestCase):
    def setUp(self):
        source = Path(__file__).resolve().parents[1] / 'armpi_pro/src/armpi_pro_bringup/launch/start_dependence.launch'
        node = ET.parse(source).getroot().find("node[@name='base_to_imu']")
        args = node.attrib['args'].split()
        self.assertEqual(node.attrib['pkg'], 'tf')
        self.assertEqual(node.attrib['type'], 'static_transform_publisher')
        self.assertEqual(len(args), 10)  # xyz + qxyzw + parent + child + period
        self.assertEqual(args[7:9], ['base_link', 'imu_link'])
        self.assertEqual([float(v) for v in args[:3]], [0, 0, 0])
        self.q = [float(v) for v in args[3:7]]
        self.assertAlmostEqual(sum(v*v for v in self.q), 1)

    def rotate(self, v):
        x, y, z, w = self.q
        return [
            (1-2*(y*y+z*z))*v[0] + 2*(x*y-z*w)*v[1] + 2*(x*z+y*w)*v[2],
            2*(x*y+z*w)*v[0] + (1-2*(x*x+z*z))*v[1] + 2*(y*z-x*w)*v[2],
            2*(x*z-y*w)*v[0] + 2*(y*z+x*w)*v[1] + (1-2*(x*x+y*y))*v[2],
        ]

    def check_vector(self, actual, expected):
        for a, b in zip(actual, expected):
            self.assertAlmostEqual(a, b)

    def test_forward_and_backward(self):
        self.check_vector(self.rotate([0, -1, 0]), [1, 0, 0])
        self.check_vector(self.rotate([0, 1, 0]), [-1, 0, 0])

    def test_vertical_and_gyro_z_preserved(self):
        self.check_vector(self.rotate([0, 0, 9.80665]), [0, 0, 9.80665])
        self.check_vector(self.rotate([0, 0, .1]), [0, 0, .1])

    def test_nominal_yaw_and_implied_left(self):
        self.check_vector(self.rotate([1, 0, 0]), [0, 1, 0])
        self.assertAlmostEqual(2*math.atan2(self.q[2], self.q[3]), math.pi/2)


if __name__ == '__main__':
    unittest.main()
