import math
import unittest
from types import SimpleNamespace
from armpi_ros2.bmi270_transport import read_sample


class BmiTransportTests(unittest.TestCase):
    def test_si_units_and_signed_samples(self):
        # +1g X, -1g Y, zero Z; +500deg/s X, -500deg/s Y, zero Z.
        data = [0, 64, 0, 192, 0, 0] * 2
        calls = []
        def read(address, register, count):
            calls.append((address, register, count))
            return data
        sample = SimpleNamespace(address=0x68, bus=SimpleNamespace(read_i2c_block_data=read))
        accel, gyro = read_sample(sample)
        self.assertEqual(calls, [(0x68, 0x0c, 12)])
        self.assertAlmostEqual(accel[0], 9.80665)
        self.assertAlmostEqual(accel[1], -9.80665)
        self.assertAlmostEqual(gyro[0], math.radians(500))
        self.assertAlmostEqual(gyro[1], -math.radians(500))

    def test_incomplete_transfer_rejected(self):
        sample = SimpleNamespace(address=0x68, bus=SimpleNamespace(read_i2c_block_data=lambda *_: [0] * 8))
        with self.assertRaises(OSError):
            read_sample(sample)


if __name__ == '__main__':
    unittest.main()
