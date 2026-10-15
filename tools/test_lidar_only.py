"""Offline safety boundaries for isolated sensor recovery."""
import importlib.util
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('lidar_runner', ROOT / 'car_setup/ros2/run_lidar_only.py')
RUNNER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(RUNNER)


class LidarOnlyTest(unittest.TestCase):
    def test_only_lidar_mapped(self):
        args = RUNNER.container_args('test', 'verified', '/opt/test.py')
        self.assertEqual(args.count('--device'), 1)
        self.assertEqual(args[args.index('--device') + 1], '/dev/ld06:/dev/ld06')
        self.assertNotIn('--privileged', args)
        self.assertIn('--rm', args)
        self.assertEqual(args[args.index('--log-driver') + 1], 'journald')
        self.assertEqual(args[args.index('-v') + 1], '/opt/test.py:/opt/armpi_lidar_only.launch.py:ro')

    def test_other_hardware_rejected(self):
        for devices in ([], ['/dev/rrc'], ['/dev/ld06', '/dev/i2c-1']):
            with self.assertRaises(ValueError):
                RUNNER.validate_devices(devices)

    def test_sensor_only_launch(self):
        launch = (ROOT / 'car_setup/ros2/lidar_only.launch.py').read_text()
        self.assertNotIn("executable='arm'", launch)
        self.assertNotIn("executable='chassis'", launch)
        self.assertIn("'address': '127.0.0.1'", launch)
        self.assertIn('target_action=node', launch)

    def test_no_whole_car_restart(self):
        text = (ROOT / 'car_setup/ros2/run_lidar_only.py').read_text()
        self.assertIn("if not stopping:", text)
        self.assertIn('Sensor container exited unexpectedly', text)
        unit = (ROOT / 'car_setup/ros2/armpi-ros2-lidar.service').read_text()
        self.assertIn('Conflicts=armpi-ros2.service', unit)
        self.assertIn('Restart=always', unit)
        self.assertNotIn('[Install]', unit)


if __name__ == '__main__':
    unittest.main()
