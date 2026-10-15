import json
import tempfile
import unittest
import importlib.util
from unittest import mock
from pathlib import Path
from core_profile import load_core_profile


class CoreProfileTests(unittest.TestCase):
    def test_missing_profile_defaults_to_ros2(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).resolve().parent) as d:
            self.assertEqual(load_core_profile(Path(d)/'missing'),{'ros_version':2,'remote_ros_port':9091})

    def test_exact_profile_pairs(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).resolve().parent) as d:
            p=Path(d)/'profile.json'
            for version,port in ((1,9090),(2,9091)):
                p.write_text(json.dumps({'ros_version':version,'remote_ros_port':port}))
                self.assertEqual(load_core_profile(p)['remote_ros_port'],port)

    def test_mixed_or_unrecognized_profile_is_rejected(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).resolve().parent) as d:
            p=Path(d)/'profile.json'
            for data in ({'ros_version':2,'remote_ros_port':9090},{'ros_version':1,'remote_ros_port':9091},{}, {'ros_version':3,'remote_ros_port':9091},{'ros_version':True,'remote_ros_port':9090},{'ros_version':2,'remote_ros_port':'9091'}):
                p.write_text(json.dumps(data))
                with self.assertRaises(ValueError):load_core_profile(p)

class NativeRunnerTests(unittest.TestCase):
    def test_container_wait_does_not_overflow_linux_poll_timeout(self):
        p=Path(__file__).resolve().parents[1]/'car_setup'/'ros2'/'run_native_core.py'
        s=importlib.util.spec_from_file_location('native_core_runner',p)
        m=importlib.util.module_from_spec(s);s.loader.exec_module(m)
        with mock.patch.object(m,'run',return_value='0') as run:
            self.assertEqual(m.wait_container('armpi_ros2_native_test'),0)
            run.assert_called_once_with(['docker','wait','armpi_ros2_native_test'],timeout=None)

if __name__=='__main__':unittest.main()
