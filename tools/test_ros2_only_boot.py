import ast
import urllib.parse
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
source = (ROOT/'car_setup/ros2/lidar_preflight.py').read_text()
# termios is Unix-only; test the pure packet decoder on Windows too.
tree = ast.parse(source)
pure = ast.Module(body=[n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name in ('crc8','packets')], type_ignores=[])
scope = {}
exec(compile(pure, 'lidar_preflight.py', 'exec'), scope)


class BootTests(unittest.TestCase):
    def frame(self):
        body = b'\x54\x2c' + bytes(range(44))
        return body + bytes([scope['crc8'](body)])

    def test_split_frame_and_noise(self):
        frame=self.frame();buffer=bytearray(b'noise'+frame[:20])
        self.assertEqual(scope['packets'](buffer),0)
        buffer.extend(frame[20:]+frame)
        self.assertEqual(scope['packets'](buffer),2)

    def test_corrupt_frames_never_ready(self):
        frame=bytearray(self.frame());frame[-1]^=1
        buffer=bytearray(frame*10)
        self.assertEqual(scope['packets'](buffer),0)

    def test_unexpected_clean_exit_also_recovers(self):
        text=(ROOT/'car_setup/ros2/run_native_core.py').read_text()
        self.assertIn('if not stopping:\n            raise RuntimeError',text)
        self.assertNotIn('if not stopping and result:',text)

    def test_ros1_no_dependency_and_conflicts(self):
        text=(ROOT/'car_setup/ros2/armpi-ros2.service').read_text()
        self.assertIn('After=docker.service\n',text)
        self.assertIn('Conflicts=start_node.service',text)
        self.assertIn('Restart=on-failure',text)

    def test_camera_is_ros2_native_and_no_motion_driver_changes(self):
        text=(ROOT/'ros2_ws/Dockerfile.native_only').read_text()
        self.assertIn('ros-jazzy-v4l2-camera',text)
        self.assertIn('ros-jazzy-web-video-server',text)
        self.assertNotIn('COPY src/armpi_ros2 src/armpi_ros2',text)
        self.assertIn('WaitLidarCommConnect(10000)',text)

    def test_ros2_video_uses_sensor_qos(self):
        from types import SimpleNamespace
        tree=ast.parse((ROOT/'web_console/proxy_server(1).py').read_text(encoding='utf-8'))
        handler=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Handler')
        method=next(n for n in handler.body if isinstance(n,ast.FunctionDef) and n.name=='_car_url')
        scope={'urllib':__import__('urllib'),'load_core_profile':lambda:{'ros_version':2}}
        exec(compile(ast.Module(body=[method],type_ignores=[]),'proxy','exec'),scope)
        url=scope['_car_url'](SimpleNamespace(path='/snapshot?ip=192.168.55.178'),'/snapshot')
        self.assertIn('qos_profile=sensor_data',url)

    def test_live_acceptance_is_readonly_and_reports_new_arm_faults(self):
        import verify_ros2_only_live
        text=verify_ros2_only_live.WATCH
        self.assertNotIn('create_publisher',text)
        self.assertNotIn('create_client',text)
        self.assertIn('new arm fault during observation',text)
        self.assertIn('historical fault remains latched',text)
        self.assertIn('assert not failures',text)


if __name__=='__main__':unittest.main()
