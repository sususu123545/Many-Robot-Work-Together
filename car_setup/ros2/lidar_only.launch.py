"""Sensor-only recovery: no chassis, arm, IMU or motion device access."""
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    lidar = Node(package='ldlidar_stl_ros2', executable='ldlidar_stl_ros2_node',
                 name='LD06', output='screen', parameters=[{
                     'product_name': 'LDLiDAR_LD06', 'topic_name': '/scan',
                     'frame_id': 'base_laser', 'port_name': '/dev/ld06',
                     'port_baudrate': 230400, 'laser_scan_dir': True,
                     'enable_angle_crop_func': False,
                     'angle_crop_min': 135.0, 'angle_crop_max': 225.0}])
    bridge = Node(package='rosbridge_server', executable='rosbridge_websocket',
                  name='rosbridge_websocket', output='screen',
                  parameters=[{'port': 9091, 'address': '127.0.0.1'}])
    # Restart only this sensor container, including renewed USB device mappings.
    # Never respawn the whole-car launcher or authorize motion on sensor return.
    guards = [RegisterEventHandler(OnProcessExit(target_action=node,
              on_exit=[EmitEvent(event=Shutdown(reason='Sensor-only node exited'))]))
              for node in (lidar, bridge)]
    return LaunchDescription(guards + [lidar, bridge])
