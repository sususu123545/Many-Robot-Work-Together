"""Run the read-only eye-in-hand checkerboard capture/solve services."""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    params = os.path.join(get_package_share_directory('armpi_ros2'),
                          'config', 'hand_eye_calibration.yaml')
    calibration = Node(package='armpi_ros2', executable='hand_eye_calibration',
                       name='arm_camera_hand_eye_calibration', output='screen',
                       parameters=[params])
    return LaunchDescription([calibration])
