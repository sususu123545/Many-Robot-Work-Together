"""Launch the liveness supervisor.

Publishes a latched verdict on /vla/safety_estop. It is NOT wired into the
hardware stop by default; see config/robot_safety.yaml and the package README
before changing that.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('robot_safety')
    params = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(share, 'config', 'robot_safety.yaml'),
        description='YAML file with vla_safety_supervisor parameters')

    supervisor = Node(
        package='robot_safety',
        executable='supervisor',
        name='vla_safety_supervisor',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        emulate_tty=True,
    )

    return LaunchDescription([params, supervisor])
