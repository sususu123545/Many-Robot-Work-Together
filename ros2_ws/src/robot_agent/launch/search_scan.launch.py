"""Launch the search sweep.

Does not enable chassis motion. Grant it first::

    ros2 param set /chassis_control enable_motion true

then start the sweep::

    ros2 service call /vla/search/start std_srvs/srv/Trigger
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('robot_agent')
    params = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(share, 'config', 'robot_agent.yaml'),
        description='YAML file with vla_search_scan parameters')

    node = Node(
        package='robot_agent',
        executable='search_scan',
        name='vla_search_scan',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        emulate_tty=True,
    )

    return LaunchDescription([params, node])
