"""Launch the VLA chunk executor.

Deliberately does not touch motion permissions or the arm's safety latch. Start
this alongside the existing core, then grant motion from the console (or with
``ros2 param set /arm_controller enable_arm_motion true``) once the executor
reports ``arm_ready: true``.

Per-run overrides go through ROS arguments rather than more launch arguments::

    ros2 run vla_controller chunk_executor --ros-args \\
        -p control_dt:=0.5 -p allow_projection:=true

or, keeping the YAML::

    ros2 launch vla_controller vla_controller.launch.py params_file:=/path/to.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('vla_controller')
    params = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(share, 'config', 'vla_controller.yaml'),
        description='YAML file with vla_chunk_executor parameters')

    executor = Node(
        package='vla_controller',
        executable='chunk_executor',
        name='vla_chunk_executor',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        emulate_tty=True,
    )

    return LaunchDescription([params, executor])
