"""Launch the VLA head: observation builder + policy bridge (action client).

The executor is NOT launched here -- it belongs to ``vla_controller`` and must
already be running (``ros2 launch vla_controller vla_controller.launch.py``).
Nothing in this file grants arm motion permission; the operator still has to
enable it from the console before any goal can reach the hardware.

Bench bring-up, after the executor reports arm_ready::

    ros2 launch vla_ros2_bridge vla_bridge.launch.py
    ros2 topic pub --once /vla/instruction std_msgs/msg/String \
        "{data: 'pick up the red ball'}"

The bridge runs with one_shot:=true, so exactly one chunk is sent.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('vla_ros2_bridge')
    params = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(share, 'config', 'vla_bridge.yaml'),
        description='YAML file with vla_observation_node / vla_bridge_node parameters')
    parameters = [LaunchConfiguration('params_file')]

    observation = Node(
        package='vla_ros2_bridge',
        executable='vla_observation_node',
        name='vla_observation_node',
        output='screen',
        parameters=parameters,
        emulate_tty=True,
    )
    bridge = Node(
        package='vla_ros2_bridge',
        executable='vla_bridge_node',
        name='vla_bridge_node',
        output='screen',
        parameters=parameters,
        emulate_tty=True,
    )
    return LaunchDescription([params, observation, bridge])
