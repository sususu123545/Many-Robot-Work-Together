"""Bring up occupancy-map localisation: map_server + amcl + lifecycle manager.

Why a launch file instead of three `ros2 run` calls
---------------------------------------------------
Both map_server and amcl are lifecycle nodes: they come up `unconfigured` and
do nothing until they are configured and activated.  Forgetting that step is
exactly the mistake that made the first mapping run look like a dead mapper
(the node was running, the topic was empty).  nav2_lifecycle_manager does the
configure/activate dance, and keeps a bond on both nodes so a crash is visible.

Run INSIDE the core container (shares DDS domain and the TF tree):

  source /opt/ros/jazzy/setup.bash
  source /opt/armpi_ros2/install/setup.bash
  ros2 launch /path/to/nav2_localization.launch.py

Verify:
  ros2 lifecycle get /amcl          -> active [3]
  ros2 topic echo /amcl_pose --once
  ros2 run tf2_ros tf2_echo map base_link
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            description='Absolute path to nav2_localization_params.yaml'),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            parameters=[params_file],
        ),
    ])
