"""ROS 2 launch for slam_toolbox on the ArmPi Pro.

Replaces the ROS 1 XML launch car_setup/ld06_slam_toolbox.launch, which cannot
run under the ROS 2 Jazzy core (the whole car is ROS 2 only).

Run INSIDE the core container (so it shares the DDS domain and TF tree):

  source /opt/armpi_ros2/install/setup.bash
  ros2 launch /path/to/slam_toolbox.launch.py \
      params_file:=/path/to/slam_toolbox_mapping.yaml \
      executable:=async_slam_toolbox_node

For localisation on a saved map:

  ros2 launch /path/to/slam_toolbox.launch.py \
      params_file:=/path/to/slam_toolbox_localization.yaml \
      executable:=localization_slam_toolbox_node

Both executables were verified present via `ros2 pkg executables slam_toolbox`.
Only ONE of these may run at a time: both publish map->odom and would fight.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    executable = LaunchConfiguration('executable')
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            description='Absolute path to the slam_toolbox ROS 2 parameter YAML'),
        DeclareLaunchArgument(
            'executable', default_value='async_slam_toolbox_node',
            description='slam_toolbox executable: async_slam_toolbox_node for '
                        'mapping, localization_slam_toolbox_node for a saved map'),
        DeclareLaunchArgument(
            'max_laser_range', default_value='8.0',
            description='Clip /scan beyond this many metres.  Set 2.5-3.0 to '
                        'map ONLY a small patch around the car instead of the '
                        'whole corridor.  Anything past this never enters the '
                        'occupancy grid.'),
        Node(
            package='slam_toolbox',
            executable=executable,
            name='slam_toolbox',
            output='screen',
            parameters=[params_file,
                        {'max_laser_range': ParameterValue(
                            LaunchConfiguration('max_laser_range'),
                            value_type=float)}],
        ),
    ])
