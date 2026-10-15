import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    params = os.path.join(get_package_share_directory('armpi_ros2'), 'config', 'slam.yaml')
    slam = os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
    return LaunchDescription([IncludeLaunchDescription(PythonLaunchDescriptionSource(slam),
        launch_arguments={'slam_params_file': params, 'use_sim_time': 'false'}.items())])
