"""Start passive eye-in-hand red-ball perception; this launch never moves joints."""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    params = os.path.join(get_package_share_directory('armpi_ros2'),
                          'config', 'grasp_vision.yaml')
    transform_params = os.path.join(get_package_share_directory('armpi_ros2'),
                                    'config', 'grasp_transform.yaml')
    camera_tf_params = os.path.join(get_package_share_directory('armpi_ros2'),
                                    'config', 'arm_camera_tf.yaml')
    detector = Node(package='armpi_ros2', executable='red_ball_detector',
                    name='red_ball_detector', output='screen', parameters=[params])
    target_transform = Node(package='armpi_ros2', executable='grasp_target_transform',
                            name='grasp_target_transform', output='screen',
                            parameters=[transform_params])
    camera_tf = Node(package='armpi_ros2', executable='arm_camera_transform',
                     name='arm_camera_transform', output='screen',
                     parameters=[camera_tf_params])
    return LaunchDescription([detector, target_transform, camera_tf])
