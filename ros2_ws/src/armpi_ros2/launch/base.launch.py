import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.conditions import IfCondition
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('armpi_ros2'), 'config', 'base.yaml')
    arguments = [DeclareLaunchArgument(k, default_value=v) for k, v in {
        'dry_run': 'true', 'enable_motion': 'false', 'lidar': 'false',
        'imu': 'false', 'arm': 'false', 'enable_arm_motion': 'false',
        'enable_gripper_motion': 'false',
        'arm_port': '/dev/rrc', 'rosbridge': 'true', 'rosbridge_port': '9091',
        # Rear camera is a plain UVC device (UGREEN 1080P), previously driven by
        # ROS 1 usb_cam on /dev/video0 at 640x480 yuyv. It is a peripheral, so it
        # is OFF by default and, unlike the five hardware controllers, it is NOT
        # wired into the shutdown guards below: an unplugged camera must not take
        # the arm and chassis down with it.
        'camera': 'false', 'camera_device': '/dev/video0',
        'video_server': 'false',
        # User measured 2026-10-11: 1 cm behind, 1 cm left of wheel rectangle centre.
        'lidar_port': '/dev/ld06', 'lidar_x': '-0.01', 'lidar_y': '0.01',
        'lidar_z': '0.18', 'lidar_yaw': '0.0', 'imu_yaw': '1.57079632679',
        'initial_x': '0.0', 'initial_y': '0.0', 'initial_yaw': '0.0',
    }.items()]
    chassis = Node(package='armpi_ros2', executable='chassis', output='screen',
                   parameters=[cfg, {'dry_run': ParameterValue(LaunchConfiguration('dry_run'), value_type=bool),
                                     'enable_motion': ParameterValue(LaunchConfiguration('enable_motion'), value_type=bool)}])
    odom = Node(package='armpi_ros2', executable='odom', output='screen', parameters=[cfg, {
        'imu_yaw': ParameterValue(LaunchConfiguration('imu_yaw'), value_type=float),
        **{k: ParameterValue(LaunchConfiguration(k), value_type=float)
           for k in ('initial_x', 'initial_y', 'initial_yaw')}}])
    bmi = Node(package='armpi_ros2', executable='bmi270', output='screen',
               parameters=[cfg], condition=IfCondition(LaunchConfiguration('imu')))
    arm = Node(package='armpi_ros2', executable='arm', output='screen',
               condition=IfCondition(LaunchConfiguration('arm')), parameters=[cfg, {
                   'dry_run': ParameterValue(LaunchConfiguration('dry_run'), value_type=bool),
                   'enable_arm_motion': ParameterValue(LaunchConfiguration('enable_arm_motion'), value_type=bool),
                   'enable_gripper_motion': ParameterValue(LaunchConfiguration('enable_gripper_motion'), value_type=bool),
                   'serial_port': LaunchConfiguration('arm_port')}])
    lidar = Node(package='ldlidar_stl_ros2', executable='ldlidar_stl_ros2_node',
                 name='LD06', output='screen', condition=IfCondition(LaunchConfiguration('lidar')),
                 parameters=[{'product_name': 'LDLiDAR_LD06', 'topic_name': '/scan',
                              'frame_id': 'base_laser', 'port_name': LaunchConfiguration('lidar_port'),
                              'port_baudrate': 230400, 'laser_scan_dir': True,
                              'enable_angle_crop_func': False, 'angle_crop_min': 135.0,
                              'angle_crop_max': 225.0}])
    laser_tf = Node(package='tf2_ros', executable='static_transform_publisher',
                    name='base_to_laser', arguments=[
                        '--x', LaunchConfiguration('lidar_x'), '--y', LaunchConfiguration('lidar_y'),
                        '--z', LaunchConfiguration('lidar_z'), '--yaw', LaunchConfiguration('lidar_yaw'),
                        '--frame-id', 'base_link', '--child-frame-id', 'base_laser'])
    imu_tf = Node(package='tf2_ros', executable='static_transform_publisher',
                  name='base_to_imu', arguments=['--yaw', LaunchConfiguration('imu_yaw'),
                    '--frame-id', 'base_link', '--child-frame-id', 'imu_link'])
    bridge_path = os.path.join(get_package_share_directory('rosbridge_server'),
                               'launch', 'rosbridge_websocket_launch.xml')
    bridge = IncludeLaunchDescription(FrontendLaunchDescriptionSource(bridge_path),
        launch_arguments={'port': LaunchConfiguration('rosbridge_port'), 'address': '127.0.0.1'}.items(),
        condition=IfCondition(LaunchConfiguration('rosbridge')))
    camera = Node(package='v4l2_camera', executable='v4l2_camera_node',
                  name='camera', namespace='rear_camera', output='screen',
                  condition=IfCondition(LaunchConfiguration('camera')),
                  parameters=[{
                      'video_device': LaunchConfiguration('camera_device'),
                      # Matches the ROS 1 usb_cam profile exactly (640x480 yuyv).
                      'image_size': [640, 480],
                      'pixel_format': 'YUYV',
                      'output_encoding': 'rgb8',
                      'camera_frame_id': 'rear_camera'}])
    video = Node(package='web_video_server', executable='web_video_server',
                 name='web_video_server', output='screen',
                 condition=IfCondition(LaunchConfiguration('video_server')),
                 parameters=[{'port': 8080, 'address': '0.0.0.0'}])
    # A partial launch must not look like a healthy whole-car controller.
    # Do not silently respawn an arm into an unknown hardware state.
    # camera is deliberately ABSENT from this list: it is an optional peripheral,
    # and a USB camera dropout must never shut the arm and chassis down.
    guards = [RegisterEventHandler(OnProcessExit(target_action=node,
              on_exit=[EmitEvent(event=Shutdown(reason='Required hardware node exited'))]))
              for node in (chassis, odom, bmi, arm, lidar)]
    return LaunchDescription(arguments + guards + [chassis, odom, bmi, arm, lidar,
                                                   laser_tf, imu_tf, bridge, camera, video])
