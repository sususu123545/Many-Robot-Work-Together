from glob import glob
from setuptools import find_packages, setup

setup(
    name='armpi_ros2',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/armpi_ros2']),
        ('share/armpi_ros2', ['package.xml']),
        ('share/armpi_ros2/launch', glob('launch/*.launch.py')),
        ('share/armpi_ros2/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='ArmPi project maintainers',
    maintainer_email='maintainer@example.com',
    description='Native ROS 2 ArmPi migration baseline; hardware motion disabled by default',
    license='MIT',
    entry_points={'console_scripts': [
        'chassis = armpi_ros2.chassis_node:main',
        'odom = armpi_ros2.odom_node:main',
        'bmi270 = armpi_ros2.bmi270_node:main',
        'arm = armpi_ros2.arm_node:main',
    ]},
)
