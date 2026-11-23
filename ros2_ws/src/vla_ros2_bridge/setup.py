from glob import glob
from setuptools import find_packages, setup

package_name = 'vla_ros2_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='ArmPi project maintainers',
    maintainer_email='maintainer@example.com',
    description='Observation builder, policy hook and action client for VLA policies',
    license='MIT',
    entry_points={'console_scripts': [
        'vla_observation_node = vla_ros2_bridge.observation_node:main',
        'vla_bridge_node = vla_ros2_bridge.bridge_node:main',
    ]},
)
