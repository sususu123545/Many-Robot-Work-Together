from glob import glob
from setuptools import find_packages, setup

package_name = 'robot_safety'

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
    description='Liveness supervision and latched stop for the VLA control path',
    license='MIT',
    entry_points={'console_scripts': [
        'supervisor = robot_safety.supervisor_node:main',
    ]},
)
