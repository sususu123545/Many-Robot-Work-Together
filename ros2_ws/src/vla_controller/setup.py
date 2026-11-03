from glob import glob
from setuptools import find_packages, setup

package_name = 'vla_controller'

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
    description='Action adapter, safety filter and chunk executor for VLA policies',
    license='MIT',
    entry_points={'console_scripts': [
        'chunk_executor = vla_controller.chunk_executor_node:main',
    ]},
)
