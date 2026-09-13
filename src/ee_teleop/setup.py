import os
from glob import glob
from setuptools import setup

package_name = 'ee_teleop'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='e22e',
    maintainer_email='e22e@todo.todo',
    description='End-effector teleop frontend for ROS 2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'ee_teleop = ee_teleop.ee_teleop:main',
            'self_check = ee_teleop.self_check:main',
        ],
    },
)
