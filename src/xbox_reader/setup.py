import os
from glob import glob
from setuptools import setup

package_name = 'xbox_reader'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='e22e',
    maintainer_email='e22e@todo.todo',
    description='Xbox controller joystick reader for ROS 2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'joy_reader = xbox_reader.joy_reader:main',
        ],
    },
)
