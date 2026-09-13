from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    dev = LaunchConfiguration('dev')

    return LaunchDescription([
        DeclareLaunchArgument(
            'dev', default_value='Xbox Gamepad (userspace driver)',
            description='手柄名称（SDL 识别的设备名，用 ls /dev/input 对应不了时用这个）'),
        Node(
            package='joy', executable='joy_node', name='joy_node',
            parameters=[{'device_name': dev}]),
        Node(
            package='xbox_reader', executable='joy_reader', name='joy_reader',
            output='screen'),
    ])
