from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    dev = LaunchConfiguration('dev')
    config_file = LaunchConfiguration('config_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'dev', default_value='Xbox Gamepad (userspace driver)',
            description='手柄名称（SDL 识别的设备名，用 ls /dev/input 对应不了时用这个）'),
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([
                get_package_share_directory('ee_teleop'),
                'config', 'ee_teleop.yaml']),
            description='teleop 参数文件路径，可用自己的 yaml 覆盖'),
        Node(
            package='joy', executable='joy_node', name='joy_node',
            parameters=[{'device_name': dev}]),
        Node(
            package='xbox_reader', executable='joy_reader', name='joy_reader',
            output='screen'),
        Node(
            package='ee_teleop', executable='ee_teleop', name='ee_teleop',
            parameters=[config_file, {'use_sim_time': True}],
            output='screen'),
    ])
