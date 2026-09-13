from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 自检脚本自己发布 /gamepad/state，不能同时起 joy_node / joy_reader
    return LaunchDescription([
        Node(
            package='ee_teleop', executable='ee_teleop',
            name='ee_teleop', output='screen'),
        Node(
            package='ee_teleop', executable='self_check',
            name='ee_teleop_self_check', output='screen'),
    ])
