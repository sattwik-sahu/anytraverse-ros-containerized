"""Hound drive base: /cmd_vel -> Sabertooth motor controllers."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([FindPackageShare('hound_base'), 'config', 'base_driver.yaml'])),
        Node(package='hound_base', executable='base_driver', name='base_driver', output='screen',
             parameters=[LaunchConfiguration('params_file')],
             ros_arguments=['--log-level', LaunchConfiguration('log_level', default='info')]),
    ])
