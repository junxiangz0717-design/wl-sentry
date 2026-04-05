import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('stair_mode_controller')
    default_params = os.path.join(package_share, 'config', 'stair_mode_controller.yaml')

    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Full path to the stair mode controller parameters file',
        ),
        Node(
            package='stair_mode_controller',
            executable='stair_mode_controller_node',
            name='stair_mode_controller',
            output='screen',
            parameters=[params_file],
        ),
    ])
