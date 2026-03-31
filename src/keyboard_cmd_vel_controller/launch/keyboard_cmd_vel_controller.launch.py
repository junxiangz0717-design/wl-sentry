from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='keyboard_cmd_vel_controller',
            executable='keyboard_cmd_vel_node',
            name='keyboard_cmd_vel_controller',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'linear_step': 0.1,
                'angular_step': 0.1,
                'max_linear_speed': 2.0,
                'max_angular_speed': 2.0,
                'publish_rate': 20.0,
            }],
        )
    ])
