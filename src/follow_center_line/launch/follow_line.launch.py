from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='follow_center_line',
            executable='create_line',
            name='create_line',
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='follow_center_line',
            executable='pure_pursuit',
            name='pure_pursuit',
            output='screen',
            emulate_tty=True
        )
    ])
