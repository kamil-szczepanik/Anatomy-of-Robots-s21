from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package='turtlesim',
                executable='turtlesim_node'
            ),
            Node(
                package='lab1',
                executable='my_teleop'
                name='my_teleop',
            ),
            Node(
                package='lab1',
                executable='keyboard_reader',
                name='keyboard_reader',
                prefix=["gnome-terminal ", "-- "],
                output='screen'
            )
        ]
    )