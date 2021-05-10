import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='rviz2',
            executable='rviz2',
            name='bogson_rviz2',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,}],
            arguments=['-d', os.path.join(get_package_share_directory('lab5'), 'oint_config.rviz')]),
        Node(
            package='lab5',
            executable='oint_srv',
            name='oint_srv',
            output='screen',
        )
  ])
