import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():

  use_sim_time = LaunchConfiguration('use_sim_time', default='false')
  xacro_path = LaunchConfiguration('xacro_path', default=None)
  urdf_file_name = 'robot.urdf.xacro'

  print("urdf_file_name : {}".format(urdf_file_name))

  urdf = os.path.join(
      get_package_share_directory('lab2'),
      urdf_file_name)

  return LaunchDescription([
      DeclareLaunchArgument(
          'use_sim_time',
          default_value='false',
          description='Use simulation (Gazebo) clock if true'),
      DeclareLaunchArgument(
          'xacro_path',
          default_value=None,
          description='path to urdf.xacro file to publish'),
      Node(
          package='robot_state_publisher',
          executable='robot_state_publisher',
          name='robot_state_publisher',
          output='screen',
          parameters=[{
              'use_sim_time': use_sim_time,
              'robot_description': Command(['xacro',' ', urdf])
          }]),
          
      Node(
          package='lab2',
          executable='state_publisher',
          name='state_publisher',
          output='screen'),
  ])