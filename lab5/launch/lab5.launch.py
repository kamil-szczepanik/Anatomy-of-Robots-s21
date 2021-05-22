import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def get_package_file(package, file_path):
    """Get the location of a file installed in an ament package"""
    package = 'lab5'
    package_path = get_package_share_directory(package)
    absolute_file_path = os.path.join(get_package_share_directory('lab5'), file_path)
    return absolute_file_path

def run_xacro(xacro_file):
    """Run xacro and output a file in the same directory with the same name, w/o a .xacro suffix"""
    urdf_file, ext = os.path.splitext(xacro_file)
    if ext != '.xacro':
        raise RuntimeError(f'Input file to xacro must have a .xacro extension, got {xacro_file}')
    os.system(f'xacro {xacro_file} -o {urdf_file}')
    return urdf_file

def generate_launch_description():

    param_file_name = 'params.yaml'
    param_file = get_package_file('lab5', param_file_name)

    pos_param_file_name = 'pos_params.yaml'
    pos_param_file = get_package_file('lab5', pos_param_file_name)

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    xacro_file_name = 'robot.urdf.xacro'
    print("urdf_file_name : {}".format(xacro_file_name))
    
    xacro_file = get_package_file('lab5', xacro_file_name)
    urdf_file = run_xacro(xacro_file)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[urdf_file]),
        Node(
            package='rviz2',
            executable='rviz2',
            name='bogson_rviz2',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,}],
            arguments=['-d', os.path.join(get_package_share_directory('lab5'), 'robot.rviz')]),
        Node(
            package='lab5',
            executable='oint_srv',
            name='oint_srv',
            output='screen',
        ),
        Node(
            package='lab5',
            executable='ikin',
            name='ikin_node',
            output='screen',
        )
  ])
