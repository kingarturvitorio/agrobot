import os
import yaml
import launch
import launch_ros
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def get_package_file(package, file_path):
    """Get the location of a file installed in an ament package"""
    package_path = get_package_share_directory(package)
    absolute_file_path = os.path.join(package_path, file_path)
    return absolute_file_path

def run_xacro(xacro_file):
    """Run xacro and output a file in the same directory with the same name, w/o a .xacro suffix"""
    urdf_file, ext = os.path.splitext(xacro_file)
    if ext != '.xacro':
        raise RuntimeError(f'Input file to xacro must have a .xacro extension, got {xacro_file}')
    os.system(f'xacro {xacro_file} -o {urdf_file}')
    return urdf_file

def generate_launch_description():
    xacro_file = get_package_file('agroBot_description', 'urdf/agrobot.xacro')
    urdf_file = run_xacro(xacro_file)

    return LaunchDescription(
        [
            ExecuteProcess(
                cmd=['gazebo', '-s', 'libgazebo_ros_factory.so'],
                output='screen',
            ),
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=['-entity','agroBot_description', '-b', '-file', urdf_file],
            ),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                output='screen',
                arguments=[urdf_file],
            ),
        ]
    )