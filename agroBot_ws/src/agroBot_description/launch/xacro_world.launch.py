from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():

    spawn_x_val = '-6.0'
    spawn_y_val = '0.3'
    spawn_z_val = '0.8'
    spawn_yaw_val = '4.7'
    
    package_name = 'agroBot_description'
    package_name2 = 'agroBot_description'
    
    modelFileRelativePath = 'urdf/agrobot.xacro'
    worldFileRelativePath = 'worlds/farmWith1CropRow.world'
    
    pathModelFile = os.path.join(get_package_share_directory(package_name), modelFileRelativePath)
    pathWorldFile = os.path.join(get_package_share_directory(package_name2), worldFileRelativePath)

    robotDescription = xacro.process_file(pathModelFile).toxml()
    
    gazebo_rosPackageLaunch = PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'))

    gazeboLaunch = IncludeLaunchDescription(gazebo_rosPackageLaunch, launch_arguments={'world': pathWorldFile}.items())

    spawnModelNode = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'montagem-pandora',
                   '-x', spawn_x_val, '-y', spawn_y_val, '-z', spawn_z_val, '-Y', spawn_yaw_val],
        output='screen'
    )

    nodeRobotStatePublisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robotDescription, 'use_sim_time': True}]
    )

    # Publish the joint states of the robot
    start_joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher')
    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            ],
        )

    simple_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "simple_velocity_controller",
            "--controller-manager",
            "/controller_manager"
            ]
        )

    simple_controller_py = Node(
        package="agroBot_description",
        executable="simple_controller",
        )


    ld = LaunchDescription()

    # Launch Gazebo with the specified world
    ld.add_action(gazeboLaunch)

    # Add robot spawn, state publisher, and controller nodes
    ld.add_action(spawnModelNode)
    ld.add_action(nodeRobotStatePublisher)
    ld.add_action(start_joint_state_publisher_cmd)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(simple_controller)

    # ld.add_action(simple_position_controller)
    ld.add_action(simple_controller_py)

    return ld