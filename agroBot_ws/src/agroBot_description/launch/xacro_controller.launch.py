from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
import os
from ament_index_python import get_package_share_directory, get_package_prefix
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():


    # Pose where we want to spawn the robot
    spawn_x_val = '0.0' #frente e traz (-) traz (+) frente
    spawn_y_val = '0.0 ' #sobe e desce (-) desce (+) sobe
    spawn_z_val = '1.0' 
    spawn_yaw_val = '1.6'   

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(get_package_share_directory("agroBot_description"), "urdf", "agrobot.xacro"),
        description="Absolute path to the robot URDF file"
    )

    env_var = SetEnvironmentVariable("GAZEBO_MODEL_PATH", os.path.join(get_package_prefix("agroBot_description"), "share"))

    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]), value_type=str)
 
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzserver.launch.py"))
    )

    start_gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzclient.launch.py"))
    )

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "agroBot", "-topic", "robot_description",                                                          
                                                         '-x', spawn_x_val,
                                                         '-y', spawn_y_val,
                                                         '-z', spawn_z_val,
                                                         '-Y', spawn_yaw_val],
        output='screen',
    )

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


    return LaunchDescription([
        env_var,
        model_arg,
        joint_state_broadcaster_spawner,
        robot_state_publisher,
        simple_controller,
        simple_controller_py,
        start_gazebo_server,
        start_gazebo_client,
        spawn_robot
    ])