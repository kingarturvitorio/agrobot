import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'launch', 'lic_gazebo', 'urdf_world_launch.py'],
            output='screen',
        ),
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'launch', 'rosbridge_server', 'rosbridge_websocket_launch.xml'],
            output='screen',
        ),
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'run', 'lic_control', 'salve_data'],
            output='screen',
        ),
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'run', 'lic_control', 'odometry'],
            output='screen',
        ),
        # launch.actions.ExecuteProcess(
        #     cmd=['ros2', 'run', 'lic_control', 'return'],
        #     output='screen',
        # ),

        # launch.actions.ExecuteProcess(
        #     cmd=['ros2', 'launch', 'teleop_twist_joy', 'teleop-launch.py'],
        #     output='screen',
        # ),
        # launch.actions.ExecuteProcess(
        #     cmd=['ros2', 'run', 'roboagricola', 'roboagricola_joy_original'],
        #     output='screen',
        # ),
        # launch.actions.ExecuteProcess(
        #     cmd=['ros2', 'launch', 'roboagricola', 'joystic_teleop_launch.py'],
        #     output='screen',
        # ),
        # launch.actions.ExecuteProcess(
        #     cmd=['ros2', 'run', 'roboagricola', 'simple_controller_v2'],
        #     output='screen',
        # ),
        # launch.actions.ExecuteProcess(
        #     cmd=['ros2', 'launch', 'roboagricola', 'gazebo_launch.py'],
        #     output='screen',
        # ),
    ])
