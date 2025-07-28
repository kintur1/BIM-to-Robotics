import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    robot_description_pkg = get_package_share_directory('robot_description')
    robot_controller_pkg = get_package_share_directory('robot_controller')
    robot_moveit_pkg = get_package_share_directory('robot_moveit_config')
    your_pipe_pkg = get_package_share_directory('your_pipe_pkg')  # Added pipe package reference

    # Launch Gazebo with robot
    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_description_pkg, 'launch', 'gazebo.launch.py'),
        )
    )

    # Spawn pipe after 3 seconds (Gazebo should be ready)
    spawn_pipe = TimerAction(
        period=1.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(your_pipe_pkg, 'launch', 'spawn_pipe.launch.py')  # Corrected path
                )
            )
        ]
    )

    # Launch controllers after pipe is spawned (5 seconds total)
    launch_controllers = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(robot_controller_pkg, 'launch', 'robot_controller.launch.py'),
                )
            )
        ]
    )

    # Launch MoveIt after everything else is ready (15 seconds total)
    launch_moveit = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(robot_moveit_pkg, 'launch', 'move_group.launch.py'),
                )
            )
        ]
    )

    return LaunchDescription([
        launch_gazebo,
        spawn_pipe,
        launch_controllers,
        launch_moveit
    ])