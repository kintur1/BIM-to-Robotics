from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_pipe_assembly',
            output='screen',
            arguments=[
                '-name', 'pipe_system',
                '-file', PathJoinSubstitution([
                    FindPackageShare('your_pipe_pkg'),
                    'sdf',
                    'pipe_system.sdf'
                ]),
                '-x', '0.30',
                '-y', '0.08',
                '-z', '0.4'
            ]
        )
        
    ])