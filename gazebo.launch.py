from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from pathlib import Path
import os

def generate_launch_description():
    declared_arguments = []

    # UR specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            description="IP address of used UR robot.",
            default_value="0.0.0.0",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            description="Type/series of used UR robot.",
            default_value="ur5e",
            choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e", "ur20", "ur30"],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        )
    )

    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="robot_description",
            description="Description package with robot URDF/XACRO files.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="robot.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value='""',
            description="Prefix of the joint names, useful for multi-robot setup.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_ignition",
            default_value="true",
            description="Set to 'true' to use Gazebo Ignition (Gazebo Sim) for simulation.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time for ROS nodes.",
        )
    )

    # Initialize LaunchConfigurations
    robot_ip = LaunchConfiguration("robot_ip")
    ur_type = LaunchConfiguration("ur_type")
    safety_limits = LaunchConfiguration("safety_limits")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    tf_prefix = LaunchConfiguration("tf_prefix")
    sim_ignition = LaunchConfiguration("sim_ignition")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Generate robot_description parameter by running xacro on the robot urdf file
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
            " ",
            "safety_limits:=",
            safety_limits,
            " ",
            "name:=",
            "ur5e",
            " ",
            "ur_type:=",
            ur_type,
            " ",
            "robot_ip:=",
            robot_ip,
            " ",
            "sim_ignition:=",
            sim_ignition,
            " ",
            "tf_prefix:=",
            tf_prefix,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Paths for resources and world
    robot_description_path = get_package_share_directory("ur_description")
    gripper_description_path = get_package_share_directory("robotiq_description")

    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            str(Path(robot_description_path).parent.resolve()),
            ":",
            str(Path(gripper_description_path).parent.resolve()),
        ],
    )

    custom_world_path = os.path.join(
        get_package_share_directory("robot_simulation_launch"),
        "worlds",
        "custom_world.sdf",
    )

    # Gazebo Ignition launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments=[("gz_args", f" -v 4 -r {custom_world_path} ")],
    )

    # Spawn robot entity in Gazebo Ignition
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "/robot_description", "-name", "ur5e"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # ROS2-Gazebo bridge for clock topic
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        parameters=[
            {"use_sim_time": use_sim_time},
            {"qos_overrides./clock.publisher.reliability": "reliable"},
            {"qos_overrides./clock.publisher.durability": "transient_local"},
        ],
    )

    # ** Add robot_state_publisher node **
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    nodes_to_start = [
        gazebo_resource_path,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge,
        robot_state_publisher_node,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
