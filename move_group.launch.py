from launch import LaunchDescription
from moveit_configs_utils import MoveItConfigsBuilder
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from ur_moveit_config.launch_common import load_yaml

def generate_launch_description():
    # 1. Enhanced MoveIt configuration
    moveit_config = (
        MoveItConfigsBuilder("ur5e", package_name="robot_moveit_config")
        .robot_description(file_path=os.path.join(
          get_package_share_directory("robot_moveit_config"), "config", "ur5e.urdf"))
        .robot_description_semantic(file_path=os.path.join(
          get_package_share_directory("robot_moveit_config"), "config", "ur5e.srdf"))
        .trajectory_execution(file_path=os.path.join(
          get_package_share_directory("robot_moveit_config"), "config", "moveit_controllers.yaml"))
        .robot_description_kinematics(file_path=os.path.join(
          get_package_share_directory("robot_moveit_config"), "config", "kinematics.yaml"))
        .planning_scene_monitor(
          publish_robot_description=True,
          publish_robot_description_semantic=True,
          publish_geometry_updates=True,
          publish_state_updates=True,
          publish_transforms_updates=True)
        .to_moveit_configs()
    )

    # 2. Enhanced planning scene parameters
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "planning_scene_monitor_options": {
            "name": "planning_scene_monitor",
            "robot_description": "robot_description",
            "joint_state_topic": "/joint_states",
            "attached_collision_object_topic": "/move_group/attached_collision_object",
            "publish_planning_scene_topic": "/move_group/publish_planning_scene",
            "monitored_planning_scene_topic": "/move_group/monitored_planning_scene",
        }
    }

    # 3. Collision object parameters
    collision_object_parameters = {
        "collision_detector": "Bullet",  # Use hybrid collision detector for better performance
        "collision_objects": {
            "pipe": {
                "type": "cylinder",
                "dimensions": [0.5, 0.05],  # height, radius
                "pose": {
                    "position": [0.4, 0.0, 0.25],  # x, y, z
                    "orientation": [0.0, 0.0, 0.0, 1.0]  # identity quaternion
                },
                "operation": "add"
            }
        }
    }

    # 4. Enhanced planning pipelines configuration
    planning_pipelines_config = {
        "default_planning_pipeline": "ompl",
        "planning_pipelines": ["pilz", "ompl"],
        "pilz": {
            "planning_plugin": "pilz_industrial_motion_planner/CommandPlanner",
            "request_adapters": "",
            "start_state_max_bounds_error": 0.1,
        },
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization " 
                                "default_planner_request_adapters/FixWorkspaceBounds " 
                                "default_planner_request_adapters/FixStartStateBounds " 
                                "default_planner_request_adapters/FixStartStateCollision " 
                                "default_planner_request_adapters/FixStartStatePathConstraints",
            "start_state_max_bounds_error": 0.1,
            "collision_checking_resolution": 0.01,  # More precise collision checking
            "longest_valid_segment_fraction": 0.05,  # Better for obstacle-rich environments
        },
    }

    # Load OMPL planning config
    ompl_planning_yaml = load_yaml(
        "robot_moveit_config", "config/ompl_planning.yaml"
    )
    if ompl_planning_yaml:
        planning_pipelines_config["ompl"].update(ompl_planning_yaml)
    else:
        print("WARNING: Could not load ompl_planning.yaml")

    # 5. Enhanced move_group node configuration
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        emulate_tty=True,
        parameters=[
            moveit_config.to_dict(),
            planning_scene_monitor_parameters,
            collision_object_parameters,
            planning_pipelines_config,
            {
                "use_sim_time": True,
                "allow_trajectory_execution": True,
                "max_safe_path_cost": 1,
                "default_planning_pipeline": "ompl",
                "capabilities": "",  # Add specific capabilities if needed
                "disable_capabilities": "",  # Disable specific capabilities if needed
                "octomap_frame": "",  # Disable octomap if not needed
                "octomap_resolution": 0.0,
                "publish_planning_scene": True,
                "publish_geometry_updates": True,
                "publish_state_updates": True,
                "publish_transforms_updates": True,
            }
        ],
    )

    # 6. Enhanced RViz configuration
    rviz_config = os.path.join(
        get_package_share_directory("robot_moveit_config"),
        "config",
        "moveit.rviz"
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {"use_sim_time": True},
        ],
        remappings=[
            ("/display_planned_path", "/move_group/display_planned_path"),
            ("/robot_description", "/move_group/robot_description"),
        ]
    )

    return LaunchDescription([
        move_group_node,
        rviz_node,
    ])