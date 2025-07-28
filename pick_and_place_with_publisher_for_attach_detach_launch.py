#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.msg import RobotTrajectory, PlanningScene
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from geometry_msgs.msg import Pose
from std_msgs.msg import String
import tf2_geometry_msgs
from tf2_ros import TransformListener, Buffer
import threading
import time
from rclpy.executors import MultiThreadedExecutor

class PickPlaceNode(Node):
    def __init__(self):
        super().__init__('pick_place_publisher_demo')
        
        # Define planning groups
        self.arm_group_name = "ur_manipulator"
        self.gripper_group_name = "gripper"
        
        # Initialize MoveGroup action client
        self.move_group_client = ActionClient(self, MoveGroup, '/move_action')
        self.execute_trajectory_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        
        # Planning scene monitor
        self.planning_scene_monitor = self.create_subscription(
            PlanningScene,
            '/monitored_planning_scene',
            self.planning_scene_callback,
            10)
        
        # Wait for action servers
        if not self.move_group_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("MoveGroup action server not available")
            raise RuntimeError("Action server not available")
            
        if not self.execute_trajectory_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("ExecuteTrajectory action server not available")
            raise RuntimeError("Action server not available")
        
        # Publishers for status
        self.arm_status_publisher = self.create_publisher(String, '/ur_manipulator/status', 10)
        self.gripper_status_publisher = self.create_publisher(String, '/gripper/status', 10)
        
        # Synchronization variables
        self.lock = threading.Lock()
        self.current_planning_scene = None
    
    def planning_scene_callback(self, msg):
        with self.lock:
            self.current_planning_scene = msg
    
    def publish_arm_status(self, status):
        msg = String()
        msg.data = f"[UR Manipulator] {status}"
        self.arm_status_publisher.publish(msg)
        self.get_logger().info(msg.data)
    
    def publish_gripper_status(self, status):
        msg = String()
        msg.data = f"[Gripper] {status}"
        self.gripper_status_publisher.publish(msg)
        self.get_logger().info(msg.data)
    
    def execute_pick_and_place_task(self):
        # 1. Move to Pregrasp Position
        self.get_logger().info("Step 1: Moving to Pregrasp Position")
        if not self.move_to_pregrasp_position():
            self.get_logger().error("Failed to move to pregrasp position. Aborting task.")
            return
        
        # 2. Open Gripper
        self.get_logger().info("Step 2: Opening Gripper")
        if not self.open_gripper():
            self.get_logger().error("Failed to open gripper. Aborting task.")
            return
        
        # 3. Move to Grasp Position (30 cm down)
        self.get_logger().info("Step 3: Moving to Grasp Position (30 cm down)")
        if not self.move_down_to_grasp_position():
            self.get_logger().error("Failed to move to grasp position. Aborting task.")
            return
        
        # 4. Close Gripper to 50%
        self.get_logger().info("Step 4: Closing Gripper to half")
        if not self.close_gripper_partially():
            self.get_logger().error("Failed to close gripper. Aborting task.")
            return
        
        # 5. Lift to Pregrasp Position (30 cm up)
        self.get_logger().info("Step 5: Lifting to Pregrasp Position")
        if not self.lift_to_pregrasp_position():
            self.get_logger().error("Failed to lift to pregrasp position. Aborting task.")
            return
        
        # 6. Move to Drop Location
        self.get_logger().info("Step 6: Moving to Drop Location")
        if not self.moving_to_drop_location():
            self.get_logger().error("Failed to move to drop location. Aborting task.")
            return
        
        # 7. Move Down to Drop Position
        self.get_logger().info("Step 7: Moving Down to Drop Position")
        if not self.move_down_to_drop_position():
            self.get_logger().error("Failed to move down to drop position. Aborting task.")
            return
        
        # 8. Open Gripper FULLY to release object
        self.get_logger().info("Step 8: Opening Gripper Fully to Release Object")
        if not self.open_gripper_fully():
            self.get_logger().error("Failed to open gripper fully. Aborting task.")
            return
        
        # 9. Move Up from Drop Position
        self.get_logger().info("Step 9: Moving Up from Drop Position")
        if not self.move_up_from_drop_position():
            self.get_logger().error("Failed to move up from drop position. Aborting task.")
            return
        
        self.get_logger().info("Pick and Place task completed successfully!")
    
    def send_move_group_goal(self, group_name, target, is_pose_target=True, cartesian_path=False):
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = group_name
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 20.0
        goal_msg.request.max_velocity_scaling_factor = 0.5
        goal_msg.request.max_acceleration_scaling_factor = 0.5
        
        if is_pose_target:
            goal_msg.request.goal_constraints.append(self.create_pose_constraint(target))
        else:
            goal_msg.request.goal_constraints.append(self.create_joint_constraint(target))
        
        if cartesian_path:
            goal_msg.request.pipeline_id = "pilz_industrial_motion_planner"
            goal_msg.request.planner_id = "PTP"
        
        future = self.move_group_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        
        if not future.result().accepted:
            return False
        
        result_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        return result_future.result().result.error_code.val == MoveGroup.Result.SUCCESS
    
    def create_pose_constraint(self, pose):
        from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
        from shape_msgs.msg import SolidPrimitive
        
        constraints = Constraints()
        
        # Position constraint
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = "world"
        pos_constraint.link_name = "tool0"
        
        # Create a bounding box for position tolerance
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.01, 0.01, 0.01]  # 1cm tolerance
        
        pos_constraint.constraint_region.primitives.append(box)
        pos_constraint.constraint_region.primitive_poses.append(pose)
        pos_constraint.weight = 1.0
        
        # Orientation constraint
        orient_constraint = OrientationConstraint()
        orient_constraint.header.frame_id = "world"
        orient_constraint.link_name = "tool0"
        orient_constraint.orientation = pose.orientation
        orient_constraint.absolute_x_axis_tolerance = 0.1
        orient_constraint.absolute_y_axis_tolerance = 0.1
        orient_constraint.absolute_z_axis_tolerance = 0.1
        orient_constraint.weight = 1.0
        
        constraints.position_constraints.append(pos_constraint)
        constraints.orientation_constraints.append(orient_constraint)
        
        return constraints
    
    def create_joint_constraint(self, joint_values):
        from moveit_msgs.msg import Constraints, JointConstraint
        
        constraints = Constraints()
        
        for joint_name, value in joint_values.items():
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = value
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        
        return constraints
    
    def execute_trajectory(self, trajectory):
        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.trajectory = trajectory
        
        future = self.execute_trajectory_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        
        if not future.result().accepted:
            return False
        
        result_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        return result_future.result().result.error_code.val == ExecuteTrajectory.Result.SUCCESS
    
    def move_to_pregrasp_position(self):
        target_pose = Pose()
        target_pose.orientation.x = 0.721
        target_pose.orientation.y = -0.693
        target_pose.orientation.z = -0.006
        target_pose.orientation.w = 0.006
        target_pose.position.x = 0.695
        target_pose.position.y = 0.168
        target_pose.position.z = 0.510
        
        self.publish_arm_status("Planning motion to pregrasp position")
        success = self.send_move_group_goal(self.arm_group_name, target_pose)
        
        if success:
            self.publish_arm_status("Trajectory execution completed successfully")
        else:
            self.publish_arm_status("Trajectory execution failed")
        
        return success
    
    def open_gripper(self):
        self.publish_gripper_status("Planning gripper opening")
        success = self.send_move_group_goal(
            self.gripper_group_name, 
            {"robotiq_85_left_knuckle_joint": 0.0},
            is_pose_target=False
        )
        
        if success:
            self.publish_gripper_status("Gripper opened successfully")
        else:
            self.publish_gripper_status("Gripper opening failed")
        
        return success
    
    def move_down_to_grasp_position(self):
        from moveit_msgs.msg import RobotTrajectory
        from geometry_msgs.msg import Pose
        
        self.publish_arm_status("Moving down to grasp position")
        
        # Get current pose
        current_pose = self.get_current_pose()
        
        # Create waypoints for cartesian path (30 cm down)
        waypoints = []
        num_segments = 10
        step_size = 0.3 / num_segments
        
        for i in range(1, num_segments + 1):
            intermediate_pose = Pose()
            intermediate_pose.orientation = current_pose.orientation
            intermediate_pose.position.x = current_pose.position.x
            intermediate_pose.position.y = current_pose.position.y
            intermediate_pose.position.z = current_pose.position.z - (step_size * i)
            waypoints.append(intermediate_pose)
        
        # Compute cartesian path
        (success, trajectory, fraction) = self.compute_cartesian_path(waypoints)
        
        if not success or fraction < 0.9:
            self.publish_arm_status("Attempting joint-space planning instead")
            target_pose = Pose()
            target_pose.orientation = current_pose.orientation
            target_pose.position.x = current_pose.position.x
            target_pose.position.y = current_pose.position.y
            target_pose.position.z = current_pose.position.z - 0.3
            
            success = self.send_move_group_goal(self.arm_group_name, target_pose)
        
        if success:
            self.publish_arm_status("Trajectory execution completed successfully")
        else:
            self.publish_arm_status("Trajectory execution failed")
        
        return success
    
    def compute_cartesian_path(self, waypoints):
        from moveit_msgs.srv import GetCartesianPath
        from geometry_msgs.msg import Pose
        
        client = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Cartesian path service not available")
            return (False, None, 0.0)
        
        request = GetCartesianPath.Request()
        request.header.frame_id = "world"
        request.group_name = self.arm_group_name
        request.link_name = "tool0"
        request.waypoints = waypoints
        request.max_step = 0.01
        request.jump_threshold = 0.0
        request.avoid_collisions = True
        
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is None:
            return (False, None, 0.0)
        
        response = future.result()
        return (True, response.solution, response.fraction)
    
    def get_current_pose(self):
        from tf2_ros import TransformException
        from geometry_msgs.msg import PoseStamped
        
        tf_buffer = Buffer()
        tf_listener = TransformListener(tf_buffer, self)
        
        pose = PoseStamped()
        pose.header.frame_id = "tool0"
        pose.header.stamp = self.get_clock().now().to_msg()
        
        try:
            transform = tf_buffer.lookup_transform(
                "world",
                "tool0",
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0))
            
            current_pose = Pose()
            current_pose.position.x = transform.transform.translation.x
            current_pose.position.y = transform.transform.translation.y
            current_pose.position.z = transform.transform.translation.z
            current_pose.orientation = transform.transform.rotation
            
            return current_pose
        except TransformException as ex:
            self.get_logger().error(f"Could not get current pose: {ex}")
            return None
    
    def close_gripper_partially(self):
        self.publish_gripper_status("Planning gripper closing")
        success = self.send_move_group_goal(
            self.gripper_group_name,
            {"robotiq_85_left_knuckle_joint": 0.4551},
            is_pose_target=False
        )
        
        if success:
            self.publish_gripper_status("Gripper closed successfully")
        else:
            self.publish_gripper_status("Gripper closing failed")
        
        return success
    
    def lift_to_pregrasp_position(self):
        from moveit_msgs.msg import RobotTrajectory
        from geometry_msgs.msg import Pose
        
        self.publish_arm_status("Lifting to pregrasp position")
        
        # Get current pose
        current_pose = self.get_current_pose()
        
        # Create waypoints for cartesian path (30 cm up)
        waypoints = []
        num_segments = 10
        step_size = 0.3 / num_segments
        
        for i in range(1, num_segments + 1):
            intermediate_pose = Pose()
            intermediate_pose.orientation = current_pose.orientation
            intermediate_pose.position.x = current_pose.position.x
            intermediate_pose.position.y = current_pose.position.y
            intermediate_pose.position.z = current_pose.position.z + (step_size * i)
            waypoints.append(intermediate_pose)
        
        # Compute cartesian path
        (success, trajectory, fraction) = self.compute_cartesian_path(waypoints)
        
        if not success or fraction < 0.9:
            self.publish_arm_status("Attempting joint-space planning instead")
            target_pose = Pose()
            target_pose.orientation = current_pose.orientation
            target_pose.position.x = current_pose.position.x
            target_pose.position.y = current_pose.position.y
            target_pose.position.z = current_pose.position.z + 0.3
            
            success = self.send_move_group_goal(self.arm_group_name, target_pose)
        
        if success:
            self.publish_arm_status("Trajectory execution completed successfully")
        else:
            self.publish_arm_status("Trajectory execution failed")
        
        return success
    
    def moving_to_drop_location(self):
        target_pose = Pose()
        target_pose.orientation.x = 0.509
        target_pose.orientation.y = -0.491
        target_pose.orientation.z = -0.494
        target_pose.orientation.w = 0.506
        target_pose.position.x = -0.603
        target_pose.position.y = -0.139
        target_pose.position.z = 0.650
        
        self.publish_arm_status("Planning motion to drop location")
        success = self.send_move_group_goal(self.arm_group_name, target_pose)
        
        if success:
            self.publish_arm_status("Trajectory execution completed successfully")
        else:
            self.publish_arm_status("Trajectory execution failed")
        
        return success
    
    def move_down_to_drop_position(self):
        from moveit_msgs.msg import RobotTrajectory
        from geometry_msgs.msg import Pose
        
        self.publish_arm_status("Moving down to drop position")
        
        # Get current pose
        current_pose = self.get_current_pose()
        
        # Create waypoints for cartesian path (30 cm down)
        waypoints = []
        num_segments = 10
        step_size = 0.3 / num_segments
        
        for i in range(1, num_segments + 1):
            intermediate_pose = Pose()
            intermediate_pose.orientation = current_pose.orientation
            intermediate_pose.position.x = current_pose.position.x
            intermediate_pose.position.y = current_pose.position.y
            intermediate_pose.position.z = current_pose.position.z - (step_size * i)
            waypoints.append(intermediate_pose)
        
        # Compute cartesian path
        (success, trajectory, fraction) = self.compute_cartesian_path(waypoints)
        
        if not success or fraction < 0.9:
            self.publish_arm_status("Attempting joint-space planning instead")
            target_pose = Pose()
            target_pose.orientation = current_pose.orientation
            target_pose.position.x = current_pose.position.x
            target_pose.position.y = current_pose.position.y
            target_pose.position.z = current_pose.position.z - 0.3
            
            success = self.send_move_group_goal(self.arm_group_name, target_pose)
        
        if success:
            self.publish_arm_status("Trajectory execution completed successfully")
        else:
            self.publish_arm_status("Trajectory execution failed")
        
        return success
    
    def open_gripper_fully(self):
        self.publish_gripper_status("Planning gripper opening to fully release object")
        success = self.send_move_group_goal(
            self.gripper_group_name,
            {"robotiq_85_left_knuckle_joint": 0.0},
            is_pose_target=False
        )
        
        if success:
            self.publish_gripper_status("Gripper fully opened successfully - object released")
        else:
            self.publish_gripper_status("Failed to fully open gripper for object release")
        
        return success
    
    def move_up_from_drop_position(self):
        from moveit_msgs.msg import RobotTrajectory
        from geometry_msgs.msg import Pose
        
        self.publish_arm_status("Moving up from drop position")
        
        # Get current pose
        current_pose = self.get_current_pose()
        
        # Create waypoints for cartesian path (30 cm up)
        waypoints = []
        num_segments = 10
        step_size = 0.3 / num_segments
        
        for i in range(1, num_segments + 1):
            intermediate_pose = Pose()
            intermediate_pose.orientation = current_pose.orientation
            intermediate_pose.position.x = current_pose.position.x
            intermediate_pose.position.y = current_pose.position.y
            intermediate_pose.position.z = current_pose.position.z + (step_size * i)
            waypoints.append(intermediate_pose)
        
        # Compute cartesian path
        (success, trajectory, fraction) = self.compute_cartesian_path(waypoints)
        
        if not success or fraction < 0.9:
            self.publish_arm_status("Attempting joint-space planning instead")
            target_pose = Pose()
            target_pose.orientation = current_pose.orientation
            target_pose.position.x = current_pose.position.x
            target_pose.position.y = current_pose.position.y
            target_pose.position.z = current_pose.position.z + 0.3
            
            success = self.send_move_group_goal(self.arm_group_name, target_pose)
        
        if success:
            self.publish_arm_status("Trajectory execution completed successfully")
        else:
            self.publish_arm_status("Trajectory execution failed")
        
        return success

def main(args=None):
    rclpy.init(args=args)
    
    try:
        # Create and set up a multi-threaded executor for improved performance
        executor = MultiThreadedExecutor()
        
        # Create the node
        pick_place_node = PickPlaceNode()
        executor.add_node(pick_place_node)
        
        # Start the executor in a separate thread
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()
        
        # Wait a moment for the executor to start properly
        time.sleep(1)
        
        pick_place_node.get_logger().info("Initializing pick and place node...")
        
        # Execute the pick and place task
        pick_place_node.execute_pick_and_place_task()
        
        # Clean up
        executor.shutdown()
        executor_thread.join()
        
    except Exception as e:
        pick_place_node.get_logger().error(f"Exception during pick and place execution: {str(e)}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()