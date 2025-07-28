#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from moveit_msgs.msg import PlanningScene, CollisionObject, AttachedCollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from std_msgs.msg import Header

class PipeCollisionManager(Node):
    def __init__(self):
        super().__init__('pipe_collision_manager')

        # Publisher for planning scene on monitored_planning_scene topic
        self.planning_scene_pub = self.create_publisher(
            PlanningScene,
            '/monitored_planning_scene',
            10
        )

        self.pipe_name = "pipe_1"
        self.eef_link = "wrist_3_link"  # Change to your end-effector link name
        self.robot_frame = "world"       # Change to your robot's global frame if different

        self.is_pipe_attached = False
        self.is_pipe_added = False

        self.get_logger().info("PipeCollisionManager node started.")

    def add_pipe(self):
        self.get_logger().info("Adding pipe to planning scene...")

        co = CollisionObject()
        co.id = self.pipe_name
        co.header.frame_id = self.robot_frame

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.CYLINDER
        primitive.dimensions = [0.5, 0.05]  # [height(length), radius]

        co.primitives = [primitive]

        pose = Pose()
        pose.position.x = 0.70
        pose.position.y = 0.15
        pose.position.z = 0.015
        pose.orientation.x = 0.0
        pose.orientation.y = -0.707
        pose.orientation.z = 0.0
        pose.orientation.w = -0.707

        co.primitive_poses = [pose]

        co.operation = CollisionObject.ADD

        ps = PlanningScene()
        ps.is_diff = True
        ps.world.collision_objects = [co]

        self.planning_scene_pub.publish(ps)
        self.is_pipe_added = True

        self.get_logger().info("Pipe added.")

    def attach_pipe(self):
        if not self.is_pipe_added:
            self.get_logger().warn("Pipe not added yet. Cannot attach.")
            return

        self.get_logger().info("Attaching pipe to end-effector...")

        aco = AttachedCollisionObject()
        aco.link_name = self.eef_link
        aco.object.id = self.pipe_name
        aco.object.operation = CollisionObject.ADD
        aco.object.header.frame_id = self.robot_frame

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.CYLINDER
        primitive.dimensions = [0.5, 0.05]

        aco.object.primitives = [primitive]

        pose = Pose()
        pose.position.x = 0.60
        pose.position.y = 0.15
        pose.position.z = 0.015
        pose.orientation.x = 0.0
        pose.orientation.y = -0.707
        pose.orientation.z = 0.0
        pose.orientation.w = -0.707
        
        aco.object.primitive_poses = [pose]

        ps = PlanningScene()
        ps.is_diff = True
        ps.robot_state.attached_collision_objects = [aco]
        ps.world.collision_objects = []  # remove from world, now attached

        self.planning_scene_pub.publish(ps)
        self.is_pipe_attached = True
        self.get_logger().info("Pipe attached.")

    def detach_pipe(self):
        if not self.is_pipe_attached:
            self.get_logger().warn("Pipe not attached yet. Cannot detach.")
            return

        self.get_logger().info("Detaching pipe from end-effector...")

        aco = AttachedCollisionObject()
        aco.link_name = self.eef_link
        aco.object.id = self.pipe_name
        aco.object.operation = CollisionObject.REMOVE

        ps = PlanningScene()
        ps.is_diff = True
        ps.robot_state.attached_collision_objects = [aco]

        self.planning_scene_pub.publish(ps)
        self.is_pipe_attached = False
        self.get_logger().info("Pipe detached.")

    def remove_pipe(self):
        if not self.is_pipe_added:
            self.get_logger().warn("Pipe not added yet. Cannot remove.")
            return

        self.get_logger().info("Removing pipe from planning scene...")

        co = CollisionObject()
        co.id = self.pipe_name
        co.operation = CollisionObject.REMOVE
        co.header.frame_id = self.robot_frame

        ps = PlanningScene()
        ps.is_diff = True
        ps.world.collision_objects = [co]

        self.planning_scene_pub.publish(ps)
        self.is_pipe_added = False
        self.get_logger().info("Pipe removed.")

def main(args=None):
    rclpy.init(args=args)
    node = PipeCollisionManager()

    try:
        while rclpy.ok():
            cmd = input("Enter command (add, attach, detach, remove, exit): ").strip()
            if cmd == "add":
                node.add_pipe()
            elif cmd == "attach":
                node.attach_pipe()
            elif cmd == "detach":
                node.detach_pipe()
            elif cmd == "remove":
                node.remove_pipe()
            elif cmd == "exit":
                break
            else:
                node.get_logger().warn("Unknown command")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
