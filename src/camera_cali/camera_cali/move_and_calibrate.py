import rclpy
import os
import time
import subprocess

from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint
from shape_msgs.msg import SolidPrimitive
from rclpy.action import ActionClient


class MoveAndCalibrate(Node):
    def __init__(self):
        super().__init__("move_and_calibrate")

        self.client = ActionClient(self, MoveGroup, "/move_action")
        self.script_dir = os.path.dirname(os.path.abspath(__file__))

    def move_to_pose(self, x, y, z):
        self.client.wait_for_server()

        goal = MoveGroup.Goal()

        request = MotionPlanRequest()
        request.group_name = "ur_manipulator"
        request.num_planning_attempts = 10
        request.allowed_planning_time = 5.0
        request.max_velocity_scaling_factor = 0.2
        request.max_acceleration_scaling_factor = 0.2

        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0

        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = "base_link"
        position_constraint.link_name = "tool0"

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.01, 0.01, 0.01]

        position_constraint.constraint_region.primitives.append(box)
        position_constraint.constraint_region.primitive_poses.append(pose.pose)
        position_constraint.weight = 1.0

        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = "base_link"
        orientation_constraint.link_name = "tool0"
        orientation_constraint.orientation = pose.pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = 0.2
        orientation_constraint.absolute_y_axis_tolerance = 0.2
        orientation_constraint.absolute_z_axis_tolerance = 3.14
        orientation_constraint.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(position_constraint)
        constraints.orientation_constraints.append(orientation_constraint)

        request.goal_constraints.append(constraints)
        goal.request = request

        self.get_logger().info(f"Går til x={x}, y={y}, z={z}")

        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("MoveGroup goal ble avvist")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result

        if result.error_code.val == 1:
            self.get_logger().info("Bevegelse fullført")
            return True

        self.get_logger().error(f"MoveGroup feilet med error_code: {result.error_code.val}")
        return False

    def run_get_image(self):
        script = os.path.join(self.script_dir, "getImages.py")
        subprocess.run(["python3", script], check=True)

    def run_calibration(self):
        script = os.path.join(self.script_dir, "calibration.py")
        subprocess.run(["python3", script], check=True)

    def run(self):
        positions = [
            (0.45, -0.10, 0.30),
            (0.45,  0.10, 0.30),
            (0.60, -0.10, 0.30),
            (0.60,  0.10, 0.30),
        ]

        for i, (x, y, z) in enumerate(positions):
            self.get_logger().info(f"Posisjon {i + 1}/4")

            success = self.move_to_pose(x, y, z)

            if success:
                time.sleep(1.0)
                self.get_logger().info("Tar bilde")
                self.run_get_image()
            else:
                self.get_logger().warn("Hopper over bilde")

        self.get_logger().info("Starter kalibrering")
        self.run_calibration()
        self.get_logger().info("Ferdig")


def main():
    rclpy.init()
    node = MoveAndCalibrate()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()