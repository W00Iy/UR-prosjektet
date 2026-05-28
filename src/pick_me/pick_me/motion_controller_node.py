import json
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Pose

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    PlanningOptions,
    Constraints,
    JointConstraint,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
    AttachedCollisionObject,
    CollisionObject,
    MoveItErrorCodes,
)

from shape_msgs.msg import SolidPrimitive
from visualization_msgs.msg import Marker, MarkerArray


class MotionController(Node):
    def __init__(self):
        super().__init__("motion_controller")

        self.group_name = "ur_manipulator"
        self.tool_link = "ur5e_tool0"
        self.base_frame = "ur5e_base_link"

        self.controller_name = "scaled_joint_trajectory_controller"

        self.joint_names = [
            "ur5e_shoulder_pan_joint",
            "ur5e_shoulder_lift_joint",
            "ur5e_elbow_joint",
            "ur5e_wrist_1_joint",
            "ur5e_wrist_2_joint",
            "ur5e_wrist_3_joint",
        ]

        # Adjust these if your SRDF "home" is different.
        self.home_joint_positions = {
            "ur5e_shoulder_pan_joint": 0.7853981634,      # 45°
            "ur5e_shoulder_lift_joint": -1.0297442587,   # -59°
            "ur5e_elbow_joint": -1.5707963268,           # -90°
            "ur5e_wrist_1_joint": -2.0943951024,         # -120°
            "ur5e_wrist_2_joint": 1.5707963268,          # 90°
            "ur5e_wrist_3_joint": 0.0,                   # 0°
        }

        # Extended-search poses are stored here, in the motion controller.
        # The color-picker node only sends: {"command": "search_next_pose"}
        # Format: (x, y, z) in self.base_frame.
        self.extended_search_positions = [
            (-0.218, -0.375, 0.408),
            (-0.276, -0.318, 0.410),
            (-0.172, -0.211, 0.411),
            (-0.113, -0.268, 0.409),
            (-0.141, -0.239, 0.468),
            (-0.245, -0.346, 0.468),
        ]
        self.current_search_pose_index = 0

        # Default orientation for search poses.
        # Change this quaternion if your camera/tool needs a different fixed orientation.
        self.search_orientation = {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0,
            "w": 1.0,
        }

        self.busy = False
        self.saved_poses = []
        self.max_saved_poses = 3

        self.motion_status_pub = self.create_publisher(
            String,
            "/cube_robot/motion_status",
            10,
        )

        self.marker_pub = self.create_publisher(
            MarkerArray,
            "/saved_goal_poses",
            10,
        )

        self.search_marker_pub = self.create_publisher(
            MarkerArray,
            "/extended_search_poses",
            10,
        )

        self.attached_collision_pub = self.create_publisher(
            AttachedCollisionObject,
            "/attached_collision_object",
            10,
        )

        self.pose_subscriber = self.create_subscription(
            PoseStamped,
            "/goal_pose",
            self.subscriber_callback,
            10,
        )

        self.command_subscriber = self.create_subscription(
            String,
            "/cube_robot/command",
            self.command_callback,
            10,
        )

        self.move_group_client = ActionClient(
            self,
            MoveGroup,
            "/move_action",
        )

        self.get_logger().info("Waiting for /move_action server...")
        self.move_group_client.wait_for_server()
        self.get_logger().info("Connected to /move_action")

        self.publish_extended_search_pose_markers()

        # Publish camera collision object shortly after startup.
        # self.camera_mount_timer = self.create_timer(
        #     1.0,
        #     self.publish_camera_mount_once,
        # )

        self.get_logger().info("Motion controller started without MoveItPy")

    def publish_camera_mount_once(self):
        self.camera_mount_timer.cancel()

        try:
            attached_camera_mount = AttachedCollisionObject()
            attached_camera_mount.link_name = self.tool_link
            attached_camera_mount.touch_links = [
                "ur5e_tool0",
                "ur5e_flange",
                "ur5e_wrist_3_link",
            ]

            camera_mount = CollisionObject()
            camera_mount.header.frame_id = self.tool_link
            camera_mount.id = "camera_mount"

            mount_pose = Pose()
            mount_pose.position.x = 0.06
            mount_pose.position.y = 0.0
            mount_pose.position.z = 0.005
            mount_pose.orientation.w = 1.0

            mount = SolidPrimitive()
            mount.type = SolidPrimitive.BOX
            mount.dimensions = [0.12, 0.08, 0.01]

            camera_mount.primitives.append(mount)
            camera_mount.primitive_poses.append(mount_pose)
            camera_mount.operation = CollisionObject.ADD

            attached_camera_mount.object = camera_mount

            self.attached_collision_pub.publish(attached_camera_mount)

            self.get_logger().info(
                f"Published attached camera_mount collision object on {self.tool_link}"
            )

        except Exception as e:
            self.get_logger().error(f"Failed to publish camera mount: {e}")

    def publish_motion_status(self, status):
        msg = String()
        msg.data = json.dumps({"status": status})
        self.motion_status_pub.publish(msg)

    def make_common_goal(self):
        goal = MoveGroup.Goal()

        request = MotionPlanRequest()
        request.group_name = self.group_name
        request.num_planning_attempts = 10
        request.allowed_planning_time = 5.0
        request.max_velocity_scaling_factor = 0.1
        request.max_acceleration_scaling_factor = 0.1

        planning_options = PlanningOptions()
        planning_options.plan_only = False
        planning_options.look_around = False
        planning_options.replan = True
        planning_options.replan_attempts = 3
        planning_options.planning_scene_diff.is_diff = True
        planning_options.planning_scene_diff.robot_state.is_diff = True

        goal.request = request
        goal.planning_options = planning_options

        return goal

    def make_home_goal(self):
        goal = self.make_common_goal()

        constraints = Constraints()
        constraints.name = "home"

        for joint_name, position in self.home_joint_positions.items():
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = float(position)
            joint_constraint.tolerance_above = 0.01
            joint_constraint.tolerance_below = 0.01
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)

        goal.request.goal_constraints.append(constraints)

        return goal

    def make_pose_goal(self, pose_stamped: PoseStamped):
        goal = self.make_common_goal()

        constraints = Constraints()
        constraints.name = "pose_goal"

        position_constraint = PositionConstraint()
        position_constraint.header = pose_stamped.header
        position_constraint.link_name = self.tool_link
        position_constraint.weight = 1.0

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [
            0.01,
            0.01,
            0.01,
        ]

        region = BoundingVolume()
        region.primitives.append(box)
        region.primitive_poses.append(pose_stamped.pose)

        position_constraint.constraint_region = region

        orientation_constraint = OrientationConstraint()
        orientation_constraint.header = pose_stamped.header
        orientation_constraint.link_name = self.tool_link
        orientation_constraint.orientation = pose_stamped.pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = math.radians(5.0)
        orientation_constraint.absolute_y_axis_tolerance = math.radians(5.0)
        orientation_constraint.absolute_z_axis_tolerance = math.radians(5.0)
        orientation_constraint.weight = 1.0

        constraints.position_constraints.append(position_constraint)
        constraints.orientation_constraints.append(orientation_constraint)

        goal.request.goal_constraints.append(constraints)

        return goal

    def make_search_pose_stamped(self, position_tuple):
        x, y, z = position_tuple

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self.base_frame
        pose_stamped.header.stamp = self.get_clock().now().to_msg()

        pose_stamped.pose.position.x = float(x)
        pose_stamped.pose.position.y = float(y)
        pose_stamped.pose.position.z = float(z)

        pose_stamped.pose.orientation.x = float(self.search_orientation["x"])
        pose_stamped.pose.orientation.y = float(self.search_orientation["y"])
        pose_stamped.pose.orientation.z = float(self.search_orientation["z"])
        pose_stamped.pose.orientation.w = float(self.search_orientation["w"])

        return pose_stamped

    def send_move_group_goal(self, goal, description):
        if self.busy:
            self.get_logger().warn("Robot is busy, ignoring command")
            return

        self.busy = True
        self.publish_motion_status("moving")

        self.get_logger().info(f"Sending MoveGroup goal: {description}")

        send_future = self.move_group_client.send_goal_async(goal)
        send_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f"Failed to send MoveGroup goal: {e}")
            self.publish_motion_status("failed")
            self.busy = False
            return

        if not goal_handle.accepted:
            self.get_logger().error("MoveGroup goal was rejected")
            self.publish_motion_status("failed")
            self.busy = False
            return

        self.get_logger().info("MoveGroup goal accepted")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        try:
            result = future.result().result
        except Exception as e:
            self.get_logger().error(f"MoveGroup result failed: {e}")
            self.publish_motion_status("failed")
            self.busy = False
            return

        error_code = result.error_code.val

        if error_code == MoveItErrorCodes.SUCCESS:
            self.get_logger().info("Motion completed successfully")
            self.publish_motion_status("done")
        else:
            self.get_logger().error(f"MoveGroup failed with error code: {error_code}")
            self.publish_motion_status("failed")

        self.busy = False

    def execute_home_motion(self):
        # Reset the extended-search sequence whenever a new search starts from home.
        self.current_search_pose_index = 0
        goal = self.make_home_goal()
        self.send_move_group_goal(goal, "home")

    def execute_next_search_pose(self):
        if not self.extended_search_positions:
            self.get_logger().error("No extended search positions are configured")
            self.publish_motion_status("failed")
            return

        if self.current_search_pose_index >= len(self.extended_search_positions):
            self.get_logger().error("No more extended search poses left")
            self.publish_motion_status("failed")
            return

        pose_index = self.current_search_pose_index
        position = self.extended_search_positions[pose_index]
        self.current_search_pose_index += 1

        pose_stamped = self.make_search_pose_stamped(position)
        goal = self.make_pose_goal(pose_stamped)

        self.get_logger().info(
            f"Moving to extended search pose {pose_index + 1}/{len(self.extended_search_positions)}: "
            f"x={position[0]}, y={position[1]}, z={position[2]}"
        )

        self.send_move_group_goal(goal, f"extended search pose {pose_index + 1}")

    def command_callback(self, msg):
        try:
            command_data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error("Could not parse /cube_robot/command JSON")
            self.publish_motion_status("failed")
            return

        command = command_data.get("command")

        if command == "go_home":
            self.execute_home_motion()

        elif command == "search_next_pose":
            self.execute_next_search_pose()

        else:
            self.get_logger().warn(f"Unsupported command: {command}")

    def subscriber_callback(self, goal_pose):
        goal = self.make_pose_goal(goal_pose)
        self.send_move_group_goal(goal, "pose goal")

    def publish_saved_pose_markers(self):
        marker_array = MarkerArray()

        for i, pose_stamped in enumerate(self.saved_poses):
            sphere = Marker()
            sphere.header.frame_id = pose_stamped.header.frame_id
            sphere.header.stamp = self.get_clock().now().to_msg()
            sphere.ns = "saved_goal_poses"
            sphere.id = i * 2
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose = pose_stamped.pose
            sphere.scale.x = 0.04
            sphere.scale.y = 0.04
            sphere.scale.z = 0.04
            sphere.color.a = 1.0

            if i == 0:
                sphere.color.r = 1.0
                sphere.color.g = 0.0
                sphere.color.b = 0.0
            elif i == 1:
                sphere.color.r = 1.0
                sphere.color.g = 1.0
                sphere.color.b = 0.0
            else:
                sphere.color.r = 0.0
                sphere.color.g = 0.0
                sphere.color.b = 1.0

            marker_array.markers.append(sphere)

            text = Marker()
            text.header.frame_id = pose_stamped.header.frame_id
            text.header.stamp = self.get_clock().now().to_msg()
            text.ns = "saved_goal_pose_labels"
            text.id = i * 2 + 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose = pose_stamped.pose
            text.pose.position.z += 0.07
            text.scale.z = 0.04
            text.color.a = 1.0
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.text = f"Pose {i + 1}"

            marker_array.markers.append(text)

        self.marker_pub.publish(marker_array)

    def publish_extended_search_pose_markers(self):
        marker_array = MarkerArray()

        for i, position in enumerate(self.extended_search_positions):
            pose_stamped = self.make_search_pose_stamped(position)

            sphere = Marker()
            sphere.header.frame_id = self.base_frame
            sphere.header.stamp = self.get_clock().now().to_msg()
            sphere.ns = "extended_search_poses"
            sphere.id = i * 2
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose = pose_stamped.pose
            sphere.scale.x = 0.035
            sphere.scale.y = 0.035
            sphere.scale.z = 0.035
            sphere.color.a = 1.0
            sphere.color.r = 0.0
            sphere.color.g = 1.0
            sphere.color.b = 1.0
            marker_array.markers.append(sphere)

            text = Marker()
            text.header.frame_id = self.base_frame
            text.header.stamp = self.get_clock().now().to_msg()
            text.ns = "extended_search_pose_labels"
            text.id = i * 2 + 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose = pose_stamped.pose
            text.pose.position.z += 0.06
            text.scale.z = 0.035
            text.color.a = 1.0
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.text = f"Search {i + 1}"
            marker_array.markers.append(text)

        self.search_marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)

    node = MotionController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
