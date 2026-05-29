import json

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseStamped, Pose

from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from tf2_ros import (
    Buffer,
    TransformListener,
    LookupException,
    ConnectivityException,
    ExtrapolationException,
)

# This node subscribes to camera detections and target color commands, transforms the detected pixel coordinates of the target object into world coordinates, 
# and publishes a goal pose for the robot to pick the object. It also publishes visualization markers for the target pose.
class CamToWorld(Node):
    def __init__(self):
        super().__init__("cam_to_pose")

        self.sent_goal = False
        self.goal_requested = False
        self.saved_targets = []
        self.current_target_index = 0
        self.snapshot_taken = False
        self.latest_cam = None
        self.target_color = None

        # Updated frames after ur5e_ prefix
        self.base_frame = "ur5e_base_link"
        self.ee_frame = "ur5e_tool0"

        self.camera_frame = "ur5e_tool0"

        # Camera matrix fallback.
        # This gets overwritten if /camera/camera_info publishes.
        self.K = np.array([
            [803.08080926841205, 0.0, 272.24043631557367],
            [0.0, 931.70126479938983, 278.03098639361787],
            [0.0, 0.0, 1.0],
        ])

        self.cube_height = 0.05 # Adjust cube height in meters
        self.table_z = 0.0
        self.approach_offset = 0.10
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.target_color_sub = self.create_subscription(
            String,
            "/cube_robot/target_color",
            self.target_color_callback,
            10,
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            "/camera_calibration/result",
            self.cam_matrix_callback,
            10,
        )

        self.detections_sub = self.create_subscription(
            String,
            "/cube_vision/detections",
            self.cam_callback,
            10,
        )

        self.goal_publisher = self.create_publisher(
            PoseStamped,
            "/goal_pose",
            10,
        )

        marker_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.marker_pub = self.create_publisher(
            MarkerArray,
            "/cube_robot/target_markers",
            marker_qos,
        )

        self.last_marker_array = None
        self.marker_republish_timer = self.create_timer(
            1.0,
            self.republish_last_marker,
        )

        self.timer = self.create_timer(0.5, self.procedure)

        self.get_logger().info("cam_to_world node started")
        self.get_logger().info(f"Using base_frame: {self.base_frame}")
        self.get_logger().info(f"Using ee_frame: {self.ee_frame}")
        self.get_logger().info(f"Using camera_frame: {self.camera_frame}")

    # Callback for receiving target color commands. Expects JSON string with format: {"color": "action": "point"}
    def target_color_callback(self, msg):
        try:
            data = json.loads(msg.data)

            color = data.get("color")
            action = data.get("action", "point")

            if action != "point":
                self.get_logger().warn(f"Unsupported target action: {action}")
                return

            if color is None:
                self.get_logger().error("Target color command did not include 'color'")
                return

            self.target_color = color
            self.goal_requested = True
            self.sent_goal = False

            self.get_logger().info(f"Received target color: {self.target_color}")

        except json.JSONDecodeError:
            self.get_logger().error("Could not parse target color command")

    def cam_matrix_callback(self, msg):
        self.K = np.array(msg.k).reshape(3, 3)
        self.get_logger().info("Updated camera matrix from /camera/camera_info")

    def cam_callback(self, msg):
        try:
            self.latest_cam = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error("Could not parse /cube_vision/detections JSON")

    # Convert a TF transform message to a 4x4 homogeneous transformation matrix.
    def transform_to_matrix(self, tf):
        t = tf.transform.translation
        q = tf.transform.rotation

        x = q.x
        y = q.y
        z = q.z
        w = q.w

        R = np.array([
            [1 - 2 * y * y - 2 * z * z, 2 * x * y - 2 * z * w,     2 * x * z + 2 * y * w],
            [2 * x * y + 2 * z * w,     1 - 2 * x * x - 2 * z * z, 2 * y * z - 2 * x * w],
            [2 * x * z - 2 * y * w,     2 * y * z + 2 * x * w,     1 - 2 * x * x - 2 * y * y],
        ])

        T = np.eye(4)
        T[0:3, 0:3] = R
        T[0, 3] = t.x
        T[1, 3] = t.y
        T[2, 3] = t.z

        return T

    def lookup_transform_latest(self, target_frame, source_frame, timeout_seconds=1.0):
        try:
            return self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time(),
                timeout=Duration(seconds=timeout_seconds),
            )

        except (LookupException, ConnectivityException, ExtrapolationException) as ex:
            self.get_logger().error(
                f"Could not lookup transform {target_frame} -> {source_frame}: {ex}"
            )
            return None
    def republish_last_marker(self):
        if self.last_marker_array is not None:
            now = self.get_clock().now().to_msg()

            for marker in self.last_marker_array.markers:
                marker.header.stamp = now

            self.marker_pub.publish(self.last_marker_array)
    def get_current_ee_pose(self):
        tf = self.lookup_transform_latest(
            self.base_frame,
            self.ee_frame,
            timeout_seconds=1.0,
        )

        if tf is None:
            return None, None

        return tf.transform.translation, tf.transform.rotation

    def camera_to_point(self, u, v, object_z):
        tf = self.lookup_transform_latest(
            self.base_frame,
            self.ee_frame,
            timeout_seconds=1.0,
        )

        if tf is None:
            return None

        T_base_tool = self.transform_to_matrix(tf)

        # Camera offset from tool
        T_tool_cam = np.eye(4)
        T_tool_cam[0, 3] = 0.09

        T_base_cam = T_base_tool @ T_tool_cam

        fx = self.K[0, 0]
        fy = self.K[1, 1]
        cx = self.K[0, 2]
        cy = self.K[1, 2]

        # Normalized ray
        x = (u - cx) / fx
        y = (v - cy) / fy

        ray_cam = np.array([x, y, 1.0])
        ray_cam = ray_cam / np.linalg.norm(ray_cam)

        R = T_base_cam[0:3, 0:3]
        t = T_base_cam[0:3, 3]

        ray_world = R @ ray_cam

        # Intersect ray with plane z = object_z
        scale = (object_z - t[2]) / ray_world[2]

        p_world = t + scale * ray_world

        return p_world

    def get_detection_pixel(self, detection):
        """
        Supports formats like:
        {"x": 123, "y": 456}
        {"pixel": {"x": 123, "y": 456}}
        {"center": {"x": 123, "y": 456}}
        """

        if not isinstance(detection, dict):
            return None

        source = detection

        if "pixel" in detection and isinstance(detection["pixel"], dict):
            source = detection["pixel"]
        elif "center" in detection and isinstance(detection["center"], dict):
            source = detection["center"]

        if "x" not in source or "y" not in source:
            return None

        try:
            x = float(source["x"])
            y = float(source["y"])
            return x, y

        except (TypeError, ValueError):
            return None

    def procedure(self):

        if not self.goal_requested:
            return

        if self.latest_cam is None:
            self.get_logger().warn("No detections")
            return

        # TAKE SNAPSHOT ONLY ONCE
        if not self.snapshot_taken:

            detections = self.latest_cam.get("detections", {})

            self.saved_targets = []

            cube_z = self.table_z + self.cube_height

            for color_name, detection in detections.items():

                pixel = self.get_detection_pixel(detection)

                if pixel is None:
                    continue

                u, v = pixel

                p_world = self.camera_to_point(u, v, cube_z)

                if p_world is None:
                    continue

                self.saved_targets.append({
                    "color": color_name,
                    "x": float(p_world[0]),
                    "y": float(p_world[1]),
                    "z": cube_z + self.approach_offset,
                })

                self.get_logger().info(
                    f"Saved {color_name} target at "
                    f"{p_world[0]:.3f}, "
                    f"{p_world[1]:.3f}, "
                    f"{cube_z:.3f}"
                )

            self.snapshot_taken = True
            self.current_target_index = 0

        # DONE
        if self.current_target_index >= len(self.saved_targets):
            self.get_logger().info("Finished all targets")
            self.goal_requested = False
            return

        # SEND NEXT TARGET
        target = self.saved_targets[self.current_target_index]

        translation, rotation = self.get_current_ee_pose()

        goal_pose = PoseStamped()
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.header.frame_id = self.base_frame

        goal_pose.pose.position.x = target["x"]
        goal_pose.pose.position.y = target["y"]
        goal_pose.pose.position.z = target["z"]

        goal_pose.pose.orientation.x = rotation.x
        goal_pose.pose.orientation.y = rotation.y
        goal_pose.pose.orientation.z = rotation.z
        goal_pose.pose.orientation.w = rotation.w

        self.goal_publisher.publish(goal_pose)

        self.publish_target_marker(goal_pose, target["color"])

        self.get_logger().info(
            f"Moving to {target['color']} cube"
        )

        self.current_target_index += 1

        # WAIT for next command cycle
        self.goal_requested = False

    def publish_target_marker(self, goal_pose, color_name):
        marker_array = MarkerArray()

        sphere = Marker()
        sphere.header = goal_pose.header
        sphere.ns = "target_color_points"
        sphere.id = 0
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose = goal_pose.pose
        sphere.scale.x = 0.05
        sphere.scale.y = 0.05
        sphere.scale.z = 0.05
        sphere.color.a = 1.0

        self.set_marker_color(sphere, color_name)

        marker_array.markers.append(sphere)

        text = Marker()
        text.header = goal_pose.header
        text.ns = "target_color_labels"
        text.id = 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose = Pose()
        text.pose.position.x = goal_pose.pose.position.x
        text.pose.position.y = goal_pose.pose.position.y
        text.pose.position.z = goal_pose.pose.position.z + 0.08
        text.pose.orientation.w = 1.0
        text.scale.z = 0.05
        text.color.a = 1.0
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 1.0
        text.text = color_name

        marker_array.markers.append(text)

        self.last_marker_array = marker_array
        self.marker_pub.publish(marker_array)

        self.get_logger().info(
            f"Published target marker for {color_name} on /cube_robot/target_markers"
        )

    def set_marker_color(self, marker, color_name):
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        if color_name == "red":
            marker.color.r = 1.0
        elif color_name == "yellow":
            marker.color.r = 1.0
            marker.color.g = 1.0
        elif color_name == "blue":
            marker.color.b = 1.0
        else:
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0


def main(args=None):
    rclpy.init(args=args)

    node = CamToWorld()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
