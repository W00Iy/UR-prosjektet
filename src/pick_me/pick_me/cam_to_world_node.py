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

    def camera_to_point(self, pixel_ray, z_world):
        tf = self.lookup_transform_latest(
            self.base_frame,
            self.camera_frame,
            timeout_seconds=1.0,
        )

        if tf is None:
            self.get_logger().error("Could not convert from camera to world point")
            return None

        # Estimate target plane height.
        z = z_world - self.cube_height

        # Camera offset relative to tool frame.
        tool_to_cam = np.array([
            [1.0, 0.0, 0.0, 0.09],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ])

        world_to_camera = self.transform_to_matrix(tf) @ tool_to_cam

        PI_tilde = np.vstack((
            z * np.eye(3),
            np.array([[0.0, 0.0, 1.0]]),
        ))

        try:
            p_world = world_to_camera @ PI_tilde @ np.linalg.inv(self.K) @ pixel_ray
            return p_world

        except np.linalg.LinAlgError:
            self.get_logger().error("Camera matrix is not invertible")
            return None

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

        if self.sent_goal:
            return

        if self.target_color is None:
            return

        if self.latest_cam is None:
            self.get_logger().warn("No camera detections received yet")
            return

        translation, rotation = self.get_current_ee_pose()

        if translation is None:
            self.get_logger().error("Translation does not exist")
            return

        if rotation is None:
            self.get_logger().error("Rotation does not exist")
            return

        detections = self.latest_cam.get("detections", {})
        detection = detections.get(self.target_color)

        if detection is None:
            self.get_logger().warn(
                f"Did not find {self.target_color} in detections"
            )
            return

        pixel = self.get_detection_pixel(detection)

        if pixel is None:
            self.get_logger().error(
                f"Detection for {self.target_color} does not contain valid x/y pixel data: {detection}"
            )
            return

        x, y = pixel

        pixel_ray = np.array([
            [x],
            [y],
            [1.0],
        ])

        p_world = self.camera_to_point(pixel_ray, translation.z)

        if p_world is None:
            self.get_logger().error("World point does not exist")
            return

        goal_pose = PoseStamped()
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.header.frame_id = self.base_frame

        goal_pose.pose.position.x = float(p_world[0, 0])
        goal_pose.pose.position.y = float(p_world[1, 0])

        goal_pose.pose.position.z = float(translation.z)

        goal_pose.pose.orientation.x = rotation.x
        goal_pose.pose.orientation.y = rotation.y
        goal_pose.pose.orientation.z = rotation.z
        goal_pose.pose.orientation.w = rotation.w

        self.goal_publisher.publish(goal_pose)
        self.publish_target_marker(goal_pose, self.target_color)

        self.sent_goal = True
        self.goal_requested = False

        self.get_logger().info(
            f"Published /goal_pose for {self.target_color}: "
            f"x={goal_pose.pose.position.x:.3f}, "
            f"y={goal_pose.pose.position.y:.3f}, "
            f"z={goal_pose.pose.position.z:.3f}"
        )

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