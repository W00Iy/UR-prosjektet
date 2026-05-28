import json
import math
import os
import pickle
import threading
import time

import cv2 as cv
import numpy as np
import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    PlanningOptions,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
    MoveItErrorCodes,
)

from shape_msgs.msg import SolidPrimitive


class SimpleCameraCalibrationNode(Node):
    def __init__(self):
        super().__init__("camera_calibration_node")

        # ---------------- USER SETTINGS ----------------

        self.image_topic = "/camera/image_raw"

        self.group_name = "ur_manipulator"
        self.base_frame = "ur5e_base_link"
        self.tool_link = "ur5e_tool0"

        self.chessboard_size = (7, 5)  # inner corners: columns, rows
        self.square_size_mm = 20.0

        self.capture_delay_seconds = 1.0

        self.save_debug_files = True
        self.output_dir = os.path.join(
            os.path.expanduser("~"),
            "ros2_ws",
            "camera_calibration_output",
        )

        # These are example calibration poses.
        # Adjust them to safe/reachable poses where the chessboard is visible.
        self.calibration_poses = [
            (-0.218, -0.375, 0.408),
            (-0.276, -0.318, 0.410),
            (-0.172, -0.211, 0.411),
            (-0.113, -0.268, 0.409),
            (-0.141, -0.239, 0.468),
            (-0.245, -0.346, 0.468),
        ]

        # Keep a simple fixed tool orientation for now.
        # If planning fails, change these to match a known good RViz pose.
        self.goal_orientation = {
            "x": 0.3,
            "y": 0.707,
            "z": 0.0,
            "w": 0.0,
        }

        # ---------------- ROS INTERFACE ----------------

        self.command_sub = self.create_subscription(
            String,
            "/camera_calibration/command",
            self.command_callback,
            10,
        )

        self.result_pub = self.create_publisher(
            String,
            "/camera_calibration/result",
            10,
        )

        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_image_time = None
        self.image_lock = threading.Lock()

        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10,
        )

        self.move_group_client = ActionClient(
            self,
            MoveGroup,
            "/move_action",
        )

        self.busy = False

        self.get_logger().info("Simple camera calibration node started")
        self.get_logger().info(f"Subscribing to camera topic: {self.image_topic}")
        self.get_logger().info("Send command on /camera_calibration/command:")
        self.get_logger().info('  {"command": "calibrate"}')

    # ---------------- IMAGE TOPIC ----------------

    def image_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"Could not convert image message: {e}")
            return

        with self.image_lock:
            self.latest_image = image.copy()
            self.latest_image_time = self.get_clock().now()

    # ---------------- COMMAND HANDLING ----------------

    def command_callback(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.publish_result(
                success=False,
                message="Could not parse command JSON",
            )
            return

        command = data.get("command")

        if command != "calibrate":
            self.publish_result(
                success=False,
                message=f"Unsupported command: {command}",
            )
            return

        if self.busy:
            self.publish_result(
                success=False,
                message="Calibration is already running",
            )
            return

        self.busy = True

        thread = threading.Thread(target=self.run_calibration_sequence)
        thread.daemon = True
        thread.start()

    # ---------------- MAIN SEQUENCE ----------------

    def run_calibration_sequence(self):
        try:
            self.get_logger().info("Starting calibration sequence")

            os.makedirs(self.output_dir, exist_ok=True)

            captured_images = []

            for i, (x, y, z) in enumerate(self.calibration_poses):
                self.get_logger().info(
                    f"Calibration pose {i + 1}/{len(self.calibration_poses)}: "
                    f"x={x:.3f}, y={y:.3f}, z={z:.3f}"
                )

                move_ok = self.move_to_pose(x, y, z)

                if not move_ok:
                    self.get_logger().warn(
                        f"Skipping image at pose {i + 1} because motion failed"
                    )
                    continue

                time.sleep(self.capture_delay_seconds)

                image = self.capture_image()

                if image is None:
                    self.get_logger().warn(f"Skipping pose {i + 1}: no image captured")
                    continue

                captured_images.append(image)

                if self.save_debug_files:
                    image_path = os.path.join(self.output_dir, f"calibration_{i:02d}.png")
                    cv.imwrite(image_path, image)
                    self.get_logger().info(f"Saved image: {image_path}")

            if len(captured_images) == 0:
                self.publish_result(
                    success=False,
                    message="No images captured",
                )
                return

            calibration_result = self.calibrate_from_images(captured_images)

            if calibration_result is None:
                self.publish_result(
                    success=False,
                    message="Calibration failed: no valid chessboard images",
                )
                return

            camera_matrix, dist, reprojection_error, valid_images = calibration_result

            if self.save_debug_files:
                with open(os.path.join(self.output_dir, "cameraMatrix.pkl"), "wb") as f:
                    pickle.dump(camera_matrix, f)

                with open(os.path.join(self.output_dir, "dist.pkl"), "wb") as f:
                    pickle.dump(dist, f)

                with open(os.path.join(self.output_dir, "calibration.pkl"), "wb") as f:
                    pickle.dump((camera_matrix, dist), f)

            self.publish_result(
                success=True,
                message="Calibration complete",
                k_matrix=camera_matrix.tolist(),
                distortion=dist.flatten().tolist(),
                reprojection_error=float(reprojection_error),
                valid_images=int(valid_images),
            )

            self.get_logger().info("Calibration complete")
            self.get_logger().info(f"K matrix:\n{camera_matrix}")
            self.get_logger().info(f"Distortion:\n{dist}")
            self.get_logger().info(f"Reprojection error: {reprojection_error}")

        except Exception as e:
            self.get_logger().error(f"Calibration crashed: {e}")
            self.publish_result(
                success=False,
                message=f"Calibration crashed: {e}",
            )

        finally:
            self.busy = False

    # ---------------- ROBOT MOTION ----------------

    def move_to_pose(self, x, y, z):
        self.get_logger().info("Waiting for /move_action server...")

        if not self.move_group_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("/move_action server not available")
            return False

        goal = self.make_move_group_goal(x, y, z)

        send_future = self.move_group_client.send_goal_async(goal)

        if not self.wait_for_future(send_future, timeout_seconds=10.0):
            self.get_logger().error("Timed out sending MoveGroup goal")
            return False

        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("MoveGroup goal rejected")
            return False

        self.get_logger().info("MoveGroup goal accepted")

        result_future = goal_handle.get_result_async()

        if not self.wait_for_future(result_future, timeout_seconds=30.0):
            self.get_logger().error("Timed out waiting for MoveGroup result")
            return False

        result = result_future.result().result

        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            self.get_logger().info("Motion completed successfully")
            return True

        self.get_logger().error(
            f"MoveGroup failed with error code: {result.error_code.val}"
        )
        return False

    def make_move_group_goal(self, x, y, z):
        goal = MoveGroup.Goal()

        request = MotionPlanRequest()
        request.group_name = self.group_name
        request.num_planning_attempts = 10
        request.allowed_planning_time = 5.0
        request.max_velocity_scaling_factor = 0.1
        request.max_acceleration_scaling_factor = 0.1

        pose = PoseStamped()
        pose.header.frame_id = self.base_frame
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        pose.pose.orientation.x = self.goal_orientation["x"]
        pose.pose.orientation.y = self.goal_orientation["y"]
        pose.pose.orientation.z = self.goal_orientation["z"]
        pose.pose.orientation.w = self.goal_orientation["w"]

        constraints = Constraints()
        constraints.name = "calibration_pose"

        position_constraint = PositionConstraint()
        position_constraint.header = pose.header
        position_constraint.link_name = self.tool_link
        position_constraint.weight = 1.0

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.03, 0.03, 0.03]

        region = BoundingVolume()
        region.primitives.append(box)
        region.primitive_poses.append(pose.pose)

        position_constraint.constraint_region = region

        orientation_constraint = OrientationConstraint()
        orientation_constraint.header = pose.header
        orientation_constraint.link_name = self.tool_link
        orientation_constraint.orientation = pose.pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = math.radians(2.0)
        orientation_constraint.absolute_y_axis_tolerance = math.radians(2.0)
        orientation_constraint.absolute_z_axis_tolerance = math.radians(2.0)
        orientation_constraint.weight = 1.0

        constraints.position_constraints.append(position_constraint)
        constraints.orientation_constraints.append(orientation_constraint)

        request.goal_constraints.append(constraints)

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

    def wait_for_future(self, future, timeout_seconds):
        event = threading.Event()

        def callback(_):
            event.set()

        future.add_done_callback(callback)
        return event.wait(timeout=timeout_seconds)

    # ---------------- IMAGE CAPTURE ----------------

    def capture_image(self):
        # The physical camera should be opened by one camera driver/node only.
        # This node just uses the newest frame received on self.image_topic.
        deadline = time.time() + 5.0

        while time.time() < deadline:
            with self.image_lock:
                if self.latest_image is not None:
                    return self.latest_image.copy()

            time.sleep(0.05)

        self.get_logger().error(
            f"No image received on topic {self.image_topic}. "
            "Check that your camera publisher is running."
        )
        return None

    # ---------------- CALIBRATION ----------------

    def calibrate_from_images(self, images):
        criteria = (
            cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER,
            30,
            0.001,
        )

        cols, rows = self.chessboard_size

        objp = np.zeros((cols * rows, 3), np.float32)
        objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
        objp *= self.square_size_mm

        objpoints = []
        imgpoints = []

        frame_size = None

        for i, image in enumerate(images):
            gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)

            if frame_size is None:
                h, w = gray.shape[:2]
                frame_size = (w, h)

            found, corners = cv.findChessboardCorners(
                gray,
                self.chessboard_size,
                None,
            )

            self.get_logger().info(
                f"Image {i + 1}/{len(images)} chessboard found: {found}"
            )

            if not found:
                continue

            corners_refined = cv.cornerSubPix(
                gray,
                corners,
                (11, 11),
                (-1, -1),
                criteria,
            )

            objpoints.append(objp)
            imgpoints.append(corners_refined)

        valid_images = len(objpoints)

        if valid_images == 0:
            return None

        ret, camera_matrix, dist, rvecs, tvecs = cv.calibrateCamera(
            objpoints,
            imgpoints,
            frame_size,
            None,
            None,
        )

        reprojection_error = self.calculate_reprojection_error(
            objpoints,
            imgpoints,
            rvecs,
            tvecs,
            camera_matrix,
            dist,
        )

        return camera_matrix, dist, reprojection_error, valid_images

    def calculate_reprojection_error(
        self,
        objpoints,
        imgpoints,
        rvecs,
        tvecs,
        camera_matrix,
        dist,
    ):
        total_error = 0.0

        for i in range(len(objpoints)):
            projected_points, _ = cv.projectPoints(
                objpoints[i],
                rvecs[i],
                tvecs[i],
                camera_matrix,
                dist,
            )

            error = cv.norm(
                imgpoints[i],
                projected_points,
                cv.NORM_L2,
            ) / len(projected_points)

            total_error += error

        return total_error / len(objpoints)

    # ---------------- RESULT ----------------

    def publish_result(
        self,
        success,
        message,
        k_matrix=None,
        distortion=None,
        reprojection_error=None,
        valid_images=None,
    ):
        result = {
            "success": success,
            "message": message,
        }

        if k_matrix is not None:
            result["k_matrix"] = k_matrix

        if distortion is not None:
            result["distortion"] = distortion

        if reprojection_error is not None:
            result["reprojection_error"] = reprojection_error

        if valid_images is not None:
            result["valid_images"] = valid_images

        msg = String()
        msg.data = json.dumps(result)
        self.result_pub.publish(msg)

        self.get_logger().info(f"Calibration result: {msg.data}")


def main(args=None):
    rclpy.init(args=args)

    node = SimpleCameraCalibrationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()