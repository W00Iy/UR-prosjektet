#!/usr/bin/env python3

import cv2 as cv
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class SimpleCameraPublisherNode(Node):
    def __init__(self):
        super().__init__("simple_camera_publisher")

        # ---------------- USER SETTINGS ----------------

        self.camera_device = "/dev/video4"
        self.image_topic = "/camera/image_raw"
        self.publish_fps = 30.0

        # Optional camera settings. Set to None to leave unchanged.
        self.frame_width = 640
        self.frame_height = 480

        # ---------------- ROS INTERFACE ----------------

        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, self.image_topic, 10)

        # ---------------- CAMERA ----------------

        self.cap = cv.VideoCapture(self.camera_device, cv.CAP_V4L2)

        if not self.cap.isOpened():
            self.get_logger().error(f"Could not open camera: {self.camera_device}")
            raise RuntimeError(f"Could not open camera: {self.camera_device}")

        if self.frame_width is not None:
            self.cap.set(cv.CAP_PROP_FRAME_WIDTH, float(self.frame_width))

        if self.frame_height is not None:
            self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, float(self.frame_height))

        timer_period = 1.0 / self.publish_fps
        self.timer = self.create_timer(timer_period, self.publish_frame)

        self.get_logger().info("Simple camera publisher started")
        self.get_logger().info(f"Camera device: {self.camera_device}")
        self.get_logger().info(f"Publishing images on: {self.image_topic}")
        self.get_logger().info(f"Publish FPS: {self.publish_fps}")

    def publish_frame(self):
        success, frame = self.cap.read()

        if not success or frame is None:
            self.get_logger().warn("Failed to read frame from camera")
            return

        try:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"cv_bridge conversion failed: {e}")
            return

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_frame"

        self.image_pub.publish(msg)

    def destroy_node(self):
        if hasattr(self, "cap") and self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = None

    try:
        node = SimpleCameraPublisherNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if node is not None:
            node.get_logger().error(f"Node crashed: {e}")
        else:
            print(f"Node crashed before startup: {e}")
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
