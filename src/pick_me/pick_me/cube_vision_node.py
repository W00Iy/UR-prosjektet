import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import cv2
import numpy as np
import json


def find_positions_by_color(img, color="red", min_area=300):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    color_ranges = {
        "red": [
            (np.array([0, 50, 50]), np.array([10, 255, 255])),
            (np.array([170, 50, 50]), np.array([180, 255, 255])),
        ],
        "blue": [
            (np.array([100, 50, 50]), np.array([130, 255, 255])),
        ],
        "yellow": [
            (np.array([20, 50, 50]), np.array([35, 255, 255])),
        ],
    }

    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)

    for lower, upper in color_ranges[color]:
        mask |= cv2.inRange(hsv, lower, upper)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )

    targets = []

    for contour in contours:
        area = cv2.contourArea(contour)

        if area < min_area:
            continue

        x, y, w, h = cv2.boundingRect(contour)
        cx = x + w // 2
        cy = y + h // 2

        targets.append(
            {
                "x": int(cx),
                "y": int(cy),
                "area": float(area),
                "box": [int(x), int(y), int(w), int(h)],
            }
        )

    targets.sort(key=lambda t: t["area"], reverse=True)

    return targets, mask


class CubeVisionNode(Node):
    def __init__(self):
        super().__init__("cube_vision_node")

        # Subscribe to the camera publisher node
        self.image_topic = "/camera/image_raw"

        self.detection_pub = self.create_publisher(
            String,
            "/cube_vision/detections",
            10,
        )

        self.debug_image_pub = self.create_publisher(
            Image,
            "/cube_vision/debug_image",
            10,
        )

        self.bridge = CvBridge()

        self.latest_frame = None
        self.latest_frame_time = None

        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10,
        )

        self.colors = ["red", "yellow", "blue"]

        self.timer = self.create_timer(0.1, self.process_frame)

        self.get_logger().info("Cube vision node started")
        self.get_logger().info(f"Subscribing to image topic: {self.image_topic}")
        self.get_logger().info("Publishing detections on: /cube_vision/detections")
        self.get_logger().info("Publishing debug image on: /cube_vision/debug_image")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"cv_bridge failed: {e}")
            return

        self.latest_frame = frame
        self.latest_frame_time = self.get_clock().now()

    def process_frame(self):
        if self.latest_frame is None:
            self.get_logger().warn(
                f"No image received yet on {self.image_topic}",
                throttle_duration_sec=2.0,
            )
            return

        frame = self.latest_frame.copy()
        debug = frame.copy()

        detections = {}
        missing = []

        for color in self.colors:
            targets, mask = find_positions_by_color(
                frame,
                color=color,
                min_area=300,
            )

            if len(targets) > 0:
                target = targets[0]
                detections[color] = target

                x = target["x"]
                y = target["y"]
                bx, by, bw, bh = target["box"]

                cv2.rectangle(debug, (bx, by), (bx + bw, by + bh), (0, 255, 0), 2)
                cv2.circle(debug, (x, y), 5, (0, 255, 0), -1)
                cv2.putText(
                    debug,
                    f"{color}: ({x}, {y})",
                    (bx, max(by - 10, 20)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2,
                )
            else:
                detections[color] = None
                missing.append(color)

        output = {
            "detections": detections,
            "missing": missing,
        }

        msg = String()
        msg.data = json.dumps(output)
        self.detection_pub.publish(msg)

        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
            self.debug_image_pub.publish(debug_msg)
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to publish debug image: {e}")

    def destroy_node(self):
        # No camera release needed because this node does not own the camera anymore.
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = CubeVisionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
