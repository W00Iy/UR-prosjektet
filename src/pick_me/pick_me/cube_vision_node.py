import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np
import json
import os
from ament_index_python.packages import get_package_share_directory

def find_positions_by_color(img, color="red", min_area=300):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    color_ranges = {
        "red": [
            (np.array([0, 50, 50]), np.array([10, 255, 255])),
            (np.array([170, 50, 50]), np.array([180, 255, 255]))
        ],
        "blue": [
            (np.array([100, 50, 50]), np.array([130, 255, 255]))
        ],
        "yellow": [
            (np.array([20, 50, 50]), np.array([35, 255, 255]))
        ]
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
        cv2.CHAIN_APPROX_SIMPLE
    )

    targets = []

    for contour in contours:
        area = cv2.contourArea(contour)

        if area < min_area:
            continue

        x, y, w, h = cv2.boundingRect(contour)
        cx = x + w // 2
        cy = y + h // 2

        targets.append({
            "x": int(cx),
            "y": int(cy),
            "area": float(area),
            "box": [int(x), int(y), int(w), int(h)]
        })

    targets.sort(key=lambda t: t["area"], reverse=True)

    return targets, mask


class CubeVisionNode(Node):
    def __init__(self):
        super().__init__("cube_vision_node")

        self.detection_pub = self.create_publisher(
            String,
            "/cube_vision/detections",
            10
        )

        self.debug_image_pub = self.create_publisher(
            Image,
            "/cube_vision/debug_image",
            10
        )

        self.bridge = CvBridge()

        self.cap = cv2.VideoCapture("/dev/video0", cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        package_share = get_package_share_directory("mega_project")
        self.image_path = os.path.join(package_share, "tricoloris.jpg")
        if not self.cap.isOpened():
            self.get_logger().error("Could not open camera")
        else:
            self.get_logger().info("Camera opened successfully")

        for _ in range(10):
            self.cap.read()

        self.colors = ["red", "yellow", "blue"]

        self.timer = self.create_timer(0.1, self.process_frame)

    def process_frame(self):
        frame = cv2.imread(self.image_path)

        if frame is None:
            self.get_logger().warn(f"Failed to read image: {self.image_path}")
            return

        debug = frame.copy()

        detections = {}
        missing = []

        for color in self.colors:
            targets, mask = find_positions_by_color(
                frame,
                color=color,
                min_area=300
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
                    (bx, by - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2
                )
            else:
                detections[color] = None
                missing.append(color)

        output = {
            "detections": detections,
            "missing": missing
        }

        msg = String()
        msg.data = json.dumps(output)
        self.detection_pub.publish(msg)

        debug_msg = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
        self.debug_image_pub.publish(debug_msg)

    def destroy_node(self):
        self.cap.release()
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
