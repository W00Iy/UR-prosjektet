import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from visualization_msgs import msg
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose

from enum import Enum
import json
import time


class RobotState(Enum):
    BOOT = 0
    IDLE = 1
    SEARCH = 2
    EXTENDED_SEARCH = 3
    WAIT_FOR_SEARCH_MOTION = 4
    POINT_AT_COLOR = 5
    WAIT_FOR_POINT_MOTION = 6
    DONE = 7
    ERROR = 8
    CALIBRATION = 9


class ColorPickerNode(Node):

    def __init__(self):
        super().__init__("color_picker_node")

        self.state = RobotState.BOOT
        self.previous_state = None

        self.latest_detections = None
        self.isCalibrating = False
        self.colors_to_find = ["red", "yellow", "blue"]
        self.current_color_index = 0

        self.last_motion_status = None

        self.search_attempts = 0

        self.max_search_attempts = 6

        self.last_state_time = time.time()

        self.marker_frame = "ur5e_base_link"

        self.detection_sub = self.create_subscription(
            String,
            "/cube_vision/detections",
            self.detection_callback,
            10
        )

        self.motion_status_sub = self.create_subscription(
            String,
            "/cube_robot/motion_status",
            self.motion_status_callback,
            10
        )

        self.calibration_status_sub = self.create_subscription(
            String,
            "/camera_calibration/result",
            self.calibration_status_callback,
            10
        )

        self.change_to_state = self.create_subscription(
            String,
            "/state_machine/change_to_state",
            self.change_to_state_callback,
            10
        )

        self.robot_command_pub = self.create_publisher(
            String,
            "/cube_robot/command",
            10
        )

        self.target_color_pub = self.create_publisher(
            String,
            "/cube_robot/target_color",
            10
        )

        self.marker_pub = self.create_publisher(
            MarkerArray,
            "/cube_robot/color_markers",
            10
        )
        self.calibration_pub = self.create_publisher(
            String,
            "/camera_calibration/command",
            10
        )

        self.timer = self.create_timer(0.5, self.state_machine)

        # Start after a short delay.
        self.start_timer = self.create_timer(2.0, self.auto_start_once)

        self.get_logger().info("Color picker main controller started")

    def auto_start_once(self):
        self.start_timer.cancel()

        if self.state == RobotState.BOOT:
            self.set_state(RobotState.IDLE)

        if self.state == RobotState.IDLE:
            self.start_search()
    
    def change_to_state_callback(self, msg):
        if self.state == RobotState.ERROR or self.state == RobotState.IDLE:
            self.get_logger().info("Trying to change state based on external command")
        else :
            self.get_logger().warn("Ignoring external command to change state because current state is not ERROR or IDLE")
            return
        try:
            data = json.loads(msg.data)
            new_state_str = data.get("state")

            if new_state_str is None:
                self.get_logger().error("Received change_to_state command without 'state' field")
                return

            try:
                new_state = RobotState[new_state_str]
            except KeyError:
                self.get_logger().error(f"Received invalid state in change_to_state command: {new_state_str}")
                return

            self.get_logger().info(f"Received external command to change state to {new_state.name}")
            self.set_state(new_state)

        except json.JSONDecodeError:
            self.get_logger().error("Could not parse change_to_state command, ignoring,use this format : {\"state\": \"STATE_NAME\"}")

    def calibration_status_callback(self, msg):
        
        
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error("Could not parse calibration result, trying again...")
            self.isCalibrating = False
            self.calibration()  # Try again
            return

        if data.get("success"):
            k_matrix = data.get("k_matrix")
            self.get_logger().info(f"Calibration succeeded. K={k_matrix}, moving on to preform a search")
            self.start_search()
            self.isCalibrating = False
        else:
            self.get_logger().error(f"Calibration failed: {data.get('message')}, Trying again...")
            self.isCalibrating = False
            self.calibration()  # Try again


    def detection_callback(self, msg):
        try:
            self.latest_detections = json.loads(msg.data)
            self.publish_color_markers()
        except json.JSONDecodeError:
            self.get_logger().error("Could not parse detection data")
            self.set_state(RobotState.ERROR)

    def motion_status_callback(self, msg):
        try:
            data = json.loads(msg.data)
            self.last_motion_status = data.get("status")
            self.get_logger().info(f"Received motion status: {self.last_motion_status}")
            
        except json.JSONDecodeError:
            self.get_logger().error("Could not parse motion status, trying again...")
            

    def set_state(self, new_state):
        if self.state != new_state:
            self.get_logger().info(
                f"State change: {self.state.name} -> {new_state.name}"
            )
            self.previous_state = self.state
            self.state = new_state
            self.last_state_time = time.time()

    def state_machine(self):
        match self.state:
            case RobotState.BOOT:
                self.boot_robot()

            case RobotState.IDLE:
                self.idle()

            case RobotState.SEARCH:
                self.search()

            case RobotState.EXTENDED_SEARCH:
                self.extended_search()

            case RobotState.WAIT_FOR_SEARCH_MOTION:
                self.wait_for_search_motion()

            case RobotState.POINT_AT_COLOR:
                self.point_at_current_color()

            case RobotState.WAIT_FOR_POINT_MOTION:
                self.wait_for_point_motion()

            case RobotState.DONE:
                self.done()

            case RobotState.ERROR:
                self.error_state()
            case RobotState.CALIBRATION:
                self.calibration();

    def boot_robot(self):
        self.get_logger().info("Booting system")
        self.set_state(RobotState.CALIBRATION)

    def idle(self):
        pass
    
    def calibration(self):
        if self.isCalibrating:
            return
        self.get_logger().info("Starting camera calibration")
        self.set_state(RobotState.CALIBRATION)
        self.calibration_pub.publish(String(data="{\"command\":\"calibrate\"}"))
        self.isCalibrating = True

    def start_search(self):
        self.search_attempts = 0
        self.current_color_index = 0
        self.last_motion_status = None
        self.latest_detections = None

        self.get_logger().info("Moving robot to home position before taking search image")

        self.send_robot_command({
            "command": "go_home"
        })

        self.set_state(RobotState.WAIT_FOR_SEARCH_MOTION)

    def search(self):
        self.get_logger().info("Checking for cubes")

        if self.latest_detections is None:
            self.get_logger().warn("No detections received yet")
            return

        self.publish_color_markers()

        if self.all_colors_found():
            self.get_logger().info("All colors found")
            self.current_color_index = 0
            self.set_state(RobotState.POINT_AT_COLOR)
        else:
            missing = self.get_missing_colors()
            self.get_logger().warn(f"Missing colors: {missing}")
            self.set_state(RobotState.EXTENDED_SEARCH)

    def extended_search(self):
        if self.search_attempts >= self.max_search_attempts:
            self.get_logger().error("Max search attempts reached")
            self.set_state(RobotState.ERROR)
            return

        self.search_attempts += 1

        self.get_logger().info(
            f"Extended search attempt {self.search_attempts}/{self.max_search_attempts}"
        )

        # Clear old camera results before moving to the next search pose.
        # This prevents SEARCH from reusing detections from the previous pose.
        self.latest_detections = None
        self.last_motion_status = None

        # The motion controller owns the actual search-pose list.
        # This node only asks it to move to the next one.
        self.send_robot_command({
            "command": "search_next_pose"
        })

        self.set_state(RobotState.WAIT_FOR_SEARCH_MOTION)

    def wait_for_search_motion(self):
        if self.last_motion_status == "done":
            self.get_logger().info("Search movement finished, checking cubes")
            self.last_motion_status = None
            self.set_state(RobotState.SEARCH)

        elif self.last_motion_status == "failed":
            self.get_logger().error("Search movement failed")
            self.set_state(RobotState.ERROR)

        elif self.state_timed_out(timeout_seconds=200.0):
            self.get_logger().error("Timed out waiting for search motion")
            self.set_state(RobotState.ERROR)

    def point_at_current_color(self):
        if self.current_color_index >= len(self.colors_to_find):
            self.set_state(RobotState.DONE)
            return

        color = self.colors_to_find[self.current_color_index]

        if not self.color_found(color):
            self.get_logger().warn(f"{color} is no longer visible")
            self.set_state(RobotState.SEARCH)
            return

        self.get_logger().info(f"Commanding robot to point at {color}")

        self.last_motion_status = None

        self.send_target_color({
            "action": "point",
            "color": color
        })

        # Wait until the robot reports motion_status == "done".
        self.set_state(RobotState.WAIT_FOR_POINT_MOTION)

    def wait_for_point_motion(self):
        if self.current_color_index >= len(self.colors_to_find):
            self.set_state(RobotState.DONE)
            return

        color = self.colors_to_find[self.current_color_index]

        if self.last_motion_status == "done":
            self.get_logger().info(f"Finished pointing at {color}")

            self.current_color_index += 1
            self.last_motion_status = None

            self.set_state(RobotState.POINT_AT_COLOR)

        elif self.last_motion_status == "failed":
            self.get_logger().error(f"Failed to point at {color}")
            self.set_state(RobotState.ERROR)

        elif self.state_timed_out(timeout_seconds=200.0):
            self.get_logger().error(f"Timed out pointing at {color}")
            self.set_state(RobotState.ERROR)

    def done(self):
        self.get_logger().info("Finished pointing at all colors, returning robot to home")

        self.last_motion_status = None

        self.send_robot_command({
            "command": "go_home"
        })

        self.set_state(RobotState.IDLE)

    def error_state(self):
        self.get_logger().error("System is in error state")

    def all_colors_found(self):
        for color in self.colors_to_find:
            if not self.color_found(color):
                return False

        return True

    def color_found(self, color):
        if self.latest_detections is None:
            return False

        detections = self.latest_detections.get("detections", {})
        return detections.get(color) is not None

    def get_missing_colors(self):
        missing = []

        for color in self.colors_to_find:
            if not self.color_found(color):
                missing.append(color)

        return missing

    def send_robot_command(self, command_dict):
        msg = String()
        msg.data = json.dumps(command_dict)
        self.robot_command_pub.publish(msg)

        self.get_logger().info(f"Sent robot command: {msg.data}")

    def send_target_color(self, command_dict):
        msg = String()
        msg.data = json.dumps(command_dict)
        self.target_color_pub.publish(msg)

        self.get_logger().info(f"Sent target color command: {msg.data}")

    def state_timed_out(self, timeout_seconds):
        return time.time() - self.last_state_time > timeout_seconds

    def publish_color_markers(self):
        if self.latest_detections is None:
            return

        detections = self.latest_detections.get("detections", {})
        marker_array = MarkerArray()

        marker_id = 0

        for color in self.colors_to_find:
            detection = detections.get(color)

            if detection is None:
                continue

            pose = self.get_marker_pose_from_detection(detection)

            if pose is None:
                continue

            sphere = Marker()
            sphere.header.frame_id = self.marker_frame
            sphere.header.stamp = self.get_clock().now().to_msg()
            sphere.ns = "detected_colors"
            sphere.id = marker_id
            marker_id += 1
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose = pose
            sphere.scale.x = 0.04
            sphere.scale.y = 0.04
            sphere.scale.z = 0.04
            sphere.color.a = 1.0

            self.set_marker_color(sphere, color)

            marker_array.markers.append(sphere)

            text = Marker()
            text.header.frame_id = self.marker_frame
            text.header.stamp = self.get_clock().now().to_msg()
            text.ns = "detected_color_labels"
            text.id = marker_id
            marker_id += 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose = Pose()
            text.pose.position.x = pose.position.x
            text.pose.position.y = pose.position.y
            text.pose.position.z = pose.position.z + 0.07
            text.pose.orientation.w = 1.0
            text.scale.z = 0.04
            text.color.a = 1.0
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.text = color

            marker_array.markers.append(text)

        if marker_array.markers:
            self.marker_pub.publish(marker_array)

    def get_marker_pose_from_detection(self, detection):

        try:
            source = detection

            if isinstance(detection, dict) and "pose" in detection:
                pose_data = detection["pose"]

                if isinstance(pose_data, dict) and "position" in pose_data:
                    source = pose_data["position"]
                else:
                    source = pose_data

            elif isinstance(detection, dict) and "position" in detection:
                source = detection["position"]

            elif isinstance(detection, dict) and "world" in detection:
                source = detection["world"]

            if not isinstance(source, dict):
                return None

            if not all(axis in source for axis in ["x", "y", "z"]):
                return None

            pose = Pose()
            pose.position.x = float(source["x"])
            pose.position.y = float(source["y"])
            pose.position.z = float(source["z"])
            pose.orientation.w = 1.0

            return pose

        except Exception as e:
            self.get_logger().warn(f"Could not create marker pose from detection: {e}")
            return None

    def set_marker_color(self, marker, color):
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        if color == "red":
            marker.color.r = 1.0
        elif color == "yellow":
            marker.color.r = 1.0
            marker.color.g = 1.0
        elif color == "blue":
            marker.color.b = 1.0
        else:
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0


def main(args=None):
    rclpy.init(args=args)

    node = ColorPickerNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
