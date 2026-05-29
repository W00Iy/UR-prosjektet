import rclpy
from rclpy.node import Node

from std_msgs.msg import String

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


class ColorPickerNode(Node):

    def __init__(self):
        super().__init__("color_picker_node")

        self.state = RobotState.BOOT
        self.previous_state = None

        self.latest_detections = None

        self.colors_to_find = ["red", "yellow", "blue"]
        self.current_color_index = 0

        self.waiting_for_motion = False
        self.last_motion_status = None

        self.search_attempts = 0
        self.max_search_attempts = 5

        self.last_state_time = time.time()

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

        self.timer = self.create_timer(0.5, self.state_machine)

        self.get_logger().info("Color picker main controller started")

    def detection_callback(self, msg):
        try:
            self.latest_detections = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error("Could not parse detection data")
            self.set_state(RobotState.ERROR)

    def motion_status_callback(self, msg):
        try:
            data = json.loads(msg.data)
            self.last_motion_status = data.get("status")
        except json.JSONDecodeError:
            self.get_logger().error("Could not parse motion status")
            self.set_state(RobotState.ERROR)

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

    def boot_robot(self):
        self.get_logger().info("Booting system")

        

        self.set_state(RobotState.IDLE)

    def idle(self):
        # In the future this could wait for a UI button, service call, or keyboard command.
        pass

    def start_search(self):
        self.search_attempts = 0
        self.current_color_index = 0
        self.last_motion_status = None
        self.get_logger().info("Moving robot to home position before taking search image")
        self.send_robot_command({"command": "go_home"})
        self.set_state(RobotState.WAIT_FOR_SEARCH_MOTION)

    def search(self):
        self.get_logger().info("Checking for cubes")

        if self.latest_detections is None:
            self.get_logger().warn("No detections received yet")
            return

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

        self.last_motion_status = None

        self.send_robot_command({
            "command": "search_next_pose",
            "attempt": self.search_attempts
        })

        self.set_state(RobotState.WAIT_FOR_SEARCH_MOTION)

    def wait_for_search_motion(self):
        if self.last_motion_status == "done":
            self.get_logger().info("Search movement finished, taking image and checking cubes")
            self.last_motion_status = None
            self.set_state(RobotState.SEARCH)

        elif self.last_motion_status == "failed":
            self.get_logger().error("Search movement failed")
            self.set_state(RobotState.ERROR)

        elif self.state_timed_out(timeout_seconds=10.0):
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
        self.current_color_index +=1
        #self.set_state(RobotState.WAIT_FOR_POINT_MOTION)

    def wait_for_point_motion(self):
        color = self.colors_to_find[self.current_color_index]

        if self.last_motion_status == "done":
            self.get_logger().info(f"Finished pointing at {color}")

            self.current_color_index += 1
            self.last_motion_status = None

            self.set_state(RobotState.POINT_AT_COLOR)

        elif self.last_motion_status == "failed":
            self.get_logger().error(f"Failed to point at {color}")
            self.set_state(RobotState.ERROR)

        elif self.state_timed_out(timeout_seconds=15.0):
            self.get_logger().error(f"Timed out pointing at {color}")
            self.set_state(RobotState.ERROR)

    def done(self):
        self.get_logger().info("Finished pointing at all colors")
        self.set_state(RobotState.IDLE)

    def error_state(self):
        self.get_logger().error("System is in error state")

        # Later:
        # publish stop command
        # wait for reset command
        # move robot to safe position

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


def main(args=None):
    rclpy.init(args=args)

    node = ColorPickerNode()

    try:
        # Automatically start search after boot for now.
        # Later you can trigger this from a service or UI.
        node.start_search()

        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()