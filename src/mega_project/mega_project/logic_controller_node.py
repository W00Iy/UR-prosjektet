import rclpy

from rclpy.node import Node
from std_msgs.msg import String


from tf2_ros import Buffer, TransformListener
from sensor_msgs.msg import CameraInfo

from geometry_msgs.msg import Pose, PoseStamped

from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from rclpy.duration import Duration

import numpy as np
import json


class LogicController(Node):
    def __init__(self):
        super().__init__('logic_controller')


        self.positions = None

        # Subscribing to cam_to_world-node
        self.position_subscription = self.create_subscription(
            String,
            '/cube_pos',
            self.positions_callback,
            10
        )

        self.goal_publisher = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10
        )



    def positions_callback(self, msg):
        self.positions = msg
        #self.get_logger().info(msg.data)
        #print(msg.data)
        scan(self)

def scan(self):    
    goal_pose = PoseStamped()
    goal_pose.header.stamp = self.get_clock().now().to_msg()
    goal_pose.header.frame_id = "base_link"
    
    #goal_pose.pose.position.x = float(p_world[0, 0])
    #goal_pose.pose.position.y = float(p_world[1, 0])
 
    goal_pose.pose.position.x = 0.5
    goal_pose.pose.position.y = 0.5

    self.get_logger().info('goal x: ' + str(goal_pose.pose.position.x) + '  goal y:       ' + str(goal_pose.pose.position.y))
  
    goal_pose.pose.position.z = 0.5#translation.z

    goal_pose.pose.orientation.x = 1.0
    goal_pose.pose.orientation.y = 0.0
    goal_pose.pose.orientation.z = 0.0
    goal_pose.pose.orientation.w = 0.0
  
    self.goal_publisher.publish(goal_pose)


def main(args=None):
    rclpy.init(args=args)

    node = LogicController()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


