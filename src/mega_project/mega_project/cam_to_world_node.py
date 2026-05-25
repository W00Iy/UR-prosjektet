import rclpy

from rclpy.node import Node
from std_msgs.msg import String


from tf2_ros import Buffer, TransformListener
from sensor_msgs.msg import CameraInfo

from geometry_msgs.msg import PoseStamped
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from rclpy.duration import Duration


import numpy as np
import json



class cam_to_world(Node):
    def __init__(self):
        super().__init__('cam_to_pose')
        
        self.latest_cam = None
        self.K = np.array([
            [803.08080926841205, 0, 272.24043631557367], 
            [0, 931.70126479938983, 278.03098639361787],
            [0, 0, 1]
        ])
    
        self.base_frame = 'base_link'
        self.ee_frame = 'tool0'
        self.camera_frame = self.ee_frame
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Camera-Matrix K
        #self.subscription = self.create_subscription(
        #    CameraInfo,
        #    '/camera/camera_info',
        #    self.cam_matrix_callback,
        #    10
        #)
    
        # Pixels from camera
        self.cam_subscription = self.create_subscription(
            String,
            '/cube_vision/detections', #.data
            self.cam_callback,
            10
        )
        
        #self.goal_publisher = self.create_publisher(
        #    PoseStamped, 
        #    '/goal_pose', 
        #    10
        #)
        
        self.cube_pos_publisher = self.create_publisher(
            String,
            '/cube_pos',
            10
        )

        self.cube_height = 0.05 #50mm = 0.05m

    
        
    #def cam_matrix_callback(self, msg):
    #    self.K = np.array(msg.k).reshape(3, 3)
    #    self.get_logger().info("[Cam_matrix_callback]: " + str(msg))
    
    def cam_callback(self, msg):
        self.latest_cam = json.loads(msg.data)
        self.process_data()
        #self.get_logger().info("[Cam_callback]: " + str(msg))
    def transform_to_matrix(self, tf):
        t = tf.transform.translation
        q = tf.transform.rotation

        x = q.x
        y = q.y
        z = q.z
        w = q.w

        R = np.array([
            [1 - 2*y*y - 2*z*z,     2*x*y - 2*z*w,     2*x*z + 2*y*w],
            [    2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z,     2*y*z - 2*x*w],
            [    2*x*z - 2*y*w,     2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y]
        ])

        T = np.eye(4)
        T[0:3, 0:3] = R
        T[0, 3] = t.x
        T[1, 3] = t.y
        T[2, 3] = t.z

        return T
    
    def camera_to_point(self, r, z_world):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.camera_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2)
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().error("Could convert from camera_to_point")
            return None

        z = z_world - self.cube_height
        
        tool_to_cam = np.array([
            [1, 0, 0, 0.09],
            [0, 1, 0,    0],
            [0, 0, 1,    0],
            [0, 0, 0,    1]
        ])

        wTc = self.transform_to_matrix(tf) @ tool_to_cam
        
        #self.get_logger().info("[world_to_camera transform]: \n" + str(wTc))

        PI_tilde = np.vstack((z * np.eye(3), np.array([[0, 0, 1]])))
        

        k = self.K

        p_world = wTc @ PI_tilde @ np.linalg.inv(k) @ r
        return p_world
    
    def get_current_ee_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.ee_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2)
            )
            return tf.transform.translation, tf.transform.rotation

        except:
            self.get_logger().error("Could not get end-effector pose")
            return None, None
    
    
    def process_data(self):
        # procedure
        
        translation, rotation = self.get_current_ee_pose()
        
        if self.latest_cam is None:
            return
        
        #if self.K is None:
        #    self.get_logger().error("Camera Matrix(K) doesn't exist")
        #    return
        
        if translation is None:
            self.get_logger().error("Translation doesn't exist")
            return
        if rotation is None:
            self.get_logger().error("Rotation doesn't exist")
            return
        

        x_red = None
        y_red = None

        x_blue = None
        y_blue = None

        x_blue = None
        y_blue = None

        r_red = None
        r_blue = None
        r_yellow = None

        p_world_red = None
        p_world_blue = None
        p_world_yellow = None

        
        try:
            x_red = self.latest_cam['detections']['red']['x']
            y_red = self.latest_cam['detections']['red']['y']
            
            r_red = np.array([
                [x_red],
                [y_red],
                [1.0]
            ])

            p_world_red = self.camera_to_point(r_red, translation.z)

        except (KeyError, TypeError, ValueError):
            self.get_logger().warn('Did not find red cube')
        try:
            x_blue = self.latest_cam['detections']['blue']['x']
            y_blue = self.latest_cam['detections']['blue']['y']

            r_blue = np.array([
                [x_blue],
                [y_blue],
                [1.0]
            ])

            p_world_blue = self.camera_to_point(r_blue, translation.z)
            
        except (KeyError, TypeError, ValueError):
            self.get_logger().warn('Did not find blue cube')
        
        try:
            x_yellow = self.latest_cam['detections']['yellow']['x']
            y_yellow = self.latest_cam['detections']['yellow']['y']

            r_yellow = np.array([
                [x_yellow],
                [y_yellow],
                [1.0]
            ])

            p_world_yellow = self.camera_to_point(r_yellow, translation.z)

        except (KeyError, TypeError, ValueError):
            self.get_logger().warn('Did not find yellow cube')
            #return
        

        
        data ={
                "detected": {
                    "red": (p_world_red is not None),
                    "blue": (p_world_blue is not None),
                    "yellow": (p_world_yellow is not None)
                },
                "position": {
                    "red": (p_world_red.tolist() if p_world_red is not None else None),
                    "blue": (p_world_blue.tolist() if p_world_blue is not None else None),
                    "yellow": (p_world_yellow.tolist() if p_world_yellow is not None else None),
                }
        }

        json_string = json.dumps(data, indent=4)

        #self.get_logger().debug(json_string)
        
        msg = String()
        msg.data = json_string

        self.cube_pos_publisher.publish(msg)

        #goal_pose = PoseStamped()
        #goal_pose.header.stamp = self.get_clock().now().to_msg()
        #goal_pose.header.frame_id = self.base_frame
        
        #goal_pose.pose.position.x = float(p_world[0, 0])
        #goal_pose.pose.position.y = float(p_world[1, 0])
        
        #goal_pose.pose.position.x = 0.5
        #goal_pose.pose.position.y = 0.5


        #self.get_logger().info('goal x: ' + str(goal_pose.pose.position.x) + '  goal y: ' + str(goal_pose.pose.position.y))
        
            
        #goal_pose.pose.position.z = 0.5#translation.z

        #goal_pose.pose.orientation.x = 1.0
        #goal_pose.pose.orientation.y = 0.0
        #goal_pose.pose.orientation.z = 0.0
        #goal_pose.pose.orientation.w = 0.0
            
        #self.goal_publisher.publish(goal_pose)
        
    



def main(args=None):
    rclpy.init(args=args)

    node = cam_to_world()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
    
