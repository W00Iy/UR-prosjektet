import rclpy

from rclpy.node import Node
from std_msgs.msg import String

from geometry_msgs.msg import Pose, PoseStamped

from moveit.planning import MoveItPy

# Collsion-protection
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from shape_msgs.msg import SolidPrimitive



class motion_controller(Node):
    def __init__(self):
        super().__init__('motion_controller_ros')

        self.moveit = MoveItPy(node_name="motion_controller")
        self.arm = self.moveit.get_planning_component("ur_manipulator")
        # Default starting position
        self.arm.set_goal_state(configuration_name="home")


        self.planning_scene_monitor = self.moveit.get_planning_scene_monitor()


        self.busy = False
         
        # Adding camera (mount) to planning scene inspired by the "Motion Planning Python API"-tutorial
        with self.planning_scene_monitor.read_write() as scene:
            attached_camera_mount = AttachedCollisionObject()
            attached_camera_mount.link_name = "tool0"
            attached_camera_mount.touch_links = [
                "tool0",
                "flange",
                "wrist_3_link"
            ]


            camera_mount = CollisionObject()
            camera_mount.header.frame_id = "tool0"
            camera_mount.id = "camera_mount"
             
            mount_pose = Pose()
            mount_pose.position.x = 0.06
            mount_pose.position.y = 0.0
            mount_pose.position.z = 0.005


            mount = SolidPrimitive()

            mount.type = SolidPrimitive.BOX
            mount.dimensions = [0.12, 0.08, 0.01]

            camera_mount.primitives.append(mount)
            camera_mount.primitive_poses.append(mount_pose)
            camera_mount.operation = CollisionObject.ADD

            attached_camera_mount.object = camera_mount
            
            scene.process_attached_collision_object(attached_camera_mount)
            scene.current_state.update()


        self.pose_subscriber = self.create_subscription(
            PoseStamped,
            "/goal_pose",
            self.subscriber_callback,
            10
        )

    def subscriber_callback(self, goal_pose):
        if self.busy:
            self.get_logger().warn("The robot is already in motion")
            return
        self.busy = True
        
        self.arm.set_start_state_to_current_state()

        self.arm.set_goal_state(
            pose_stamped_msg = goal_pose,
            pose_link="tool0"
        )

        #self.arm.set_goal_state(configuration_name="home")

        result = self.arm.plan()

        if result:
            self.get_logger().info("Plan found, moving now")
            self.moveit.execute(result.trajectory, controllers=["scaled_joint_trajectory_controller"])
        else:
            self.get_logger().error("no valid plan")
        self.busy = False

def main(args=None):
    rclpy.init(args=args)
    
    node = motion_controller()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

