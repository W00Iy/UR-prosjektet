from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            "my_robot_cell",
            package_name="my_robot_cell_moveit_config"
        )
        .robot_description(file_path="config/my_robot_cell.urdf.xacro")
        .robot_description_semantic(file_path="config/my_robot_cell.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    return LaunchDescription([
        Node(
            package="camera_cali",
            executable="move_and_calibrate",
            output="screen",
            parameters=[
                moveit_config.to_dict()
            ],
        )
    ])