from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder(
            "my_robot_cell",
            package_name="my_robot_cell_moveit_config"
        )
        .to_moveit_configs()
    )

    # Your already-working robot + MoveIt/RViz launch
    robot_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("bringup"),
                "launch",
                "cell_brinpup.launch.py",
            ])
        )
    )

    cube_vision_node = Node(
        package="pick_me",
        executable="cube_vision_node",
        name="cube_vision_node",
        output="screen",
    )

    cam_to_world_node = Node(
        package="pick_me",
        executable="cam_to_world_node",
        name="cam_to_world_node",
        output="screen",
    )

    motion_controller_node = Node(
        package="pick_me",
        executable="motion_controller_node",
        name="motion_controller",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
        ],
    )

    main_controller_node = Node(
        package="pick_me",
        executable="main_controller_node",
        name="main_controller_node",
        output="screen",
    )
    camera_calibration_node = Node(
        package="pick_me",
        executable="camera_calibration_node",
        name="camera_calibration_node",
        output="screen",
    )
    start_pick_me_nodes = TimerAction(
        period=10.0,
        actions=[
            cube_vision_node,
            cam_to_world_node,
            motion_controller_node,
            main_controller_node,
            camera_calibration_node,
        ],
    )
    

    return LaunchDescription([
        robot_moveit_launch,

        # Start motion controller after move_group/RViz stack has had time to start
        TimerAction(
            period=8.0,
            actions=[
                motion_controller_node, cam_to_world_node, cube_vision_node, camera_calibration_node
            ],
        ),

        # Start the main logic later, so motion_controller is already subscribed
        TimerAction(
            period=12.0,
            actions=[
                main_controller_node,
            ],
        ),

        # Add these later when basic robot command flow works
        # TimerAction(period=12.0, actions=[cube_vision_node, cam_to_world_node]),
    ])