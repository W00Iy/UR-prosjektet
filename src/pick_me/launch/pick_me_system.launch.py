from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    robot_ip = LaunchConfiguration("robot_ip")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    headless_mode = LaunchConfiguration("headless_mode")
    launch_rviz = LaunchConfiguration("launch_rviz")
    ur_type = LaunchConfiguration("ur_type")

    declared_arguments = [
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur5e",
            description="UR robot type.",
        ),
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.56.101",
            description="IP address of the robot.",
        ),
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="true",
            description="Use mock hardware instead of real robot.",
        ),
        DeclareLaunchArgument(
            "headless_mode",
            default_value="true",
            description="Run UR driver in headless mode.",
        ),
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="false",
            description="Launch UR driver RViz.",
        ),
    ]

    moveit_config = (
        MoveItConfigsBuilder(
            "my_robot_cell",
            package_name="my_robot_cell_moveit_config"
        )
        .to_moveit_configs()
    )

    robot_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("bringup"),
                "launch",
                "cell_brinpup.launch.py",
            ])
        ),
        launch_arguments={
            "ur_type": ur_type,
            "robot_ip": robot_ip,
            "use_mock_hardware": use_mock_hardware,
            "headless_mode": headless_mode,
            "launch_rviz": launch_rviz,
        }.items(),
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

    simple_camera_publisher_node = Node(
        package="pick_me",
        executable="simple_camera_publisher_node",
        name="simple_camera_publisher_node",
        output="screen",
    )

    return LaunchDescription(
        declared_arguments
        + [
            robot_moveit_launch,

            TimerAction(
                period=8.0,
                actions=[
                    motion_controller_node,
                    cam_to_world_node,
                    cube_vision_node,
                    camera_calibration_node,
                    simple_camera_publisher_node,
                ],
            ),

            TimerAction(
                period=12.0,
                actions=[
                    main_controller_node,
                ],
            ),
        ]
    )