from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("my_robot_cell_control"),
                "launch",
                "start_robot.launch.py",
            ])
        ),
        launch_arguments={
            "use_mock_hardware": "true",
            "headless_mode": "true",
            "launch_rviz": "false",
        }.items(),
    )

    wait_robot_description = Node(
        package="ur_robot_driver",
        executable="wait_for_robot_description",
        output="screen",
    )

    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("my_robot_cell_moveit_config"),
                "launch",
                "move_group.launch.py",
            ])
        )
    )

    moveit_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("my_robot_cell_moveit_config"),
                "launch",
                "moveit_rviz.launch.py",
            ])
        )
    )

    return LaunchDescription([
        control_launch,
        wait_robot_description,
        RegisterEventHandler(
            OnProcessExit(
                target_action=wait_robot_description,
                on_exit=[
                    move_group_launch,
                    moveit_rviz_launch,

                ],
            )
        ),
    ])