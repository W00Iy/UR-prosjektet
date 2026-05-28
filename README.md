# UR-prosjektet

A ROS 2 workspace for a Universal Robots cell with robot description, ros2_control setup, MoveIt configuration, and a bringup launch package.

## Repository structure

- `src/bringup` - Top-level launch package for starting the full robot cell and MoveIt.
- `src/my_robot_cell_description` - URDF/XACRO robot and workspace description, RViz configuration and visualization launch files.
- `src/my_robot_cell_control` - ros2_control configuration, robot control launch files, and UR integration.
- `src/my_robot_cell_moveit_config` - MoveIt configuration package generated for the robot cell.
- `src/pick_me` - Python ROS package that launches both bringup and all the coustom nodes for movement and camera detection.

## Package overview

- `bringup`
  - Provides a single launch entrypoint to start robot control and MoveIt together.
- `my_robot_cell_description`
  - Contains the shared workspace URDF and visualization setup.
- `my_robot_cell_control`
  - Integrates the UR robot with ROS 2 control, including robot description, controllers, and calibration.
- `my_robot_cell_moveit_config`
  - Holds MoveIt planning and RViz launch configurations for the robot cell.
- `pick_me`
  - Launches both bringup and all the custom nodes for movment and camera detection.

## Prerequisites

- ROS 2 Jazzy installed.
- `colcon` build tool.
- Required ROS 2 packages including `ament_cmake`, `xacro`, `robot_state_publisher`, `rviz2`, `moveit_ros_move_group`, `ur_robot_driver`, and related UR/MoveIt packages.
- A Universal Robots arm reachable on the network if you want to run with real hardware.

## Build instructions

```bash
cd /home/wooly/UR-prosjektet
source /opt/ros/Jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Launch examples

### Start the full pick_me including MoveIt

```bash
source /home/wooly/UR-prosjektet/install/setup.bash
ros2 launch pick_me pick_me_system.launch.py 
```

This pick_me launch file starts the robot control stack, waits for the robot description, then starts MoveIt, RViz and all the nodes.

## Configuration notes

- `src/my_robot_cell_control/config/my_robot_calibration.yaml` contains kinematics calibration parameters.
- `src/my_robot_cell_control/config/ros2_controllers.yaml` contains controller definitions.
- `src/my_robot_cell_description/urdf/my_robot_cell.urdf.xacro` defines the shared workspace model.
- `src/my_robot_cell_moveit_config/config/my_robot_cell.srdf` and generated MoveIt launch files configure motion planning.

## Trobelshooting 

### View robot description in RViz

```bash
source /home/wooly/UR-prosjektet/install/setup.bash
ros2 launch my_robot_cell_description view_robot.launch.py ur_type:=ur5e
```

### Start robot control for a UR arm

```bash
source /home/wooly/UR-prosjektet/install/setup.bash
ros2 launch my_robot_cell_control rsp.launch.py robot_ip:=<robot_ip> ur_type:=ur5e launch_rviz:=true
```

Replace `<robot_ip>` with your robot controller IP address.

## Tips

- If you do not have a real robot connected, set `use_mock_hardware:=true` in the control launch arguments to enable mock hardware mode.
- Update the default `robot_ip` in `src/my_robot_cell_control/launch/start_robot.launch.py` when deploying to your robot.
