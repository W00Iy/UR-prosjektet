# UR-prosjektet

A ROS 2 workspace for a Universal Robots cell that detects and points to colored
cubes using machine vision. The cubes can be placed randomly on a table, and the robot should
identify the cubes and point to them in a specific order based on color. The system is designed
to operate autonomously by combining image processing, robot control, and motion planning.

## Repository structure

- `src/bringup` - Top-level launch package for starting the full robot cell and MoveIt.
- `src/my_robot_cell_description` - URDF/XACRO robot and workspace description, RViz configuration and visualization launch files.
- `src/my_robot_cell_control` - ros2_control configuration, robot control launch files, and UR integration.
- `src/my_robot_cell_moveit_config` - MoveIt configuration package generated for the robot cell.
- `src/pick_me` - Python ROS package that launches both bringup and all the custom nodes for movement and camera detection.

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
  - Launches both bringup and all the custom nodes for movement and camera detection.

## Node overview
## cam_to_world_node
Cam_to_world_node converts cube detections from camera image coordinates into real-world robot coordinates.
It subscribes to camera detections and target color commands, then calculates the cube position using TF transforms and camera calibration data.
The node publishes a goal pose for the robot and visualization markers for RViz.
It acts as the connection between the vision system and robot motion planning.

## camera_calibration_node
Camera_calibration_node performs automatic camera calibration using a chessboard pattern and robot-guided image capture.
It moves the robot through predefined calibration poses using MoveIt and captures images from the camera.
The node uses OpenCV to detect chessboard corners and compute the camera matrix and lens distortion parameters.
The calibration results are then published and saved for later use in the vision system.

## color_picker
color_picker acts as the main state machine and controller for the robot system.
It controls the search process, calibration, and robot actions for pointing at colored cubes in the correct order.
The node communicates with the vision system and motion controller using ROS topics.
It also publishes visualization markers and handles system errors and retries.

## cube_vision_node
cube_vision_node detects colored cubes in the camera image using OpenCV image processing.
It subscribes to the camera image topic and searches for red, yellow, and blue objects using HSV color filtering.
The node calculates the pixel position of each detected cube and publishes the results as JSON data.
It also publishes a debug image with bounding boxes and labels for visualization in RViz or rqt_image_view.

## motion_controller_node
Motion_controller_node controls the robot movements using MoveIt through the /move_action action server.
It receives goal poses from /goal_pose and robot commands from /cube_robot/command.
The node can move the robot home, move to search poses, or point at detected cube positions.
It also publishes motion status messages so the state machine knows when movement is done or failed.

## simple_camera_publisher_node
Simple_camera_publisher_node reads images directly from the connected USB camera using OpenCV.
It converts each camera frame into a ROS 2 Image message using cv_bridge.
The images are published on /camera/image_raw at a fixed frame rate.
This gives the vision system a live camera feed to detect the colored cubes.

## Prerequisites

- ROS 2 Jazzy installed.
- `colcon` build tool.
- Required ROS 2 packages including `ament_cmake`, `xacro`, `robot_state_publisher`, `rviz2`, `moveit_ros_move_group`, `ur_robot_driver`, and related UR/MoveIt packages.
- A Universal Robots arm reachable on the network if you want to run with real hardware.

## Build instructions

```bash
cd /home/user/UR-prosjektet
source /opt/ros/Jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Launch examples

### Start the first the drivers and MoveIt

```bash
source install/setup.bash
ros2 launch bringup cell_brinpup.launch.py
```
This `bringup` launch file starts the robot control stack, waits for the robot description, and then starts MoveIt and RViz
ensure the nodes are up and running before launching

```bash
source install/setup.bash
ros2 launch pick_me pick_me_system.launch.py
```
in a new terminal.

This `pick_me` launch file starts the cam_to_world, camera_calibration, motion_control, cube_vision and simple_camera_publisher. After a short timer to ensure the nodes have started, the main state machine node is started, color_picker.

## Configuration notes

* `src/my_robot_cell_control/config/my_robot_calibration.yaml` contains kinematics calibration parameters.
* `src/my_robot_cell_control/config/ros2_controllers.yaml` contains controller definitions.
* `src/my_robot_cell_description/urdf/my_robot_cell.urdf.xacro` defines the shared workspace model.
* `src/my_robot_cell_moveit_config/config/my_robot_cell.srdf` and the generated MoveIt launch files configure motion planning.

## Troubleshooting

### View robot description in RViz

```bash
source /home/user/UR-prosjektet/install/setup.bash
ros2 launch my_robot_cell_description view_robot.launch.py ur_type:=ur5e
```

### Start robot control for a UR arm

```bash
source /home/user/UR-prosjektet/install/setup.bash
ros2 launch my_robot_cell_control rsp.launch.py robot_ip:=<robot_ip> ur_type:=ur5e launch_rviz:=true
```

Replace `<robot_ip>` with your robot controller IP address.

## Tips

* If you do not have a real robot connected, set `use_mock_hardware:=true` in the control launch arguments to enable mock hardware mode.
* Update the default `robot_ip` in `src/my_robot_cell_control/launch/start_robot.launch.py` when deploying to your robot.
