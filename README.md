# ROS ArUco Robot Project

This workspace contains a simple set of Python nodes to calibrate a camera, detect an ArUco marker and publish its pose, and drive a robot using odometry feedback.

## Nodes

### `camera_intrinsics_node.py`
Calibrates the camera using a chessboard pattern. After capturing at least three images press `q` to finish and the node will continuously publish the resulting `sensor_msgs/CameraInfo` on `/camera/camera_info`.

### `camera_node.py`
Detects an ArUco ID 0 marker from the camera feed, overlays the position on the image, and publishes the marker trajectory as a `nav_msgs/Path` on `/trajectory`. The raw annotated frames are published on `/camera/image_raw`.

### `crtmovement.py`
Basic controller that subscribes to `/trajectory` and drives the robot forward or backward on the x‐axis depending on the marker’s y position relative to the camera.

### `odom_control_movement.py`
New controller that uses odometry feedback. It subscribes to `/trajectory` for the target marker distance and to `/odom` to track robot motion. The robot moves forward or backward until the travelled distance matches the detected marker distance.

## Usage
1. Source your workspace and make sure the robot is running.
2. Calibrate the camera:
   ```
   rosrun QRfollowing camera_intrinsics_node.py
   ```
3. In a new terminal, start the marker detection:
   ```
   rosrun QRfollowing camera_node.py
   ```
4. To drive using odometry feedback run:
   ```
   rosrun QRfollowing odom_control_movement.py
   ```

Adjust parameters such as `~kp`, `~tolerance` and `~max_speed` with ROS parameters if needed.
