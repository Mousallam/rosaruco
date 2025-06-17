# ROS ArUco Robot Project

This workspace now includes a basic framework for a real-time ArUco based localization
and navigation system.  The original example nodes are kept in the `QRfollowing`
package while the new implementation lives in `aruco_localization`.

## Packages

### `QRfollowing`
Contains the original example nodes:
- `camera_intrinsics_node.py` – calibrate the camera.
- `camera_node.py` – detect a single marker and publish its path.
- `crtmovement.py` – simple forward/backward controller.
- `odom_control_movement.py` – controller using odometry feedback.

### `aruco_localization`
Folder layout:
```
aruco_localization/
  scripts/                # Python nodes
  launch/                 # Launch files
  config/                 # Camera calibration YAML
  maps/                   # Static marker map
```

Provided node templates:
- `camera_calibration_node.py`
- `aruco_detector.py`
- `localization_manager.py`
- `motion_commander.py`

## Example Usage
```
# Calibrate the camera
roslaunch aruco_localization calibration.launch

# Run detection and localization
roslaunch aruco_localization bringup.launch

# Send a motion command
rostopic pub /command std_msgs/String "forward"
```

These nodes are only skeletons and need further implementation to be fully
functional.

